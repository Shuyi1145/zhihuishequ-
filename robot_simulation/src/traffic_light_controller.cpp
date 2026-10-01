#include <array>
#include <cmath>
#include <functional>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
// Gazebo 模型插件：按仿真时间驱动红、绿、黄三盏灯，并通过 Gazebo
// transport 将新的材质脚本发送给客户端。灯的显示和灯态计算都在插件
// 内完成，因此不会依赖 ROS 节点或墙上时间。
class TrafficLightController : public ModelPlugin
{
  public: void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    // 保存模型和承载灯面 visual 的 link。当前 SDF 将三个灯面放在 body
    // link 下，后续通过 link->Visuals() 找到它们的完整 scoped name。
    this->model = model;
    this->link = model->GetLink("body");
    if (!this->link)
    {
      gzerr << "[traffic_light] Missing body link\n";
      return;
    }

    // 从插件 SDF 读取每个颜色的持续时间。数组下标约定为：
    // 0 = 红灯，1 = 绿灯，2 = 黄灯；缺省值对应一个 30 秒周期。
    this->durations = {{
      this->Duration(sdf, "red_duration", 10.0),
      this->Duration(sdf, "green_duration", 15.0),
      this->Duration(sdf, "yellow_duration", 5.0)
    }};

    // ~/visual 是 Gazebo 当前世界的 visual 主题。发布 msgs::Visual 可以
    // 在模型已经生成后更新材质，而不需要重新生成模型或修改 SDF。
    this->node.reset(new transport::Node());
    this->node->Init(model->GetWorld()->Name());
    this->visualPub = this->node->Advertise<msgs::Visual>("~/visual");
    // 使用仿真时钟而不是系统时钟：暂停、快进或重置 Gazebo 时，灯的
    // 周期会和世界状态一起暂停、调整或重新开始。
    this->cycleStart = model->GetWorld()->SimTime();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&TrafficLightController::OnUpdate, this, std::placeholders::_1));
  }

  // 读取一个正的有限持续时间；SDF 缺少该参数或参数非法时回退到
  // fallback，避免出现除零、负周期或 NaN 导致插件无法更新。
  private: double Duration(sdf::ElementPtr sdf, const std::string &name,
                           double fallback)
  {
    if (!sdf->HasElement(name))
      return fallback;
    const double value = sdf->Get<double>(name);
    if (!std::isfinite(value) || value <= 0.0)
    {
      gzerr << "[traffic_light] Invalid " << name << ", using "
            << fallback << " seconds\n";
      return fallback;
    }
    return value;
  }

  // 查找三个灯面 visual。模型加载和 visual 注册可能不是同一个时刻，
  // 因此该函数允许失败并由 OnUpdate 在后续仿真帧重试。
  private: bool ResolveVisuals()
  {
    // 只匹配名称后缀，兼容世界中 traffic_light、traffic_light_0 等
    // 不同实例名产生的 scoped name。
    const std::array<std::string, 3> suffixes = {{
      "::red_lamp", "::green_lamp", "::yellow_lamp"
    }};
    for (std::size_t i = 0; i < suffixes.size(); ++i)
    {
      bool found = false;
      for (const auto &entry : this->link->Visuals())
      {
        const std::string &name = entry.second.name();
        if (name.size() >= suffixes[i].size() &&
            name.compare(name.size() - suffixes[i].size(),
                         suffixes[i].size(), suffixes[i]) == 0)
        {
          this->visuals[i] = entry.second;
          found = true;
          break;
        }
      }
      if (!found)
      {
        if (!this->warnedMissing)
        {
          gzerr << "[traffic_light] Waiting for visual " << suffixes[i]
                << " in " << this->link->GetScopedName() << "\n";
          this->warnedMissing = true;
        }
        return false;
      }
    }
    this->visualsReady = true;
    gzmsg << "[traffic_light] Visuals ready for " << this->model->GetName()
          << "\n";
    return true;
  }

  // 每个世界更新周期调用一次。函数只在状态变化时立即发布材质，状态
  // 未变化时最多每秒发布一次，降低 transport 消息量，同时保证新加入
  // 的 Gazebo 客户端仍能收到当前材质。
  private: void OnUpdate(const common::UpdateInfo &info)
  {
    if (!this->visualsReady && !this->ResolveVisuals())
      return;

    // 世界重置后仿真时间会跳回较小值；重新设置周期起点，避免 elapsed
    // 变成负数并让灯卡在错误的颜色。
    if (info.simTime < this->cycleStart)
    {
      this->cycleStart = info.simTime;
      this->published = false;
    }

    // 将已经经过的仿真时间折算到一个周期内，再根据三个持续时间确定
    // 当前颜色。std::fmod 让周期自然循环，不需要累加浮点计数器。
    const double elapsed = (info.simTime - this->cycleStart).Double();
    const double cycle = this->durations[0] + this->durations[1] +
                         this->durations[2];
    const double phase = std::fmod(elapsed, cycle);
    const std::size_t next = phase < this->durations[0] ? 0 :
        (phase < this->durations[0] + this->durations[1] ? 1 : 2);
    const bool changed = !this->published || next != this->active;
    if (!changed && (info.simTime - this->lastPublish).Double() < 1.0)
      return;

    // 为三个 visual 分别选择 On/Dark 材质。即使颜色没有变化，定期发布
    // 也会把完整的 Visual 消息补给后来连接的客户端。
    this->active = next;
    const char *materials[] = {"Red", "Green", "Yellow"};
    for (std::size_t i = 0; i < this->visuals.size(); ++i)
    {
      msgs::Visual visual = this->visuals[i];
      visual.mutable_material()->mutable_script()->set_name(
          std::string("TrafficLight/") + materials[i] +
          (i == this->active ? "On" : "Dark"));
      visual.set_transparency(0.0);
      this->visualPub->Publish(visual);
    }
    // 日志只在颜色切换时输出，避免每个仿真帧刷屏。
    if (changed)
    {
      const char *states[] = {"red", "green", "yellow"};
      gzmsg << "[traffic_light] " << this->model->GetName() << " state="
            << states[this->active] << " sim_time=" << info.simTime.Double()
            << "\n";
    }
    this->lastPublish = info.simTime;
    this->published = true;
  }

  // 模型和灯所在 link。
  private: physics::ModelPtr model;
  private: physics::LinkPtr link;
  // 用于发布 visual 更新的 Gazebo transport 对象及世界更新回调。
  private: transport::NodePtr node;
  private: transport::PublisherPtr visualPub;
  private: event::ConnectionPtr updateConnection;
  // 按红、绿、黄顺序保存三个灯面的 visual 消息和持续时间。
  private: std::array<msgs::Visual, 3> visuals;
  private: std::array<double, 3> durations;
  // 当前周期起点及最近一次发布时间，均使用 Gazebo 仿真时间。
  private: common::Time cycleStart;
  private: common::Time lastPublish;
  // active 为当前颜色数组下标；published 用于第一次更新时强制发布。
  private: std::size_t active = 0;
  private: bool published = false;
  // visual 尚未解析成功时，OnUpdate 会继续重试；warning 只打印一次。
  private: bool visualsReady = false;
  private: bool warnedMissing = false;
};

GZ_REGISTER_MODEL_PLUGIN(TrafficLightController)
}
