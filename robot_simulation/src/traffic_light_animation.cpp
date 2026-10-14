#include <cmath>
#include <functional>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>

namespace gazebo
{
// 图片序列播放器：不切换三盏灯的材质，只改变整面贴图的当前帧。
// anim_texture 的自动计时在 GUI 与相机渲染中并不一致，故将材质中的
// duration 设为 0，由每个渲染场景的仿真时间统一控制帧号。
class TrafficLightAnimation : public VisualPlugin
{
  public: void Load(rendering::VisualPtr visual, sdf::ElementPtr) override
  {
    this->visual = visual;
    this->connection = event::Events::ConnectPreRender(
        std::bind(&TrafficLightAnimation::OnRender, this));
  }

  private: void OnRender()
  {
    const double now = this->visual->GetScene()->SimTime().Double();
    if (!std::isfinite(now) || now < 0.0)
      return;

    // 使用绝对仿真时间，GUI、机器人相机和后来加载的实例保持同一灯态。
    // [0,10) 红、[10,15) 黄、[15,30) 绿；世界重置时自动回到红灯。
    const unsigned int frame = static_cast<unsigned int>(
        std::floor(std::fmod(now, 30.0) / 5.0));
    auto material = Ogre::MaterialManager::getSingleton().getByName(
        this->visual->GetMaterialName());
    if (material.isNull())
      return;

    // Gazebo 的 RTShader 会复制 technique；只设置原始 technique 会造成
    // 查询帧号正确，但相机实际渲染的仍是另一个帧。每次渲染同步所有副本，
    // 同时兼容加载后才创建的 shader technique。
    for (unsigned short t = 0; t < material->getNumTechniques(); ++t)
    {
      auto technique = material->getTechnique(t);
      for (unsigned short p = 0; p < technique->getNumPasses(); ++p)
      {
        auto pass = technique->getPass(p);
        for (unsigned short u = 0; u < pass->getNumTextureUnitStates(); ++u)
        {
          auto texture = pass->getTextureUnitState(u);
          if (texture->getNumFrames() == 6)
            texture->setCurrentFrame(frame);
        }
      }
    }
  }

  private: rendering::VisualPtr visual;
  private: event::ConnectionPtr connection;
};

GZ_REGISTER_VISUAL_PLUGIN(TrafficLightAnimation)
}
