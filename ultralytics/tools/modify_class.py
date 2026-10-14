import os
import argparse

def modify_yolo_labels(input_folder, target_class=None):
    """
    合并 communitydda 与 community，或将标注统一为指定类别 ID。
    
    参数:
        input_folder (str): 包含YOLO标签文件的文件夹路径
        target_class (int | None): 指定时保留原有统一类别 ID 的功能
    """
    if not os.path.isdir(input_folder):
        raise ValueError(f"标签文件夹不存在: {input_folder}")
    if target_class is not None and target_class < 0:
        raise ValueError("目标类别 ID 不能为负数")

    classes_path = os.path.join(input_folder, 'classes.txt')
    class_mapping = None
    if target_class is None:
        with open(classes_path, 'r', encoding='utf-8') as f:
            class_names = f.read().splitlines()
        if not class_names:
            raise ValueError(f"类别文件为空: {classes_path}")
        target_names = ['community', 'non-community']
        class_mapping = {}
        for class_id, name in enumerate(class_names):
            name = name.strip()
            if name == 'communitydda':
                name = 'community'
            if name not in target_names:
                raise ValueError(f"不支持的类别: {name!r}")
            class_mapping[class_id] = target_names.index(name)

    # 先验证所有标注，再写入，避免格式错误造成部分数据被修改。
    pending_files = []
    for filename in sorted(os.listdir(input_folder)):
        filepath = os.path.join(input_folder, filename)
        if filename == 'classes.txt' or not filename.endswith('.txt') or not os.path.isfile(filepath):
            continue
        with open(filepath, 'r', encoding='utf-8', newline='') as f:
            lines = f.readlines()
        modified_lines = []
        for line_number, line in enumerate(lines, start=1):
            parts = line.split(maxsplit=1)
            if not parts:
                modified_lines.append(line)
                continue
            try:
                class_id = int(parts[0])
            except ValueError as exc:
                raise ValueError(f"{filepath} 第 {line_number} 行类别 ID 无效") from exc
            if class_mapping is not None:
                if class_id not in class_mapping:
                    raise ValueError(f"{filepath} 第 {line_number} 行类别 ID 越界: {class_id}")
                new_class_id = class_mapping[class_id]
            else:
                new_class_id = target_class
            if new_class_id == class_id:
                modified_lines.append(line)
            else:
                modified_lines.append(str(new_class_id) + (' ' + parts[1] if len(parts) > 1 else '\n'))
        if modified_lines != lines:
            pending_files.append((filepath, modified_lines))

    for filepath, lines in pending_files:
        with open(filepath, 'w', encoding='utf-8', newline='') as f:
            f.writelines(lines)
        print(f"已修改: {os.path.basename(filepath)}")
    if class_mapping is not None:
        with open(classes_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(target_names) + '\n')
    print(f"处理完成，修改了 {len(pending_files)} 个标签文件")

if __name__ == "__main__":
    # 设置命令行参数解析
    parser = argparse.ArgumentParser(description='将 communitydda 合并为 community，仅保留 community 和 non-community')
    default_input = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'raw_data', 'label'))
    parser.add_argument('--input', type=str, default=default_input, help='包含YOLO标签和 classes.txt 的文件夹路径')
    parser.add_argument('--class', type=int, dest='target_class', help='可选：将所有标注统一为指定类别 ID')
    args = parser.parse_args()

    try:
        modify_yolo_labels(args.input, args.target_class)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"错误：{exc}\n")
