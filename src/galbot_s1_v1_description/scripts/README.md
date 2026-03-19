# URDF左臂零位调整工具

这套工具用于调整Galbot机器人URDF文件中左臂关节的零位位置。通过修改关节限位范围来实现零位偏移，保持实际物理限位不变。

## 文件说明

### 主要脚本

1. **`adjust_left_arm_zero_position.py`** - 完整功能版本
   - 支持多种输入方式（命令行参数、交互式输入）
   - 提供详细的状态信息和错误处理
   - 支持角度和弧度输入
   - 支持指定特定关节调整

2. **`adjust_left_arm_zero_position_simple.py`** - 简化版本
   - 简单易用的接口
   - 支持命令行和交互式输入
   - 自动备份原文件

3. **`example_zero_adjustment.py`** - 使用示例
   - 提供预设的调整示例
   - 演示不同的使用场景
   - 包含备份恢复功能

## 零位调整原理

零位调整通过修改关节的limit参数来实现：

```xml
<!-- 原始关节限位 -->
<limit effort="60" lower="-2.53" upper="3.58" velocity="1.5"/>

<!-- 零位偏移0.1弧度后的限位 -->
<limit effort="60" lower="-2.63" upper="3.48" velocity="1.5"/>
```

- **零位偏移量为正值**：限位范围向负方向偏移（相当于零位向正方向移动）
- **零位偏移量为负值**：限位范围向正方向偏移（相当于零位向负方向移动）
- **实际可动范围保持不变**，只是改变了零位参考点

## 左臂关节说明

左臂包含7个关节，按顺序为：

1. `left_arm_joint1` - 肩部旋转（水平转动）
2. `left_arm_joint2` - 肩部俯仰（上下摆动）
3. `left_arm_joint3` - 肩部内外展（内外摆动）
4. `left_arm_joint4` - 肘部（屈伸）
5. `left_arm_joint5` - 前臂旋转（前臂扭转）
6. `left_arm_joint6` - 腕部俯仰（手腕上下）
7. `left_arm_joint7` - 腕部旋转（手腕扭转）

## 使用方法

### 方法1：使用示例脚本（推荐新手）

```bash
cd /home/zbs/Code/zbs_workspace/remote_ws/src/galbot_description/galbot_one_delta_description/scripts
python3 example_zero_adjustment.py
```

按提示选择预设的调整示例或自定义输入。

### 方法2：使用简化版脚本

```bash
# 交互式输入
python3 adjust_left_arm_zero_position_simple.py /path/to/your/robot.urdf

# 命令行指定偏移量（7个关节，单位：度）
python3 adjust_left_arm_zero_position_simple.py /path/to/your/robot.urdf 5.7 -11.5 8.6 0 -5.7 2.9 17.2
```

### 方法3：使用完整功能版本

```bash
# 查看当前关节限位
python3 adjust_left_arm_zero_position.py --urdf_file robot.urdf --show-limits

# 角度输入（度）
python3 adjust_left_arm_zero_position.py --urdf_file robot.urdf --offsets-deg 5.7 -11.5 8.6 0 -5.7 2.9 17.2

# 弧度输入
python3 adjust_left_arm_zero_position.py --urdf_file robot.urdf --offsets 0.1 -0.2 0.15 0 -0.1 0.05 0.3

# 指定特定关节
python3 adjust_left_arm_zero_position.py --urdf_file robot.urdf --joint-offsets left_arm_joint1:0.1 left_arm_joint3:-0.2

# 交互式输入
python3 adjust_left_arm_zero_position.py --urdf_file robot.urdf --interactive

# 保存到新文件
python3 adjust_left_arm_zero_position.py --urdf_file robot.urdf --offsets 0.1 0 0 0 0 0 0 --output new_robot.urdf
```

## 使用场景示例

### 场景1：微调机器人初始姿态
当机器人的零位姿态不理想时，可以进行小幅度调整：
```bash
# 轻微调整各关节，使机器人姿态更自然
python3 adjust_left_arm_zero_position_simple.py robot.urdf 2.0 -1.5 3.0 0.0 -2.0 1.0 0.5
```

### 场景2：适配不同的硬件标定
当硬件关节的物理零位与设计零位有偏差时：
```bash
# 根据硬件标定结果调整零位
python3 adjust_left_arm_zero_position_simple.py robot.urdf 5.2 -8.1 6.3 -2.1 4.7 -1.8 3.2
```

### 场景3：改变机器人默认工作姿态
使机器人以不同的初始姿态启动：
```bash
# 让机器人以举手姿态为零位
python3 adjust_left_arm_zero_position_simple.py robot.urdf 15.0 -30.0 25.0 45.0 -10.0 15.0 0.0
```

## 安全注意事项

1. **自动备份**：脚本会自动创建`.backup`备份文件
2. **限位检查**：调整后请检查关节限位是否合理
3. **仿真验证**：建议在仿真中验证调整效果
4. **物理验证**：实际使用前请确认调整值在硬件安全范围内

## 恢复原始文件

如果需要恢复到调整前的状态：

```bash
# 使用示例脚本的恢复功能
python3 example_zero_adjustment.py
# 选择"2. 从备份恢复"

# 或者手动恢复
cp robot.urdf.backup robot.urdf
```

## 验证调整效果

### 在RViz中查看
```bash
# 启动RViz显示机器人模型
roslaunch galbot_one_delta_description display.launch
```

### 在Gazebo中仿真
```bash
# 启动Gazebo仿真验证关节运动
roslaunch galbot_one_delta_description gazebo.launch
```

## 故障排除

### 常见问题

1. **权限错误**
   ```bash
   chmod +x *.py
   ```

2. **文件路径错误**
   - 确认URDF文件路径正确
   - 使用绝对路径

3. **XML格式错误**
   - 检查URDF文件是否为有效的XML格式
   - 查看脚本输出的错误信息

4. **关节名称不匹配**
   - 确认URDF文件中左臂关节名称与脚本中定义的一致

### 调试建议

1. 使用`--show-limits`参数查看当前关节限位
2. 先进行小幅度调整测试
3. 在仿真环境中验证后再应用到实际机器人

## 开发说明

### 扩展脚本功能

如需添加新功能，可以修改：
- `URDFLeftArmZeroAdjuster`类添加新方法
- `parse_arguments()`函数添加新参数
- `main()`函数添加新逻辑

### 适配其他机器人

修改以下部分：
- `left_arm_joints`列表（关节名称）
- 关节数量相关的常数
- 文件路径和命名规则

---

**版本**: 1.0  
**作者**: Robot Development Team  
**最后更新**: 2025-07-30
