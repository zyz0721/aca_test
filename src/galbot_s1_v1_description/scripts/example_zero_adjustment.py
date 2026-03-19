#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
URDF左臂零位调整示例脚本
演示如何使用零位调整功能
"""

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from adjust_left_arm_zero_position_simple import adjust_left_arm_zero_position

def example_adjustments():
    """零位调整示例"""
    
    # 当前URDF文件路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_file = os.path.join(current_dir, "..", "urdf", "S1_0", "S1_ROBOT-O4.urdf")
    
    if not os.path.exists(urdf_file):
        print(f"URDF文件不存在: {urdf_file}")
        print("请确认文件路径正确")
        return
    
    print("URDF左臂零位调整示例")
    print("=" * 50)
    print(f"目标文件: {urdf_file}")
    
    # 示例1: 轻微调整
    print("\n示例1: 轻微零位调整")
    print("适用于微调机器人姿态")
    offsets_example1 = [2.0, -1.5, 3.0, 0.0, -2.0, 1.0, 0.5]  # 度
    print("偏移量(度):", offsets_example1)
    
    # 示例2: 较大调整
    print("\n示例2: 较大零位调整") 
    print("适用于改变机器人初始姿态")
    offsets_example2 = [15.0, -25.0, 20.0, 30.0, -15.0, 10.0, 5.0]  # 度
    print("偏移量(度):", offsets_example2)
    
    # 示例3: 单关节调整
    print("\n示例3: 仅调整特定关节")
    print("只调整肩部关节")
    offsets_example3 = [10.0, -5.0, 8.0, 0.0, 0.0, 0.0, 0.0]  # 度
    print("偏移量(度):", offsets_example3)
    
    # 让用户选择
    print("\n请选择要执行的示例:")
    print("1. 轻微调整")
    print("2. 较大调整") 
    print("3. 单关节调整")
    print("4. 自定义调整")
    print("0. 退出")
    
    try:
        choice = input("\n请输入选择 (0-4): ").strip()
        
        if choice == "0":
            print("退出")
            return
        elif choice == "1":
            offsets = offsets_example1
        elif choice == "2":
            offsets = offsets_example2
        elif choice == "3":
            offsets = offsets_example3
        elif choice == "4":
            offsets = custom_input()
        else:
            print("无效选择")
            return
        
        # 执行调整
        print(f"\n应用偏移量: {offsets}")
        confirm = input("确认执行? (y/N): ").strip().lower()
        
        if confirm in ['y', 'yes']:
            if adjust_left_arm_zero_position(urdf_file, offsets_deg=offsets):
                print("\n✓ 零位调整成功完成！")
                print(f"✓ 已备份原文件为: {urdf_file}.backup")
                print(f"✓ 修改后的文件: {urdf_file}")
                
                print("\n后续操作建议:")
                print("1. 使用rviz查看机器人模型：")
                print("   roslaunch S1_ROBOT display.launch")
                print("2. 检查关节运动范围是否合理")
                print("3. 在仿真中测试机器人运动")
            else:
                print("\n✗ 调整失败！")
        else:
            print("操作已取消")
            
    except KeyboardInterrupt:
        print("\n操作已取消")

def custom_input():
    """自定义输入偏移量"""
    print("\n自定义零位偏移量输入")
    print("请输入7个关节的偏移量（角度）:")
    
    joint_names = [
        "肩部旋转 (left_arm_joint1)",
        "肩部俯仰 (left_arm_joint2)", 
        "肩部内外展 (left_arm_joint3)",
        "肘部 (left_arm_joint4)",
        "前臂旋转 (left_arm_joint5)",
        "腕部俯仰 (left_arm_joint6)",
        "腕部旋转 (left_arm_joint7)"
    ]
    
    offsets = []
    for name in joint_names:
        while True:
            try:
                value = input(f"{name}: ").strip()
                if not value:
                    offsets.append(0.0)
                    break
                offsets.append(float(value))
                break
            except ValueError:
                print("请输入有效数字")
    
    return offsets

def restore_backup():
    """从备份恢复原文件"""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_file = os.path.join(current_dir, "..", "urdf", "S1_0", "galbot_one_echo-O1.urdf")
    backup_file = urdf_file + ".backup"
    
    if os.path.exists(backup_file):
        import shutil
        shutil.copy2(backup_file, urdf_file)
        print(f"已从备份恢复: {urdf_file}")
    else:
        print("备份文件不存在")

def main():
    """主函数"""
    print("URDF左臂零位调整工具")
    print("=" * 30)
    print("1. 运行调整示例")
    print("2. 从备份恢复")
    print("0. 退出")
    
    try:
        choice = input("\n请选择 (0-2): ").strip()
        
        if choice == "1":
            example_adjustments()
        elif choice == "2":
            restore_backup()
        elif choice == "0":
            print("退出")
        else:
            print("无效选择")
            
    except KeyboardInterrupt:
        print("\n退出")

if __name__ == "__main__":
    main()
