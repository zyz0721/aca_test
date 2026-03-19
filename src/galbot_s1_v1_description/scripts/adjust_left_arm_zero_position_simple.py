#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版URDF左臂零位调整脚本
"""

import os
import sys
import xml.etree.ElementTree as ET
import math

def adjust_left_arm_zero_position(urdf_file, offsets_deg=None, offsets_rad=None):
    """
    调整URDF文件中左臂关节的零位
    
    Args:
        urdf_file: URDF文件路径
        offsets_deg: 角度偏移量列表（7个值）
        offsets_rad: 弧度偏移量列表（7个值）
    """
    
    # 左臂关节名称
    left_arm_joints = [
        # "left_arm_joint1",  # 肩部旋转
        # "left_arm_joint2",  # 肩部俯仰  
        # "left_arm_joint3",  # 肩部内外展
        # "left_arm_joint4",  # 肘部
        # "left_arm_joint5",  # 前臂旋转
        # "left_arm_joint6",  # 腕部俯仰
        # "left_arm_joint7"   # 腕部旋转

        "S1_left_arm_joint_1", 
        "S1_left_arm_joint_2", 
        "S1_left_arm_joint_3", 
        "S1_left_arm_joint_4", 
        "S1_left_arm_joint_5", 
        "S1_left_arm_joint_6", 
        "S1_left_arm_joint_7"
    ]
    
    # 转换角度到弧度
    if offsets_deg is not None:
        offsets = [math.radians(deg) for deg in offsets_deg]
    elif offsets_rad is not None:
        offsets = offsets_rad
    else:
        print("错误：必须提供offsets_deg或offsets_rad参数")
        return False
    
    if len(offsets) != 7:
        print(f"错误：偏移量数量必须为7个，当前为{len(offsets)}个")
        return False
    
    try:
        # 加载URDF文件
        tree = ET.parse(urdf_file)
        root = tree.getroot()
        
        # 备份原文件
        backup_file = urdf_file + ".backup"
        if not os.path.exists(backup_file):
            import shutil
            shutil.copy2(urdf_file, backup_file)
            print(f"已创建备份文件: {backup_file}")
        
        print("开始调整左臂关节零位...")
        print("=" * 50)
        
        # 调整每个关节
        for joint_name, offset in zip(left_arm_joints, offsets):
            if abs(offset) < 1e-6:  # 跳过零偏移
                continue
                
            # 查找关节
            for joint in root.findall('joint'):
                if joint.get('name') == joint_name:
                    limit_elem = joint.find('limit')
                    if limit_elem is not None:
                        # 获取当前限位
                        current_lower = float(limit_elem.get('lower', 0))
                        current_upper = float(limit_elem.get('upper', 0))
                        
                        # 计算新限位（通过偏移限位实现零位调整）
                        new_lower = current_lower - offset
                        new_upper = current_upper - offset
                        
                        # 更新限位
                        limit_elem.set('lower', str(new_lower))
                        limit_elem.set('upper', str(new_upper))
                        
                        print(f"{joint_name}:")
                        print(f"  零位偏移: {math.degrees(offset):.2f}° ({offset:.4f} rad)")
                        print(f"  新限位: [{math.degrees(new_lower):.2f}°, {math.degrees(new_upper):.2f}°]")
                    break
        
        # 保存文件
        tree.write(urdf_file, encoding='utf-8', xml_declaration=True)
        print(f"\n成功保存到: {urdf_file}")
        return True
        
    except Exception as e:
        print(f"错误：{e}")
        return False

def main():
    """主函数 - 提供简单的命令行接口"""
    if len(sys.argv) < 2:
        print("用法:")
        print("python adjust_left_arm_zero_position_simple.py <urdf_file> [offset1] [offset2] ... [offset7]")
        print("偏移量单位：度（角度）")
        print("\n示例:")
        print("python adjust_left_arm_zero_position_simple.py robot.urdf 5.7 -11.5 8.6 0 -5.7 2.9 17.2")
        return
    
    urdf_file = sys.argv[1]
    
    if not os.path.exists(urdf_file):
        print(f"错误：文件不存在: {urdf_file}")
        return
    
    # 如果提供了偏移量参数
    if len(sys.argv) == 9:  # 文件名 + 7个偏移量
        try:
            offsets_deg = [float(x) for x in sys.argv[2:9]]
        except ValueError:
            print("错误：偏移量必须是数字")
            return
    else:
        # 交互式输入
        print("请输入左臂各关节的零位偏移量（角度）:")
        print("关节顺序：肩旋转、肩俯仰、肩内外展、肘部、前臂旋转、腕俯仰、腕旋转")
        
        offsets_deg = []
        joint_names = ["肩部旋转", "肩部俯仰", "肩部内外展", "肘部", "前臂旋转", "腕部俯仰", "腕部旋转"]
        
        for i, name in enumerate(joint_names):
            while True:
                try:
                    value = input(f"{name} (度，回车输入0): ").strip()
                    if not value:
                        offsets_deg.append(0.0)
                        break
                    offsets_deg.append(float(value))
                    break
                except ValueError:
                    print("请输入有效数字")
                except KeyboardInterrupt:
                    print("\n操作已取消")
                    return
    
    # 显示将要应用的偏移量
    print(f"\n将对文件 {urdf_file} 应用以下偏移量:")
    for i, offset in enumerate(offsets_deg):
        if abs(offset) > 0.01:
            print(f"  关节{i+1}: {offset:.2f}°")
    
    confirm = input("\n确认继续? (y/N): ").strip().lower()
    if confirm in ['y', 'yes']:
        if adjust_left_arm_zero_position(urdf_file, offsets_deg=offsets_deg):
            print("\n零位调整完成！")
        else:
            print("\n调整失败！")
    else:
        print("操作已取消")

if __name__ == "__main__":
    main()
