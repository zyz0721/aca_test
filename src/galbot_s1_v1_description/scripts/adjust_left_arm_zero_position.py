#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
URDF左臂零位调整脚本
零位调整方式：保持实际限位不变，调整限位上下限来实现零位偏移

用法示例：
python adjust_left_arm_zero_position.py --urdf_file path/to/robot.urdf --offsets 0.1 -0.2 0.15 0 -0.1 0.05 0.3
"""

import os
import sys
import re
import xml.etree.ElementTree as ET
import argparse
import math
from typing import List, Dict, Tuple, Optional

class URDFLeftArmZeroAdjuster:
    """URDF左臂零位调整器"""
    
    def __init__(self, urdf_file: str):
        """
        初始化调整器
        
        Args:
            urdf_file: URDF文件路径
        """
        self.urdf_file = urdf_file
        self.tree = None
        self.root = None
        
        # 左臂关节名称列表（按顺序）
        self.left_arm_joints = [
        "S1_left_arm_joint_1", 
        "S1_left_arm_joint_2", 
        "S1_left_arm_joint_3", 
        "S1_left_arm_joint_4", 
        "S1_left_arm_joint_5", 
        "S1_left_arm_joint_6", 
        "S1_left_arm_joint_7"
        ]
        
        # 默认零位偏移值（弧度）
        self.default_offsets = [0.0] * len(self.left_arm_joints)
        
    def load_urdf(self) -> bool:
        """
        加载URDF文件
        
        Returns:
            加载是否成功
        """
        try:
            if not os.path.exists(self.urdf_file):
                print(f"错误：URDF文件不存在: {self.urdf_file}")
                return False
                
            self.tree = ET.parse(self.urdf_file)
            self.root = self.tree.getroot()
            print(f"成功加载URDF文件: {self.urdf_file}")
            return True
            
        except ET.ParseError as e:
            print(f"错误：解析URDF文件失败: {e}")
            return False
        except Exception as e:
            print(f"错误：加载URDF文件失败: {e}")
            return False
    
    def get_joint_limits(self, joint_name: str) -> Optional[Tuple[float, float]]:
        """
        获取关节的当前限位
        
        Args:
            joint_name: 关节名称
            
        Returns:
            (lower_limit, upper_limit) 或 None
        """
        for joint in self.root.findall('joint'):
            if joint.get('name') == joint_name:
                limit_elem = joint.find('limit')
                if limit_elem is not None:
                    lower = float(limit_elem.get('lower', 0))
                    upper = float(limit_elem.get('upper', 0))
                    return (lower, upper)
        return None
    
    def adjust_joint_limits(self, joint_name: str, zero_offset: float) -> bool:
        """
        调整单个关节的限位来实现零位偏移
        
        Args:
            joint_name: 关节名称
            zero_offset: 零位偏移量（弧度，正值表示正方向偏移）
            
        Returns:
            调整是否成功
        """
        for joint in self.root.findall('joint'):
            if joint.get('name') == joint_name:
                limit_elem = joint.find('limit')
                if limit_elem is not None:
                    # 获取当前限位
                    current_lower = float(limit_elem.get('lower', 0))
                    current_upper = float(limit_elem.get('upper', 0))
                    
                    # 计算新的限位（通过调整限位来实现零位偏移）
                    # 零位偏移的实现：将限位范围整体偏移相反的值
                    new_lower = current_lower - zero_offset
                    new_upper = current_upper - zero_offset
                    
                    # 更新限位
                    limit_elem.set('lower', str(new_lower))
                    limit_elem.set('upper', str(new_upper))
                    
                    print(f"关节 {joint_name}:")
                    print(f"  零位偏移: {zero_offset:.4f} rad ({math.degrees(zero_offset):.2f}°)")
                    print(f"  原始限位: [{current_lower:.4f}, {current_upper:.4f}] rad")
                    print(f"  新限位: [{new_lower:.4f}, {new_upper:.4f}] rad")
                    
                    return True
                else:
                    print(f"警告：关节 {joint_name} 没有limit元素")
                    return False
        
        print(f"错误：未找到关节 {joint_name}")
        return False
    
    def adjust_all_joints(self, offsets: List[float]) -> bool:
        """
        调整所有左臂关节的零位
        
        Args:
            offsets: 各关节的零位偏移量列表（弧度）
            
        Returns:
            调整是否成功
        """
        if len(offsets) != len(self.left_arm_joints):
            print(f"错误：偏移量数量({len(offsets)})与关节数量({len(self.left_arm_joints)})不匹配")
            return False
        
        print("开始调整左臂关节零位...")
        print("=" * 60)
        
        success_count = 0
        for i, (joint_name, offset) in enumerate(zip(self.left_arm_joints, offsets)):
            if abs(offset) < 1e-6:  # 忽略很小的偏移
                print(f"关节 {joint_name}: 偏移量为0，跳过")
                success_count += 1
                continue
                
            if self.adjust_joint_limits(joint_name, offset):
                success_count += 1
            print("-" * 40)
        
        print(f"调整完成：成功 {success_count}/{len(self.left_arm_joints)} 个关节")
        return success_count == len(self.left_arm_joints)
    
    def save_urdf(self, output_file: Optional[str] = None) -> bool:
        """
        保存URDF文件
        
        Args:
            output_file: 输出文件路径，None表示覆盖原文件
            
        Returns:
            保存是否成功
        """
        try:
            save_file = output_file if output_file else self.urdf_file
            
            # 备份原文件
            if save_file == self.urdf_file:
                backup_file = self.urdf_file + ".backup"
                if not os.path.exists(backup_file):
                    import shutil
                    shutil.copy2(self.urdf_file, backup_file)
                    print(f"已创建备份文件: {backup_file}")
            
            # 保存修改后的文件
            self.tree.write(save_file, encoding='utf-8', xml_declaration=True)
            print(f"成功保存URDF文件: {save_file}")
            return True
            
        except Exception as e:
            print(f"错误：保存URDF文件失败: {e}")
            return False
    
    def print_current_limits(self):
        """打印当前所有左臂关节的限位信息"""
        print("当前左臂关节限位信息:")
        print("=" * 60)
        
        for joint_name in self.left_arm_joints:
            limits = self.get_joint_limits(joint_name)
            if limits:
                lower, upper = limits
                range_deg = math.degrees(upper - lower)
                print(f"{joint_name}:")
                print(f"  限位: [{lower:.4f}, {upper:.4f}] rad")
                print(f"  限位: [{math.degrees(lower):.2f}°, {math.degrees(upper):.2f}°]")
                print(f"  范围: {upper-lower:.4f} rad ({range_deg:.2f}°)")
            else:
                print(f"{joint_name}: 未找到限位信息")
            print("-" * 40)

def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="URDF左臂零位调整脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 查看当前限位信息
  python adjust_left_arm_zero_position.py --urdf_file robot.urdf --show-limits
  
  # 调整所有关节零位（弧度）
  python adjust_left_arm_zero_position.py --urdf_file robot.urdf --offsets 0.1 -0.2 0.15 0 -0.1 0.05 0.3
  
  # 调整所有关节零位（角度）
  python adjust_left_arm_zero_position.py --urdf_file robot.urdf --offsets-deg 5.7 -11.5 8.6 0 -5.7 2.9 17.2
  
  # 只调整特定关节
  python adjust_left_arm_zero_position.py --urdf_file robot.urdf --joint-offsets left_arm_joint1:0.1 left_arm_joint3:-0.2
  
  # 保存到新文件
  python adjust_left_arm_zero_position.py --urdf_file robot.urdf --offsets 0.1 0 0 0 0 0 0 --output new_robot.urdf

关节顺序:
  1. left_arm_joint1 (肩部旋转)
  2. left_arm_joint2 (肩部俯仰)  
  3. left_arm_joint3 (肩部内外展)
  4. left_arm_joint4 (肘部)
  5. left_arm_joint5 (前臂旋转)
  6. left_arm_joint6 (腕部俯仰)
  7. left_arm_joint7 (腕部旋转)
        """
    )
    
    parser.add_argument('--urdf_file', '-f', required=True,
                       help='URDF文件路径')
    
    parser.add_argument('--offsets', nargs=7, type=float, metavar='OFFSET',
                       help='7个左臂关节的零位偏移量（弧度）')
    
    parser.add_argument('--offsets-deg', nargs=7, type=float, metavar='OFFSET_DEG',
                       help='7个左臂关节的零位偏移量（角度）')
    
    parser.add_argument('--joint-offsets', nargs='+', metavar='JOINT:OFFSET',
                       help='指定关节的零位偏移，格式：joint_name:offset_rad')
    
    parser.add_argument('--output', '-o',
                       help='输出文件路径（默认覆盖原文件）')
    
    parser.add_argument('--show-limits', action='store_true',
                       help='显示当前关节限位信息')
    
    parser.add_argument('--interactive', '-i', action='store_true',
                       help='交互式输入偏移量')
    
    return parser.parse_args()

def interactive_input(adjuster: URDFLeftArmZeroAdjuster) -> List[float]:
    """交互式输入零位偏移量"""
    print("\n交互式零位调整")
    print("=" * 40)
    
    # 显示当前限位
    adjuster.print_current_limits()
    
    offsets = []
    for i, joint_name in enumerate(adjuster.left_arm_joints):
        while True:
            try:
                prompt = f"请输入 {joint_name} 的零位偏移量（弧度，回车跳过）: "
                user_input = input(prompt).strip()
                
                if not user_input:  # 回车跳过
                    offsets.append(0.0)
                    break
                
                offset = float(user_input)
                offsets.append(offset)
                break
                
            except ValueError:
                print("输入无效，请输入数字")
            except KeyboardInterrupt:
                print("\n用户取消")
                sys.exit(0)
    
    return offsets

def parse_joint_offsets(joint_offset_strs: List[str]) -> Dict[str, float]:
    """解析关节偏移量字符串"""
    joint_offsets = {}
    for joint_offset_str in joint_offset_strs:
        try:
            joint_name, offset_str = joint_offset_str.split(':')
            offset = float(offset_str)
            joint_offsets[joint_name.strip()] = offset
        except ValueError:
            print(f"警告：无效的关节偏移格式: {joint_offset_str}")
    return joint_offsets

def main():
    """主函数"""
    args = parse_arguments()
    
    # 创建调整器
    adjuster = URDFLeftArmZeroAdjuster(args.urdf_file)
    
    # 加载URDF文件
    if not adjuster.load_urdf():
        sys.exit(1)
    
    # 显示当前限位信息
    if args.show_limits:
        adjuster.print_current_limits()
        return
    
    # 确定零位偏移量
    offsets = None
    
    if args.interactive:
        # 交互式输入
        offsets = interactive_input(adjuster)
        
    elif args.offsets:
        # 直接指定偏移量（弧度）
        offsets = args.offsets
        
    elif args.offsets_deg:
        # 指定偏移量（角度转弧度）
        offsets = [math.radians(deg) for deg in args.offsets_deg]
        
    elif args.joint_offsets:
        # 指定特定关节的偏移量
        joint_offsets = parse_joint_offsets(args.joint_offsets)
        offsets = []
        for joint_name in adjuster.left_arm_joints:
            offsets.append(joint_offsets.get(joint_name, 0.0))
    
    else:
        print("错误：必须指定偏移量参数")
        print("使用 --help 查看使用方法")
        sys.exit(1)
    
    # 显示将要应用的偏移量
    print("\n将要应用的零位偏移量:")
    print("=" * 50)
    for i, (joint_name, offset) in enumerate(zip(adjuster.left_arm_joints, offsets)):
        if abs(offset) > 1e-6:
            print(f"{joint_name}: {offset:.4f} rad ({math.degrees(offset):.2f}°)")
    
    # 确认操作
    if not args.interactive:
        confirm = input("\n确认应用这些偏移量? (y/N): ").strip().lower()
        if confirm not in ['y', 'yes']:
            print("操作已取消")
            return
    
    # 调整零位
    if adjuster.adjust_all_joints(offsets):
        # 保存文件
        if adjuster.save_urdf(args.output):
            print("\n零位调整完成！")
            
            # 如果需要进行其他操作，可以在这里添加
            print("\n可以进行的后续操作:")
            print("1. 使用rviz查看调整后的机器人模型")
            print("2. 运行仿真验证关节运动范围") 
            print("3. 生成关节测试脚本")
            
        else:
            print("保存文件失败")
            sys.exit(1)
    else:
        print("零位调整失败")
        sys.exit(1)

if __name__ == "__main__":
    main()
