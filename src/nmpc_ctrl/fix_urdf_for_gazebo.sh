#!/bin/bash
# fix_urdf_for_gazebo.sh — 修复 xacro 让 Gazebo 正常运行
#
# 修复 1: 取消注释被注释掉的 <geometry> (torso_arm_amount_link)
# 修复 2: 手臂 (14个) + 头部 (2个) 关节改成 fixed, 防止重力下垂
#
# 用法: bash fix_urdf_for_gazebo.sh
# 恢复: cp <xacro>.bak <xacro>

set -e

XACRO_FILE=$(rospack find galbot_s1_v1_description)/urdf/galbot_S1_V1_with_GE10301.urdf.xacro

if [ ! -f "$XACRO_FILE" ]; then
    echo "ERROR: 找不到 $XACRO_FILE"
    exit 1
fi

# 备份 (只在没有备份时)
if [ ! -f "${XACRO_FILE}.bak" ]; then
    cp "$XACRO_FILE" "${XACRO_FILE}.bak"
    echo "[OK] 备份: ${XACRO_FILE}.bak"
else
    echo "[OK] 备份已存在, 跳过"
fi

# --- 修复 1: 取消注释 geometry ---
echo ""
echo "=== 修复 1: 取消注释 <geometry> ==="
sed -i \
    -e 's|<!-- <geometry>|<geometry>|g' \
    -e 's|</geometry>  -->|</geometry>|g' \
    -e 's|</geometry> -->|</geometry>|g' \
    "$XACRO_FILE"
echo "[OK] geometry 标签已恢复"

# --- 修复 2: 手臂和头部关节改成 fixed ---
echo ""
echo "=== 修复 2: 冻结手臂和头部关节 ==="

# 要冻结的关节列表
JOINTS=(
    "left_arm_joint1"
    "left_arm_joint2"
    "left_arm_joint3"
    "left_arm_joint4"
    "left_arm_joint5"
    "left_arm_joint6"
    "left_arm_joint7"
    "right_arm_joint1"
    "right_arm_joint2"
    "right_arm_joint3"
    "right_arm_joint4"
    "right_arm_joint5"
    "right_arm_joint6"
    "right_arm_joint7"
    "head_joint1"
    "head_joint2"
)

for j in "${JOINTS[@]}"; do
    # 把 name="xxx" 后面同行或下行的 type="revolute" 改成 type="fixed"
    # xacro 格式可能是:
    #   <joint name="left_arm_joint1" type="revolute">
    # 或者:
    #   <joint
    #     name="left_arm_joint1"
    #     type="revolute">
    
    # 方法: 找到包含这个 joint name 的行号, 然后在附近找 type="revolute" 改成 fixed
    LINE=$(grep -n "name=\"${j}\"" "$XACRO_FILE" | head -1 | cut -d: -f1)
    if [ -z "$LINE" ]; then
        echo "  [SKIP] $j 未找到"
        continue
    fi
    
    # 在该行及后5行范围内替换 type="revolute" → type="fixed"
    sed -i "${LINE},$((LINE+5))s/type=\"revolute\"/type=\"fixed\"/" "$XACRO_FILE"
    echo "  [OK] $j → fixed"
done

# --- 验证 ---
echo ""
echo "=== 验证 ==="

# 检查 xacro 能否正常处理
if xacro "$XACRO_FILE" > /tmp/test_urdf.xml 2>/tmp/xacro_err.txt; then
    echo "[OK] xacro 处理成功"
    
    # 检查手臂关节是否都变成 fixed 了
    STILL_REVOLUTE=$(grep -c 'arm_joint.*type="revolute"\|head_joint.*type="revolute"' /tmp/test_urdf.xml 2>/dev/null || true)
    if [ "$STILL_REVOLUTE" -eq 0 ]; then
        echo "[OK] 所有手臂/头部关节已变为 fixed"
    else
        echo "[WARN] 还有 $STILL_REVOLUTE 个手臂/头部关节是 revolute"
    fi
    
    # 检查 geometry 问题
    # 一个简单的检查: 找 <visual> 后面没有 <geometry> 的情况
    EMPTY_VIS=$(python3 -c "
import xml.etree.ElementTree as ET
tree = ET.parse('/tmp/test_urdf.xml')
count = 0
for link in tree.findall('.//link'):
    for vis in link.findall('visual'):
        if vis.find('geometry') is None:
            count += 1
            print(f'  [WARN] {link.get(\"name\")}: visual 缺少 geometry')
print(count)
" 2>/dev/null | tail -1)
    
    if [ "$EMPTY_VIS" = "0" ]; then
        echo "[OK] 所有 visual 都有 geometry"
    fi
    
    # 统计当前关节类型
    echo ""
    echo "当前关节统计:"
    echo "  revolute:   $(grep -c 'type="revolute"' /tmp/test_urdf.xml || echo 0)"
    echo "  continuous: $(grep -c 'type="continuous"' /tmp/test_urdf.xml || echo 0)"
    echo "  fixed:      $(grep -c 'type="fixed"' /tmp/test_urdf.xml || echo 0)"
    echo "  prismatic:  $(grep -c 'type="prismatic"' /tmp/test_urdf.xml || echo 0)"
else
    echo "[FAIL] xacro 处理失败:"
    cat /tmp/xacro_err.txt
fi

echo ""
echo "恢复命令: cp ${XACRO_FILE}.bak $XACRO_FILE"
echo "完成!"
