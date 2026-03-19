#!/usr/bin/env python3
"""
test_wheels.py — Gazebo 舵轮管道测试 (不依赖 MPC/acados)

验证清单:
  [1] Gazebo 启动, 模型加载
  [2] controller_spawner 成功, 9 个控制器就位
  [3] /joint_states 有数据 (joint_state_controller 工作)
  [4] /odom 有数据 (p3d 插件工作)
  [5] 舵角命令 → 轮子实际转动
  [6] 驱动命令 → 轮子实际旋转
  [7] 轮子映射正确 (FL/FR/RL/RR 对应正确的 Wheel_N)

用法:
  rosrun nmpc_ctrl test_wheels.py
  或通过 test_gazebo.launch 自动启动
"""
import rospy
import math
import sys
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

# ============================================================
# URDF 轮子 → 逻辑轮子 映射 (和 gazebo_controllers.yaml 一致)
#   FL = Wheel_2  (0.247, +0.247)  前左
#   FR = Wheel_1  (0.247, -0.247)  前右
#   RL = Wheel_4  (-0.247, +0.247) 后左
#   RR = Wheel_3  (-0.247, -0.247) 后右
# ============================================================
WHEEL_MAP = {
    'FL': {'steer': 'Wheel_2_direction_joint', 'drive': 'Wheel_2_drive_joint'},
    'FR': {'steer': 'Wheel_1_direction_joint', 'drive': 'Wheel_1_drive_joint'},
    'RL': {'steer': 'Wheel_4_direction_joint', 'drive': 'Wheel_4_drive_joint'},
    'RR': {'steer': 'Wheel_3_direction_joint', 'drive': 'Wheel_3_drive_joint'},
}
WHEEL_ORDER = ['FL', 'FR', 'RL', 'RR']


class WheelTester:
    def __init__(self):
        rospy.init_node('test_wheels', anonymous=True)

        # --- 发布器 (和 gazebo_controllers.yaml 话题名对应) ---
        ctrl_names = ['fl', 'fr', 'rl', 'rr']
        self.steer_pubs = {}
        self.drive_pubs = {}
        for i, name in enumerate(WHEEL_ORDER):
            cn = ctrl_names[i]
            self.steer_pubs[name] = rospy.Publisher(
                '/%s_steer_controller/command' % cn, Float64, queue_size=1)
            self.drive_pubs[name] = rospy.Publisher(
                '/%s_drive_controller/command' % cn, Float64, queue_size=1)

        # --- 订阅 ---
        self.js_data = None
        self.js_count = 0
        self.odom_data = None
        self.odom_count = 0
        rospy.Subscriber('/joint_states', JointState, self._js_cb)
        rospy.Subscriber('/odom', Odometry, self._odom_cb)

        self.pass_count = 0
        self.fail_count = 0
        self.warn_count = 0

    def _js_cb(self, msg):
        self.js_data = msg
        self.js_count += 1

    def _odom_cb(self, msg):
        self.odom_data = msg
        self.odom_count += 1

    # --- 辅助 ---
    def get_joint(self, js, name, field='position'):
        try:
            idx = list(js.name).index(name)
            if field == 'position':
                return js.position[idx] if idx < len(js.position) else 0.0
            elif field == 'velocity':
                return js.velocity[idx] if idx < len(js.velocity) else 0.0
        except (ValueError, IndexError):
            return None
        return 0.0

    def send_all(self, steer_val, drive_val, duration=2.0, rate_hz=50):
        """给所有轮子发相同命令, 持续 duration 秒"""
        r = rospy.Rate(rate_hz)
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < duration and not rospy.is_shutdown():
            for name in WHEEL_ORDER:
                self.steer_pubs[name].publish(Float64(data=steer_val))
                self.drive_pubs[name].publish(Float64(data=drive_val))
            r.sleep()

    def send_individual(self, steer_dict, drive_dict, duration=2.0, rate_hz=50):
        """给各轮子发不同命令"""
        r = rospy.Rate(rate_hz)
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < duration and not rospy.is_shutdown():
            for name in WHEEL_ORDER:
                self.steer_pubs[name].publish(Float64(data=steer_dict.get(name, 0.0)))
                self.drive_pubs[name].publish(Float64(data=drive_dict.get(name, 0.0)))
            r.sleep()

    def log_result(self, test_name, passed, msg=""):
        if passed:
            self.pass_count += 1
            rospy.loginfo("  [PASS] %s %s", test_name, msg)
        else:
            self.fail_count += 1
            rospy.logerr("  [FAIL] %s %s", test_name, msg)

    def log_warn(self, test_name, msg=""):
        self.warn_count += 1
        rospy.logwarn("  [WARN] %s %s", test_name, msg)

    # ============================================================
    # 测试用例
    # ============================================================
    def test_1_topics(self):
        """检查 /joint_states 和 /odom 是否有数据"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("TEST 1: 话题连接检查")
        rospy.loginfo("  等待 3 秒收集数据...")
        rospy.sleep(3.0)

        # joint_states
        self.log_result("joint_states",
                        self.js_count > 0,
                        "(收到 %d 条)" % self.js_count)
        if self.js_data:
            rospy.loginfo("  关节列表 (%d 个): %s",
                          len(self.js_data.name), list(self.js_data.name))

        # odom (p3d 插件)
        self.log_result("/odom (p3d)",
                        self.odom_count > 0,
                        "(收到 %d 条)" % self.odom_count)
        if self.odom_data:
            p = self.odom_data.pose.pose.position
            rospy.loginfo("  初始位置: x=%.3f y=%.3f z=%.3f", p.x, p.y, p.z)

    def test_2_steer_response(self):
        """发送舵角命令, 检查轮子是否响应"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("TEST 2: 舵角响应 — 所有轮子转到 +0.5 rad (28.6 deg)")

        # 先归零
        self.send_all(0.0, 0.0, duration=1.0)
        rospy.sleep(0.5)

        # 发 +0.5
        self.send_all(0.5, 0.0, duration=2.0)
        rospy.sleep(0.5)

        if not self.js_data:
            self.log_result("舵角响应", False, "没有 joint_states 数据")
            return

        all_ok = True
        for name in WHEEL_ORDER:
            jname = WHEEL_MAP[name]['steer']
            pos = self.get_joint(self.js_data, jname, 'position')
            if pos is None:
                self.log_result("%s 舵角" % name, False,
                                "关节 %s 不在 joint_states 中!" % jname)
                all_ok = False
            else:
                err = abs(pos - 0.5)
                ok = err < 0.15  # 允许 0.15 rad 误差
                self.log_result("%s 舵角" % name, ok,
                                "%s = %.3f rad (目标 0.5, 误差 %.3f)" % (jname, pos, err))
                if not ok:
                    all_ok = False

        # 归零
        self.send_all(0.0, 0.0, duration=1.5)

    def test_3_drive_response(self):
        """发送驱动命令, 检查轮速响应"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("TEST 3: 驱动响应 — 舵角=0, 轮速=5 rad/s 前进")

        self.send_all(0.0, 0.0, duration=1.0)

        # 记录初始 odom
        odom_before = None
        if self.odom_data:
            odom_before = (self.odom_data.pose.pose.position.x,
                           self.odom_data.pose.pose.position.y)

        # 发驱动命令
        self.send_all(0.0, 5.0, duration=3.0)
        rospy.sleep(0.5)

        if not self.js_data:
            self.log_result("驱动响应", False, "没有 joint_states 数据")
            return

        all_ok = True
        for name in WHEEL_ORDER:
            jname = WHEEL_MAP[name]['drive']
            vel = self.get_joint(self.js_data, jname, 'velocity')
            if vel is None:
                self.log_result("%s 轮速" % name, False,
                                "关节 %s 不在 joint_states 中!" % jname)
                all_ok = False
            else:
                ok = abs(vel) > 1.0  # 至少转起来了
                self.log_result("%s 轮速" % name, ok,
                                "%s = %.2f rad/s (目标 5.0)" % (jname, vel))
                if not ok:
                    all_ok = False

        # 检查 odom 是否变化
        if self.odom_data and odom_before:
            dx = self.odom_data.pose.pose.position.x - odom_before[0]
            dy = self.odom_data.pose.pose.position.y - odom_before[1]
            dist = math.sqrt(dx * dx + dy * dy)
            self.log_result("odom 位移", dist > 0.1,
                            "移动了 %.3f m (dx=%.3f, dy=%.3f)" % (dist, dx, dy))

        # 停车
        self.send_all(0.0, 0.0, duration=2.0)

    def test_4_mapping_verify(self):
        """单独控制每个轮子, 验证映射正确"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("TEST 4: 轮子映射验证 — 逐个转动")

        self.send_all(0.0, 0.0, duration=1.0)

        for name in WHEEL_ORDER:
            rospy.loginfo("  --- 单独转 %s 舵角到 +0.8 rad ---", name)

            # 只给这个轮子发舵角命令
            steer_dict = {n: 0.0 for n in WHEEL_ORDER}
            steer_dict[name] = 0.8
            drive_dict = {n: 0.0 for n in WHEEL_ORDER}

            self.send_individual(steer_dict, drive_dict, duration=1.5)
            rospy.sleep(0.3)

            if self.js_data:
                # 检查目标轮子转了
                target_joint = WHEEL_MAP[name]['steer']
                target_pos = self.get_joint(self.js_data, target_joint, 'position')

                # 检查其他轮子没转 (或回到0)
                others_ok = True
                for other in WHEEL_ORDER:
                    if other == name:
                        continue
                    other_joint = WHEEL_MAP[other]['steer']
                    other_pos = self.get_joint(self.js_data, other_joint, 'position')
                    if other_pos is not None and abs(other_pos) > 0.3:
                        others_ok = False
                        rospy.logwarn("    %s 也转了! %s=%.3f (应该是 ~0)",
                                      other, other_joint, other_pos)

                if target_pos is not None:
                    ok = abs(target_pos - 0.8) < 0.2 and others_ok
                    self.log_result(
                        "%s 映射" % name, ok,
                        "%s=%.3f (目标 0.8), 其他轮子%s" %
                        (target_joint, target_pos,
                         "正常" if others_ok else "异常!"))
                else:
                    self.log_result("%s 映射" % name, False, "关节不存在")

            # 归零
            self.send_all(0.0, 0.0, duration=0.5)

    def test_5_forward_drive(self):
        """所有轮子舵角=0, 前进, 检查 odom x 增加"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("TEST 5: 直线前进 — 检查 odom 方向")

        self.send_all(0.0, 0.0, duration=2.0)

        if self.odom_data:
            x0 = self.odom_data.pose.pose.position.x
            y0 = self.odom_data.pose.pose.position.y
        else:
            x0, y0 = 0, 0

        # 前进 3 秒
        self.send_all(0.0, 5.0, duration=3.0)
        rospy.sleep(0.5)

        if self.odom_data:
            x1 = self.odom_data.pose.pose.position.x
            y1 = self.odom_data.pose.pose.position.y
            dx = x1 - x0
            dy = y1 - y0
            rospy.loginfo("  odom 变化: dx=%.3f dy=%.3f", dx, dy)
            # 舵角=0 应该主要沿 x 方向移动
            self.log_result("前进方向", abs(dx) > 0.2,
                            "dx=%.3f (应该明显 >0)" % dx)
            if abs(dy) > abs(dx) * 0.5:
                self.log_warn("横向偏移",
                              "dy=%.3f 偏大, 可能映射有问题" % dy)

        self.send_all(0.0, 0.0, duration=2.0)

    def test_6_spin_in_place(self):
        """原地旋转测试"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("TEST 6: 原地旋转")

        self.send_all(0.0, 0.0, duration=2.0)

        # 原地旋转的舵角配置:
        # 每个轮子指向圆心 (对角线方向)
        # FL(+lx,+ly): steer = atan2(lx, -ly) ≈ atan2(0.247, -0.247) ≈ 2.356 (135°)
        # 但限制在 [-pi/2, pi/2], 所以用翻转: -0.785 + 反转轮速
        # 简化: 所有轮子 45° 交叉
        steer_dict = {
            'FL':  -0.785,   # -45°
            'FR': 0.785,   # 45°
            'RL': 0.785,   # 45°
            'RR':  -0.785,   # -45°
        }
        drive_dict = {
                    'FL': -3.0,
                    'FR':  3.0,
                    'RL': -3.0,
                    'RR':  3.0,
                }

        if self.odom_data:
            q = self.odom_data.pose.pose.orientation
            yaw0 = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        else:
            yaw0 = 0

        self.send_individual(steer_dict, drive_dict, duration=3.0)
        rospy.sleep(0.5)

        if self.odom_data:
            q = self.odom_data.pose.pose.orientation
            yaw1 = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            dyaw = yaw1 - yaw0
            # 归一化
            while dyaw > math.pi:
                dyaw -= 2*math.pi
            while dyaw < -math.pi:
                dyaw += 2*math.pi
            self.log_result("原地旋转", abs(dyaw) > 0.2,
                            "yaw 变化 = %.3f rad (%.1f deg)" % (dyaw, math.degrees(dyaw)))

        self.send_all(0.0, 0.0, duration=2.0)

    # ============================================================
    # 运行
    # ============================================================
    def run(self):
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("  Gazebo 舵轮管道测试 (无 MPC)")
        rospy.loginfo("=" * 60)

        self.test_1_topics()
        if self.js_count == 0:
            rospy.logerr("")
            rospy.logerr("致命: /joint_states 没有数据, 后续测试无法进行!")
            rospy.logerr("请检查:")
            rospy.logerr("  1. URDF 中是否有 <gazebo> + gazebo_ros_control 插件")
            rospy.logerr("  2. URDF 中是否有 <transmission> 标签")
            rospy.logerr("  3. controller_spawner 是否报错")
            rospy.logerr("  4. Gazebo 是否正常运行 (没有 exit code 255)")
            return

        self.test_2_steer_response()
        self.test_3_drive_response()
        self.test_4_mapping_verify()
        self.test_5_forward_drive()
        self.test_6_spin_in_place()

        # --- 汇总 ---
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("  测试汇总: %d PASS, %d FAIL, %d WARN",
                      self.pass_count, self.fail_count, self.warn_count)
        rospy.loginfo("=" * 60)

        if self.fail_count == 0:
            rospy.loginfo("")
            rospy.loginfo("  全部通过! 可以接入 MPC 了:")
            rospy.loginfo("  roslaunch nmpc_ctrl gazebo_mpc.launch")
        else:
            rospy.logerr("")
            rospy.logerr("  有 %d 项失败, 请先修复再接 MPC", self.fail_count)

        rospy.loginfo("")
        rospy.loginfo("  测试节点保持运行, 你可以在 Gazebo 中观察机器人状态")
        rospy.loginfo("  也可以手动发命令:")
        rospy.loginfo("    rostopic pub /fl_steer_controller/command std_msgs/Float64 'data: 0.5'")
        rospy.loginfo("    rostopic pub /fl_drive_controller/command std_msgs/Float64 'data: 3.0'")
        rospy.loginfo("")

        rospy.spin()


if __name__ == '__main__':
    try:
        tester = WheelTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass
