#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
import tty
import termios
import select
import time

# 轮子参数（和C++程序对齐）
WHEEL_CIRCUMFERENCE_CM = 31.4159  # π*10cm
WHEEL_BASE_CM = 37.6               # 376mm → cm
RPM_TO_CM_PER_S = WHEEL_CIRCUMFERENCE_CM / 60.0
CM_PER_S_TO_M_PER_S = 0.01

# 极速响应配置（核心解决按键延迟）
KEY_READ_TIMEOUT = 0.001   # 按键读取超时（1ms，原10ms，提速10倍）
LONG_PRESS_THRESHOLD = 0.1 # QE长按判定阈值
LONG_PRESS_INTERVAL = 0.05 # QE长按重复触发间隔

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        
        # 1. QoS配置（保持可靠传输）
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        
        # 2. 终端极速配置（关闭所有缓冲，无延迟）
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        self._set_fast_terminal()
        self.should_exit = False

        # 3. 速度配置（保持原参数）
        self.base_rpm = 20
        self.rpm_step = 2
        self.max_rpm = 100
        self.min_rpm = 5
        
        # 4. 状态保持核心变量
        self.current_action = None   # 记录当前运动状态（保持不变）
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        
        # 5. QE长按状态记录
        self.last_qe_time = 0.0
        self.last_key = ''

        # 6. 按键映射（完全保留原逻辑）
        self.KEY_ACTION_MAP = {
            'w': ('forward',),
            's': ('backward',),
            'a': ('turn_left',),
            'd': ('turn_right',),
            'q': ('speed_down',),
            'e': ('speed_up',),
            '0': ('stop',),
            'c': ('emergency_stop',),
            'x': ('exit',),
        }

        # 7. 提高发布频率至50Hz（原10Hz，指令下发更快）
        self.pub_freq = 50
        self.timer = self.create_timer(1.0/self.pub_freq, self.publish_cmd_vel)
        
        self.print_help()

    def _set_fast_terminal(self):
        """极速终端配置：关闭回显/行缓冲/信号延迟，按键按下立即响应"""
        settings = termios.tcgetattr(self.fd)
        # 关闭规范模式(无行缓冲)、关闭回显、关闭中断信号
        settings[3] &= ~(termios.ICANON | termios.ECHO | termios.IGNBRK | termios.BRKINT)
        # 最小读取1字节，读取超时0ms
        settings[6][termios.VMIN] = 1
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, settings) # 立即生效

    def print_help(self):
        self.get_logger().info("="*60)
        self.get_logger().info("差速小车键盘控制（状态保持版·极速响应）")
        self.get_logger().info(f"初始轮速：{self.base_rpm}rpm（线速度≈{self.rpm_to_lin_vel(self.base_rpm):.2f}m/s）")
        self.get_logger().info("【控制逻辑】按一下WASD切换状态，不按键保持当前运动")
        self.get_logger().info("【速度调整】Q长按减速 | E长按加速（每次±2rpm）")
        self.get_logger().info("【急停/停止】C急停 | 0停止 | X退出并停止")
        self.get_logger().info("="*60)

    # ------------- 工具函数 -------------
    def rpm_to_lin_vel(self, rpm):
        return (rpm * RPM_TO_CM_PER_S) * CM_PER_S_TO_M_PER_S

    def rpm_to_ang_vel(self, rpm):
        lin_vel_cm = rpm * RPM_TO_CM_PER_S
        return (2 * lin_vel_cm) / WHEEL_BASE_CM

    def get_key(self):
        """非阻塞极速读键，1ms超时，无延迟"""
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], KEY_READ_TIMEOUT)
            if rlist:
                key = sys.stdin.read(1).lower()
                # 过滤无效字符
                return key if ord(key) > 31 and ord(key) != 127 else ''
            return ''
        except:
            return ''

    def send_stop_cmd(self, immediate_publish=False):
        """停止并清空状态"""
        self.current_action = None
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        if immediate_publish:
            stop_twist = Twist()
            # 连续发3次确保底层收到
            for _ in range(3):
                self.cmd_vel_pub.publish(stop_twist)
                time.sleep(0.001)
            self.get_logger().info("🚨 已停止所有运动")

    def refresh_action_speed(self):
        """根据当前状态刷新速度（调速后自动更新）"""
        if not self.current_action:
            self.current_linear_x = 0.0
            self.current_angular_z = 0.0
            return

        lin_vel = self.rpm_to_lin_vel(self.base_rpm)
        ang_vel = self.rpm_to_ang_vel(self.base_rpm)
        
        if self.current_action == 'forward':
            self.current_linear_x = lin_vel
            self.current_angular_z = 0.0
        elif self.current_action == 'backward':
            self.current_linear_x = -lin_vel
            self.current_angular_z = 0.0
        elif self.current_action == 'turn_left':
            self.current_linear_x = 0.0
            self.current_angular_z = -ang_vel
        elif self.current_action == 'turn_right':
            self.current_linear_x = 0.0
            self.current_angular_z = ang_vel

    # ------------- 按键处理（状态保持核心） -------------
    def handle_key(self, key):
        if key not in self.KEY_ACTION_MAP:
            return

        action = self.KEY_ACTION_MAP[key][0]
        now = time.time()

        # 1. 急停/退出/普通停止
        if action == 'emergency_stop':
            self.send_stop_cmd(immediate_publish=True)
            self.get_logger().info("⚠️ C键紧急停止")
        elif action == 'exit':
            self.send_stop_cmd(immediate_publish=True)
            self.should_exit = True
            self.get_logger().info("⚠️ X键退出，小车已停止")
        elif action == 'stop':
            self.send_stop_cmd(immediate_publish=True)

        # 2. QE长按加减速（保持原逻辑）
        elif action in ['speed_down', 'speed_up']:
            # 首次按下或长按间隔满足才触发
            if key != self.last_key or now - self.last_qe_time > LONG_PRESS_INTERVAL:
                self.base_rpm = max(self.min_rpm, self.base_rpm - self.rpm_step) if action == 'speed_down' else min(self.max_rpm, self.base_rpm + self.rpm_step)
                self.last_qe_time = now
                self.get_logger().info(f"⚙️ 当前轮速：{self.base_rpm}rpm")
                self.refresh_action_speed() # 刷新当前运动的速度
            self.last_key = key

        # 3. WASD状态切换（按一下就保持，核心需求）
        elif action in ['forward', 'backward', 'turn_left', 'turn_right']:
            self.current_action = action
            self.refresh_action_speed()
            self.get_logger().info(f"▶️ 切换状态：{action}")

    def publish_cmd_vel(self):
        """50Hz高频发布，保持当前状态速度"""
        twist = Twist()
        twist.linear.x = self.current_linear_x
        twist.angular.z = self.current_angular_z
        self.cmd_vel_pub.publish(twist)

    # ------------- 主循环 -------------
    def run(self):
        try:
            while rclpy.ok() and not self.should_exit:
                key = self.get_key()
                if key:
                    self.handle_key(key)
                # 1ms自旋，无空转延迟
                rclpy.spin_once(self, timeout_sec=0.001)
        finally:
            # 退出必停+恢复终端
            self.send_stop_cmd(immediate_publish=True)
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            self.destroy_timer(self.timer)
            self.get_logger().info("✅ 节点安全退出，终端已恢复")

# ------------- 主函数 -------------
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"运行异常：{e}")
        node.send_stop_cmd(immediate_publish=True)
        termios.tcsetattr(node.fd, termios.TCSADRAIN, node.old_settings)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()