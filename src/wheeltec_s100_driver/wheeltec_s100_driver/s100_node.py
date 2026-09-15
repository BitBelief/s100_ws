#!/usr/bin/env python3
"""WHEELTEC S100 ROS 2 驅動節點（Jazzy）。

/cmd_vel  ──► 11-byte 序列指令（固定 20 Hz 重送）──► STM32
/odom /imu/data_raw /battery_state ◄── 24-byte 回傳幀（20 Hz）

兩條鐵律（2026-09-09 實機量到的，不是猜的）：
  1. STM32 看門狗約 1.0 秒斷訊歸零 → 必須固定重送，送一次就不管會走走停停。
  2. 1 秒太久（0.5 m/s 會滑 50 cm）→ 本節點自己 0.2 秒沒收到 /cmd_vel 就主動送零。
"""
import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, Imu
from tf2_ros import TransformBroadcaster

from .protocol import FrameParser, encode_cmd

# 底盤回傳標稱 20 Hz。只用來給「第一批」一個時間基準，之後一律用實測間隔。
NOMINAL_RX_PERIOD_NS = 50_000_000
# 間隔超過這個秒數視為斷線，不積分——但會計數並警告，不再靜默丟棄位移。
MAX_INTEGRATE_DT = 0.5


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class S100Driver(Node):

    def __init__(self):
        super().__init__('s100_driver')

        p = self.declare_parameter
        p('port', '/dev/ttyACM0')
        p('baudrate', 115200)
        p('tx_rate', 20.0)              # 指令重送頻率（Hz）
        p('cmd_timeout', 0.2)           # 多久沒收到 /cmd_vel 就送零（秒）
        p('cmd_vel_type', 'twist')      # 'twist' 或 'twist_stamped'（Nav2 Jazzy 用後者）
        p('max_linear', 0.8)            # 保守上限，量到實際極速後再放寬
        p('max_angular', 2.0)
        p('linear_scale', 1.0)          # 輪徑校正：實測距離 / 指令距離
        p('angular_scale', 1.0)         # 輪距校正：實測角度 / 指令角度
        p('publish_tf', True)
        p('odom_frame', 'odom')
        p('base_frame', 'base_footprint')
        p('imu_frame', 'imu_link')
        # ⚠ 這兩組是 placeholder，不是估計出來的不確定度。見下方啟動警告。
        p('pose_covariance_diagonal', [0.05, 0.05, 1e6, 1e6, 1e6, 0.1])
        p('twist_covariance_diagonal', [0.05, 0.05, 1e6, 1e6, 1e6, 0.1])

        g = self.get_parameter
        self.port = g('port').value
        self.baud = g('baudrate').value
        self.cmd_timeout = g('cmd_timeout').value
        self.max_lin = g('max_linear').value
        self.max_ang = g('max_angular').value
        self.lin_scale = g('linear_scale').value
        self.ang_scale = g('angular_scale').value
        self.publish_tf = g('publish_tf').value
        self.odom_frame = g('odom_frame').value
        self.base_frame = g('base_frame').value
        self.imu_frame = g('imu_frame').value
        self.pose_cov = list(g('pose_covariance_diagonal').value)
        self.twist_cov = list(g('twist_covariance_diagonal').value)

        self._cmd = (0.0, 0.0)
        self._cmd_stamp = 0.0
        self._lock = threading.Lock()
        self._ser = None
        self._running = True
        self._rx_thread = None
        self._zeroed = True          # 已進入「送零」狀態，避免重複印警告
        self._last_rx_ns = None      # 上一幀的時間戳（整數奈秒），用來積分
        self._gap_drops = 0          # 間隔過大而未積分的次數
        self.x = self.y = self.th = 0.0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_odom = self.create_publisher(Odometry, 'odom', qos)
        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', qos)
        self.pub_bat = self.create_publisher(BatteryState, 'battery_state', qos)
        self.tf_bc = TransformBroadcaster(self) if self.publish_tf else None

        ctype = g('cmd_vel_type').value
        if ctype == 'twist_stamped':
            self.create_subscription(TwistStamped, 'cmd_vel',
                                     lambda m: self.on_cmd(m.twist), qos)
        else:
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd, qos)

        self.get_logger().warn(
            'odom 的 covariance 是 placeholder，不是估計出來的不確定度 —— '
            '底盤韌體不回報它。需要校準過的位姿不確定度時不可直接採用。')
        self.get_logger().warn(
            'IMU 是 MPU6050（6 軸、無磁力計）→ 不發布 orientation，yaw 只能靠積分，必然漂移。')

        self.open_serial()
        self._rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self._rx_thread.start()
        self.create_timer(1.0 / g('tx_rate').value, self.tx_tick)
        self.get_logger().info(f'S100 driver 啟動：{self.port} @ {self.baud}')

    # ---------- 序列 ----------

    def open_serial(self):
        # serial_for_url：真車給 /dev/ttyACM0，測試可給 socket://host:port（假底盤）
        try:
            s = serial.serial_for_url(self.port, baudrate=self.baud,
                                      timeout=0.05, do_not_open=True)
            try:
                s.dtr = False       # CH343：別讓 DTR/RTS 去重置 STM32
                s.rts = False
            except (AttributeError, ValueError, OSError):
                pass                # 虛擬裝置沒有 modem control line
            s.open()
            time.sleep(0.3)
            s.reset_input_buffer()
            self._ser = s
            return True
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'開啟 {self.port} 失敗：{e}')
            self._ser = None
            return False

    def rx_loop(self):
        parser = FrameParser()
        while self._running:
            ser = self._ser          # 取本地參考，避免 check-then-use 競態
            if ser is None:
                time.sleep(1.0)
                self.open_serial()
                continue
            try:
                data = ser.read(256)
            except (serial.SerialException, OSError, AttributeError) as e:
                self.get_logger().error(f'序列讀取中斷：{e}')
                self._ser = None
                continue
            if not data:
                continue
            frames = parser.feed(data)
            if not frames:
                continue
            # 一次 read 可能一口氣收到多幀。若每幀都蓋上「解析當下」的時鐘，
            # 批內 dt≈0，那幾幀的位移會被積分整個吃掉。改成把「上一批到這一批」
            # 的實際間隔平均分配給批內各幀 —— 總時間守恆，位移不會憑空消失。
            now_ns = self.get_clock().now().nanoseconds
            if self._last_rx_ns is None:
                base_ns = now_ns - NOMINAL_RX_PERIOD_NS * len(frames)
            else:
                base_ns = self._last_rx_ns
            step_ns = (now_ns - base_ns) // len(frames)
            for i, st in enumerate(frames):
                self.on_state(st, base_ns + step_ns * (i + 1))

    def tx_tick(self):
        with self._lock:
            vx, wz = self._cmd
            age = time.monotonic() - self._cmd_stamp
        if age > self.cmd_timeout:
            if not self._zeroed:
                self.get_logger().warn(
                    f'{age:.2f}s 沒收到 /cmd_vel（門檻 {self.cmd_timeout}s）→ 送零')
                self._zeroed = True
            vx, wz = 0.0, 0.0
        ser = self._ser              # 取本地參考，避免 check-then-use 競態
        if ser is None:
            return
        try:
            ser.write(encode_cmd(vx / self.lin_scale, 0.0, wz / self.ang_scale))
        except (serial.SerialException, OSError, AttributeError) as e:
            self.get_logger().error(f'序列寫入中斷：{e}')
            self._ser = None

    # ---------- 回呼 ----------

    def on_cmd(self, msg):
        vx = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        wz = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        with self._lock:
            self._cmd = (vx, wz)
            self._cmd_stamp = time.monotonic()
        self._zeroed = False

    def on_state(self, st, t_ns):
        if not self._running or not rclpy.ok():
            return          # 關閉中，context 可能已失效
        stamp = Time(nanoseconds=t_ns).to_msg()

        # 用回傳的實測速度積分（編碼器 + STM32 算的，不是指令值）
        if self._last_rx_ns is not None:
            dt = (t_ns - self._last_rx_ns) * 1e-9
            if 0.0 < dt < MAX_INTEGRATE_DT:
                vx = st.vx * self.lin_scale
                wz = st.wz * self.ang_scale
                th_mid = self.th + wz * dt * 0.5
                self.x += vx * math.cos(th_mid) * dt
                self.y += vx * math.sin(th_mid) * dt
                self.th = math.atan2(math.sin(self.th + wz * dt),
                                     math.cos(self.th + wz * dt))
            elif dt >= MAX_INTEGRATE_DT:
                # 不積分，但絕不靜默 —— 里程會少掉這一整段位移。
                self._gap_drops += 1
                self.get_logger().warn(
                    f'回傳中斷 {dt:.2f}s（門檻 {MAX_INTEGRATE_DT}s）→ '
                    f'這段位移未積分，累計 {self._gap_drops} 次')
        self._last_rx_ns = t_ns

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation = yaw_to_quat(self.th)
        od.twist.twist.linear.x = st.vx * self.lin_scale
        od.twist.twist.angular.z = st.wz * self.ang_scale
        for i in range(6):
            od.pose.covariance[i * 7] = self.pose_cov[i]
            od.twist.covariance[i * 7] = self.twist_cov[i]
        self.pub_odom.publish(od)

        if self.tf_bc is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = yaw_to_quat(self.th)
            self.tf_bc.sendTransform(tf)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        imu.orientation_covariance[0] = -1.0    # ROS 慣例：沒有姿態估計
        imu.angular_velocity.x = st.gx
        imu.angular_velocity.y = st.gy
        imu.angular_velocity.z = st.gz
        imu.linear_acceleration.x = st.ax
        imu.linear_acceleration.y = st.ay
        imu.linear_acceleration.z = st.az
        self.pub_imu.publish(imu)

        bat = BatteryState()
        bat.header.stamp = stamp
        bat.voltage = st.voltage
        bat.present = st.voltage > 1.0
        self.pub_bat.publish(bat)

    # ---------- 收尾 ----------

    def shutdown(self):
        self._running = False
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=2.0)   # 先讓讀取執行緒停，再關 context
        ser = self._ser              # 取本地參考：停車路徑不能因競態而拋例外
        if ser is not None:
            try:
                for _ in range(5):
                    ser.write(encode_cmd(0.0, 0.0, 0.0))
                    ser.flush()
                    time.sleep(0.05)
                ser.close()
            except (serial.SerialException, OSError, AttributeError):
                pass
        self.get_logger().info('已送零速度並關閉序列埠')


def main():
    rclpy.init()
    node = S100Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
