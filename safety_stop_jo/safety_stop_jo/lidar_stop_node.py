# 이 코드 전체를 복사하세요
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Float32

def ang_diff(a: float, b: float) -> float:
    """원형 각도 차이 ([-pi, pi])"""
    return math.atan2(math.sin(a - b), math.cos(a - b))

class LidarStop(Node):
    """
    지정한 중심각(center_deg)을 기준으로 FOV(±fov_deg/2) 안에서
    trigger_dist_m 이하 장애물이 연속 frames_confirm 프레임 감지되면
    /cmd_stop(Empty)을 퍼블리시. one_shot=True면 멈춤 후 래치.
    /cmd_relock 수신 또는 auto_resume_sec 경과 시 래치 해제.

    LaserScan 각도는 라디안, 기준은 LiDAR 프레임의 +x(전방)가 0rad.
    뒤쪽(180°) 감지는 center_deg=180.0 사용.
    """
    def __init__(self):
        super().__init__('lidar_stop')
        self.declare_parameters('', [
            ('trigger_dist_m', 2.0),         # 트리거 거리 [m]
            ('fov_deg', 50.0),               # 시야각(전체) [deg], 예: ±25°
            ('center_deg', 180.0),           # 감지 중심각 [deg], 0=정면, 180=후방
            ('frames_confirm', 2),           # 연속 프레임 수
            ('min_speed_cps', 0.0),          # 휠 cps 하한(없으면 0)
            ('one_shot', True),              # 한 번만 멈춤(래치)
            ('publish_cmd_stop', True),      # /cmd_stop 발행 여부
            ('auto_resume_enable', True),    # 자동 재출발(자동 릴록) 사용
            ('auto_resume_sec', 5.0),        # 정지 후 N초 뒤 /cmd_relock 자동 발행
            ('relock_topic', '/cmd_relock'),
            ('scan_topic', '/scan'),         # LiDAR 스캔 토픽
            ('wheel_cps_topic', '/wheel_cps')
        ])

        # 파라미터 로드
        self.trigger = float(self.get_parameter('trigger_dist_m').value)
        fov_deg = float(self.get_parameter('fov_deg').value)
        self.fov_rad = math.radians(fov_deg)
        self.center_rad = math.radians(float(self.get_parameter('center_deg').value))
        self.need_frames = int(self.get_parameter('frames_confirm').value)
        self.min_speed_cps = float(self.get_parameter('min_speed_cps').value)
        self.one_shot = bool(self.get_parameter('one_shot').value)
        self.pub_stop = bool(self.get_parameter('publish_cmd_stop').value)
        self.auto_resume_enable = bool(self.get_parameter('auto_resume_enable').value)
        self.auto_resume_sec = float(self.get_parameter('auto_resume_sec').value)

        self.already_stopped = False
        self.ok_frames = 0
        self.cur_cps = 0.0
        self._resume_timer = None  # 자동 릴록 타이머 핸들

        # 통신 설정
        scan_topic = str(self.get_parameter('scan_topic').value)
        wheel_cps_topic = str(self.get_parameter('wheel_cps_topic').value)
        self.sub_scan = self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)
        self.sub_cps  = self.create_subscription(Float32, wheel_cps_topic, self.on_cps, 10)
        self.pub_cmd_stop = self.create_publisher(Empty, '/cmd_stop', 1)

        relock_topic = str(self.get_parameter('relock_topic').value)
        self.sub_relock = self.create_subscription(Empty, relock_topic, self.on_relock, 1)
        # 오토-릴록을 위해 같은 토픽으로 퍼블리셔도 보유
        self.pub_relock = self.create_publisher(Empty, relock_topic, 1)

        self.get_logger().info(
            f'[lidar_stop] trigger={self.trigger} m, CENTER={math.degrees(self.center_rad):.1f}°, '
            f'FOV=±{math.degrees(self.fov_rad)/2:.1f}°, frames={self.need_frames}, one_shot={self.one_shot}, '
            f'auto_resume={self.auto_resume_enable} @{self.auto_resume_sec:.1f}s'
        )

    def on_cps(self, msg: Float32):
        self.cur_cps = msg.data

    def on_relock(self, _):
        # 외부 또는 내부(오토-릴록)에서 relock 수신 시 래치 해제
        self.already_stopped = False
        self.ok_frames = 0
        # 타이머가 살아있다면 정리
        if self._resume_timer is not None:
            try:
                self._resume_timer.cancel()
            except Exception:
                pass
            self._resume_timer = None
        self.get_logger().info('[lidar_stop] /cmd_relock received → latch cleared (resume)')

    def _start_auto_resume_timer(self):
        if not self.auto_resume_enable:
            return
        # 이미 타이머가 있으면 재생성하지 않음
        if self._resume_timer is not None:
            return

        def _timer_cb():
            # 1회성 실행: /cmd_relock 발행하여 다시 주행 가능 상태로
            self.pub_relock.publish(Empty())
            self.get_logger().warn(f'[lidar_stop] AUTO-RELOCK after {self.auto_resume_sec:.1f}s → resume driving')
            # 타이머 정리
            if self._resume_timer is not None:
                try:
                    self._resume_timer.cancel()
                except Exception:
                    pass
                self._resume_timer = None

        # create_timer는 주기 타이머라 콜백 한 번 실행 후 우리가 취소
        self._resume_timer = self.create_timer(self.auto_resume_sec, _timer_cb)

    def on_scan(self, scan: LaserScan):
        # --- 디버깅 로그 ---
        self.get_logger().info('>>> on_scan callback received a message!')

        if self.one_shot and self.already_stopped:
            return

        angle = scan.angle_min
        has_close = False
        half_fov = self.fov_rad / 2.0

        for r in scan.ranges:
            if math.isfinite(r):
                # 중심각(self.center_rad)과의 최소 원형 각도 차이로 FOV 판정
                if abs(ang_diff(angle, self.center_rad)) <= half_fov and (r <= self.trigger):
                    has_close = True
                    break
            angle += scan.angle_increment

        # --- 디버깅 로그 ---
        self.get_logger().info(f'    Scan Check: has_close={has_close}')

        # 속도 하한 적용(원하면 0으로 끄기)
        if has_close and self.cur_cps < self.min_speed_cps:
            has_close = False

        self.ok_frames = self.ok_frames + 1 if has_close else 0
        self.get_logger().info(f'    Frame Count: ok_frames={self.ok_frames} / needed={self.need_frames}')

        if self.ok_frames >= self.need_frames:
            self.ok_frames = 0
            if self.pub_stop:
                self.pub_cmd_stop.publish(Empty())
                self.get_logger().warn(
                    f'[lidar_stop] E-STOP TRIGGERED (≤{self.trigger} m, center={math.degrees(self.center_rad):.1f}°)'
                )
            # 래치 + 자동 릴록 예약
            if self.one_shot:
                self.already_stopped = True
            # 정지 후 자동 재출발 타이머 시작
            self._start_auto_resume_timer()

def main():
    rclpy.init()
    node = LidarStop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
