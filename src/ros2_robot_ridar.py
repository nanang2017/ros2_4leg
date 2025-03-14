import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FilteredScanPublisher(Node):
    def __init__(self):
        super().__init__('filtered_scan_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # 기존 LIDAR 토픽
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            LaserScan,
            '/filtered_scan',  # 4° 간격으로 필터링한 데이터 발행할 토픽
            10)

    def listener_callback(self, msg):
        angle_min = math.degrees(msg.angle_min)  # 시작 각도 (라디안 → 도 변환)
        angle_increment = math.degrees(msg.angle_increment)  # 각 데이터 간의 각도 차이 (도 단위)
        total_points = len(msg.ranges)  # 총 데이터 개수

        step = int(4.0 / angle_increment)  # 4° 간격 샘플링
        filtered_ranges = [None] * 90 # 90개 저장장

        print("\n===== Filtered Lidar Data (4° Step) =====")
        for i in range(0, total_points, step):
            angle = angle_min + (i * angle_increment)  # 현재 각도 계산 (도)
            
            index = int((angle + 180) / 4)  # ✅ -180° ~ 180° 범위를 4° 간격 인덱스로 변환
            if 0 <= index < 90:
                filtered_ranges[index] = msg.ranges[i]
                print(f"각도 {angle:.1f}°: 거리 {msg.ranges[i]:.2f}m")

        # 새로운 LaserScan 메시지 생성
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max 
        filtered_msg.angle_increment = math.radians(4.0)  # 4° 간격
        filtered_msg.time_increment = msg.time_increment * step
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = filtered_ranges  # 필터링된 데이터 저장

        # 필터링된 메시지 퍼블리시
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FilteredScanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
