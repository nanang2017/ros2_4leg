"""
Example to move the robot in blind mode using ROS2 API without services.
"""
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import UInt32
from geometry_msgs.msg import Vector3, Twist
from nav_msgs.msg import Odometry  # ë©”ì‹œì§€ íƒ€ìž… ë³€ê²½ ê°€ëŠ¥
from std_srvs.srv import Empty

Iteration = 100
Speed_X = 0.5
Speed_Y = 0.5
Speed_Angular = 1.0

class VelocityOdomActivator(Node):
    def __init__(self):
        super().__init__('velocity_odom_activator')
        self.client = self.create_client(Empty, '/activate_velocity_odom')

        while not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('ì„œë¹„ìŠ¤ë¥¼ ê¸°ë‹¤ë¦¬ëŠ” ì¤‘...')

    def call_service(self):
        request = Empty.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('ì„œë¹„ìŠ¤ í˜¸ì¶œ ì„±ê³µ!')
        else:
            self.get_logger().error('ì„œë¹„ìŠ¤ í˜¸ì¶œ ì‹¤íŒ¨!')

class TestMoveBlindNoService(Node):
    def __init__(self):
        super().__init__("MoveBlindNoService")
        self.pub_action = self.create_publisher(UInt32, '/command/setAction', 10)
        self.pub_run = self.create_publisher(UInt32, '/command/setRun', 10)
        self.pub_control_mode = self.create_publisher(UInt32, '/command/setControlMode', 10)
        self.pub_twist = self.create_publisher(Twist, '/mcu/command/manual_twist', 10)
        self.minimal_subscriber = MinimalSubscriber()

    def Initialize(self):
        self.get_logger().info("Setting control mode=170")
        self.pub_control_mode.publish(UInt32(data=170))
        time.sleep(0.01)
        self.pub_action.publish(UInt32(data=1)) # stand
        time.sleep(1)

        self.get_logger().info("Setting action=2")
        self.pub_action.publish(UInt32(data=2)) # walk mode

    def Forward(self):
        self.get_logger().info("Commanding forward twist")
        for i in range(Iteration):
            self.pub_twist.publish(Twist(linear=Vector3(x=Speed_X)))
            time.sleep(0.01) # do this instead of sleep(2) to avoid timeout

        self.get_logger().info("Setting action=0")
        self.pub_twist.publish(Twist()) # zero twist

    def Backward(self):
        self.get_logger().info("Commanding Backward twist")
        for i in range(Iteration):

            self.pub_twist.publish(Twist(linear=Vector3(x=-Speed_X)))
            time.sleep(0.01) # do this instead of sleep(2) to avoid timeout

        self.get_logger().info("Setting action=0")
        self.pub_twist.publish(Twist()) # zero twist

    def RightSide(self):
        self.get_logger().info("Commanding RightSide twist")
        for i in range(Iteration):
            self.pub_twist.publish(Twist(linear=Vector3(y=Speed_Y)))
            time.sleep(0.01) # do this instead of sleep(2) to avoid timeout

        self.get_logger().info("Setting action=0")
        self.pub_twist.publish(Twist()) # zero twist

    def LeftSide(self):
        self.get_logger().info("Commanding LeftSide twist")
        for i in range(Iteration):
            self.pub_twist.publish(Twist(linear=Vector3(y=-Speed_Y)))
            time.sleep(0.01) # do this instead of sleep(2) to avoid timeout

        self.get_logger().info("Setting action=0")
        self.pub_twist.publish(Twist()) # zero twist

    def TurnRight(self, angular):
        self.get_logger().info("Commanding TurnRight twist")
        rclpy.spin_once(self.minimal_subscriber)
        initial_angular = self.minimal_subscriber.angular
        changed_angular = angular
        while abs(changed_angular - initial_angular - angular) >= 0.05:
            rclpy.spin_once(self.minimal_subscriber)
            changed_angular = self.minimal_subscriber.angular
            self.pub_twist.publish(Twist(angular=Vector3(z=(changed_angular-initial_angular-angular))))
            time.sleep(0.01) # do this instead of sleep(2) to avoid timeout
            print((changed_angular-initial_angular) * 10)

        self.get_logger().info("Setting action=0")
        self.pub_twist.publish(Twist()) # zero twist

    def TurnLeft(self, angular):
        self.get_logger().info("Commanding TurnLeft twist")
        for i in range(angular):
            self.pub_twist.publish(Twist(angular=Vector3(z=-Speed_Angular)))
            time.sleep(0.01) # do this instead of sleep(2) to avoid timeout

        self.get_logger().info("Setting action=0")
        self.pub_twist.publish(Twist()) # zero twist

    def Endmode(self):
        node.pub_action.publish(UInt32(data=0)) # sit
        time.sleep(5)
        node.get_logger().info("Setting control mode=180")
        node.pub_control_mode.publish(UInt32(data=180))


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,  # ìˆ˜ì‹ í•  ë©”ì‹œì§€ íƒ€ìž…
            '/odom',  # êµ¬ë…í•  í† í”½ ì´ë¦„
            self.odom_callback,  # ì½œë°± í•¨ìˆ˜
            10  # í í¬ê¸°
        )
        self.subscription  # ë°©ì¶œ ë°©ì§€
        self.angular =0 

    def odom_callback(self, msg):
        # ìœ„ì¹˜(Position)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # ìžì„¸(Orientation) (ì¿¼í„°ë‹ˆì–¸ ê°’)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        self.angular = qz
        qw = msg.pose.pose.orientation.w

        # ì†ë„(Twist)
        linear_x = msg.twist.twist.linear.x
        angular_z = msg.twist.twist.angular.z

        # ë¡œê·¸ ì¶œë ¥
        self.get_logger().info(f"ìœ„ì¹˜: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        self.get_logger().info(f"ìžì„¸(ì¿¼í„°ë‹ˆì–¸): qx={qx:.2f}, qy={qy:.2f}, qz={qz:.2f}, qw={qw:.2f}")
        self.get_logger().info(f"ì†ë„: ì„ ì†ë„ x={linear_x:.2f}, ê°ì†ë„ z={angular_z:.2f}")

        self.get_logger().info(msg.pose.pose)

if __name__=="__main__":
    rclpy.init(args=None)
    node = TestMoveBlindNoService()
    minimal_subscriber = MinimalSubscriber()

    node.Initialize()
    print("forward : 1 \n backward : 2 \n right : 3 \n left : 4 \n turn right : 5 \n turn left : 6 \n end : 7 \n ")
    while True:
        In = input()

        if In == "1":
            node.Forward()
            rclpy.spin_once(minimal_subscriber)
        elif In == "2":
            node.Backward()
            rclpy.spin_once(minimal_subscriber)
        elif In == "3":
            node.RightSide()
            rclpy.spin_once(minimal_subscriber)
        elif In == "4":
            node.LeftSide()
            rclpy.spin_once(minimal_subscriber)
        elif In == "5":
            node.TurnRight(Iteration)
            rclpy.spin_once(minimal_subscriber)
        elif In == "6":
            node.TurnLeft(Iteration)
            rclpy.spin_once(minimal_subscriber)
        
        elif In =="7":
            node.Endmode()
            break

    node.destroy_node()
    rclpy.shutdown()