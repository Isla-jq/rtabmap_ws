import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')

                # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.dynamic_broadcaster_ = TransformBroadcaster(self)
        self.timer_create = self.create_timer(0.1,self.dynamic_callback)
        self.dynamic_transforms=[]

        self.sub = self.create_subscription(VehicleOdometry,'fmu/out/vehicle_odometry',self.odometrycallback,qos_profile)

        self.linear_x = 0.0 # 前进速度
        self.linear_y = 0.0  # 侧向速度（通常为0）
        self.linear_z = 0.0  # 垂直速度（通常为0）
        self.angular_x = 0.0  # 绕x轴的角速度（通常为0）
        self.angular_y = 0.0  # 绕y轴的角速度（通常为0）
        self.angular_z = 0.0  # 绕z轴的角速度

        self.t1 = TransformStamped()
        self.t1.header.frame_id = 'base_link'
        self.t1.child_frame_id = 'drone_1'

    def odometrycallback(self, msgs):
        try:
            # self.get_logger().info(f'msgs.position{msgs.position[2]}')
            self.t1.transform.translation.x = float(msgs.position[0])
            self.t1.transform.translation.y = -float(msgs.position[1])
            self.t1.transform.translation.z = -float(msgs.position[2])

            self.t1.transform.rotation.w = float(msgs.q[0])
            self.t1.transform.rotation.x = float(msgs.q[1])
            self.t1.transform.rotation.y = float(msgs.q[2])
            self.t1.transform.rotation.z = float(msgs.q[3])

        except:
            pass

    def dynamic_callback(self):
        self.t1.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_broadcaster_.sendTransform(self.t1)


def main(args = None):
    rclpy.init(args=args)
    node = DynamicTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


        # q = quaternion_from_euler(0,0,self.angular_z,'rxyz')
        # self.t1.transform.rotation.x=q[0]
        # self.t1.transform.rotation.y=q[1]
        # self.t1.transform.rotation.z=q[2]
        # self.t1.transform.rotation.w=q[3]

        # self.dynamic_transforms.append(self.t1)

        # t2 = TransformStamped()
        # t2.header.frame_id = 'base_footprint'
        # t2.header.stamp = self.get_clock().now().to_msg()
        # t2.child_frame_id = 'laser'

        # t2.transform.translation.x = 0.0
        # t2.transform.translation.y = 0.0
        # t2.transform.translation.z = 0.2

        # q = quaternion_from_euler(0,0,0,'rxyz')
        # t2.transform.rotation.x=q[0]
        # t2.transform.rotation.y=q[1]
        # t2.transform.rotation.z=q[2]
        # t2.transform.rotation.w=q[3]

        # self.dynamic_transforms.append(t2)

        # t3 = TransformStamped()
        # t3.header.frame_id = 'base_link'
        # t3.header.stamp = self.get_clock().now().to_msg()
        # t3.child_frame_id = 'odom'

        # t3.transform.translation.x = 0.0
        # t3.transform.translation.y = 0.0
        # t3.transform.translation.z = 0.0

        # q = quaternion_from_euler(0,0,self.rol+0.1,'rxyz')
        # t3.transform.rotation.x=q[0]
        # t3.transform.rotation.y=q[1]
        # t3.transform.rotation.z=q[2]
        # t3.transform.rotation.w=q[3]

        # self.dynamic_transforms.append(t3)
        # self.dynamic_broadcaster_.sendTransform(self.dynamic_transforms)