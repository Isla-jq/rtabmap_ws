import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import math
from rclpy.time import Time
import sys

class StaticTFBroadcaster(Node):
    def __init__(self,transformation):
        super().__init__('static_tf_broadcaster')
        self.static_broadcaster_ = StaticTransformBroadcaster(self)
        self.static_transforms = []
        self.publish_static(transformation)
        
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        手动计算欧拉角到四元数的转换
        返回 geometry_msgs/Quaternion 格式
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0.0, 0.0, 0.0, 0.0]
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        q[3] = cr * cp * cy + sr * sp * sy  # w
        return q

    def publish_static(self,transformation):
        # 3. base_link -> laser (激光雷达，高度0.3米)
        t2 = TransformStamped()
        t2.header.frame_id = 'base_link'
        t2.header.stamp = Time().to_msg()
        t2.child_frame_id = 'laser'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.3
        q = self.quaternion_from_euler(0, 0, 0)  # 无旋转
        t2.transform.rotation.x = q[0]
        t2.transform.rotation.y = q[1]
        t2.transform.rotation.z = q[2]
        t2.transform.rotation.w = q[3]
        self.static_transforms.append(t2)

        # 修改 publish_static() 方法中的摄像头 TF 部分
        t3 = TransformStamped()
        t3.header.frame_id = 'laser'  # 父坐标系改为激光雷达
        t3.header.stamp = Time().to_msg()
        t3.child_frame_id = 'ascamera_hp60c_camera_link_0'
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.5  # 摄像头在激光雷达正上方0.5米

        # 旋转对齐：摄像头Z轴（前）对齐激光雷达X轴（前）
        q = self.quaternion_from_euler(0, -1.57, -1.57)  # roll=0, pitch=-π/2, yaw=-π/2
        t3.transform.rotation.x = q[0]
        t3.transform.rotation.y = q[1]
        t3.transform.rotation.z = q[2]
        t3.transform.rotation.w = q[3]
        self.static_transforms.append(t3)

        self.static_broadcaster_.sendTransform(self.static_transforms)

def main(args=None):
    rclpy.init()
    node = StaticTFBroadcaster(sys.argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()