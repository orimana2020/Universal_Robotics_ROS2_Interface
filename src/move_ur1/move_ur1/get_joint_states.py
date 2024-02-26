import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('print_tf')
        self.subscription_tf = self.create_subscription(JointState,'/joint_states',self.joint_states_cb,10)
        self.subscription_tf  # prevent unused variable warning
       
    def joint_states_cb(self, msg):
        pos = msg.position
        base = pos[5]
        shoulder = pos[0]
        elbow = pos[1]
        wrist1 = pos[2]
        wrist2 = pos[3]
        wrist3 = pos[4]
        print(base, shoulder, elbow, wrist1, wrist2, wrist3)


def main(args=None):
    rclpy.init(args=args)

    sub_node = MinimalSubscriber()

    rclpy.spin(sub_node)
    sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()