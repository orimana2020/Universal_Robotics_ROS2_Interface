import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64MultiArray

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('print_tf')
        self.subscription_tf = self.create_subscription(TFMessage,'/tf',self.listener_callback_tf,10)
        self.subscription_tf  # prevent unused variable warning
       
    def listener_callback_tf(self, msg):
        pass
        print(msg.transforms[0].transform.translation.x)
        # self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    sub_node = MinimalSubscriber()

    rclpy.spin(sub_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()