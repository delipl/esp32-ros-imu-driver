import matplotlib.pyplot as plt
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MessageFrequencyPlotter(Node):
    def __init__(self):
        super().__init__('message_frequency_plotter')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.message_callback,
            qos_policy
        )
        self.time = []
        self.counter = 0
        self.linear_x = []
        self.linear_y = []
        self.x = 0.0
        self.y = 0.0
        self.timer_period = 1.0/50
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def timer_callback(self):
        self.time.append(float(self.counter) * self.timer_period)
        self.linear_x.append(self.x)
        self.linear_y.append(self.y)
        self.counter += 1


    def message_callback(self, msg):
        self.x = (msg.linear.x)
        self.y = (msg.linear.y)

    def plot_values(self):
        # time = list(range(self.counter))
        plt.plot(self.time, self.linear_x, label='Linear X')
        plt.plot(self.time, self.linear_y, label='Linear Y')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Value')
        plt.title('ROS 2 Message Values on /cmd_vel Topic')
        plt.legend()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    plotter = MessageFrequencyPlotter()
    try:
        rclpy.spin(plotter)
        plotter.destroy_node()
        rclpy.shutdown()
    except:
        plotter.plot_values()

if __name__ == '__main__':
    main()
