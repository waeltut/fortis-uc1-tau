import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Float32
from fortis_interfaces.msg import AggMotivation as agg
import numpy as np

class Aggregate(Node):
    def __init__(self):
        # ROS -stuff
        super().__init__("aggregate_motivation")
        self.subscribers = {}
        self.data = {}
        self.poll_interval = Duration(seconds=5)
        self.last_poll = self.get_clock().now()
        self.log_interval = Duration(seconds=0.5)
        self.agg_pub = self.create_publisher(agg, "/steven/reasoning/agg_motivation", 1)
        self.prefix = '/steven/reasoning/motivation_'
        self.create_timer(0.10, self.spinner)
    

    def spinner(self):
        # Stuf
        start = self.get_clock().now()

        # Poll for new topics every 2 seconds
        if self.get_clock().now() - self.last_poll > self.poll_interval:
            topics = self.get_topic_names_and_types()
            for t_name, t_type in topics:
                if t_name.startswith(self.prefix) and t_name not in self.subscribers:
                    self.get_logger().info(f"Subscribed to {t_name}")
                    self.subscribers[t_name] = self.create_subscription(Float32, t_name, lambda msg, tn=t_name: self.callback(msg, tn), 1)
            self.last_poll = self.get_clock().now()

        # Then compile and send motivations
        msg = agg()
        msg.name = list(self.data.keys())
        msg.value = list(self.data.values())
        self.agg_pub.publish(msg)
    

    def callback(self, msg, topic_name):
        self.data[topic_name] = msg.data



def main(args=None):
    rclpy.init(args=args)
    aggregate = Aggregate()
    try:
        rclpy.spin(aggregate)
    except KeyboardInterrupt:
        aggregate.get_logger().warn("Shutdown called")
    except Exception as e:
        aggregate.get_logger().error(f"Unexpected error occurred: {e}")
    finally:
        pass


if __name__ == '__main__':
    main()