import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from fortis_interfaces.msg import Vision as vsn
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class Motivation(Node):
    def __init__(self):
        # Ros
        super().__init__("motivation_capability")
        self.motivation_pub = self.create_publisher(Float32, "/steven/reasoning/motivation_capability", 1)
        self.create_subscription(vsn, "/steven/sensing/vision", self.vision_callback, 1)
        self.get_logger().info("Initialized")

        # Messages
        self.msg = Float32()
        self.msg.data = 50.0

        # Variables
        self.last_seen = self.get_clock().now()

        ## Fuzzy
        # Input
        camera_downtime = ctrl.Antecedent(np.arange(0, 2.01, 0.01), 'camera_downtime')
        camera_downtime['low']  = fuzz.gaussmf(camera_downtime.universe, 0.0, 0.3)
        camera_downtime['med']  = fuzz.gaussmf(camera_downtime.universe, 1.0, 0.3)
        camera_downtime['high'] = fuzz.gaussmf(camera_downtime.universe, 2.0, 0.3)
        
        # Output
        capability_motivation = ctrl.Consequent(np.arange(0, 101, 1), 'capability_motivation')
        for mot in [capability_motivation]:
            mot['low'] = fuzz.trimf(mot.universe, [0, 0, 30])
            mot['med'] = fuzz.trimf(mot.universe, [20, 50, 80])
            mot['high'] = fuzz.trimf(mot.universe, [60, 100, 100])
        
        # Rules
        rules = []
        rules.append(ctrl.Rule(camera_downtime['high'], capability_motivation['high']))
        rules.append(ctrl.Rule(camera_downtime['med'], capability_motivation['med']))
        rules.append(ctrl.Rule(camera_downtime['low'], capability_motivation['low']))

        # Control system
        system = ctrl.ControlSystem(rules)
        self.sim = ctrl.ControlSystemSimulation(system)

        # Run
        self.create_timer(0.10, self.fuzzy_inference)
    
    
    def vision_callback(self, msg):
        self.last_seen = msg.header.stamp


    def fuzzy_inference(self):
        self.sim.input["camera_downtime"] = (self.get_clock().now() - self.last_seen).nanoseconds / 1e9
        self.sim.compute()

        self.msg.data = self.sim.output["capability_motivation"]
        self.motivation_pub.publish(self.msg)



def main(args=None):
    rclpy.init(args=args)
    motivation = Motivation()
    try:
        rclpy.spin(motivation)
    except KeyboardInterrupt:
        motivation.get_logger().warn("Shutdown called")
    except Exception as e:
        motivation.get_logger().error(f"Unexpected error occurred: {e}")
    finally:
        pass


if __name__ == '__main__':
    main()