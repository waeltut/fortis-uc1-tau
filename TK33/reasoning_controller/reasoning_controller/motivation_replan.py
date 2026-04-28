import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Float32
from fortis_interfaces.msg import Proximity as prx
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


class Motivation(Node):
    def __init__(self):
        # Ros
        super().__init__("motivation_replan")
        self.motivation_pub = self.create_publisher(Float32, "/steven/reasoning/motivation_replan", 1)    # Namespace works by leaving the leading slash off
        self.create_subscription(prx, "/steven/sensing/proximity", self.proximity_callback, 1)
        self.get_logger().info("Initialized")

        # Messages
        self.msg = Float32()

        # Variables
        self.dist = 2
        self.angle = 0
        
        ## Fuzzy ##
        # Input
        distance_to_obstacle = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'distance_to_obstacle')
        distance_to_obstacle['low']  = fuzz.gaussmf(distance_to_obstacle.universe, 0.15, 0.12)
        distance_to_obstacle['med']  = fuzz.gaussmf(distance_to_obstacle.universe, 0.45, 0.2)
        distance_to_obstacle['high'] = fuzz.gaussmf(distance_to_obstacle.universe, 0.80, 0.3)
        angle_to_obstacle = ctrl.Antecedent(np.arange(0, np.pi/2, 0.01), 'angle_to_obstacle')
        angle_to_obstacle['low']  = fuzz.gaussmf(angle_to_obstacle.universe, 0.0, 0.3)
        angle_to_obstacle['med']  = fuzz.gaussmf(angle_to_obstacle.universe, 0.5, 0.3)
        angle_to_obstacle['high'] = fuzz.gaussmf(angle_to_obstacle.universe, np.pi/2, 0.3)

        # Output
        replan_motivation = ctrl.Consequent(np.arange(0, 101, 1), 'replan_motivation')
        for mot in [replan_motivation]:
            mot['low'] = fuzz.trimf(mot.universe, [0, 0, 30])
            mot['med'] = fuzz.trimf(mot.universe, [20, 40, 60])
            mot['high'] = fuzz.trimf(mot.universe, [50, 100, 100])

        # Rules
        rules = []
        rules.append(ctrl.Rule(distance_to_obstacle['low'] & angle_to_obstacle['low'], replan_motivation['high']))
        rules.append(ctrl.Rule(distance_to_obstacle['low'] & angle_to_obstacle['med'], replan_motivation['high']))
        rules.append(ctrl.Rule(distance_to_obstacle['low'] & angle_to_obstacle['high'], replan_motivation['med']))
        rules.append(ctrl.Rule(distance_to_obstacle['med'] & angle_to_obstacle['low'], replan_motivation['med']))
        rules.append(ctrl.Rule(distance_to_obstacle['med'] & angle_to_obstacle['med'], replan_motivation['med']))
        rules.append(ctrl.Rule(distance_to_obstacle['med'] & angle_to_obstacle['high'], replan_motivation['low']))
        rules.append(ctrl.Rule(distance_to_obstacle['high'] & angle_to_obstacle['low'], replan_motivation['med']))
        rules.append(ctrl.Rule(distance_to_obstacle['high'] & angle_to_obstacle['med'], replan_motivation['low']))
        rules.append(ctrl.Rule(distance_to_obstacle['high'] & angle_to_obstacle['high'], replan_motivation['low']))

        # Control system
        system = ctrl.ControlSystem(rules)
        self.sim = ctrl.ControlSystemSimulation(system)

        # Run
        self.create_timer(0.10, self.fuzzy_inference)
    

    ## Callbacks ##    
    def proximity_callback(self, msg):
        self.dist = msg.distance
        self.angle = abs(msg.angle)

    
    def fuzzy_inference(self):
        # Compute
        self.sim.input["angle_to_obstacle"] = self.angle
        self.sim.input["distance_to_obstacle"] = self.dist
        self.sim.compute()
        self.msg.data = 0.0#self.sim.output["replan_motivation"]
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