import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from fortis_interfaces.msg import Vision as vsn
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


class Motivation(Node):
    def __init__(self):
        # Ros
        super().__init__("motivation_respond")
        self.motivation_pub = self.create_publisher(Float32, "/steven/reasoning/motivation_respond", 1)
        self.create_subscription(vsn, "/steven/sensing/vision", self.vision_callback, 1)
        self.create_subscription(Float64, "/steven/sensing/time_since_interaction", self.hearing_callback, 1)
        self.create_subscription(Float32, "/steven/reasoning/motivation_capability", self.capability_callback, 1)
        self.get_logger().info("Initialized")

        # Messages
        self.msg = Float32()

        # Variables
        self.human_visible = False
        self.timeInteraction = 10
        self.enabled = True
        
        ## Fuzzy ##
        # Input
        time_since_last_interaction = ctrl.Antecedent(np.arange(0, 20.1, 0.1), 'time_since_last_interaction')
        time_since_last_interaction['low']  = fuzz.gaussmf(time_since_last_interaction.universe, 0, 4.0)
        time_since_last_interaction['med']  = fuzz.gaussmf(time_since_last_interaction.universe, 10, 4.0)
        time_since_last_interaction['high'] = fuzz.gaussmf(time_since_last_interaction.universe, 20, 4.0)
        human_visible = ctrl.Antecedent(np.arange(0, 1.1, 0.1), 'human_visible')
        human_visible['no']  = fuzz.gaussmf(human_visible.universe, 0, 0.2)
        human_visible['yes'] = fuzz.gaussmf(human_visible.universe, 1, 0.2)

        # Output
        respond_motivation = ctrl.Consequent(np.arange(0, 101, 1), 'respond_motivation')
        for mot in [respond_motivation]:
            mot['low'] = fuzz.trimf(mot.universe, [0, 0, 30])
            mot['med'] = fuzz.trimf(mot.universe, [20, 50, 80])
            mot['high'] = fuzz.trimf(mot.universe, [60, 100, 100])

        # Rules
        rules = []
        rules.append(ctrl.Rule(time_since_last_interaction['high'] & human_visible['no'], respond_motivation['low']))
        rules.append(ctrl.Rule(time_since_last_interaction['high'] & human_visible['yes'], respond_motivation['low']))
        rules.append(ctrl.Rule(time_since_last_interaction['med'] & human_visible['no'], respond_motivation['med']))
        rules.append(ctrl.Rule(time_since_last_interaction['med'] & human_visible['yes'], respond_motivation['high']))
        rules.append(ctrl.Rule(time_since_last_interaction['low'] & human_visible['no'], respond_motivation['high']))
        rules.append(ctrl.Rule(time_since_last_interaction['low'] & human_visible['yes'], respond_motivation['high']))

        # Control system
        system = ctrl.ControlSystem(rules)
        self.sim = ctrl.ControlSystemSimulation(system)

        # Run
        self.create_timer(0.10, self.fuzzy_inference)
    

    ## Callbacks ##
    def vision_callback(self, msg):
        self.human_visible = msg.visible
    
    def hearing_callback(self, msg):
        self.timeInteraction = self.get_clock().now().nanoseconds / 1e9 - msg.data
    
    def capability_callback(self, msg):
        if msg.data > 50:
            self.enabled = False
        else:
            self.enabled = True

    
    def fuzzy_inference(self):
        # Compute
        self.sim.input["human_visible"] = self.human_visible
        self.sim.input["time_since_last_interaction"] = self.timeInteraction
        self.sim.compute()
        self.msg.data = self.sim.output["respond_motivation"] if self.enabled else 0.0
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