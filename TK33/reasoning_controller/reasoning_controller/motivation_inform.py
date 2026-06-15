import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from fortis_interfaces.msg import Vision as vsn
from fortis_interfaces.msg import Proximity as prx
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt


class Motivation(Node):
    def __init__(self):
        # Ros
        super().__init__("motivation_inform")
        self.motivation_pub = self.create_publisher(Float32, "/steven/reasoning/motivation_inform", 1)
        self.create_subscription(vsn, "/steven/sensing/vision", self.vision_callback, 1)
        self.create_subscription(prx, "/steven/sensing/proximity", self.proximity_callback, 1)
        self.create_subscription(Bool, "/steven/movement/dist_reset", self.reset_callback, 1)
        self.create_subscription(Float32, "/steven/reasoning/motivation_capability", self.capability_callback, 1)
        self.get_logger().info("Initialized")

        # Messages
        self.msg = Float32()

        # Variables
        self.human_visible = False
        self.pDist = 2.0
        self.pDistCooldown = Duration(seconds=2)
        self.pDistReset = self.get_clock().now()
        self.reset = False
        self.enabled = True
        
        ## Fuzzy ##
        # Input
        min_persistent_distance_to_obstacle = 0
        max_persistent_distance_to_obstacle = 1
        step_persistent_distance_to_obstacle = 0.01

        min_human_visible = 0
        max_human_visible = 1
        step_human_visible = 0.1

        persistent_distance_to_obstacle = ctrl.Antecedent(np.arange(min_persistent_distance_to_obstacle, max_persistent_distance_to_obstacle+step_persistent_distance_to_obstacle, step_persistent_distance_to_obstacle), 'persistent_distance_to_obstacle')
        persistent_distance_to_obstacle['low']  = fuzz.gaussmf(persistent_distance_to_obstacle.universe, 0.15, 0.12)
        persistent_distance_to_obstacle['med']  = fuzz.gaussmf(persistent_distance_to_obstacle.universe, 0.45, 0.2)
        persistent_distance_to_obstacle['high'] = fuzz.gaussmf(persistent_distance_to_obstacle.universe, 0.80, 0.3)

        human_visible = ctrl.Antecedent(np.arange(min_human_visible, max_human_visible+step_human_visible, step_human_visible), 'human_visible')
        human_visible['no']  = fuzz.gaussmf(human_visible.universe, 0, 0.2)
        human_visible['yes'] = fuzz.gaussmf(human_visible.universe, 1, 0.2)

        # Output
        inform_motivation = ctrl.Consequent(np.arange(0, 101, 1), 'inform_motivation')
        for mot in [inform_motivation]:
            mot['low'] = fuzz.trimf(mot.universe, [0, 0, 30])
            mot['med'] = fuzz.trimf(mot.universe, [20, 40, 80])
            mot['high'] = fuzz.trimf(mot.universe, [60, 100, 100])

        # Rules
        rules = []
        rules.append(ctrl.Rule(persistent_distance_to_obstacle['low'] & human_visible['no'], inform_motivation['med']))
        rules.append(ctrl.Rule(persistent_distance_to_obstacle['med'] & human_visible['no'], inform_motivation['low']))
        rules.append(ctrl.Rule(persistent_distance_to_obstacle['high'] & human_visible['no'], inform_motivation['low']))
        rules.append(ctrl.Rule(persistent_distance_to_obstacle['low'] & human_visible['yes'], inform_motivation['high']))
        rules.append(ctrl.Rule(persistent_distance_to_obstacle['med'] & human_visible['yes'], inform_motivation['med']))
        rules.append(ctrl.Rule(persistent_distance_to_obstacle['high'] & human_visible['yes'], inform_motivation['med']))

        # Control system
        system = ctrl.ControlSystem(rules)
        self.sim = ctrl.ControlSystemSimulation(system)

        # Run
        self.create_timer(0.10, self.fuzzy_inference)

        # Plotting
        # upsampled_x = np.linspace(min_persistent_distance_to_obstacle, max_persistent_distance_to_obstacle, max_persistent_distance_to_obstacle-min_persistent_distance_to_obstacle+1)
        # upsampled_y = np.linspace(min_human_visible, max_human_visible, max_human_visible-min_human_visible+1)
        # x, y = np.meshgrid(upsampled_x, upsampled_y)
        # z = np.zeros_like(x)

        # for j, x_ in enumerate(upsampled_x):
        #     for i, y_ in enumerate(upsampled_y):
        #         self.sim.input['persistent_distance_to_obstacle'] = x[i, j]
        #         self.sim.input['human_visible'] = y[i, j]
        #         self.sim.compute()
        #         z[i, j] = self.sim.output['inform_motivation']

        # fig = plt.figure(figsize=(8, 8))
        # ax = fig.add_subplot(111, projection='3d')

        # surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='viridis',
        #                     linewidth=0.4, antialiased=True)

        # cset = ax.contourf(x, y, z, zdir='z', offset=-2.5, cmap='viridis', alpha=0.5)
        # cset = ax.contourf(x, y, z, zdir='x', offset=3, cmap='viridis', alpha=0.5)
        # cset = ax.contourf(x, y, z, zdir='y', offset=3, cmap='viridis', alpha=0.5)

        # ax.view_init(30, 200)
        # plt.show()    
    

    ## Callbacks ##
    def vision_callback(self, msg):
        self.human_visible = msg.visible
    
    def proximity_callback(self, msg):
        if msg.distance < self.pDist and self.get_clock().now() - self.pDistReset > self.pDistCooldown:
            self.pDist = msg.distance
    
    def reset_callback(self, msg):
        self.reset = msg
    
    def capability_callback(self, msg):
        if msg.data > 50:
            self.enabled = False
        else:
            self.enabled = True

    
    def fuzzy_inference(self):
        # Distance reset
        if self.reset:
            self.pDist = 5.0
            self.pDistReset = self.get_clock().now()
            self.reset = False

        # Compute
        self.sim.input["human_visible"] = self.human_visible
        self.sim.input["persistent_distance_to_obstacle"] = self.pDist
        self.sim.compute()
        self.msg.data = self.sim.output["inform_motivation"] if self.enabled else 0.0
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