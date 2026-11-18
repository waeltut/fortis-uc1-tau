import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Float32, Float64
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseWithCovarianceStamped    # Just use /slam_toolbox/pose
from tf2_ros import Buffer, TransformListener
import math
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

class Motivation(Node):
    def __init__(self):
        # Ros
        super().__init__("motivation_work")
        self.motivation_pub = self.create_publisher(Float32, "/steven/reasoning/motivation_work", 1)
        self.dist_pub = self.create_publisher(Float64, "/reasoning/dist_goal", 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Bool, "/steven/movement/work_complete", self.work_callback, 1)
        self.create_subscription(String, "/steven/movement/mot_mode", self.mode_callback, 1)
        self.get_logger().info("Initialized")

        # Messages
        self.msg = Float32()
        self.msg.data = 50.0
        self.dist_msg = Float64()

        # Variables
        self.goal = [0.042, -1.921, 0]
        self.last_worked = self.get_clock().now()
        self.motivated = False

        ## Fuzzy
        # Input

        min_time_since_last_worked = 0
        max_time_since_last_worked = 180
        step_time_since_last_worked = 0.1
        
        min_distance_to_goal = 0
        max_distance_to_goal = 8
        step_distance_to_goal = 0.01

        time_since_last_worked = ctrl.Antecedent(np.arange(min_time_since_last_worked, max_time_since_last_worked+step_time_since_last_worked, step_time_since_last_worked), 'time_since_last_worked')
        time_since_last_worked['low']  = fuzz.gaussmf(time_since_last_worked.universe, 0.0, 10.0)
        time_since_last_worked['med']  = fuzz.gaussmf(time_since_last_worked.universe, 60.0, 20.0)
        time_since_last_worked['high'] = fuzz.gaussmf(time_since_last_worked.universe, 100.0, 30.0)
        time_since_last_worked['very_high'] = fuzz.gaussmf(time_since_last_worked.universe, 180.0, 40.0)

        distance_to_goal = ctrl.Antecedent(np.arange(min_distance_to_goal, max_distance_to_goal+step_distance_to_goal, step_distance_to_goal), 'distance_to_goal')
        distance_to_goal['low']  = fuzz.gaussmf(distance_to_goal.universe, 0.0, 2.0)
        distance_to_goal['med']  = fuzz.gaussmf(distance_to_goal.universe, 4.0, 2.0)
        distance_to_goal['high'] = fuzz.gaussmf(distance_to_goal.universe, 8.0, 2.0)
        
        # Output
        work_motivation = ctrl.Consequent(np.arange(0, 101, 1), 'work_motivation')
        for mot in [work_motivation]:
            mot['low'] = fuzz.trimf(mot.universe, [0, 0, 30])
            mot['med'] = fuzz.trimf(mot.universe, [20, 50, 80])
            mot['high'] = fuzz.trimf(mot.universe, [60, 100, 100])
        
        # Rules
        rules = []
        rules.append(ctrl.Rule(time_since_last_worked['very_high'], work_motivation['high']))
        rules.append(ctrl.Rule(time_since_last_worked['high'] & distance_to_goal['low'], work_motivation['high']))
        rules.append(ctrl.Rule(time_since_last_worked['high'] & distance_to_goal['med'], work_motivation['high']))
        rules.append(ctrl.Rule(time_since_last_worked['high'] & distance_to_goal['high'], work_motivation['med']))
        rules.append(ctrl.Rule(time_since_last_worked['med'] & distance_to_goal['low'], work_motivation['med']))
        rules.append(ctrl.Rule(time_since_last_worked['med'] & distance_to_goal['med'], work_motivation['med']))
        rules.append(ctrl.Rule(time_since_last_worked['med'] & distance_to_goal['high'], work_motivation['low']))
        rules.append(ctrl.Rule(time_since_last_worked['low'], work_motivation['low']))

        # Control system
        system = ctrl.ControlSystem(rules)
        self.sim = ctrl.ControlSystemSimulation(system)

        # Run
        self.create_timer(0.10, self.fuzzy_inference)

        # Plotting
        """
        upsampled_x = np.linspace(min_time_since_last_worked, max_time_since_last_worked, max_time_since_last_worked-min_time_since_last_worked+1)        # time since last 
        upsampled_y = np.linspace(min_distance_to_goal, max_distance_to_goal, max_distance_to_goal-min_distance_to_goal+1)            # distance to goal
        x, y = np.meshgrid(upsampled_x, upsampled_y)
        z = np.zeros_like(x)

        for j, x_ in enumerate(upsampled_x):
            for i, y_ in enumerate(upsampled_y):
                self.sim.input['time_since_last_worked'] = x[i, j]
                self.sim.input['distance_to_goal'] = y[i, j]
                self.sim.compute()
                z[i, j] = self.sim.output['work_motivation']

        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')

        surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='viridis',
                            linewidth=0.4, antialiased=True)

        cset = ax.contourf(x, y, z, zdir='z', offset=-2.5, cmap='viridis', alpha=0.5)
        cset = ax.contourf(x, y, z, zdir='x', offset=3, cmap='viridis', alpha=0.5)
        cset = ax.contourf(x, y, z, zdir='y', offset=3, cmap='viridis', alpha=0.5)

        ax.view_init(30, 200)
        plt.show()    
        """
        

    def work_callback(self, msg):
        if msg:
            self.last_worked = self.get_clock().now()
    
    def mode_callback(self, msg):
        print(msg.data)
        if msg.data == "automatic":
            self.motivated = True
        else:
            self.motivated = False


    def fuzzy_inference(self):
        self.motivation_pub.publish(self.msg)
        """
        try:
            # get transform from map -> base_link
            (trans, rot) = self.tf_buffer.lookup_transform(
                            'target_frame',
                            'map',
                            rclpy.time.Time()
            )
            x, y, z = trans
            #qx, qy, qz, qw = rot
            dist = math.sqrt((x-self.goal[0])**2 + (y-self.goal[1])**2)
            if not self.motivated:
                self.last_worked = self.get_clock().now().nanoseconds / 1e9 - 35
            self.sim.input["time_since_last_worked"] = (self.get_clock().now() - self.last_worked).nanoseconds / 1e9
            self.sim.input["distance_to_goal"] = dist
            self.dist_msg.data = dist
            self.dist_pub.publish(self.dist_msg)
            self.sim.compute()
            self.msg.data = self.sim.output['work_motivation']
            self.motivation_pub.publish(self.msg)
        except:
            pass
        """

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