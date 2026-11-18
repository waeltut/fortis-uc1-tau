import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class EventCorrelatorNode(Node):
    def __init__(self):
        super().__init__('event_correlator_node')
        
        self.create_subscription(String, '/cognition/visual/content', self.vis_callback, 10)
        self.create_subscription(String, '/cognition/audio/content', self.aud_callback, 10)
        self.fused_pub = self.create_publisher(String, '/cognition/fused/events', 10)
        
        self.event_buffer = []
        self.TEMPORAL_WINDOW_SEC = 5
        
        # Run correlation check periodically
        self.create_timer(0.5, self.process_buffer)
        self.get_logger().info("Event Correlator started.")

    def vis_callback(self, msg):
        data = json.loads(msg.data)
        self.event_buffer.append({"type": "visual", "data": data["content"], "desc": data["descriptor"], "time": time.time()})

    def aud_callback(self, msg):
        data = json.loads(msg.data)
        self.event_buffer.append({"type": "audio", "data": data["content"], "desc": data["descriptor"], "time": time.time()})

    def process_buffer(self):
        current_time = time.time()
        i = 0
        while i < len(self.event_buffer):
            oldest = self.event_buffer[i]
            best_match, best_match_idx = None, -1
            
            for j, other in enumerate(self.event_buffer):
                if i != j and other["type"] != oldest["type"]:
                    time_diff = abs(oldest["time"] - other["time"])
                    if time_diff <= self.TEMPORAL_WINDOW_SEC:
                        best_match, best_match_idx = other, j
                        break # Found a match
                        
            if best_match:
                vis = oldest if oldest["type"] == "visual" else best_match
                aud = oldest if oldest["type"] == "audio" else best_match
                
                payload = {
                    "status": "related",
                    "visual_content": {"data": vis["data"], "descriptor_data": vis["desc"]},
                    "audio_content": {"data": aud["data"], "descriptor_data": aud["desc"]}
                }
                self.fused_pub.publish(String(data=json.dumps(payload)))
                
                del self.event_buffer[max(i, best_match_idx)]
                del self.event_buffer[min(i, best_match_idx)]
                i = 0
                continue
                
            elif (current_time - oldest["time"]) > self.TEMPORAL_WINDOW_SEC:
                # Timeout, send as unrelated
                payload = {
                    "status": "unrelated",
                    oldest["type"] + "_content": {"data": oldest["data"], "descriptor_data": oldest["desc"]}
                }
                self.fused_pub.publish(String(data=json.dumps(payload)))
                del self.event_buffer[i]
                i = 0
                continue
            i += 1

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EventCorrelatorNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()