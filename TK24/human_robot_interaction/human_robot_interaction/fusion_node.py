import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import matplotlib.pyplot as plt
import google.generativeai as genai
import time
import math
import os
from math import floor
from collections import deque
from ament_index_python.packages import get_package_share_directory
import numpy as np


def calculate_fechner_intensity(physical_strength, optimal_threshold, k_factor, is_log_scale=False, inverse=False):
    """
    Applies the Weber-Fechner Law to calculate perceived sensory intensity.
    Psi = k * ln(S / S0)
    """
    try:
        if is_log_scale:
            # dB is already logarithmic. Fechner's law on a log scale is just a linear difference.
            diff = optimal_threshold - physical_strength if inverse else physical_strength - optimal_threshold
            return k_factor * diff if diff > 0 else np.finfo(float).eps
        else:
            # Prevent division by zero or log(0) errors
            strength_safe = np.maximum(physical_strength, 0.001)

            # The ratio S / S0
            ratio = optimal_threshold / strength_safe if inverse else strength_safe / optimal_threshold

            # Only apply intensity scaling if the ratio exceeds the baseline (1.0)
            return k_factor * np.log(ratio) if ratio > 1.0 else np.finfo(float).eps

    except Exception as e:
        print(f"Intensity calculation error: {e}")
        return 0.0

def calculate_modality_weights_fechner(psi_light,psi_contrast,psi_blur, psi_voice,psi_noise):
    # 1. Calculate Perceived Intensities (Psi) using Fechner's Law
    # (Physical Strength, Optimal Threshold, K-Factor, is_log, inverse)
    """psi_light = calculate_fechner_intensity(visual_descriptors['light'], 100.0, 2.0, False, True)
    psi_contrast = calculate_fechner_intensity(visual_descriptors['contrast'], 30.0, 3.0, False, True)
    psi_blur = calculate_fechner_intensity(visual_descriptors['speed'], 0.05, 5.0, False, False)

    psi_voice = calculate_fechner_intensity(audio_descriptors['volume'], -20.0, 0.1, True, True)
    psi_noise = calculate_fechner_intensity(audio_descriptors['noise'], -50.0, 0.2, True, False)
"""
    try:
        epsilon_num = np.finfo(float).eps
    except:
        epsilon_num = 0.00001
    # 2. Combine into an overall Modality Intensity Score
    #intensity_visual = (psi_light * psi_contrast) / (epsilon_num + psi_blur)
    #intensity_audio = psi_voice / (epsilon_num + psi_noise)
    intensity_visual = (psi_light * psi_contrast) *  psi_blur
    intensity_audio = psi_voice *psi_noise
    # 3. Normalize into Bayesian-style Weights
    total_intensity = intensity_visual + intensity_audio

    # Avoid division by zero if both intensities are 0
    if total_intensity <= 0:
        return {"visual_content": 0.0, "audio_content": 0.0}
    return {
        "visual_content": intensity_visual ,
        "audio_content": intensity_audio  
    }
    return {
        "visual_content": intensity_visual / total_intensity,
        "audio_content": intensity_audio / total_intensity
    }
    
def calculate_level(current_value: float, min_value: float, max_value: float) -> int:
    """
    Scales a value from a given range [min_value, max_value] to an integer level from 1 to 10.
    """
    # 1. Validate that the min/max range is valid
    if min_value >= max_value:
        raise ValueError("Error: 'max_value' must be strictly greater than 'min_value'.")
    # 2. Handle edge cases

    if current_value <= min_value: return 1
    if current_value >= max_value: return 10
    # 3. Calculate interval
    total_range = max_value - min_value
    level_interval = total_range / 10.0
    # 4. Determine level
    level = floor((current_value - min_value) / level_interval) + 1
    return int(level)


def scale_reliability_weights(data):
    # Find the highest reliability_weight in the dictionary
    max_weight = max(item['reliability_weight'] for item in data.values())

    # Check to prevent division by zero in case all weights are 0
    if max_weight > 0:
        for item in data.values():
            # Divide each weight by the max_weight to scale it
            item['reliability_weight'] /= max_weight

    return data

def is_float(input_number):
    try:
        k = float(input_number)
        return True
    except:
        return False
# -----------------------------------------------------------------
# INIT WORLDMODEL (MOCK) CLASS
# -----------------------------------------------------------------



class UnifiedWorldModel:
    def __init__(self, context_file="world_context.json", base_context_file="base_context.txt", history_len=20):
        self.context_file = context_file
        self.base_context_file = base_context_file
        self.dynamic_history = deque(maxlen=history_len)
        self.file_path = get_package_share_directory('human_robot_interaction')
        self.context_file_path = os.path.join(self.file_path,self.context_file)
        self.base_context_file_path = os.path.join(self.file_path,"context_config",self.base_context_file)
        # --- NEW: Load Base Context from Text File ---
        self.static_context = self._load_base_context()

        self.state = self._load_state()
        
        
        self.cue_library = self.state.get("cue_library", {})
        self.facts = self.state.get("verified_facts", {})
        self.objects = self.state.get("spatial_map", {})
        self.current_context = self.state.get("summary", "Initializing...")
        self.context_reliability = self.state.get("reliability", 0.1)
        #self._save_state()
    def _load_base_context(self):
        """
        Loads the static system persona/rules from a text file.
        If the file doesn't exist, it falls back to a default string.
        """
        default_context = """
        SYSTEM PROFILE:
        - Role: Cognitive Perception Engine.
        - Environment: Generic Indoor Setting.
        - Directives: Prioritize human safety. Interpret sensory data conservatively.
        """

        if os.path.exists(self.base_context_file_path):
            try:
                with open(self.base_context_file_path, 'r', encoding='utf-8') as f:
                    content = f.read().strip()
                    if content:
                        print(f"[WorldModel] Loaded Base Context from {self.base_context_file_path}")
                        return content
            except Exception as e:
                print(f"[WorldModel] Error reading base context file: {e}")

        print("[WorldModel] Using Default Base Context.")
        return default_context

    def _load_state(self):
        if os.path.exists(self.context_file_path):
            try:
                with open(self.context_file_path, 'r') as f:
                    return json.load(f)
            except:
                pass
        return {"spatial_map": {}, "verified_facts": {}, "cue_library": {}, "summary": "", "reliability": 0.1}

    def _save_state(self):
        #self.state["spatial_map"] = self.objects
        self.state["verified_facts"] = self.facts
        self.state["static_context"] = self.static_context
        self.state["cue_library"] = self.cue_library  # Save cue interpretations
        self.state["summary"] = self.current_context
        self.state["reliability"] = self.context_reliability

        with open(self.context_file_path, 'w') as f:
            json.dump(self.state, f, indent=4)

    # --- CUE INTERPRETATION MEMORY ---
    def update_cue_interpretation(self, raw_label, meaning, confidence):
        """
        Updates the meaning of a specific raw signal (e.g. 'wave_hand').
        Uses weighted average to refine understanding over time.
        """
        if not raw_label or raw_label == "None": return

        current_time = time.time()

        if raw_label in self.cue_library:
            # REINFORCE EXISTING MEANING
            prev = self.cue_library[raw_label]

            # If the meaning is roughly the same, boost confidence
            # (In a real system, you'd use semantic similarity check here)
            # For now, we assume LLM keeps wording consistent or we overwrite contextually

            # Weighted Update: 70% History + 30% New Observation
            new_conf = (prev['confidence'] * 0.7) + (confidence * 0.3)

            self.cue_library[raw_label] = {
                'meaning': meaning,  # Update to latest nuance
                'confidence': min(1.0, new_conf),
                'last_seen': current_time
            }
        else:
            # LEARN NEW CUE MEANING
            self.cue_library[raw_label] = {
                'meaning': meaning,
                'confidence': confidence,
                'last_seen': current_time
            }

    # --- (Previous Spatial/History Methods) ---
    def update_spatial_entity(self, label, x, y):
        current_time = time.time()
        matched_id = None
        for obj_id, data in self.objects.items():
            dist = math.sqrt((data['x'] - x) ** 2 + (data['y'] - y) ** 2)
            if dist < 0.5 and data['label'] == label:
                matched_id = obj_id;
                break
        if matched_id:
            self.objects[matched_id].update({'x': x, 'y': y, 'last_seen': current_time})
            self.objects[matched_id]['confidence'] = min(1.0, self.objects[matched_id]['confidence'] + 0.1)
        else:
            new_id = f"{label}_{int(current_time * 100)}"
            self.objects[new_id] = {'label': label, 'x': x, 'y': y, 'last_seen': current_time, 'confidence': 0.5}

    def prune_stale_data(self):
        current_time = time.time()

        # 1. Prune Spatial Objects (Fast Decay)
        to_delete_obj = []
        for obj_id, data in self.objects.items():
            age = current_time - data['last_seen']
            time_limit = 10.0 + (data['confidence'] * 300.0)
            if age > time_limit:
                if data['confidence'] > 0.2:
                    data['confidence'] -= 0.1
                    data['last_seen'] = current_time
                else:
                    to_delete_obj.append(obj_id)
        for obj_id in to_delete_obj: del self.objects[obj_id]

        # 2. Prune Facts (Slow Decay)
        # Facts decay very slowly unless reinforced
        to_delete_facts = []
        for fact_text, meta in self.facts.items():
            age = current_time - meta['last_verified']
            # Strong facts last for hours (e.g. 3600s), weak ones for minutes
            retention_period = meta['reliability'] * 3600.0

            if age > retention_period:
                # Decay reliability
                meta['reliability'] -= 0.05
                meta['last_verified'] = current_time  # Reset clock for gradual fade

                if meta['reliability'] < 0.2:
                    print(f"[Memory] Forgetting Fact: '{fact_text}'")
                    to_delete_facts.append(fact_text)

        for f in to_delete_facts: del self.facts[f]

    def update_history(self, data_packet):
        if isinstance(data_packet, dict):
            entry = {
                't': time.strftime("%H:%M:%S", time.localtime()),
                'vis': data_packet.get('visual_content', {}).get('data cue'),
                'aud': data_packet.get('audio_content', {}).get('data cue')
            }
            self.dynamic_history.append(entry)

    def commit_long_term_memory(self, new_summary, new_facts_list, context_rel):
        self.current_context = new_summary
        self.context_reliability = context_rel
        current_time = time.time()
        for item in new_facts_list:
            fact_str = item.get('fact')
            incoming_conf = item.get('confidence', 0.5)
            if fact_str in self.facts:
                old_rel = self.facts[fact_str]['reliability']
                new_rel = (old_rel * 0.8) + (incoming_conf * 0.2)
                self.facts[fact_str]['reliability'] = min(1.0, new_rel)
                self.facts[fact_str]['last_verified'] = current_time
            else:
                self.facts[fact_str] = {'reliability': incoming_conf, 'last_verified': current_time}
        self._save_state()

    def get_full_context_str(self, current_vis_cue, current_aud_cue):
        """
        Includes the static base context in the prompt payload.
        """
        vis_prior = self.cue_library.get(current_vis_cue, {})
        aud_prior = self.cue_library.get(current_aud_cue, {})

        cue_context = f"""
        [Visual Prior]: '{current_vis_cue}' usually means "{vis_prior.get('meaning', 'Unknown')}" (Conf: {vis_prior.get('confidence', 0):.2f})
        [Audio Prior]: '{current_aud_cue}' usually means "{aud_prior.get('meaning', 'Unknown')}" (Conf: {aud_prior.get('confidence', 0):.2f})
        """

        sorted_facts = sorted(self.facts.items(), key=lambda x: x[1]['reliability'], reverse=True)
        facts_str = "; ".join([f"'{k}' ({v['reliability']:.2f})" for k, v in sorted_facts])
        spatial_desc = [f"{d['label']} ({d['confidence']:.1f})" for d in self.objects.values()]
        spatial_str = ", ".join(spatial_desc) if spatial_desc else "Empty"

        return {
            "summary": self.current_context,
            "facts": facts_str,
            "spatial": spatial_str,
            "history": json.dumps(list(self.dynamic_history)),
            "reliability": self.context_reliability,
            "cue_priors": cue_context,
            "static_base": self.static_context  # <--- Included here
        }
        

class FusionEngineNode(Node):
    def __init__(self):
        super().__init__('fusion_engine_node')
        
        self.create_subscription(String, '/cognition/fused/events', self.fused_callback, 10)
        self.create_subscription(String, '/cognition/visual/descriptors', self.vis_desc_callback, 10)
        self.create_subscription(String, '/cognition/audio/descriptors', self.aud_desc_callback, 10)
        
        """self.intensity_noise_pub = self.create_publisher(Float32, '/cognition/audio/intensity_noise_pub', 10)
        self.intensity_volume_pub = self.create_publisher(Float32, '/cognition/audio/intensity_volume_pub', 10)
        self.intensity_light_pub = self.create_publisher(Float32, '/cognition/visual/intensity_light_pub', 10)
        self.intensity_speed_pub = self.create_publisher(Float32, '/cognition/visual/intensity_speed_pub', 10)
        self.intensity_contrast_pub = self.create_publisher(Float32, '/cognition/visual/intensity_contrast_pub', 10)"""
        
     

        #self.visual_pub = self.create_publisher(Float32, '/cognition/visual_weight', 10)
        #self.audio_pub = self.create_publisher(Float32, '/cognition/audio_weight', 10)
        
        self.world_model = UnifiedWorldModel()
        genai.configure(api_key="AIzaSyBVRKMCU_eBUWhvVDdyb6ntFAsvRiWPz3M",transport="rest") #uh, this is my own api key
        self.get_logger().info(str(self.world_model.static_context))
        
        self.llm_model = genai.GenerativeModel('gemini-2.5-flash')
        #self.world_model = UnifiedWorldModel()
        #self.get_logger().info(self.world_model.static_context)
        #self.get_logger().info(os.getcwd())
        #self.get_logger().info(self.world_model.file_path)
        
        
        # Plotting Setup
        #plt.ion()
        #self.fig, self.ax = plt.subplots()
        #self.line, = self.ax.plot([], []) # Setup your complex plots here
        
        # Render plot periodically (Matplotlib inside ROS needs this)
        #self.create_timer(0.1, self.update_plot)
        
        self.latest_vis_desc = {}
        self.latest_aud_desc = {}
        
        self.visual_descriptors = {"visual_light","visual_contrast","visual_speed"}
        self.audio_descriptors = {"audio_noise","audio_volume","audio_pitch"}
        self._build_dynamic_data_input()
        self.raw_data ={}

        self.declare_parameter('base_S_light', 10.0)
        self.declare_parameter('k_light', 5.0)
        self.declare_parameter('base_S_contrast', 30.0)
        self.declare_parameter('k_contrast', 5.0)
        self.declare_parameter('base_S_speed', 0.2)
        self.declare_parameter('k_speed', 5.0)
        
        self.declare_parameter('base_S_noise', -10.0)
        self.declare_parameter('k_noise', 0.2)
        self.declare_parameter('base_S_volume', -30.0)
        self.declare_parameter('k_volume', 0.1)
        
        self.base_S_light = self.get_parameter('base_S_light').get_parameter_value().double_value
        self.k_light = self.get_parameter('k_light').get_parameter_value().double_value
        self.base_S_contrast = self.get_parameter('base_S_contrast').get_parameter_value().double_value
        self.k_contrast = self.get_parameter('k_contrast').get_parameter_value().double_value
        self.base_S_speed = self.get_parameter('base_S_speed').get_parameter_value().double_value
        self.k_speed = self.get_parameter('k_speed').get_parameter_value().double_value
        
        self.base_S_noise = self.get_parameter('base_S_noise').get_parameter_value().double_value
        self.k_noise = self.get_parameter('k_noise').get_parameter_value().double_value
        self.base_S_volume = self.get_parameter('base_S_volume').get_parameter_value().double_value
        self.k_volume = self.get_parameter('k_volume').get_parameter_value().double_value
        self.get_logger().error(f"param test: {self.base_S_light}")
   
        
    def vis_desc_callback(self, msg):
        self.latest_vis_desc = json.loads(msg.data)
        
    def aud_desc_callback(self, msg):
        self.latest_aud_desc = json.loads(msg.data)

    def update_plot(self):
        #self.latest_aud_desc = 
        #self.latest_vis_desc = 
        plt.pause(0.01)
        
    def _build_dynamic_data_input(self):
        print("Building dynamic data input...")
        self.base_data_input = {
            "audio_content": {"reliability": 1, "content": {}, "modality": "audio", "type": "content"},
            "visual_content": {"reliability": 1, "content": {}, "modality": "visual", "action": "gesture",
                               "type": "content"},
        }
        for descriptor in self.audio_descriptors:
            class_name = f"{descriptor}"
            self.base_data_input[class_name] = {"magnitude": 1, "source": "audio_content", "modality": "audio",
                                                "class": class_name, "type": "descriptor"}
        for descriptor in self.visual_descriptors:
            class_name = f"{descriptor}"
            self.base_data_input[class_name] = {"magnitude": 1, "modality": "visual", "source": "visual_content",
                                                "class": class_name, "type": "descriptor"}

        self.base_preference = {"audio": 0.5, "visual": 0.5, "haptic": 0.2}
    
    def _cal_weight(self, descriptor_variances):
        # 1. Base intrinsic variance of your sensors in perfect conditions
        # (You can tune these based on empirical calibration. Lower = more reliable baseline)
        variance_audio = 1.0  # Whisper's baseline uncertainty
        variance_visual = 1.5 # MediaPipe's baseline uncertainty

        # 2. Add extrinsic variances caused by the environment
        for desc, var_penalty in descriptor_variances.items():
            if "audio" in desc:
                variance_audio += var_penalty
            elif "visual" in desc:
                variance_visual += var_penalty

        # 3. Calculate Precision (Inverse Variance)
        precision_audio = 1.0 / variance_audio
        precision_visual = 1.0 / variance_visual

        # 4. Normalize weights (Maximum Likelihood Estimation)
        total_precision = precision_audio + precision_visual

        return {
            "audio_content": precision_audio / total_precision,
            "visual_content": precision_visual / total_precision
        }

    def _graft_data(self, latest_avg, scaling_rules, scaled_array_list):
        descriptor_variances = {}
        for key, rules in scaling_rules.items():
            if key in latest_avg:
                avg_val = latest_avg[key]
                # Unpack the Fechner calibration rules
                opt_thresh, k_factor, is_log, inverse = rules

                # Calculate the exact variance penalty
                var_penalty = calculate_fechner_intensity(avg_val, opt_thresh, k_factor, is_log, inverse)
                descriptor_variances[key] = var_penalty

                # Update plotting array (plotting variance penalty instead of 1-10 magnitude)
                scaled_array_list[key].append(var_penalty)
                scaled_array_list[key] = scaled_array_list[key][-20:]
            else:
                descriptor_variances[key] = 0.0

        return descriptor_variances, scaled_array_list
    
    
    def _gemini_active_inference(self, model, sensory_data, full_state):
        # Extract the cues
        vis_cue = sensory_data.get('visual_content', {}).get('data cue', 'None')
        aud_cue = sensory_data.get('audio_content', {}).get('data cue', 'None')

        # Extract the psychophysical intensities
        vis_intensities = sensory_data.get('visual_content', {}).get('intensities', {})
        aud_intensities = sensory_data.get('audio_content', {}).get('intensities', {})

        # Format the intensities into a readable string for the LLM
        vis_env_str = json.dumps({k: round(v, 2) for k, v in vis_intensities.items() if 'visual' in k})
        aud_env_str = json.dumps({k: round(v, 2) for k, v in aud_intensities.items() if 'audio' in k})

        prompt = f"""
                ACT AS: Cognitive Engine.

                --- 0. SYSTEM DIRECTIVES (STATIC) ---
                {full_state['static_base']}

                --- 1. PRIOR KNOWLEDGE ---
                Context: "{full_state['summary']}" (Rel: {full_state['reliability']:.2f})
                Facts: {full_state['facts']}
                Sensor History: {full_state['cue_priors']} 

                --- 2. LIVE PERCEPTION ---
                Spatial Map: {full_state['spatial']}

                [SENSORY INPUTS]
                Visual Cue: "{vis_cue}"
                Visual Interference Intensities: {vis_env_str}

                Audio Cue: "{aud_cue}"
                Audio Interference Intensities: {aud_env_str}

                --- TASK ---
                Evaluate the live perception inputs. The "Interference Intensities" represent calculated degradation penalties using the Weber-Fechner law. Values > 0.0 indicate increasing levels of environmental intensity (e.g., high motion blur, high noise, low light). Beneficial descriptors: visual_light, visual_contrast, audio_volume. Negative descriptors: visual_speed, audio_noise. A value of 0.0 for negative descriptors means perfect conditions.

                A. INTERPRET CUES: What does each specific signal mean in this context? 
                B. UPDATE: Extract new facts.
                C. INTEGRATE: Combine the cues into a unified situation summary. If the visual and audio cues contradict each other, you MUST evaluate the Interference Intensities. Trust the modality with the lowest interference values. If a modality has high interference (high 'audio_noise' or 'visual_speed' blur), discount it as unreliable. If there is only one source of data, use that source of data
                D. MATCH SCORE: Check if the data is relevant to the context, assign match_score from (0.0-1.0).

                Output JSON:
                {{
                    "cue_analysis": {{
                        "visual": {{"meaning": "Interpretation of visual cue", "confidence": 0.0-1.0}},
                        "audio": {{"meaning": "Interpretation of audio cue", "confidence": 0.0-1.0}}
                    }},
                    "current_situation": "Integrated summary of reality, noting which cue was trusted if a conflict occurred.",
                    "new_facts": [
                        {{"fact": "User is sitting", "confidence": 0.9}}
                    ],
                    "match_score": 0.8
                }}
                """
        try:
            #self.get_logger().info(str(prompt))
            response = model.generate_content(prompt)
            return json.loads(response.text.replace("```json", "").replace("```", ""))
        except Exception as e:
            self.get_logger().info(str(f"[LLM Error] {e}"))
            return {"match_score": 0.5, "new_facts": [], "cue_analysis": {}}
        
        
    def fused_callback(self, msg):
        data_packet = json.loads(msg.data)
        self.get_logger().info("Received cognitive event for LLM processing.")
        
        # Offload this to a separate thread if LLM calls take > 1 second
        # so it doesn't block ROS topic ingestion.
        self.process_cognition(data_packet)

    def process_cognition(self, data_packet):
        descriptor_data ={}
        packet_to_send = {}
        
        """latest_averages = {
                    "audio_volume": avg_sound_vol, "audio_noise": avg_noise_vol, "audio_pitch": avg_pitch,
                    "visual_light": avg_light, "visual_speed": avg_speed, "visual_contrast": avg_contrast,
                }"""
        scaling_rules = {
                    # dB is log_scale=True. Noise above -50dB reduce intensity. Inverse=true.
                    "audio_noise": (self.base_S_noise, self.k_noise, True, True),

                    # Audio Volume (Surprise/Distortion). Drops below -20dB reduce intensity. Inverse=false.
                    "audio_volume": (self.base_S_volume, self.k_volume, True, False),

                    #"audio_pitch": (8000.0, 0.0, False, False),  # Ignoring pitch for variance currently

                    # Light: linear. Drops below 180 reduce intensity. Inverse=false.
                    "visual_light": (self.base_S_light, self.k_light, False, False),

                    # Speed: linear. Speeds above 0.2 reduce intensity (motion blur). Inverse=true.
                    "visual_speed": ( self.base_S_speed, self.k_speed, False, True),

                    # Contrast: linear. Drops below 30 reduce intensity. Inverse=false.
                    "visual_contrast": (self.base_S_contrast, self.k_contrast, False, False),
                    
                    #more can be added, this can later be split and put into descriptor calculation function, and loaded dynamically
                }
        
        scaled_array_list_interaction_only = {
            "audio_volume": [1], "audio_noise": [1], "audio_pitch": [1],
            "visual_light": [1], "visual_speed": [1], "visual_contrast": [1],
        }
        
        
        for modality in data_packet:
            try:
                for descriptor in data_packet[modality]["descriptor_data"]:
                    descriptor_data[descriptor] = data_packet[modality]["descriptor_data"][descriptor]
            except:
                pass
            
        
        """self.base_data_input_immediate = self.base_data_input.copy()
        self.base_data_input_immediate,scaled_array_list_interaction_only = self._graft_data(self.base_data_input_immediate,
                                                            descriptor_data, scaling_rules,scaled_array_list_interaction_only)"""
        
        self.base_data_input_immediate = self.base_data_input.copy()
        descriptor_intensities, scaled_array_list_interaction_only = self._graft_data(
                            descriptor_data, scaling_rules, scaled_array_list_interaction_only
                        )
        
        #self.get_logger().info(str(self.base_data_input))
        #current_pref_dict = self._cal_weight(self.base_data_input)
        
        
        if data_packet["status"] == "related":
            # We iterate over the weights to ensure we grab the latest reliability scores
            for modality in ["visual_content", "audio_content"]:
                if data_packet.get(modality):
                    packet_to_send[modality] = {
                        "data cue": data_packet[modality]["data"],
                        # Pass the full dictionary of calculated intensities
                        "intensities": descriptor_intensities,
                        "name": modality
                    }

        # CASE B: UNRELATED (Single Modality)
        # We package ONLY the available modality, but we still SEND it.
        elif data_packet["status"] == "unrelated":
            # Check which modality triggered this event
            if data_packet.get("visual_content"):
                packet_to_send["visual_content"] = {
                                    "data cue": data_packet["visual_content"]["data"],
                                    "intensities": descriptor_intensities,
                                    "name": "visual_content"
                                }
                # Explicitly mark the other as None so the LLM knows it's missing
                packet_to_send["audio_content"] = {"data cue": "None", "reliability_weight": 0.0}

            elif data_packet.get("audio_content"):
                packet_to_send["audio_content"] = {
                                    "data cue": data_packet["audio_content"]["data"],
                                    "intensities": descriptor_intensities,
                                    "name": "audio_content"
                                }
                # Explicitly mark the other as None
                packet_to_send["visual_content"] = {"data cue": "None", "reliability_weight": 0.0}
        
        
        self.get_logger().info(f"{packet_to_send}")
        vis_cue = packet_to_send.get('visual_content', {}).get('data', 'None')
        aud_cue = packet_to_send.get('audio_content', {}).get('data', 'None')
        
        """if vis_cue and "static" not in vis_cue and vis_cue != "None":
            self.world_model.update_spatial_entity(vis_cue, 0.0, 1.5)"""

        self.world_model.prune_stale_data()
        self.world_model.update_history(packet_to_send)

        # 2. Cognition
        self.get_logger().info("\n[Cognition] Processing...")
        full_state = self.world_model.get_full_context_str(vis_cue, aud_cue)
        self.get_logger().info(str(packet_to_send))
        result = self._gemini_active_inference(self.llm_model, packet_to_send, full_state)
        #self.get_logger().info(f"{result}")
        cue_analysis = result.get('cue_analysis', {})

        if vis_cue and vis_cue != "None":
            v_data = cue_analysis.get('visual', {})
            self.world_model.update_cue_interpretation(vis_cue, v_data.get('meaning', ''),
                                                    v_data.get('confidence', 0.5))
            print(f"    + Visual Meaning: '{v_data.get('meaning')}' (Conf: {v_data.get('confidence'):.2f})")

        if aud_cue and aud_cue != "None":
            a_data = cue_analysis.get('audio', {})
            self.world_model.update_cue_interpretation(aud_cue, a_data.get('meaning', ''),
                                                    a_data.get('confidence', 0.5))
            print(f"    + Audio Meaning: '{a_data.get('meaning')}' (Conf: {a_data.get('confidence'):.2f})")

        # 4. Commit General Context
        new_rel = min(1.0, full_state['reliability'] + 0.05) if result.get('match_score') > 0.6 else \
        full_state['reliability']
        self.world_model.commit_long_term_memory(
            new_summary=result.get('current_situation', full_state['summary']),
            new_facts_list=result.get('new_facts', []),
            context_rel=new_rel
        )
        
        self.get_logger().info(f" > SUMMARY: {result.get('current_situation')}")
        self.get_logger().info("-" * 40)

        self.is_thinking = False
        self.get_logger().info(f"Processed: Vis({vis_cue}), Aud({aud_cue})")

def main(args=None):
    rclpy.init(args=args)
    node = FusionEngineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()