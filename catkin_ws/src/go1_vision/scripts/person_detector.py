#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from go1_vision.srv import FindPerson, FindPersonResponse
from cv_bridge import CvBridge
from ultralytics import YOLO

class PersonDetectorNode:
    def __init__(self):
        self.bridge = CvBridge()
        
        # Load the YOLOv8 segmentation model
        self.model = YOLO("yolov8x-seg.pt")
        self.person_class_id = 0  
        
        # Initialize ROS publishers and subscribers
        self.pose_pub = rospy.Publisher('/person_pose', PoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber('/camera_face/color/image_raw', Image, self.image_callback)
        self.srv = rospy.Service('/find_person', FindPerson, self.handle_find_person)
        
        # Tracking states
        self.last_pose = None
        self.last_pose_time = None  
        self.follow_active = False
        
        # Finite State Machine (FSM) variables
        self.state = 'IDLE'          
        self.state_start_time = None
        self.close_proximity_time = None 
        
        # === REACTIVE EVASION VARIABLES (IEEE Paper) ===
        self.is_threatened = False
        self.repulsive_w = 0.0
        
        # === NAVIGATION PARAMETERS ===
        self.eval_duration = 1.5      
        self.deadzone = 0.12          
        
        # SAFE ALIGNMENT PHYSICS
        self.align_gain = 1.0         
        self.max_body_align_vel = 0.5  
        self.min_ang_vel = 0.3
        
        # DISTANCE AND PROXIMITY THRESHOLDS
        self.stop_width_threshold = 0.25  
        self.blind_spot_width = 0.22       
        self.max_width_seen = 0.0         
        
        # STABLE KINEMATICS
        self.linear_speed = 0.12       
        self.micro_step = 0.03  
        
        # TACTICAL SEARCH PATTERN VARIABLES
        self.sweep_stage = 0            
        self.global_turns = 0          
        self.max_global_turns = 4      
        self.neck_spin_vel = 0.8       
        self.neck_spin_duration = 30.0  
        self.global_turn_done = False
        
        # High-frequency control loop (20 Hz)
        self.control_timer = rospy.Timer(rospy.Duration(1.0/20.0), self.control_callback)

        rospy.loginfo("==========================================================")
        rospy.loginfo("GO1 VISION: YOLOv8 (PATIENCE FSM AND REPULSIVE VECTOR)")
        rospy.loginfo("==========================================================")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # Adjust image orientation based on the camera mount
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_180) 
            
            # Lower confidence threshold to improve tracking robustness against partial occlusions
            results = self.model(cv_image, conf=0.25, verbose=False)
            
            valid_pose = False
            new_x = 0.0
            new_z = 0.0
            
            max_person_width = 0.0
            threat_detected = False
            rep_ang = 0.0
            
            # Iterate through all detected bounding boxes
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                xyxyn = box.xyxyn[0].tolist()
                xmin, ymin, xmax, ymax = xyxyn
                
                # --- TRACKING LOGIC (PRIMARY TARGET) ---
                if cls_id == self.person_class_id:
                    current_width = abs(xmax - xmin)
                    # Prioritize the target with the largest pixel area (closest to the camera)
                    if current_width > max_person_width:
                        max_person_width = current_width
                        new_x = ((xmin + xmax) / 2.0) - 0.5  
                        new_z = current_width             
                        valid_pose = True
                
                # --- REACTIVE EVASION LOGIC (OBSTACLES) ---
                else:
                    area = abs(xmax - xmin) * abs(ymax - ymin)
                    # Condition: Obstacle area exceeds 15% AND extends into the lower 40% of the visual field (ymax > 0.60)
                    if area > 0.15 and ymax > 0.60:
                        threat_detected = True
                        obst_x = ((xmin + xmax) / 2.0) - 0.5
                        
                        # Apply a repulsive angular velocity directly proportional, but opposite in sign, to the obstacle offset
                        k_repulsive = 2.0 
                        rep_ang = -k_repulsive * obst_x

            # Update threat state for the high-frequency control loop
            self.is_threatened = threat_detected
            self.repulsive_w = rep_ang
            
            if valid_pose:
                p_msg = PoseStamped()
                p_msg.pose.position.x = new_x
                p_msg.pose.position.z = new_z
                
                self.last_pose = p_msg
                self.last_pose_time = rospy.Time.now()
                self.pose_pub.publish(p_msg)
                
            else:
                if self.last_pose is not None and self.last_pose_time is not None:
                    # Patience FSM: Maintain pursuit vector for 2.5 seconds before declaring target loss
                    if (rospy.Time.now() - self.last_pose_time).to_sec() > 2.5:
                        self.last_pose = None
                        
        except Exception as e:
            rospy.logerr(f"Error in YOLO vision pipeline: {e}")

    def change_state(self, new_state):
        # Reset search parameters upon successful reacquisition
        if new_state == 'APPROACHING':
            self.close_proximity_time = None 
            self.sweep_stage = 0 
            self.global_turns = 0
            
        self.state = new_state
        self.state_start_time = rospy.Time.now()
        rospy.loginfo(f"--- FSM STATE TRANSITION: {new_state} ---")

    def control_callback(self, event):
        if not self.follow_active:
            # Idle animation handling when mission is complete
            if self.state == 'DONE':
                elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
                cycle = elapsed % 3.0
                cmd_ang = 0.15 if cycle < 0.2 else (-0.15 if 1.5 <= cycle < 1.7 else 0.0)
                twist = Twist()
                twist.angular.z = cmd_ang
                try: self.vel_pub.publish(twist)
                except: pass
            return

        cmd_lin = 0.0
        cmd_ang = 0.0
        current_time = rospy.Time.now()

        # === STATE: EVALUATING ===
        if self.state == 'EVALUATING':
            elapsed = (current_time - self.state_start_time).to_sec()
            if elapsed < self.eval_duration:
                rospy.loginfo_throttle(1.0, "Analyzing semantic environment...")
            else:
                if self.last_pose:
                    offset_x = self.last_pose.pose.position.x
                    if abs(offset_x) <= self.deadzone:
                        self.change_state('APPROACHING')
                    else:
                        self.change_state('ALIGNING')
                else:
                    # Target lost; trigger tactical search pattern
                    if self.sweep_stage == 0:
                        self.sweep_stage = 1
                        self.change_state('SPINNING_LEFT')
                    elif self.sweep_stage == 1:
                        self.sweep_stage = 2
                        self.change_state('SPINNING_RIGHT')
                    elif self.sweep_stage == 2:
                        self.sweep_stage = 3
                        self.change_state('GLOBAL_TURN')

        # === STATE: LOCAL SWEEP (LEFT) ===
        elif self.state == 'SPINNING_LEFT':
            elapsed = (current_time - self.state_start_time).to_sec()
            if self.global_turn_done:
                self.neck_spin_duration = 20.0
            
            if elapsed < self.neck_spin_duration:
                cmd_ang = self.neck_spin_vel
            else:
                self.change_state('EVALUATING')

        # === STATE: LOCAL SWEEP (RIGHT) ===
        elif self.state == 'SPINNING_RIGHT':
            elapsed = (current_time - self.state_start_time).to_sec()
            if elapsed < (self.neck_spin_duration * 1.75):
                cmd_ang = -self.neck_spin_vel
            else:
                self.change_state('EVALUATING')

        # === STATE: GLOBAL U-TURN (Spatial Grid Search) ===
        elif self.state == 'GLOBAL_TURN':
            elapsed = (current_time - self.state_start_time).to_sec()
            
            if elapsed < 30.0:
                cycle = elapsed % 3.0  
                fake_ang_vel = 0.6 
                
                if cycle < 1.0:
                    cmd_lin = self.micro_step 
                    cmd_ang = fake_ang_vel
                elif cycle < 1.25:
                    cmd_lin = 0.0
                    cmd_ang = 0.0
                elif cycle < 2.75:
                    cmd_lin = 0.15  
                    cmd_ang = 0.0
                else:
                    cmd_lin = 0.0
                    cmd_ang = 0.0
            
            elif elapsed < 50.0:
                sub_elapsed = elapsed - 30.0
                if sub_elapsed < 19.5:
                    cmd_lin = self.micro_step  
                    cmd_ang = -0.6
                else:
                    cmd_lin = 0.0
                    cmd_ang = 0.0

            elif elapsed < 65.0:
                cycle = (elapsed - 50.0) % 3.0  
                fake_ang_vel = 0.6 
                
                if cycle < 1.0:
                    cmd_lin = self.micro_step 
                    cmd_ang = fake_ang_vel
                elif cycle < 1.25:
                    cmd_lin = 0.0
                    cmd_ang = 0.0
                elif cycle < 2.75:
                    cmd_lin = 0.15  
                    cmd_ang = 0.0
                else:
                    cmd_lin = 0.0
                    cmd_ang = 0.0

            elif elapsed < 85.0:
                sub_elapsed = elapsed - 65.0
                if sub_elapsed < 19.5:
                    cmd_lin = self.micro_step  
                    cmd_ang = -0.6
                else:
                    cmd_lin = 0.0
                    cmd_ang = 0.0

            elif elapsed < 100.0:
                cycle = (elapsed - 85.0) % 3.0  
                fake_ang_vel = 0.6 
                
                if cycle < 1.0:
                    cmd_lin = self.micro_step 
                    cmd_ang = fake_ang_vel
                elif cycle < 1.25:
                    cmd_lin = 0.0
                    cmd_ang = 0.0
                elif cycle < 2.75:
                    cmd_lin = 0.15  
                    cmd_ang = 0.0
                else:
                    cmd_lin = 0.0
                    cmd_ang = 0.0
            else:
                self.global_turns += 1
                self.global_turn_done = True
                if self.global_turns >= self.max_global_turns:
                    rospy.loginfo("Tactical exploration completed. Target unrecoverable. Aborting.")
                    self.change_state('DONE')
                    self.follow_active = False
                else:
                    self.sweep_stage = 0 
                    self.change_state('STOP_SETTLE')

        # === STATE: STOPPING AND SETTLING ===
        elif self.state == 'STOP_SETTLE':
            elapsed = (current_time - self.state_start_time).to_sec()
            if elapsed >= 1.0:
                self.change_state('EVALUATING')

        # === STATE: BRAKING FOR ALIGNMENT ===
        elif self.state == 'BRAKING_FOR_ALIGN':
            elapsed = (current_time - self.state_start_time).to_sec()
            if elapsed >= 0.5: 
                self.change_state('ALIGNING')

        # === STATE: VICTORY BRAKE (Target Reached) ===
        elif self.state == 'VICTORY_BRAKE':
            elapsed = (current_time - self.state_start_time).to_sec()
            if elapsed >= 1.5: 
                self.change_state('DONE')
                self.follow_active = False

        # === STATE: ALIGNING KINEMATICS ===
        elif self.state == 'ALIGNING':
            if not self.last_pose:
                self.change_state('EVALUATING')
            else:
                offset_x = self.last_pose.pose.position.x
                if abs(offset_x) > self.deadzone:
                    elapsed = (current_time - self.state_start_time).to_sec()
                    cycle = elapsed % 2.0 
                    
                    if cycle < 1.0:
                        raw_ang = self.align_gain * offset_x 
                        cmd_ang = max(min(raw_ang, self.max_body_align_vel), -self.max_body_align_vel)
                        if abs(cmd_ang) < self.min_ang_vel:
                            cmd_ang = self.min_ang_vel if cmd_ang > 0 else -self.min_ang_vel
                        
                        cmd_lin = self.micro_step 
                        rospy.loginfo_throttle(0.5, f"Aligning base trajectory... (Yaw Error: {offset_x:.2f})")
                        
                    elif cycle < 1.25:
                        cmd_lin = 0.0
                        cmd_ang = 0.0
                    elif cycle < 1.75:
                        cmd_lin = 0.12
                        cmd_ang = 0.0
                    else:
                        cmd_lin = 0.0
                        cmd_ang = 0.0
                else:
                    self.change_state('EVALUATING')

        # === STATE: APPROACHING TARGET ===
        elif self.state == 'APPROACHING':
            box_width = 0.0
            offset_x = 0.0
            
            if self.last_pose:
                offset_x = self.last_pose.pose.position.x
                box_width = self.last_pose.pose.position.z
                if box_width > self.max_width_seen:
                    self.max_width_seen = box_width

            # Handle temporal tracking loss or critical stopping distance
            if not self.last_pose:
                if self.max_width_seen >= self.blind_spot_width:
                    self.change_state('VICTORY_BRAKE')
                    return
                else:
                    self.change_state('EVALUATING')
                    return
            else:
                if box_width >= self.stop_width_threshold:
                    self.change_state('VICTORY_BRAKE')
                    return

            # Dynamic tolerance logic based on proximity
            tolerance = self.deadzone * 2.5 if box_width > 0.18 else self.deadzone * 1.5
            
            if abs(offset_x) > tolerance:
                self.change_state('BRAKING_FOR_ALIGN')
            else:
                # ==========================================
                # OVERRIDE: PARALLEL VISUAL REPULSIVE VECTOR
                # ==========================================
                if hasattr(self, 'is_threatened') and self.is_threatened:
                    # Threat detected: Override primary trajectory, reduce linear speed, and apply repulsive yaw
                    cmd_lin = self.micro_step # Safety micro-step (0.03 m/s)
                    cmd_ang = self.repulsive_w
                    rospy.logwarn_throttle(0.5, "VISUAL THREAT DETECTED! Reactive evasive maneuver applied.")
                else:
                    # Path is clear: Continue standard approach
                    cmd_lin = self.linear_speed
                    cmd_ang = 0.0
                    rospy.loginfo_throttle(0.5, f"Approaching target... (Proximity width: {box_width:.2f})")

        # Publish final computed velocities to the low-level controller
        twist = Twist()
        twist.linear.x = cmd_lin
        twist.angular.z = cmd_ang
        try: 
            self.vel_pub.publish(twist)
        except Exception as e: 
            pass 

    def handle_find_person(self, req):
        self.follow_active = True
        self.max_width_seen = 0.0
        self.sweep_stage = 0 
        self.global_turns = 0
        self.change_state('EVALUATING')
        return FindPersonResponse(success=True, message="Robust tracking routine initiated.")

if __name__ == '__main__':
    rospy.init_node('person_detector_service')
    node = PersonDetectorNode()
    rospy.spin()
