#!/usr/bin/env python3
import rospy
import matplotlib
# ¡CRÍTICO! Matplotlib en modo Agg ANTES de importar mediapipe
matplotlib.use('Agg')
import cv2
import mediapipe as mp

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from go1_vision.srv import FindPerson, FindPersonResponse
from cv_bridge import CvBridge

class PersonDetectorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=0,
            min_detection_confidence=0.5
        )
        
        self.pose_pub = rospy.Publisher('/person_pose', PoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber('/camera_face/color/image_raw', Image, self.image_callback)
        self.srv = rospy.Service('/find_person', FindPerson, self.handle_find_person)
        
        self.last_pose = None
        self.follow_active = False
        
        self.state = 'IDLE'          
        self.state_start_time = None
        self.close_proximity_time = None 
        
        # === PARÁMETROS DE NAVEGACIÓN (Óptimos para Robot Físico) ===
        self.eval_duration = 1.0      
        self.deadzone = 0.12          
        self.align_gain = 2.0         
        self.min_ang_vel = 0.25       
        
        # --- PARADA VISUAL ---
        self.stop_width_threshold = 0.12  
        self.blind_spot_width = 0.05      
        self.max_width_seen = 0.0         
        
        # --- FÍSICAS: STOP-AND-GO (Estabiliza la cámara) ---
        self.linear_speed = 0.20       # Un poco más ágil en la realidad
        self.walk_pulse_duration = 0.5 
        self.walk_pause_duration = 0.5 
        
        # 🔥 COREOGRAFÍA DE BÚSQUEDA FLUIDA 🔥
        self.sweep_stage = 0           
        self.global_turns = 0          
        self.max_global_turns = 4      
        
        self.spin_vel = 0.5            # Velocidad de giro natural
        self.local_spin_duration = 1.5 # 1.5s a 0.5rad/s = ~45 grados
        self.global_spin_duration = 3.14 # 3.14s a 0.5rad/s = ~90 grados (1 cuadrante)
        
        self.control_timer = rospy.Timer(rospy.Duration(1.0/20.0), self.control_callback)

        rospy.loginfo("==========================================")
        rospy.loginfo("🐕 GO1 VISION: MODO HARDWARE FÍSICO ACTIVADO")
        rospy.loginfo("==========================================")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image = cv2.flip(cv_image, -1)
            
            rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            results = self.pose.process(rgb)
            
            if results.pose_landmarks:
                l_sh = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value]
                r_sh = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
                
                p_msg = PoseStamped()
                p_msg.pose.position.x = ((l_sh.x + r_sh.x) / 2.0) - 0.5
                p_msg.pose.position.z = abs(l_sh.x - r_sh.x)
                
                self.last_pose = p_msg
                self.pose_pub.publish(p_msg)
            else:
                self.last_pose = None
        except Exception as e:
            rospy.logerr(f"Error visión: {e}")

    def change_state(self, new_state):
        if new_state == 'APPROACHING':
            self.close_proximity_time = None 
            self.sweep_stage = 0 
            self.global_turns = 0
            
        self.state = new_state
        self.state_start_time = rospy.Time.now()
        rospy.loginfo(f"--- FASE: {new_state} ---")

    def control_callback(self, event):
        if not self.follow_active:
            if self.state == 'DONE':
                # Ligero meneo de victoria (opcional)
                elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
                cmd_ang = 0.15 if elapsed % 3.0 < 0.2 else 0.0
                twist = Twist()
                twist.angular.z = cmd_ang
                try: self.vel_pub.publish(twist)
                except: pass
            return

        cmd_lin = 0.0
        cmd_ang = 0.0
        current_time = rospy.Time.now()

        # === 1. MIRAR AL FRENTE ===
        if self.state == 'EVALUATING':
            elapsed = (current_time - self.state_start_time).to_sec()
            if elapsed < self.eval_duration:
                rospy.loginfo_throttle(0.5, "👀 Buscando objetivo...")
            else:
                if self.last_pose:
                    offset_x = self.last_pose.pose.position.x
                    if abs(offset_x) <= self.deadzone:
                        self.change_state('EVALUATING_ALIGN')
                    else:
                        self.change_state('ALIGNING')
                else:
                    if self.sweep_stage == 0:
                        self.sweep_stage = 1
                        self.change_state('SPINNING_LEFT')
                    elif self.sweep_stage == 1:
                        self.sweep_stage = 2
                        self.change_state('SPINNING_RIGHT')
                    elif self.sweep_stage == 2:
                        self.sweep_stage = 3
                        self.change_state('GLOBAL_TURN')

        # === 2. BARRIDO IZQUIERDO (45º y vuelta) ===
        elif self.state == 'SPINNING_LEFT':
            elapsed = (current_time - self.state_start_time).to_sec()
            if elapsed < self.local_spin_duration:
                cmd_ang = self.spin_vel
                rospy.loginfo_throttle(0.5, "📡 Radar: Izquierda...")
            else:
                self.change_state('EVALUATING')

        # === 3. BARRIDO DERECHO (45º y vuelta) ===
        elif self.state == 'SPINNING_RIGHT':
            elapsed = (current_time - self.state_start_time).to_sec()
            if elapsed < self.local_spin_duration:
                cmd_ang = -self.spin_vel
                rospy.loginfo_throttle(0.5, "📡 Radar: Derecha...")
            else:
                self.change_state('EVALUATING')

        # === 4. CAMBIO DE CUADRANTE LÍMPIDO (90º sobre sí mismo) ===
        elif self.state == 'GLOBAL_TURN':
            elapsed = (current_time - self.state_start_time).to_sec()
            if elapsed < self.global_spin_duration: 
                cmd_lin = 0.0 # El robot real gira perfecto sin avanzar
                cmd_ang = self.spin_vel
                rospy.loginfo_throttle(0.5, f"🔄 Girando cuadrante 90º ({self.global_turns + 1}/{self.max_global_turns})...")
            else:
                self.global_turns += 1
                if self.global_turns >= self.max_global_turns:
                    rospy.loginfo("❌ 360º completado sin éxito. Misión abortada.")
                    self.change_state('DONE')
                    self.follow_active = False
                else:
                    self.sweep_stage = 0 
                    self.change_state('STOP_SETTLE')

        # === 5. PAUSA DE ESTABILIZACIÓN ===
        elif self.state == 'STOP_SETTLE':
            elapsed = (current_time - self.state_start_time).to_sec()
            if elapsed < 1.0:
                cmd_lin = 0.0
                cmd_ang = 0.0
                rospy.loginfo_throttle(0.5, "🛑 Asentando chasis...")
            else:
                self.change_state('EVALUATING')

        # === 6. ALINEACIÓN CENTRADA ===
        elif self.state == 'ALIGNING':
            if not self.last_pose:
                self.change_state('EVALUATING')
            else:
                offset_x = self.last_pose.pose.position.x
                if abs(offset_x) > self.deadzone:
                    raw_ang = self.align_gain * offset_x 
                    cmd_ang = max(min(raw_ang, self.spin_vel), -self.spin_vel)
                    if abs(cmd_ang) < self.min_ang_vel:
                        cmd_ang = self.min_ang_vel if cmd_ang > 0 else -self.min_ang_vel
                    rospy.loginfo_throttle(0.5, f"🎯 Apuntando... (Error: {offset_x:.2f})")
                else:
                    self.change_state('EVALUATING_ALIGN')

        elif self.state == 'EVALUATING_ALIGN':
            elapsed = (current_time - self.state_start_time).to_sec()
            if elapsed >= self.eval_duration:
                if self.last_pose and abs(self.last_pose.pose.position.x) <= self.deadzone:
                    self.change_state('APPROACHING')
                else:
                    self.change_state('ALIGNING')

        # === 7. ACERCAMIENTO (Stop-and-Go para mejor visión) ===
        elif self.state == 'APPROACHING':
            elapsed = (current_time - self.state_start_time).to_sec()
            time_in_cycle = elapsed % (self.walk_pulse_duration + self.walk_pause_duration)
            is_stepping = time_in_cycle < self.walk_pulse_duration

            anchura_hombros = 0.0
            offset_x = 0.0
            if self.last_pose:
                offset_x = self.last_pose.pose.position.x
                anchura_hombros = self.last_pose.pose.position.z
                if anchura_hombros > self.max_width_seen:
                    self.max_width_seen = anchura_hombros

            if not self.last_pose:
                if self.max_width_seen >= self.blind_spot_width:
                    if not is_stepping:
                        rospy.loginfo(f"🙈 Punto ciego (Récord={self.max_width_seen:.2f}). ¡Atraque!")
                        self.change_state('DONE')
                        self.follow_active = False
                        self.vel_pub.publish(Twist()) 
                        return
                else:
                    if not is_stepping:
                        rospy.loginfo("⚠️ Perdido temporalmente...")
                        self.change_state('EVALUATING')
                        return
            else:
                if anchura_hombros >= self.stop_width_threshold:
                    if not is_stepping:
                        self.change_state('DONE')
                        self.follow_active = False
                        self.vel_pub.publish(Twist()) 
                        rospy.loginfo(f"🎉 ¡Misión cumplida! (Anchura={anchura_hombros:.2f})")
                        return
                
                if anchura_hombros > 0.07:
                    if self.close_proximity_time is None:
                        self.close_proximity_time = current_time
                    elif (current_time - self.close_proximity_time).to_sec() > 4.0:
                        if not is_stepping:
                            self.change_state('DONE')
                            self.follow_active = False
                            self.vel_pub.publish(Twist()) 
                            rospy.loginfo("🧱 Atasco detectado. Atraque forzado.")
                            return
                else:
                    self.close_proximity_time = None

            if is_stepping:
                if anchura_hombros > 0.07:
                    cmd_lin = self.linear_speed * 0.5
                    rospy.loginfo_throttle(0.5, f"🐾 Pasito corto... ({anchura_hombros:.2f})")
                else:
                    cmd_lin = self.linear_speed
                    rospy.loginfo_throttle(0.5, f"➡️ Acercando... ({anchura_hombros:.2f})")
            else:
                cmd_lin = 0.0
                if self.last_pose:
                    tolerancia = self.deadzone * 2.5 if anchura_hombros > 0.08 else self.deadzone * 1.5
                    if abs(offset_x) > tolerancia:
                        rospy.loginfo("⚠️ Corrigiendo rumbo...")
                        self.change_state('ALIGNING')

        twist = Twist()
        twist.linear.x = cmd_lin
        twist.angular.z = cmd_ang
        try:
            self.vel_pub.publish(twist)
        except:
            pass 

    def handle_find_person(self, req):
        self.follow_active = True
        self.max_width_seen = 0.0
        self.sweep_stage = 0 
        self.global_turns = 0
        self.change_state('EVALUATING')
        return FindPersonResponse(success=True, message="Búsqueda táctica iniciada")

if __name__ == '__main__':
    rospy.init_node('person_detector_service')
    node = PersonDetectorNode()
    rospy.spin()
