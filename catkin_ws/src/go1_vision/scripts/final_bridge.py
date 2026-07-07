#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from unitree_legged_msgs.msg import MotorCmd

class FinalBridge:
    def __init__(self):
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        # Publicamos a los 4 motores de las caderas (hip)
        self.pub_fl = rospy.Publisher('/go1_gazebo/FL_hip_controller/command', MotorCmd, queue_size=10)
        self.pub_fr = rospy.Publisher('/go1_gazebo/FR_hip_controller/command', MotorCmd, queue_size=10)
        self.pub_rl = rospy.Publisher('/go1_gazebo/RL_hip_controller/command', MotorCmd, queue_size=10)
        self.pub_rr = rospy.Publisher('/go1_gazebo/RR_hip_controller/command', MotorCmd, queue_size=10)
        rospy.loginfo("Bridge de emergencia ACTIVO. Inyectando a motores...")

    def callback(self, msg):
        m = MotorCmd()
        m.mode = 1
        m.Kp = 10.0 # Fuerza de movimiento
        m.Kd = 1.0
        
        # Hemos deshabilitado usar motores de cadera para girar, porque hacen strafe (pasos laterales)
        # en lugar de pivotar. Para pivotar, usa el control nativo cmd_vel (modo 2) del robot.
        m.q = 0.0
        
        self.pub_fl.publish(m)
        self.pub_fr.publish(m)
        self.pub_rl.publish(m)
        self.pub_rr.publish(m)

if __name__ == '__main__':
    rospy.init_node('emergency_bridge')
    FinalBridge()
    rospy.spin()
