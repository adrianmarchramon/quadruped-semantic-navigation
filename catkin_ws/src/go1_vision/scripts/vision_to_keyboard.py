#!/usr/bin/env python3
import rospy
import fcntl
import termios
from geometry_msgs.msg import Twist

class VisionKeyboardBridge:
    def __init__(self):
        # SUSTITUYE EL NUMERO POR EL DE TU TTY
        self.terminal_path = '/dev/pts/2' 
        self.fd = open(self.terminal_path, 'w')
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        rospy.loginfo(f"Inyectando teclas en {self.terminal_path}")

    def send_key(self, char):
        # Esta función "empuja" una letra dentro de la terminal del robot
        for c in char:
            fcntl.ioctl(self.fd, termios.TIOCSTI, c)

    def callback(self, msg):
        # Si la visión dice girar a la derecha
        if msg.angular.z > 0.3:
            self.send_key('l')
        # Si la visión dice girar a la izquierda
        elif msg.angular.z < -0.3:
            self.send_key('j')
        # Si la visión dice avanzar
        if msg.linear.x > 0.1:
            self.send_key('w')

if __name__ == '__main__':
    rospy.init_node('vision_keyboard_bridge')
    VisionKeyboardBridge()
    rospy.spin()
