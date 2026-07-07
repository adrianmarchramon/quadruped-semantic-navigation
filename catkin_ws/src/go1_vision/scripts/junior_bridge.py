#!/usr/bin/env python3
import rospy
import os
import pty
import subprocess
from geometry_msgs.msg import Twist

class JuniorBridge:
    def __init__(self):
        rospy.init_node('junior_bridge')
        
        # 1. Crear un "Teclado Fantasma" (Pseudo-terminal)
        self.master, slave = pty.openpty()
        
        # 2. Arrancar el controlador sordo dentro del teclado fantasma
        rospy.loginfo("🚀 Arrancando junior_ctrl en Terminal Fantasma...")
        self.proc = subprocess.Popen(['rosrun', 'unitree_guide', 'junior_ctrl'], 
                                     stdin=slave)
        
        self.last_cmd = b' ' # Espacio suele ser freno/reposo
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb)
        
        # 3. Secuencia automática de encendido (¡Magia!)
        rospy.sleep(2)
        os.write(self.master, b'2')
        rospy.loginfo("✅ Enviado: 2 (Levantarse)")
        rospy.sleep(3)
        os.write(self.master, b'4')
        rospy.loginfo("✅ Enviado: 4 (Modo Trote)")
        rospy.loginfo("🎮 PUENTE CMD_VEL -> TECLADO LISTO Y ESPERANDO")

    def cmd_cb(self, msg):
        new_cmd = b' ' # Freno por defecto (Espacio)
        
        if msg.linear.x > 0.1:
            new_cmd = b'w' # Avanzar
        # ¡LÓGICA INVERTIDA AQUÍ!
        elif msg.angular.z > 0.1:
            new_cmd = b'l' # Girar (Invertido)
        elif msg.angular.z < -0.1:
            new_cmd = b'j' # Girar (Invertido)
            
        # Solo pulsamos la tecla si la orden cambia
        if new_cmd != self.last_cmd:
            os.write(self.master, new_cmd)
            self.last_cmd = new_cmd
            rospy.loginfo(f"🤖 Teclado fantasma pulsó: [{new_cmd.decode()}]")

if __name__ == '__main__':
    try:
        JuniorBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
