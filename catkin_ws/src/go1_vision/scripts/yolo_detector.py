#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO

class UnitreeYoloDetector:
    def __init__(self):
        # 1. Inicializar el nodo de ROS
        rospy.init_node('unitree_yolo_detector', anonymous=True)
        
        # 2. Cargar el mejor modelo equilibrado para robótica: YOLOv8 Extra large
        # (Se descargará automáticamente la primera vez que se ejecute)
        self.model = YOLO("yolov8x-seg.pt") 
        
        # 3. Puente para transformar imágenes de ROS a OpenCV
        self.bridge = CvBridge()
        
        # 4. Suscribirse al topic de la cámara frontal
        self.image_sub = rospy.Subscriber(
            "/camera_face/color/image_raw", 
            Image, 
            self.camera_callback
        )
        
        rospy.loginfo("🚀 Nodo YOLO (Extra large) iniciado. Rotando imagen 180° activado.")

    def camera_callback(self, data):
        try:
            # Convertir el mensaje de imagen de ROS a una matriz BGR de OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Error en la conversión de imagen: {e}")
            return

        # 🔄 Rotación de 180 grados para corregir la cámara invertida de Gazebo
        cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)

        # Realizar la inferencia con el nuevo modelo YOLOv8
        results = self.model(cv_image, conf=0.05, verbose=False)

        # Dibujar las cajas de detección sobre la imagen rotada
        annotated_frame = results[0].plot()

        # Mostrar el resultado visual corregido en la ventana
        cv2.imshow("Camara Frontal Go1 - Ultralytics YOLO 26", annotated_frame)
        
        # Necesario para refrescar la ventana
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = UnitreeYoloDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
