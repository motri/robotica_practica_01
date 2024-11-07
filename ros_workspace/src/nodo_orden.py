# Imports
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import JointState
from launcher_robots_lab_robotica.msg import Obstaculo 

class NodoOrdenes:

    def __init__(self):
        rospy.init_node('ordenes', anonymous=True)

        self.pub_pose = rospy.Publisher('/mover_pose', Pose, queue_size=10)
        self.pub_configuracion = rospy.Publisher('/mover_configuracion', JointState, queue_size=10)
        self.pub_trayectoria = rospy.Publisher('/trayectoria_cartesiana', PoseArray, queue_size=10)
        self.pub_obstaculo = rospy.Publisher('/añadir_obstaculo', Obstaculo, queue_size=10)
        rospy.loginfo("Nodo de órdenes inicializado. Listo para recibir comandos.")

    def solicitar_orden(self):
        while not rospy.is_shutdown():
            print("Seleccione una acción para el robot:")
            print("1: Añadir obstáculo")
            print("2: Mover a pose específica")
            print("3: Mover a configuración específica")
            print("4: Seguir una trayectoria")

            try:
                orden = int(input("Ingrese el número de la acción deseada (1-4): "))
                
                if orden == 1:
                    self.enviar_obstaculo()
                elif orden == 2:
                    self.enviar_pose()
                elif orden == 3:
                    self.enviar_configuracion()
                elif orden == 4:
                    self.enviar_trayectoria()
                else:
                    rospy.logwarn("Número de orden no válido. Ingrese un número entre 1 y 4.")
            except ValueError:
                rospy.logwarn("Entrada no válida. Ingrese un número entre 1 y 4.")

    def enviar_obstaculo(self):
        obstaculo_msg = Obstaculo()
        obstaculo_msg.size = float(input("Ingrese el tamaño del obstáculo: "))
        
        pose = Pose()
        pose.position.x = float(input("Posición X del obstáculo: "))
        pose.position.y = float(input("Posición Y del obstáculo: "))
        pose.position.z = float(input("Posición Z del obstáculo: "))
        obstaculo_msg.pose = pose

        self.pub_obstaculo.publish(obstaculo_msg)
        rospy.loginfo("Obstáculo publicado en /añadir_obstaculo.")

    def enviar_pose(self):
        pose_goal = Pose()
        pose_goal.position.x = float(input("Posición X de la pose: "))
        pose_goal.position.y = float(input("Posición Y de la pose: "))
        pose_goal.position.z = float(input("Posición Z de la pose: "))
        pose_goal.orientation.x = float(input("Orientación X: "))
        pose_goal.orientation.y = float(input("Orientación Y: "))
        pose_goal.orientation.z = float(input("Orientación Z: "))
        pose_goal.orientation.w = float(input("Orientación W: "))

        self.pub_pose.publish(pose_goal)
        rospy.loginfo("Pose publicada en /mover_pose.")

    def enviar_configuracion(self):
        joint_state = JointState()
        num_joints = int(input("Ingrese el número de articulaciones: "))
        joint_state.position = []
        
        for i in range(num_joints):
            angulo = float(input(f"Ángulo para la articulación {i+1}: "))
            joint_state.position.append(angulo)

        self.pub_configuracion.publish(joint_state)
        rospy.loginfo("Configuración de articulaciones publicada en /mover_configuracion.")

    def enviar_trayectoria(self):
        pose_array = PoseArray()
        num_poses = int(input("Ingrese el número de poses para la trayectoria: "))

        for i in range(num_poses):
            pose = Pose()
            print(f"Pose {i+1}:")
            pose.position.x = float(input("  Posición X: "))
            pose.position.y = float(input("  Posición Y: "))
            pose.position.z = float(input("  Posición Z: "))
            pose.orientation.x = float(input("  Orientación X: "))
            pose.orientation.y = float(input("  Orientación Y: "))
            pose.orientation.z = float(input("  Orientación Z: "))
            pose.orientation.w = float(input("  Orientación W: "))
            pose_array.poses.append(pose)

        self.pub_trayectoria.publish(pose_array)
        rospy.loginfo("Trayectoria publicada en /trayectoria_cartesiana.")

if __name__ == '__main__':
    try:
        nodo_ordenes = NodoOrdenes()
        nodo_ordenes.solicitar_orden()
    except rospy.ROSInterruptException:
        pass
