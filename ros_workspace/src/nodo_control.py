import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped, PoseArray 
from sensor_msgs.msg import JointState
from launcher_robots_lab_robotica.msg import Obstaculo

class ControlRobot:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        self.add_floor()

        rospy.Subscriber('/mover_pose', Pose, self.orden_mover_a_pose)
        rospy.Subscriber('/mover_configuracion', JointState, self.orden_mover_a_configuracion)
        rospy.Subscriber('/trayectoria_cartesiana', PoseArray, self.orden_mover_en_trayectoria)
        rospy.Subscriber('/añadir_obstaculo', Obstaculo, self.orden_añadir_obstaculo)

    def orden_añadir_obstaculo(self, msg):
        """
        Añade un obstáculo a la escena de planificación en la pose indicada y tamaño especificado.
        """
        rospy.loginfo("Añadiendo obstáculo en la escena con tamaño y posición especificada.")
        self.add_box_to_planning_scene(msg.pose, "Obstaculo_dinamico", (msg.size, msg.size, msg.size))
        rospy.loginfo("Obstáculo añadido a la escena.")

    def orden_mover_a_pose(self, pose_goal):
        """
        Mueve el robot a una pose específica recibida en el mensaje.
        """
        rospy.loginfo("Moviendo el robot a la pose específica recibida.")
        if self.move_to_pose(pose_goal):
            rospy.loginfo("El robot ha alcanzado la pose objetivo.")
        else:
            rospy.logwarn("No se pudo alcanzar la pose objetivo.")

    def orden_mover_a_configuracion(self, joint_state):
        """
        Mueve el robot a una configuración específica de sus articulaciones recibida en el mensaje.
        """
        rospy.loginfo("Moviendo el robot a la configuración específica recibida.")
        joint_goal = joint_state.position
        if self.move_motors(joint_goal):
            rospy.loginfo("El robot ha alcanzado la configuración objetivo.")
        else:
            rospy.logwarn("No se pudo alcanzar la configuración objetivo.")

    def orden_mover_en_trayectoria(self, pose_array):
        rospy.loginfo("Moviendo el robot a lo largo de la trayectoria especificada.")
        poses = pose_array.poses
        if self.move_trajectory(poses):
            rospy.loginfo("El robot ha completado la trayectoria con éxito.")
        else:
            rospy.logwarn("No se pudo completar la trayectoria.")

    def get_motor_angles(self):
        return self.move_group.get_current_joint_values()

    def move_motors(self, joint_goal, wait=True):
        return self.move_group.go(joint_goal, wait=wait)

    def get_pose(self):
        return self.move_group.get_current_pose().pose

    def move_to_pose(self, pose_goal, wait=True):
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)

    def add_box_to_planning_scene(self, pose_caja, name, tamaño=(.1, .1, .1)):
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        self.scene.add_box(name, box_pose, size=tamaño)

    def add_floor(self):
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.add_box_to_planning_scene(pose_suelo, "suelo", (2, 2, .05))

    def move_trajectory(self, poses, wait=True):
        poses.insert(0, self.get_pose())
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01, 0.0)
        
        if fraction != 1.0:
            return False
        
        return self.move_group.execute(plan, wait=wait)

if __name__ == '__main__':
    controlador = ControlRobot()
    rospy.spin()
