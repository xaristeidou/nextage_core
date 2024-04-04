#!/usr/bin/env python3
import os
import rospy
import tf2_ros
import tf2_msgs.msg
from moveit_commander import PlanningSceneInterface,PlanningScene
from moveit_msgs.msg import ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import PoseStamped,TransformStamped

local_mesh_repo_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
# local_mesh_repo_dir = os.path.dirname(__file__)


class DynamicPlanningScene:

    def __init__(self):
        self.planning_scene_interface = PlanningSceneInterface()
        self.planning_scene_service = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
        # rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.tf_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.colors_dict = dict()
        self.tf_names_list = ['big_hob_frame','small_hob_frame','pcb_frame','double_hob_frame']


    def check_transforms(self,tf_frame_name):
        try:
            _ = self.tf_buffer.lookup_transform("world", tf_frame_name, rospy.Time(0))
            rospy.loginfo("Frame {} is being published.".format(tf_frame_name))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Frame {} is not published.".format(tf_frame_name))
            return False

    def publish_planning_scene(self):

        
        if self.check_transforms('big_hob_frame'):
            big_hob = PoseStamped()
            big_hob.header.frame_id = "big_hob_frame"
            big_hob.pose.orientation.x, big_hob.pose.orientation.y, big_hob.pose.orientation.z, big_hob.pose.orientation.w = 0.0, 0.0, 0.0, 1.0
            big_hob.pose.position.x, big_hob.pose.position.y, big_hob.pose.position.z = 0.0,0.0,0.0 
            self.planning_scene_interface.add_mesh("big_hob", big_hob, os.path.join(local_mesh_repo_dir,'elux_robot_ros/elux_ur10e_description/meshes/big_hob/big_hob_pivot_in_center.stl'))
            self.setColor(name="big_hob",r=0.5,g=0.5,b=0.5,a=0.7)
            self.sendColors()
        if self.check_transforms('small_hob_frame'):
            small_hob = PoseStamped()
            small_hob.header.frame_id = "small_hob_frame"
            small_hob.pose.orientation.x, small_hob.pose.orientation.y, small_hob.pose.orientation.z, small_hob.pose.orientation.w = 0.0, 0.0, 0.0, 1.0
            small_hob.pose.position.x, small_hob.pose.position.y, small_hob.pose.position.z = 0.0,0.0,.0
            self.planning_scene_interface.add_mesh("small_hob", small_hob, os.path.join(local_mesh_repo_dir,'elux_robot_ros/elux_ur10e_description/meshes/small_hob/small_hob_pivot_in_center.stl'))
            self.setColor(name="small_hob",r=0.5,g=0.5,b=0.5,a=0.7)
            self.sendColors()
        if self.check_transforms('pcb_frame'):
            pcb = PoseStamped()
            pcb.header.frame_id = "pcb_frame"
            pcb.pose.orientation.x, pcb.pose.orientation.y, pcb.pose.orientation.z, pcb.pose.orientation.w = 0.0, 0.0, 0.0, 1.0
            pcb.pose.position.x, pcb.pose.position.y, pcb.pose.position.z = .0,.0,.0
            self.planning_scene_interface.add_mesh("pcb", pcb, os.path.join(local_mesh_repo_dir,'elux_robot_ros/elux_ur10e_description/meshes/tool_box.stl'))
            self.setColor(name="pcb",r=.0,g=.0,b=.0,a=0.7)
            self.sendColors()
        if self.check_transforms('double_hob_frame'):
            double_hob = PoseStamped()
            double_hob.header.frame_id = "double_hob_frame"
            double_hob.pose.orientation.x, double_hob.pose.orientation.y, double_hob.pose.orientation.z, double_hob.pose.orientation.w = 0.0, 0.0, 0.0, 1.0
            double_hob.pose.position.x, double_hob.pose.position.y, double_hob.pose.position.z = .0,.0,.0
            self.planning_scene_interface.add_mesh("double_hob", double_hob, os.path.join(local_mesh_repo_dir,'elux_robot_ros/elux_ur10e_description/meshes/mini_plate/mini_plate.stl'))
            self.setColor(name="double_hob",r=0.5,g=0.5,b=0.5,a=0.7)
            self.sendColors()

    def setColor(self, name, r, g, b, a=0.9):
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors_dict[name] = color

    def sendColors(self):
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        for color in self.colors_dict.values():
            planning_scene.object_colors.append(color)
            
        resp = self.planning_scene_service.call(planning_scene)

        if not resp.success:
            rospy.logerr("Could not update colors through service, using topic instead.")
            self._scene_pub.publish(planning_scene)

if __name__ == '__main__':
    rospy.init_node('dynamic_planning_scene')
    elux_planning_scene = DynamicPlanningScene()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        elux_planning_scene.publish_planning_scene()
        rate.sleep()
    # rospy.spin()
    