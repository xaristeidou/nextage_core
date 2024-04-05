#!/usr/bin/env python3
import os
import rospy
import tf2_ros
from moveit_commander import PlanningSceneInterface,PlanningScene
from moveit_msgs.msg import ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import PoseStamped,TransformStamped

local_mesh_repo_dir = os.path.dirname(os.path.dirname((__file__)))

print(local_mesh_repo_dir)

class EluxPlanningScene:

    def __init__(self):
        self.planning_scene_interface = PlanningSceneInterface()
        self.planning_scene_service = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
        self.colors_dict = dict()
        # self.publish_TF()


    def publish_TF(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = "gamout"
        static_transformStamped.transform.translation.x = -0.21
        static_transformStamped.transform.translation.y = 0.17
        static_transformStamped.transform.translation.z = -0.17
        static_transformStamped.transform.rotation.w = 0.707
        static_transformStamped.transform.rotation.z = 0.707
        broadcaster.sendTransform(static_transformStamped)


    def publish_planning_scene(self):
        print('Initialize Planning Scene')
                
        elux_pose = PoseStamped()
        elux_pose.header.frame_id = "world"
        elux_pose.pose.orientation.x, elux_pose.pose.orientation.y, elux_pose.pose.orientation.z, elux_pose.pose.orientation.w = 0.0, 0.0, -0.7068252, 0.7073883
        elux_pose.pose.position.x, elux_pose.pose.position.y, elux_pose.pose.position.z = 0.20, 0.30, -0.85
        self.planning_scene_interface.add_mesh("elux_scene", elux_pose, os.path.join(local_mesh_repo_dir,'meshes/elux_tecnalia_scene.stl'))
        self.setColor(name="elux_scene",r=0.88,g=0.77,b=0.66,a=1.0)
        self.sendColors()
        

        base_plate = PoseStamped()
        base_plate.header.frame_id = "base_link"
        base_plate.pose.orientation.x, base_plate.pose.orientation.y, base_plate.pose.orientation.z, base_plate.pose.orientation.w = 0.0, 0.0, -0.7068252, 0.7073883
        base_plate.pose.position.x, base_plate.pose.position.y, base_plate.pose.position.z = 0.49, 0.00, -0.10
        self.planning_scene_interface.add_mesh("hobs_plate", base_plate, os.path.join(local_mesh_repo_dir,'meshes/base_plate.stl'))
        self.setColor(name="hobs_plate",r=0.75,g=0.75,b=0.75,a=1.0)
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
    rospy.init_node('static_planning_scene')
    elux_planning_scene = EluxPlanningScene()
    elux_planning_scene.publish_planning_scene()
    rospy.spin()
    