#!/usr/bin/env python  
from this import d
import rospy
import tf2_ros
import gazebo_msgs.msg
import geometry_msgs.msg
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class AccurateODOM():

    def __init__(self) -> None:
        
        self.init_pos_x     = rospy.get_param("init_pos_x"  , 0)
        self.init_pos_y     = rospy.get_param("init_pos_y"  , 0)
        self.init_pos_yaw   = rospy.get_param("init_pos_yaw", 0)

        self._broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._sub = rospy.Subscriber(   name="/gazebo/model_states",
                                        data_class=gazebo_msgs.msg.ModelStates,
                                        callback=self._sub_Callback
                                    )

    def _sub_Callback(self, data):
        
        idx = data.name.index("agvs")
        x = data.pose[idx].position.x - self.init_pos_x
        y = data.pose[idx].position.y - self.init_pos_y
        _, _, yaw = euler_from_quaternion ([    
                                                data.pose[idx].orientation.x,
                                                data.pose[idx].orientation.y,
                                                data.pose[idx].orientation.z,
                                                data.pose[idx].orientation.w
                                            ])
        yaw = yaw - self.init_pos_yaw

        print("\nPosition X: %f\nPosition Y: %f\nHeading Yaw: %f"%(x, y, yaw))
        
        # transform = geometry_msgs.msg.TransformStamped()
        # transform.header.stamp = rospy.Time.now()
        # transform.header.frame_id = "world"
        # transform.child_frame_id = data.name[idx]
        # transform.transform.translation.x = data.pose[idx].position.x
        # transform.transform.translation.y = data.pose[idx].position.y
        # transform.transform.translation.z = data.pose[idx].position.z
        # transform.transform.rotation.w = data.pose[idx].orientation.w
        # transform.transform.rotation.x = data.pose[idx].orientation.x
        # transform.transform.rotation.y = data.pose[idx].orientation.y
        # transform.transform.rotation.z = data.pose[idx].orientation.z

        # self._broadcaster.sendTransform(transform)

if __name__ == '__main__':

    try:

        AccurateODOM()
        rospy.init_node('World Position Node')
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()
            
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
