#!/usr/bin/env python  
from sre_parse import State
from this import d
import rospy
import tf2_ros
import gazebo_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, atan2

class JointStateToODOM():

    def __init__(self) -> None:
        
        self.init_pos_x     = rospy.get_param("init_pos_x"  , 0)
        self.init_pos_y     = rospy.get_param("init_pos_y"  , 0)
        self.init_pos_yaw   = rospy.get_param("init_pos_yaw", 0)

        self.last_pos_x      = self.init_pos_x
        self.last_pos_y      = self.init_pos_y
        self.last_pos_yaw    = self.init_pos_yaw

        self.radius = 0.11
        self.length = 0.479*2

        self.last_front_steering_pos  = 0
        self.last_rear_steering_pos   = 0
        self.last_front_driving_pos   = 0
        self.last_rear_driving_pos    = 0

        self._broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._sub = rospy.Subscriber(   
                                        name="/agvs/joint_states",
                                        data_class=sensor_msgs.msg.JointState,
                                        callback=self._sub_Callback
                                    )

    def _sub_Callback(self, data):
    
        idx_front_steering  = data.name.index("joint_front_motor_wheel")
        idx_rear_steering   = data.name.index("joint_back_motor_wheel")
        idx_front_driving   = data.name.index("joint_front_wheel")
        idx_rear_driving    = data.name.index("joint_back_wheel")

        diff_front_steering_pos  =  data.position[idx_front_steering] - self.last_front_steering_pos
        diff_front_driving_pos   = (data.position[idx_front_driving]  - self.last_front_driving_pos)*self.radius
        
        dx_front = diff_front_driving_pos * cos(self.last_front_steering_pos + diff_front_steering_pos/2)
        dy_front = diff_front_driving_pos * sin(self.last_front_steering_pos + diff_front_steering_pos/2)
        
        diff_rear_steering_pos   =  data.position[idx_rear_steering]  - self.last_rear_steering_pos
        diff_rear_driving_pos    = (data.position[idx_rear_driving]   - self.last_rear_driving_pos)*self.radius
        
        dx_rear = diff_rear_driving_pos * cos(self.last_rear_steering_pos + diff_rear_steering_pos/2)
        dy_rear = diff_rear_driving_pos * sin(self.last_rear_steering_pos + diff_rear_steering_pos/2)

        front_pos_x = dx_front + self.length/2*cos(self.last_pos_yaw) + self.last_pos_x
        front_pos_y = dy_front + self.length/2*sin(self.last_pos_yaw) + self.last_pos_y

        rear_pos_x = dx_rear - self.length/2*cos(self.last_pos_yaw) + self.last_pos_x
        rear_pos_y = dy_rear - self.length/2*sin(self.last_pos_yaw) + self.last_pos_y

        self.last_pos_x = (front_pos_x + rear_pos_x)/2
        self.last_pos_y = (front_pos_y + rear_pos_y)/2
        self.last_pos_yaw = atan2(self.last_pos_y, self.last_pos_x)

        print("\nPosition X: %f\nPosition Y: %f\nHeading Yaw: %f"%(self.last_pos_x, self.last_pos_y, self.last_pos_yaw))

        self.last_front_steering_pos  = data.position[idx_front_steering]
        self.last_rear_steering_pos   = data.position[idx_rear_steering] 
        self.last_front_driving_pos   = data.position[idx_front_driving]
        self.last_rear_driving_pos    = data.position[idx_rear_driving] 
        


if __name__ == '__main__':

    try:

        JointStateToODOM()
        rospy.init_node('JointState_To_Odom_Node')
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()
            
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
