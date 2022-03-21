#!/usr/bin/env python  
import math
from sre_parse import State
import rospy
import tf2_ros
import gazebo_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, atan2

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, PoseStamped
import numpy as np

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

        self.pub_odom = rospy.Publisher('/path_by_hand', Path, queue_size=1)
        self.path = Path()

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

        # Method 1: Calculate positions of two wheels and average to get center of the AMR. (BUG)
        # front_pos_x = dx_front + self.length/2*cos(self.last_pos_yaw) + self.last_pos_x
        # front_pos_y = dy_front + self.length/2*sin(self.last_pos_yaw) + self.last_pos_y

        # rear_pos_x = dx_rear - self.length/2*cos(self.last_pos_yaw) + self.last_pos_x
        # rear_pos_y = dy_rear - self.length/2*sin(self.last_pos_yaw) + self.last_pos_y

        # self.last_pos_x = (front_pos_x + rear_pos_x)/2
        # self.last_pos_y = (front_pos_y + rear_pos_y)/2
        # self.last_pos_yaw = atan2(self.last_pos_y, self.last_pos_x)
        # if self.last_pos_yaw > math.pi/2:
        #     self.last_pos_yaw = self.last_pos_yaw - math.pi
        # elif self.last_pos_yaw < -math.pi/2:
        #     self.last_pos_yaw = self.last_pos_yaw + math.pi



        # print("\nPosition X: %f\nPosition Y: %f\nHeading Yaw: %f"%(self.last_pos_x, self.last_pos_y, self.last_pos_yaw))

        # self.last_front_steering_pos  = data.position[idx_front_steering]
        # self.last_rear_steering_pos   = data.position[idx_rear_steering] 
        # self.last_front_driving_pos   = data.position[idx_front_driving]
        # self.last_rear_driving_pos    = data.position[idx_rear_driving] 
        

        # Method 2: Xi = inv(H.T*H)*H.T * Xo
        # HHH = np.array([  [ 0.5         ,0.          ,0.5         ,0.        ]
        #                   [ 0.          ,0.5         ,0.          ,0.5       ]
        #                   [ 0.          ,1.04384134  ,0.          ,-1.04384134]   ])
        diff_pos_x = dx_front/2 + dx_rear/2
        diff_pos_y = dy_front/2 + dy_rear/2
        diff_pos_yaw = 1.04384134*dy_front - 1.04384134*dy_rear

        self.last_pos_yaw = self.last_pos_yaw + diff_pos_yaw
        self.last_pos_x = self.last_pos_x + diff_pos_x*cos(self.last_pos_yaw) - diff_pos_y*sin(self.last_pos_yaw)
        self.last_pos_y = self.last_pos_y + diff_pos_x*sin(self.last_pos_yaw) + diff_pos_y*cos(self.last_pos_yaw)

        if self.last_pos_yaw > math.pi:
            self.last_pos_yaw = self.last_pos_yaw -2*math.pi
        elif self.last_pos_yaw < -math.pi:
            self.last_pos_yaw = self.last_pos_yaw +2*math.pi


        print("\nPosition X: %f\nPosition Y: %f\nHeading Yaw: %f"%(self.last_pos_x, self.last_pos_y, self.last_pos_yaw))

        cmd = PoseStamped()
        cmd.pose.position.x = self.last_pos_x
        cmd.pose.position.y = self.last_pos_y
        cmd.pose.position.z = 0.35

        quaternion = quaternion_from_euler(0, 0, self.last_pos_yaw)
        cmd.pose.orientation.x = quaternion[0]
        cmd.pose.orientation.y = quaternion[1]
        cmd.pose.orientation.z = quaternion[2]
        cmd.pose.orientation.w = quaternion[3]

        self.path.header.frame_id = "/map"
        # self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(cmd)
        self.pub_odom.publish(self.path)



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
