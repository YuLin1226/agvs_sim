import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class MakeNavGoal:
    def __init__(self) -> None:
        self.client_ = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move base server")
        self.client_.wait_for_server()

        self.carPosX_   = 0.0
        self.carPosY_   = 0.0
        self.carPosYaw_ = 0.0
        self.flag_ = False
        
        # This line should be edited for different localization nodes.
        self.sub_ = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odomCallback)
        

    def odomCallback(self, data):
        self.carPosX_ = data.pose.pose.position.x
        self.carPosY_ = data.pose.pose.position.y
        self.carPosYaw_ = euler_from_quaternion((   data.pose.pose.orientation.x,
                                                    data.pose.pose.orientation.y,
                                                    data.pose.pose.orientation.z,
                                                    data.pose.pose.orientation.w
                                                ))[2]

    def sendGoal(self):

        if self.flag_:
        
            goalYaw = self.carPosYaw_
            goalX = self.carPosX_ + math.cos(goalYaw)
            goalY = self.carPosY_ + math.sin(goalYaw)
            rospy.loginfo("Send Navigation Goal to (%f, %f, %f)" %(goalX, goalY, goalYaw))

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map' 
            goal.target_pose.pose.position.x = goalX
            goal.target_pose.pose.position.y = goalY
            q = quaternion_from_euler(0, 0, goalYaw)
            goal.target_pose.pose.orientation.w = q[3]

            self.client_.send_goal(goal)
            self.client_.wait_for_result()
        
        else:
            print("Flag is False, Sending Goal Failed.")

        self.flag_ = False
    
    def setFlag(self):
        self.flag_ = True

if __name__ == "__main__":

    try:
        rospy.init_node('send_client_goal')
        MKG = MakeNavGoal()
        while not rospy.is_shutdown():
            print("\n*******\nEnd this node: Ctrl+c followed by answering No.\n*******\n")
            isSenfGoal = int(input("Send Navigation Goal?\nif Yes, input: 1; No, input: others\n\n"))
            if(isSenfGoal == 1):
                MKG.setFlag()
                MKG.sendGoal()
            else:
                break

        rospy.spin()
    
    except KeyboardInterrupt:
        pass

    
