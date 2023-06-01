#!/usr/bin/python3
#
# Send joint values to UR5 using messages
#

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import cv2
import mediapipe as mp
import rospy
import actionlib
import control_msgs.msg
def gripper_client(value):

        # Create an action client
        client = actionlib.SimpleActionClient(
            '/gripper_controller/gripper_cmd',  # namespace of the action topics
            control_msgs.msg.GripperCommandAction # action type
        )
        
        # Wait until the action server has been started and is listening for goals
        client.wait_for_server()

        # Create a goal to send (to the action server)
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = value   # From 0.0 to 0.8
        goal.command.max_effort = -1.0  # Do not limit the effort
        client.send_goal(goal)

        client.wait_for_result()
        return client.get_result()


class handTracker():
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5,modelComplexity=1,trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.modelComplex = modelComplexity
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands,self.modelComplex,self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils


    def handsFinder(self,image,draw=True):
        imageRGB = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imageRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:

                if draw:
                    self.mpDraw.draw_landmarks(image, handLms, self.mpHands.HAND_CONNECTIONS)
        return image        
    



    def positionFinder(self,image, handNo=0, draw=True):
        lmlist = []
        if self.results.multi_hand_landmarks:
            Hand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(Hand.landmark):
                h,w,c = image.shape
                cx,cy = int(lm.x*w), int(lm.y*h)
                lmlist.append([id,cx,cy])
            if draw:
                cv2.circle(image,(cx,cy), 15 , (255,0,255), cv2.FILLED)

        return lmlist
    

    
    
    def handClosed(self,image, handNo=0, draw=True):
        lmlist = []
        fingers_closed = [False] * 5
        if self.results.multi_hand_landmarks:
            Hand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(Hand.landmark):
                h,w,c = image.shape
                cx,cy = int(lm.x*w), int(lm.y*h)
                lmlist.append([id,cx,cy])

            # Check if the thumb is closed
            if lmlist[4][1] > lmlist[3][1]:
                fingers_closed[0] = True


            if draw:
                cv2.circle(image,(cx,cy), 15 , (255,0,255), cv2.FILLED)

            print(fingers_closed[0])

        return fingers_closed[0]

    
    




# Import the module that tracks your hand position

def main():
    cap = cv2.VideoCapture(0)
    tracker = handTracker()
    rospy.init_node('send_joints')
    pub = rospy.Publisher('/trajectory_controller/command',
                          JointTrajectory,
                          queue_size=10)

    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Get the position of your hand in 3D space

        success,image = cap.read()
        image = tracker.handsFinder(image)
        lmList = tracker.positionFinder(image)
        if len(lmList) != 0:
            print(lmList[4])
            handClosed = tracker.handClosed(image)
            print(handClosed)
            if handClosed == True:
                gripper_value = 0.8
                gripper_client(gripper_value)
            else:
                gripper_value = 0.0
                gripper_client(0.0)

        cv2.imshow("Video",image)
        cv2.waitKey(1)


        hand_pos = lmList

        # Convert the hand position to the robot's coordinate system
        # ... (insert code to perform the conversion here)

        # Set the end effector position to the hand position
        pts = JointTrajectoryPoint()


        # Translate coordinates from openCV to UR5

        if len(lmList) != 0:
            print(lmList[4])
            xCoord = lmList[4][1]
            yCoord = lmList[4][2]
        else:
            xCoord = 0.0
            yCoord = 0.0

        scaled_xCoord = (((xCoord - 375.0) / 450.0) * 1.0)
        
        scaled_yCoord = min(-0.25,((yCoord - 375.0) / 450.0) * 1.0)

        print(scaled_yCoord)

        pts.positions = [scaled_xCoord, scaled_yCoord,  0.0, -1.0, -1.5, 0.0]
        #pts.positions = [0.0, 0.0, 0.0, -1.0, -1.5, 0.0]
        pts.time_from_start = rospy.Duration(1.0)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message
        pub.publish(traj)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")


