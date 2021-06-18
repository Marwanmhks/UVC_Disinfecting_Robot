import numpy as np
import rospy
import tf
import actionlib
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from move_base_seq_ros.msg import SequenceArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Initializing ROS node
rospy.init_node("apriltag_move_base_goal")
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
goal = MoveBaseGoal()

# Initializing global variable goal and transform listener for pose transformation
transformer = TransformListener()

# Defining the topic callback function
def detection_callback(detected_array):
    try:
        if len(detected_array.detections) > 0:
            # Making robot approach shopper, inversion of frames due to difference between camera frame and robot frame
            goal.target_pose.pose.position.x = np.maximum(0, detected_array.detections[0].pose.pose.pose.position.z - 0.5)
            goal.target_pose.pose.position.y = -1 * detected_array.detections[0].pose.pose.pose.position.x
            displacement_vector = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y]
            goal.target_pose.pose.orientation = Quaternion(0,0,0,1)
            goal.target_pose.header.frame_id = "/base_link"
            goal.target_pose.header.stamp = detected_array.header.stamp
            goal.target_pose = transformer.transformPose("map", goal.target_pose)
            goal.target_pose.header.frame_id = "map"

            # Only update goal if there's a safe distance between robot and goal
            if np.linalg.norm(displacement_vector) > 0.5:
                client.send_goal(goal)
                client.wait_for_result()
        else:
            rospy.logwarn("No april tag observed, goal undefined.")
    except (tf.ExtrapolationException, tf.LookupException):
        None

# Initializing ROS publisher and subscriber
goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
rospy.Subscriber("tag_detections", SequenceArray, detection_callback)

rospy.loginfo("apriltag to move_base node started.")

# Empty loop
while not rospy.is_shutdown():
    try:
        continue
    except KeyboardInterrupt:
        exit()



