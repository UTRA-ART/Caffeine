import rospy
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

def init_node(arguments):
    rospy.init_node('movebase_client_py')

    if len(arguments) < 8:
        rospy.logerr("Error: Insufficient Arguments - Usage is 'rosrun nav_goal Movebase_client x y z roll pitch yaw frame' where x is distance to move in x, y is distance to move in y, z is distance to move in z, roll, pitch, & yaw are rotations in radians, and frame is the frame of reference.")
        rospy.signal_shutdown("Error: Insufficient Arguments - Usage is 'rosrun nav_goal movebase_client x y z roll pitch yaw frame' where x is distance to move in x, y is distance to move in y, z is distance to move in z, roll, pitch, & yaw are rotations in radians, and frame is the frame of reference.")
        return 'Failure'
    else:
        try:
            x = float(arguments[1])
            y = float(arguments[2])
            z = float(arguments[3])
            roll = float(arguments[4])
            pitch = float(arguments[5])
            yaw = float(arguments[6])
            frame = str(arguments[7])
            
            # Checks if the frame inputted is valid, more frames can be added below if necessary
            if (frame != "caffeine/map") or (frame != "caffeine/base_link"):
               rospy.logger("Invalid frame")
               rospy.shutdown("Invalid frame")
               return 'Failure'

            return (x, y, z, roll, pitch, yaw, frame)
        except:
            rospy.logerr("Error: Incorrect Argument Types - x, y, z, roll, pitch, yaw arguments must be floats or integers and frame must be a string.")
            rospy.signal_shutdown("Error: Incorrect Argument Types - x, y, z, roll, pitch, yaw arguments must be floats or integers and frame must be a string.")
            return 'Failure'

def movebase_client(x, y, z, roll, pitch, yaw, frame):
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set goal position (x, y, and z are in meters)
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z

    # Convert roll, pitch, yaw to quaternion and set pose with roll, pitch and yaw in radians
    q = quaternion_from_euler(roll, pitch, yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    # Sends goal and waits until the action is completed (or aborted if it is impossible)
    client.send_goal(goal)

    rospy.loginfo(f"Navigation Goals of x:{x}, y:{y}, z:{z}, roll:{roll}, pitch:{pitch}, yaw:{yaw} sent to action server!")

    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()

    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Error in retrieving action resolution! Exiting...")
        rospy.signal_shutdown("Error in retrieving action resolution! Exiting...")
    else:
    # Result of executing the action
        return client.get_goal_status_text()

if __name__ == '__main__':
    response = init_node(sys.argv)

    if response != 'Failure':
        try:
            x, y, z, roll, pitch, yaw, frame = response

            # If the user inputs the frame under namespace "caffeine/" or "/caffeine/" which is still valid, remove it for error checking and consistency reasons.
            if frame[0:9].lower() == "caffeine/":
                frame = frame[9:].lower()
            elif frame[0:10].lower() == "/caffeine/":
                frame = frame[10:].lower()

            result = movebase_client(x, y, z, roll, pitch, yaw, f"caffeine/{frame}")
            rospy.loginfo(result)
        except:
            rospy.logerr("Error - Navigation failed for unknown reasons. Exiting...")
            rospy.signal_shutdown("Error - Navigation failed for unknown reasons. Exiting...")
~                                                                                              