#!/usr/bin/env python
import roslib; roslib.load_manifest('rct_turtlebot')
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import sys, select, termios, tty


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    nav_pub = rospy.Publisher('move_base_simple/plan', Path)
    rospy.init_node('nav_path')

    # design the path
    nav_plan_msg = Path()
    nav_plan_msg.header.stamp = rospy.Time.now()
    nav_plan_msg.header.frame_id = '/map'

    ## sample : straight line from 0m to 1m
    q = tf.transformations.quaternion_from_euler(0, 0, 0) # roll, pitch, yaw
    orientation = Quaternion(*q)
    for i in range(50) :
        single_goal = PoseStamped()
        single_goal.header.stamp = rospy.Time.now()
        single_goal.header.frame_id = '/map'
        single_goal.pose.position.x = (i + 1) * 0.02 # interval is 0.02m
        single_goal.pose.position.y = 0
        single_goal.pose.position.z = 0 
        single_goal.pose.orientation = orientation
        nav_plan_msg.poses.append(single_goal)

    try:
        while(True):
            key = getKey()
            #print "the key value is %d" % ord(key)
            if ord(key) == 13: #enter key
                print "publish the nav plan"
                nav_pub.publish(nav_plan_msg)
            else:
                if (key == '\x03'):
                    break

            rospy.sleep(0.001)
    except Exception as e:
        print e
        print repr(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
