#!/usr/bin/env python
import roslib; roslib.load_manifest('turtlebots_master')
import rospy
from std_msgs.msg import UInt8
import sys, select, termios, tty


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    cmd_pub = rospy.Publisher('npc_nodes_cmd', UInt8)
    rospy.init_node('npc_nodes_cmd')

    try:
        while(True):
            key = getKey()
            #print "the key value is %d" % ord(key)
            if ord(key) == 13: #enter key
                print "publish the start cmd"
                msg = UInt8()
                msg.data = 1
                cmd_pub.publish(msg)
            if key == 'q':
                print "publish the stop cmd"
                msg = UInt8()
                msg.data = 0
                cmd_pub.publish(msg)
            else:
                if (key == '\x03'):
                    break

            rospy.sleep(0.001)
    except Exception as e:
        print e
        print repr(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
