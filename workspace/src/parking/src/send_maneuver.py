#!/usr/bin/env python

import rospy
from parking.srv import maneuver

def handle_request(req):
	print req
	print "returning value"
	val0 = True
	val1 = [1.0, 2.0]
	val2 = [3.0, 4.0]
	val3 = [5.0, 6.0]
	respond = [val0, val1, val2, val3]
	return respond


def main():
	rospy.init_node('send_maneuver')
	s = rospy.Service('park_maneuver', maneuver, handle_request)
	print "ready to send maneuver"
	rospy.spin()

if __name__ == '__main__':
	main()