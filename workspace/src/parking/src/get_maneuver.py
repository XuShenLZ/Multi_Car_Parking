#!/usr/bin/env python

import rospy
from parking.srv import maneuver

def main():
	rospy.wait_for_service('park_maneuver')
	try:
		get_maneuver = rospy.ServiceProxy('park_maneuver', maneuver)
		final_maneuver = get_maneuver("U", "U", "F")
		print final_maneuver
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == '__main__':
	main()