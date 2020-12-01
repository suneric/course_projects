#!/usr/bin/env python
import rospy
from camera import rs_image
from robot import iiwa_kuka
from controller import auto_scan
import sys

if __name__ == '__main__':
    args = sys.argv[1:]
    print(args)
    mode = 'ETS' # explore then scan
    if args[0] != '0':
        print("scan while exploring")
        mode = 'EAS'#explore and scan
    else:
        print("scan after exploring")

    rospy.init_node("auto_scan", anonymous=True, log_level=rospy.DEBUG)
    # create a controller and perform the scanning
    scan_frame = 200 # pixel
    scan_dist = 0.3 # meter
    camera = rs_image(scan_frame)
    robot = iiwa_kuka()
    asc = auto_scan(camera,robot,scan_dist,scan_frame,mode)
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            # wait until the camera and robot is ready
            if camera.ready() and robot.ready():
                if asc.ready_to_initial():
                    asc.initialize()
                elif asc.ready_to_explore():
                    asc.explore()
                elif asc.ready_to_scan():
                    asc.scan()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
