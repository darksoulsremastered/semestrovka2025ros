#!/usr/bin/env python3
import numpy as np
import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
spinning = Twist()
going_forward = False
moving = True
depth = 0

def get_depth(msg):
    global depth
    depth_img = CvBridge().imgmsg_to_cv2(msg, "passthrough")
    distance = depth_img[240, 320]
    print(distance)
    depth = distance

def go_forward():
    global moving
    global going_forward
    spinning.linear.x = 1.0
    spinning.angular.z = 0.0
    print("going")
    if depth < 0.4:
        moving = False
        going_forward = False
        spinning.linear.x = 0.
        print("STOOOOOOOOOOP")
        return

def get_limits():
    return (0, 100, 100), (10, 255, 255)

def found_ball(img):
    global moving
    global going_forward

    x = 0
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, 'bgr8')
    if moving:
        lower_limit, upper_limit = get_limits()
        hsvImage = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)


        mask = cv.inRange(hsvImage, lower_limit, upper_limit)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv.contourArea(cnt)
            perimeter = cv.arcLength(cnt, True)
            if perimeter == 0 or area < 450:
                continue
            circularity = 4 * np.pi * area / (perimeter ** 2)

            if 0.7 < circularity < 1.3:
                (x, y), radius = cv.minEnclosingCircle(cnt)
                print(x)
                print(depth)
                cv_image = cv.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 0), 5)
                if 315 <= x <= 345 and (depth < 3.):


                    spinning.angular.z = 0.
                    go_forward()
                    going_forward = True
                    continue

        if not going_forward:
            if (x < 320):
                spinning.angular.z = .4
            else:
                spinning.angular.z = -.18
                go_forward()
                going_forward = True
        else:
            if depth < 0.4:
                moving = False
                going_forward = False
                spinning.linear.x = 0.
                print("STOOOOOOOOOOP")

    pub.publish(spinning)
    cv.imshow("Display window", cv_image)
    cv.waitKey(1)
    pass

def main():
    rospy.init_node("realsense_camera_display")
    rospy.Subscriber("/realsense/color/image_raw", Image, found_ball)
    rospy.Subscriber("/realsense/depth/image_rect_raw", Image, get_depth)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        cv.destroyAllWindows()
    finally:
        cv.destroyAllWindows()

