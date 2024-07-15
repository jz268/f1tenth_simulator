#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# from https://github.com/f1tenth/f1tenth_labs/blob/S2023/lab4/code/src/reactive_gap_follow.py

class reactive_follow_gap:
    def __init__(self):

        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/gap_drive'

        self.max_speed = rospy.get_param("~max_speed")
        self.max_steering_angle = rospy.get_param("~max_steering_angle")

        # get car parameters

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
    
    def preprocess_lidar(self, data, lo, hi):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value tomax_steering_angle the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = np.array(data.ranges)   

        # proc_ranges[np.isnan(proc_ranges)] = 0
        # proc_ranges[np.isinf(proc_ranges)] = 0
        proc_ranges[:lo] = 0
        proc_ranges[hi:] = 0  
    
        def window_avg(x, n):
            return np.convolve(x, np.ones(n)/n, mode='same')
        proc_ranges = window_avg(proc_ranges, 5)

        rmin = max(data.range_min, 0)
        rmax = min(data.range_max, 4)
        # rmin, rmax = data.range_min, data.range_max
        np.clip(proc_ranges, rmin, rmax, out=proc_ranges)

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        lo_best, hi_best, gap_best = -1, -1, 0
        lo = 0

        for hi in range(1, len(free_space_ranges)+1):
            if free_space_ranges[hi-1] != 0:
                gap = hi - lo
                if gap > gap_best:
                    gap_best = gap
                    lo_best, hi_best = lo, hi
            else:
                lo = hi

        # lo:hi -- hi exclusive, i.e. [lo, hi)
        return lo_best, hi_best

    # def find_max_gap(self, ranges):
    #     start = 0
    #     end = 0
    #     current_start = -1
    #     duration = 0
    #     longest_duration = 0
    #     for i in range(len(ranges)):
    #         if current_start < 0:
    #             if ranges[i] > 0:
    #                 current_start = i
    #         elif ranges[i] <= 0:
    #             duration = i - current_start
    #             if duration > longest_duration:
    #                 longest_duration = duration
    #                 start = current_start
    #                 end = i - 1
    #             current_start = -1
    #     if current_start >= 0:
    #         duration = len(ranges) - current_start
    #         if duration > longest_duration:
    #             longest_duration = duration
    #             start = current_start
    #             end = len(ranges) - 1
    #     return start, end+1
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        valid_ranges = ranges[start_i:end_i]
        candidates = np.argwhere(valid_ranges == np.amax(valid_ranges)).flatten()
        center_idx = (len(ranges)-1)/2 - start_i
        winner = candidates[np.argmin(abs(candidates - center_idx))]
        return winner + start_i

    # def find_best_point(self, start_i, end_i, ranges):
    #     current_max = 0.0
    #     best_idx = -1
    #     center_idx = (len(ranges)-1)/2
    #     for i in range(start_i, end_i):
    #         if ranges[i] > current_max:
    #             current_max = ranges[i]
    #             best_idx = i
    #         elif ranges[i] == current_max:
    #             if abs(i-center_idx) < abs(best_idx-center_idx):
    #                 best_idx = i
    #     return best_idx

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        # ranges = data.ranges
        # proc_ranges = self.preprocess_lidar(ranges)

        # for i in range(len(data.ranges)):
        #     print(i, data.ranges[i])

        field_of_view = math.radians(120)
        lo = int((-field_of_view/2 - data.angle_min) / data.angle_increment)
        hi = int(( field_of_view/2 - data.angle_min) / data.angle_increment)

        #Find closest point to LiDAR

        proc_ranges = self.preprocess_lidar(data, lo, hi)

        closest_idx = lo + np.argmin(proc_ranges[lo:hi])
        closest_dist = proc_ranges[closest_idx]

        closest_angle = closest_idx * data.angle_increment + data.angle_min              
        print(closest_idx, math.degrees(closest_angle), closest_dist)

        front_dist = proc_ranges[int(len(proc_ranges)/2)]

        #Eliminate all points inside 'bubble' (set them to zero) 
        for i in range(lo, hi):
        #     # current_dist = proc_ranges[i]
        #     # angle = data.angle_increment * abs(i - closest_idx)
        #     # dist = math.sqrt(closest_dist**2+current_dist**2-2*closest_dist*current_dist*math.cos(angle))
        #     # if dist < .6:
        #     #     proc_ranges[i] = 0
            if 0.6 > closest_dist * abs(i - closest_idx) * data.angle_increment:
                proc_ranges[i] = 0
        proc_ranges[closest_idx-200:closest_idx+200] = 0

        #Find max length gap 
        lo_gap, hi_gap = self.find_max_gap(proc_ranges)
        print(lo_gap, hi_gap)

        #Find the best point in the gap 
        best_idx = self.find_best_point(lo_gap, hi_gap, proc_ranges)
        best_angle = data.angle_min + data.angle_increment * best_idx
        print(best_idx, math.degrees(best_angle), proc_ranges[best_idx])

        print(front_dist)

        print("--------------------")

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = max(min(best_angle, self.max_steering_angle), -self.max_steering_angle)
        # drive_msg.drive.steering_angle = best_angle

        speed = 1.0

        speed = 0.6
        if abs(best_angle) > math.radians(20):
            speed = min(0.2, speed)
        elif abs(best_angle) > math.radians(10):
            speed = min(0.4, speed)

        if closest_dist < 0.25:
            speed = min(0.1, speed)
        elif closest_dist < 0.3:
            speed = min(0.2, speed)
        elif closest_dist < 0.35:
            speed = min(0.3, speed)
        elif closest_dist < 0.4:
            speed = min(0.4, speed)

        # if front_dist < 0.5:
        #     speed = min(0.1, speed)
        # elif front_dist < 0.75:
        #     speed = min(0.2, speed)
        # elif front_dist < 1:
        #     speed = min(0.3, speed)
        # elif front_dist < 2:
        #     speed = min(0.5, speed)
        # elif front_dist < 2.5:
        #     speed = min(0.8, speed)

        drive_msg.drive.speed = speed


        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("gap_follower", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)