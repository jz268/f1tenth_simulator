#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/gap_drive'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) 
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10) 

        # get car parameters
        self.max_speed = rospy.get_param("~max_speed")
        self.max_steering_angle = rospy.get_param("~max_steering_angle")

        self.lo = None
        self.hi = None
        self.last_closest_dist = float('inf')
    
    def preprocess_lidar(self, ranges, data, lo, hi):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
                1.Setting each value to the mean over some window
                2.Rejecting high values (eg. > 3m)
        """
        # numpy array overhead not worth it for this node?
        # proc_ranges = list(ranges)

        ranges[:lo] = [0]*lo
        ranges[hi:] = [0]*(len(ranges)-hi)

        # proc_ranges[proc_ranges > data.range_max] = data.range_max
        # proc_ranges[proc_ranges < data.range_min] = data.range_min
        ranges[ranges > 3] = 3

        # def window_avg(x, n):
        #     return np.convolve(x, np.ones(n)/n, mode='same')
        # proc_ranges = window_avg(proc_ranges, 5)

        return ranges
    
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
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        # valid_ranges = ranges[start_i:end_i]
        # candidates = np.argwhere(valid_ranges == np.amax(valid_ranges)).flatten()
        # center_idx = (len(ranges)-1)/2 - start_i
        # winner = candidates[np.argmin(abs(candidates - center_idx))]
        # return winner + start_i
        best_idx, best_val = -1, 0
        center_idx = (len(ranges)-1)/2
        for i in range(start_i, end_i):
            if ranges[i] > best_val:
                best_idx, best_val = i, ranges[i]
            elif ranges[i] == best_val:
                if abs(i-center_idx) < abs(best_idx-center_idx):
                    best_idx = i
        return best_idx

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = list(data.ranges)

        if self.lo is None or self.hi is None:
            field_of_view = math.radians(140)
            self.lo = int((-field_of_view/2 - data.angle_min) / data.angle_increment)
            self.hi = int(( field_of_view/2 - data.angle_min) / data.angle_increment)

        front_dist = ranges[int(len(ranges)/2)]

        #Find closest point to LiDAR
        closest_dist = min(ranges)
        closest_idx = ranges.index(closest_dist)
	
        #Eliminate all points inside 'bubble' (set them to zero) 
        # radius = 216
        # ranges[closest_idx-radius: closest_idx+radius] = [0]*(2*radius)
        
        for i in range(self.lo, self.hi):
            if 0.66 > closest_dist * abs(i - closest_idx) * data.angle_increment:
                ranges[i] = 0
                
        # preprocessing mutates ranges list
        self.preprocess_lidar(ranges, data, self.lo, self.hi)

        #Find max length gap 
        lo_gap, hi_gap = self.find_max_gap(ranges)
            
        #Find the best point in the gap 
        best_idx = self.find_best_point(lo_gap, hi_gap, ranges) 
        best_angle = data.angle_min + data.angle_increment * best_idx

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        
        speed = 6.5 - 3 * max(0, 2 - front_dist)

        if closest_dist < 0.5 and closest_dist < self.last_closest_dist + 0.01:
            if abs(best_angle) > math.radians(20):
                speed = min(1.0, speed)
            elif abs(best_angle) > math.radians(15):
                speed = min(2.5, speed)
        else:
            if abs(best_angle) > math.radians(20):
                speed = min(2.0, speed)
            elif abs(best_angle) > math.radians(15):
                speed = min(5.0, speed)

        # might be susceptible to noise
        # if closest_dist < 0.5 and closest_dist < self.last_closest_dist:
        #     speed = min(1.0, speed)
        
        # if abs(best_angle) < math.radians(1):
        #     best_angle = 0

        clip_speed = min(speed, self.max_speed)
        clip_angle = max(min(best_angle, self.max_steering_angle), -self.max_steering_angle)

        drive_msg.drive.speed = clip_speed
        drive_msg.drive.steering_angle = clip_angle

        self.drive_pub.publish(drive_msg)

        self.last_closest_dist = closest_dist


def main(args):
    rospy.init_node("gap_follower", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)