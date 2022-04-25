#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample, choice

import math

from random import randint

import random

from likelihood_field import LikelihoodField

# Using the code from class!
def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

def to_rad(angle):
    return angle * math.pi / 180


def draw_random_sample(arr, n):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
  
    return choice(arr, n, replace=True)


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        self.dist_noise = 0.05
        self.angle_noise = to_rad(5)

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"


        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 500
        # 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        self.lhf = LikelihoodField()

        
        rospy.sleep(5)

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True

        '''
        
        Assume origin is top left
        
        '''



    def get_map(self, data):

        self.map = data
    

    def initialize_particle_cloud(self):
        
        # [ x ] TODO

        '''
        
            1. Set particle cloud to length 1000 to start
            2. Randomly select row, col from range (0,384) each
            3. initialize particle with row*resolution, col*resolution
            4. set weight to 1/num_particles
            5. Add to cloud
        
        
        '''

        res = self.map.info.resolution
        dim = self.map.info.width

        origin = self.map.info.origin
        origin_x, origin_y = origin.position.x, origin.position.y

        coords = []

        for i, v in enumerate(self.map.data):
            if not v:
                coords.append(i)

        coords_selected = draw_random_sample(coords, self.num_particles)
        
        for i in coords_selected:
            row = (i//dim) * res + origin_y
            col = (i%dim) * res + origin_x

            weight = 1/self.num_particles

            angle = randint(0,360) * math.pi/180

            position = Point(col, row, 0)

            quaternion = Quaternion()
            t = quaternion_from_euler(0, 0, angle)

            quaternion.x = t[0]
            quaternion.y = t[1]
            quaternion.z = t[2]
            quaternion.w = t[3]
            pose = Pose(position = position, orientation = quaternion)
            particle = Particle(pose, weight)

            self.particle_cloud.append(particle)
    
        self.normalize_particles()
        self.publish_particle_cloud()
        print("Cloud published\n")

    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        
        # [ x ] TODO
        total = 0
        for particle in self.particle_cloud:
            total += particle.w

        for index, particle in enumerate(self.particle_cloud):
            particle = self.particle_cloud[index]
            particle.w = particle.w / total
            self.particle_cloud[index] = particle
        print("normalized\n")

        s = 0
        for particle in self.particle_cloud:
            s+=particle.w
        print(s)


    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:

            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)
        print("published\n")


    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        # I changed this from rospy.Time.now()
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time(0), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)


    def resample_particles(self):

        # TODO

        probs = []

        for particle in self.particle_cloud:
            probs.append(particle.w)

        print("resample: ")
        self.particle_cloud = choice(self.particle_cloud, self.num_particles, p=probs, replace = True)
        print("resampled\n")

    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return

# I changed this
        if len(self.particle_cloud):

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose
                print("iteration\n")

    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        
        # TODO
        total_row = total_col = total_angle = 0

        for particle in self.particle_cloud:
            pose = particle.pose
            x = pose.position.x
            y = pose.position.y
            
            angle = get_yaw_from_pose(pose)
        
            total_row += x
            total_col += y
            total_angle += angle

        total_row = total_row / self.num_particles
        total_col = total_col / self.num_particles
        total_angle = total_angle / self.num_particles

        position = Point(total_col, total_row, 0)

        quaternion = Quaternion()
        t = quaternion_from_euler(0, 0, total_angle)

        quaternion.x = t[0]
        quaternion.y = t[1]
        quaternion.z = t[2]
        quaternion.w = t[3]

        particle.pose.orientation = quaternion

        pose = Pose(position = position, orientation = quaternion)
        
        self.robot_estimate = pose
        print("updated estimate\n")

    # TODO OH: iterates slowly, never finished
    def update_particle_weights_with_measurement_model(self, data):
        for index, particle in enumerate(self.particle_cloud):

            for angle, measurement in enumerate(data.ranges):
                if (angle%2):
                    continue
                
                q = 1
                x,y = particle.pose.orientation.x, particle.pose.orientation.y

                theta = get_yaw_from_pose(particle.pose)

                if measurement:

                    angle = to_rad(angle)

                    x = x + measurement * math.cos(theta + angle)
                    y = y + measurement * math.sin(theta + angle)

                    dist = self.lhf.get_closest_obstacle_distance(x,y)

                    particle.pose.orientation.x = x
                    particle.pose.orientation.y = y

                    q = q * compute_prob_zero_centered_gaussian(dist, 0.1)

                    if math.isnan(q):
                        q = 0
                    particle.w = q
                    self.particle_cloud[index] = particle

        print("updated particles with measurement model\n")
        self.normalize_particles()
        

    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        '''
        
        Using sample_motion_odometry function from slack
        - starting with 0 noise
    
        
        '''

        # TODO

        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        
        ## TODO : add noise for these three
        angle1 = math.atan2(old_y - curr_y, old_x - curr_x) - old_yaw + choice((-1,1)) * self.angle_noise
        # angle1 = angle1 % (math.pi/180)
        trans = math.sqrt((old_x - curr_x)**2 + (old_y-curr_y)**2) + choice((-1,1)) * self.dist_noise
        angle2 = curr_yaw - old_yaw - angle1 

        for index, particle in enumerate(self.particle_cloud):

            yaw = get_yaw_from_pose(particle.pose)
            particle.pose.position.x += trans * math.cos(yaw + angle1)

            particle.pose.position.y += trans * math.sin(yaw + angle1)

            yaw += (angle1 + angle2)

            quaternion = Quaternion()
            t = quaternion_from_euler(0, 0, yaw)

            quaternion.x = t[0]
            quaternion.y = t[1]
            quaternion.z = t[2]
            quaternion.w = t[3]

            particle.pose.orientation = quaternion
        
            self.particle_cloud[index] = particle
        print("updated particles with motion model\n")


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









