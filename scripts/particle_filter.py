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

from random import randint, random

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


def draw_random_sample():
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    # TODO
    return


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        self.noise = 0

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
        self.num_particles = 1000
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

        print(res, dim)

        res = self.map.info.resolution
        dim = self.map.info.width

        print(res, dim)

        for i in range(self.num_particles):
            row = randint(0,dim) * res
            col = randint(0,dim) * res
            weight = 1/self.num_particles

            angle = randint(0,360) * math.pi/180

            pose = Pose(position = Quaternion(row, col, 0, angle))
            particle = Particle(pose, weight)

            self.particle_cloud.append(particle)


        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        
        # [ x ] TODO
        total = 0
        for particle in self.particle_cloud:
            total += particle.w

        for index, particle in self.particle_cloud:
            self.particle_cloud[index].w = particle.w / total 
        



    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):

        # TODO

        probs = [particle.w for particle in self.particle_cloud]

        self.particle_cloud = choice(self.num_particles, self.num_particles, probs)

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


        if self.particle_cloud:

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



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        
        # TODO
        return False


    
    def update_particle_weights_with_measurement_model(self, data):

        # TODO
        def to_rad(angle):
            return angle * math.pi / 180

        for index, particle in enumerate(self.particle_cloud):
            for angle in enumerate(data.ranges):
                
                measurement = data.ranges[angle]

                q = 1
                x,y = particle.pose.orientation.x, particle.pose.orientation.y
                theta = euler_from_quaternion([
                    particle.pose.orientation.x, 
                    particle.pose.orientation.y, 
                    particle.pose.orientation.z, 
                    particle.pose.orientation.w])[2]

                if measurement:

                    angle = to_rad(angle)

                    x = x + measurement * math.cos(theta + angle)
                    y = y + measurement * math.sin(theta + angle)

                    lhf = LikelihoodField()

                    dist = lhf.get_closest_obstacle_distance(x,y)

                    q = q * compute_prob_zero_centered_gaussian(dist, 0.1)

                    particle.w = q
                    self.particle_cloud[index] = particle


        

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
        angle1 = np.arctan([old_y - curr_y, old_x - curr_x]) - old_yaw 
        trans = math.sqrt((old_x - curr_x)**2 + (old_y-curr_y)**2) 
        angle2 = curr_yaw - old_yaw - angle1 

        for index, particle in self.particle_cloud:

            yaw = get_yaw_from_pose(particle.pose)
            particle.pose.x += trans * math.cos(yaw + angle1)

            particle.pose.y += trans * math.sin(yaw + angle1)

            particle.pose.yaw += (angle1 + angle2)
        
            self.particle_cloud[index] = particle


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









