# particle_filter_project
## Megan Morales and Matthias Ling

## Writeup

The goal of this project was to correctly develop and implement a monte carlo particle localization guide to be able to determine a robot's orientation and location within its environment. Given a map of the robots environment, the intended purpose was to use scanner readings and different movement models to translate the posible location and movement of the robot into a probobslistic area using a particle cloud. 

### High Level Description
In order to solve the problem of particle localization we first had to identify the location of the robot. The location of the robot was found after the resampling and normalization of a made particle cloud. We first initalized a large number of particles into a 2D array that represent the robot's environment. These particles contained their own position and weights which were then updated to represent the robots movement through its environment. This update was based off the difference of the distance and orientation of the robot's old position and its new position. With the odometry topic we were provided the necessary paramaters to be able to calculate these values. Eventually, the particles were resampled and the particles with the highest weights converged to reveal the robot's true position. 

### Main Steps

1. Initialization of particle cloud
- This was done in the initialize_particle_cloud function.  
- Essentially, the function reads the dimensions of the map from the self.map and randomly chooses 10,000 coordinates based on those dimensions. It then uses those coordinates multiplied by the resolution of the map to get physical coordinates for particles and randomly selects a yaw angle.  It then appends the particles each to the self.particle_cloud list and then normalizes and publishes it.

2. Movement model
- This was done in the update_particles_with_motion_model function. 
- This function updates the current particle locations x and y position as well as its yaw. It first uses the robots old and new position, based off its odometry, to determined the angle and translation distance which the robot moves to achieve this orientation and position change. The determined angle and translation values are then used to update the particles x, y and yaw so that the particles move with the robot. 

3. Measurement model
- This was in the update_particle_weights_with_measurement_model function.
- In this function we first index into the particle cloud and iterate through the laser scan readings of the robot. Based off the laser scan readings we update the x and y location of each partile in our particle cloud using the likelihood field model and then update the weights of the particle based off a zero centered gaussian distribution. 

4. Resampling
- This was done in the resample_particles function.
- This function returns an updated particle cloud with particles that contain the highest weights. Using the random.choice() function to pick a weighted random sample of particles, we are able to update the cloud since the particles that have higher weights are the more probable to get sampled. 


5. Incorporation of noise
- This was done with the inclusion of noise variables in the particle filter object (self.dist_noise and self.dist_angle) as well as in the update_particles_with_motion_model function.
- In order to account for noise we set an arbitrary value in randians for the angle and meters for the distance and applied it to the functions above to provide an error region for our particle cloud.  

6. Updating estimated robot pose
- This was done in the update_estimated_robot_pose function.
- In this function we take the average position and orientation of the particles that are left in the particle cloud after resampling. We divide each individual x and y component by the total number of particles that we originated with as well doing the same with the yaw, which is converted from the pose structure. Thus, the position and yaw of the particle are then upated with these average values in order to dial down a more accurate estimate of the robots possible position. 
 
8. Optimization of parameters
- This was done in self.num_particles 
- To optimize paramaters we lowered the number of particles that were initialized into the particle cloud. When too many partiles were initialized into the cloud i.e 1000 or more, we found that the computation load was far too significant. By lowering the particle cloud we were not only able to better visualize the convergence of the particle cloud but our code rang significantly faster without crashing out. 

### Challenges
The hardest part was starting in many ways; our UTM was corrupted and we took some time to redownload it.  Conceptually, it was hard trying to map the theoretical idea of having coordinates and particles to the code itself, pulling those particles out of the OccupancyGrid.  Additionally, figuring out of how to implement the movement model was somewhat unintuitive as we had to understand how to move the particles geometrically according to robot movement.  Finally, one of us contracted covid, so working together remotely was hard because it made UTM run extra slowly.  

Despite all this, planning ahead and scheduling time to work routinely allowed us to chunk out the work and also ask questions along the way.  The provided functions for the motion model and guidance we received in office hours were a helpful starting point for writing the functions out.  


### Future Work
The original function we wrote analyzed the values of the OccupancyGrid.data list, and categorized the information.  There were a lot of 0s and -1s, the former representing an open space and the latter being unknown.  We could have sampled all of the 0s and then randomly selected the remaining spots from the -1s, which would be faster as it would guarantee that we were looking at as many open spots as we could rather than potentially sampling invalid spaces.

### Takeaways
* We learned that writing out a plan for a large project ahead of time was helpful in scheduling time to work.  This really helped us stay ahead of the deadline and avoid any surprises the day of.

* We also started early which allowed us to get conceptual questions out of the way and give us a lot of time for implementation.  In the future, this is helpful because a lot of the work in these projects relates to debugging and tweaking parameters, rather than conceptual concepts.

## Implementation Plan
1. How you will initialize your particle cloud (initialize_particle_cloud)?
  * Implement: For each of the 10,000 particles we'll initialize it with its position and set a weight to 0.  Then we'll append it to the particle_cloud list within the ParticleFilter class.
  * Test: To see if the array has been initialized, we'll print out the length and individual values to make sure that all 10,000 particle objects have been added.
2. How you will update the position of the particles will be updated based on the movements of the robot (update_particles_with_motion_model)?
  * Implement: When the robot moves, we can get its pose from self.laser_pose and self.odom_pose and apply it to each of the individual particles in the particle cloud list using the beam model.  That'll be written in update_particles_with_motion_model.
  * Test: We can just run one iteration and eyeball a few particles in the list to see if they've been updated as we'd expect them to.  We can also define a generic comparison function that we can map over the list once this has been checked.
3. How you will compute the importance weights of each particle after receiving the robot's laser scan data?(update_particle_weights_with_measurement_model)?
  * Implement: At this point, we'll know the robot's position and each particle's position in the cloud.  We can thus apply the measurement model from class to take the inverse of the sum of weighted differences in the robot and each particle's position.  We'll then update each particle's respective weight object accordingly.
  * Test: We can check that particles near the actual position are higher weighted than those further away.
4. How you will normalize the particles' importance weights (normalize_particles) and resample the particles (resample_particles)?
  * Implement: We'll take the sum of the particle cloud list and divide each weight of each particle in the list by the sum.
  * Test: We can save each particle's preweighted score and see that it equals the normalized weight after the calculations have been made.
5. How you will update the estimated pose of the robot (update_estimated_robot_pose)?
  * Implement: We'd record the particles with the best weights in the particle cloud and rebalance the cloud based on that.  That would cause the particles to cluster more closely and thus would update the estimated pose.
  * Test: We could get the mean of the particle cloud list in terms of coordinates and see that it moves towards the robot's prior position.  This would work as the points that are more likely to be near the robot wouldn't be resampled out like the others.
6. How you will incorporate noise into your particle filter localization?
  * Implement: We would apply the velocity to each particle but add/subtract some constant value to either side.  This would allow for some variance in our model
  * Test: We could put the particles that are generated by each original particle into a dictionary and see that they're being generated correctly by looking them up.

## Timeline
1. Initialize - 4/13
2. Movement model - 4/17
3. Measurement model - 4/17
4. Resampling - 4/18
5. Noise - 4/19 - 4/21
6. Updating Robot pose - 4/22
7. Optimization - by 4/25
