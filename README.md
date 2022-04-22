# particle_filter_project
## Megan Morales and Matthias Ling

## Writeup

The goal of this project was to guide a robot through a maze using a map of its environment and scanner readings.  We achieved this goal using monte carlo particle localization to estimate the robot's position as well as a pathfinding algorithm to navigate through it.

### High Level Description

### Main Steps

1. e
2. 

### Challenges
The hardest part was starting in many ways; our UTM was corrupted and we took some time to redownload it.  Conceptually, it was hard trying to map the theoretical idea of having coordinates and particles to the code itself, pulling those particles out of the OccupancyGrid.  Additionally, figuring out of how to implement the movement model was somewhat unintuitive as we had to understand how to move the particles geometrically according to robot movement.  Finally, one of us contracted covid, so working together remotely was hard because it made UTM run extra slowly.  

Despite all this, planning ahead and scheduling time to work routinely allowed us to chunk out the work and also ask questions along the way.  The provided functions for the motion model and guidance we received in office hours were a helpful starting point for writing the functions out.  


### Future Work
The original function we wrote analyzed the values of the OccupancyGrid.data list, and categorized the information.  There were a lot of 0s and -1s, the former representing an open space and the latter being unknown.  We could have sampled all of the 0s and then randomly selected the remaining spots from the -1s, which would be faster as it would guarantee that we were looking at as many open spots as we could rather than potentially sampling invalid spaces.

### Takeaways
We learned that writing out a plan for a large project ahead of time was helpful in scheduling time to work.  This really helped us stay ahead of the deadline and avoid any surprises the day of.

We also started early which allowed us to get conceptual questions out of the way and give us a lot of time for implementation.  In the future, this is helpful because a lot of the work in these projects relates to debugging and tweaking parameters, rather than conceptual concepts.

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
