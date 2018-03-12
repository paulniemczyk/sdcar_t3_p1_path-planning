# Term 3, Project 1, Path Planning Project
Write-up by Paul Niemczyk
CarND-Path-Planning-Project
March 2018
   
## Notes
* The shell code here was provided by Udacity.
* I borrowed the approach from the project Q&A code and modified it to include lane changing.


## File Structure 
* All the project code was contained in src/main.cpp.  
* One file was added, spline.h, for the spline interpolation function.


## Reflection/Description

This section describes my brief overview of the code structure, and some recommendations.

### Overview
* Some helper functions are included in the shell code, including translation between the map's cartesian coordinate system and Frenet coordinates.
* There are 3 lanes, each 4m wide. Lane 0 is the inner-most left lane; lane 2 is the right-most lane
* The sim updates at 50 fps, or every 0.02 seconds. The car advances to a new waypoint every cycle. Hence, much of the velocity control in the code here is predicated on the idea of setting waypoints for the sim at distances so that each cycle advances the car within the desired speed limit.
* The max speed limit on the track is 50mph, so we set the car's top speed to 49.5mph.
* We want to minimize the jerk, the derivative of acceleration, so we want smooth acceleration fore/aft and laterally when changing lanes.
* Building a lane changing trajectory based on points off of the PRIOR trajectory helps to keep a smooth, low-jerk trajectory.

* To create the trajectory, we identify 5 waypoints from the car to some location out in the distance where we want to be. We fit those points to a spline, and then build a trajectory using that spline. 
* The car then scans around and identifies if it has a car in front of it and to the sides and acts accordingly. (See below)


### Seeing ahead & changing lanes
* We start by looping through all cars from the sensor fusion list...
  * Calculate the lane number by using Frenet "d" and that lanes are 4m wide
  * If there is a car in our lane, see if it's closer than 30m ahead
  * If it's within 30m, prepare to PASS...
    * If we're in lane 1 or 2 (not in the inner-most lane):
      * Check all cars again and identify if there are cars in the lane left of us, AND within a range of 5m BEHIND US to 30m AHEAD of us
        * If it's all clear, change the desired lane by decrementing the lane counter "lane--". (Note the lane shift doesn't occur until trajectory generation).
        * If it's not all clear, slow down by 0.02 but don't change lanes
    * If we're in lane (inner-most lane):
      * Repeat the above process, but looking for cars in the lane right of us, and advance the lane counter "lane++". (Note the lane shift doesn't occur until trajectory generation).
    * **NOTE**: I designed the car to only pass in lane 0 or lane 1, not to pass in lane 2.
      * In other words, if ego is in lane 0 or lane 1 and there is nowhere to pass in lane 1 or 0 (respectively, don't pass; don't pass in lane 2
      * I did this b/c of a pet peeve of mine to never pass in the right-most lane; i.e., the "slow lane"
* This worked exceptionally well even though it's so simple. A finite state machine and cost functions for each transition would be more elegant, but this worked very well in this case.


### Waypoint generation
* The code for this section came from the Q&A session.
* We create 5 evenly spaced waypoints 30m apart.
* prev_size is the number of points in the PRIOR trajectory that have NOT been travered by the car.
  * Remember that the car traverses multiple points between updates.
* If prev_size is almost empty (<2), start with the previous car point and create new points going forward.
  * Otherwise, there are many points; use the last two points as the starting point for new points.
* 3 new points are created, evenly spaced at 30m, 60m, and 90m out.
  * The 3 new points are created with a d value that puts the car in the NEWLY intended lane, based on whether "lane" was incremented or decremented above.
* Transform (shift/rotate) all 5 points into the car's reference coordinate system.



### Trajectory Generation with spline
* We take the 5 waypoints and fit them to a spline.
* The spline is now the trajectory of the car.
* We break up the spline into equal steps, and then generate enough points to add onto the previous trajectory to total 50 steps
  * For example, if the prior cycle covered 3 of the 50 points, then we only need to add 3 points this cycle
  * This keeps the trajectory nice and smooth
* Since the trajectory is a curved line in cartesian coordinates, we approx the curve as the hypotenuse of a triangle and and calculate the steps that way.
* Once we have the next points calculated, we shift/rotate them back to the map's cartesian coordinate system and push them back onto the stack next_x_vals and next_y_vals to send to the simulator. 


### Failure Cases
This code definitely isn't foolproof. It works because:
* Most cars are driving around the same speed as each other
* They don't change lanes too erratically (but occassionally do)
* Most cars stay nicely in their lanes
* Lanes are all exactly 4m wide in all places
* The curvature of the road isn't so extreme that the transform from cartesian to Frenet is too off

We will definitely collide in a few scenarios:
* Car behind us whips around us into the same lane as we are attempting to change into for a passing procedure
* Car next to us is a little out of its lane
* A car behind us to the right/left suddenly accelerates as we attempt to pass
* A car in front of us slams on its brakes and stops so quickly that our pre-programmed step-wise deceleration doesn't stop the car in time



### Potential Improvements
Now that I understand this project, I would love to improve it in a few ways:
* **Finite State Machine.** My if statements form a crude hardcoded state machine, but I would prefer to rebuild this into a FSM that keeps tracks of states including Follow Car, Prepare to Change Lane Right/Left, and Change Lane Right/Left.

* **Cost Functions.** Coupled with an FSM, it would be an elegant solution to build cost functions for each transition to penalize unwanted behaviors, and use those to programmatically decide what to do.  This allows for more modular code that allows us to alter behaviors within the FSM.

* **Quintic Polynomial.** The spline was interesting, but I will go back and attempt to implement the same functin using the lessons on quintic polynomial fitting, since they addressed the element of jerk reduction. 

## Thank You
This was a good project. Thanks for your time in reading this paper.

- Paul Niemczyk



