# CarND-Path-Planning-Project
Udacity Self-Driving Car Engineer Nanodegree Program

---
### Model Documentation Overview
---

This is the first project of term 3 in Udacity's SDCND program and
I would like to start by saying that this was one of the most challenging projects to date for various reasons. Because of this, I found myself like many others watching the walkthrough Q&A with David and Aaron then following along with them. The examples provided as well as some of the written course material was enough to get me started and familiar with the way in which the simulator worked. This allowed me to begin this project by having the vehicle drive itself down one lane in the simulator at a desired vehicle speed. From that point I began to develope my model.

---
## Waypoint Generation
---

* To recap, the "starter code" begins by using the cars xy position and two engineered xy points just behind the car (Lines 274-284 `main.cpp`) as the beginning of the car's trajectory. If there were any previous points from a previous iteration of the simulator we can instead use two of those points to create the beginning of a new trajectory (Lines 286-299 `main.cpp`). This step is important because it allows a smooth transition of waypoint paths between process iterations by using common begin/end points.

* Next, now that there is a starting point, we can move out an arbitrary value in freenet s coordinates (30, 60, and 90 meters) along with the relevant d coordinate for the lane we want to be in. From there, the returned sd coordinates can be converted into the global xy coordinates to find points ahead in the lane we want to be in based on the map waypoint information provided (Lines 305-307 `main.cpp`).

* With the new xy coordinates for the lane we want to be in ahead and the previously saved xy coordinates we now have a rough waypoint path for the desired lane (5 coordinates total). From here, we shift and rotate the cars reference from global to local space (Lines 318-324 `main.cpp`). Then fit our 5 points with a spline to create a jerk minimizing curve that the car can physically follow (Line 328 `main.cpp`).

* Once the spline has been fit, we can grab points along that spline to create the waypoint path for the simulator using the formula Aaron provided in the Q&A walkthough. The provided path generation formula is used because it creates the simulator path correctly for the desired vehicle speed in the cars local space. Once the points have been created from the fitted spline the returned xy points are shifted back in to global coordinate space and added to the simulator's next waypoint path (Lines 342-263). This allows the car to travel at the specified speed along the generated path.

---
## Finite State Machine
---

Having a car that can drive down one lane in the simulator isn't very useful, but now that I was capable of getting the car to move at all, I felt ready to tackle driving through traffic efficiently.

Before I started I set a goal of having the car incentivised to maintain the speed limit when possible, and pass any slower moving traffic when safe to do so.

The first step was following other vehicles.
I decided to start with a keep lane state that would hold the lane the vehicle was driving in and follow any other slower moving vehicles ahead when needed. To work with the simulator, I opted to use an object oriented approach. All of the code that maintains the vehicle parameters and states is located in the DataVariables class in `datavariables.cpp`.

### 1. KeepLane()

* The keep lane state method is located on lines 34-114 of `datavariables.cpp` and simply drives down a lane and scans ahead for any vehicles that are within a certain distance of the car. When no vehicle is detected the vehicle will drive the speed limit. If a vehicle ahead is detected, the car will take one of three actions.

* First there is a check for a vehicle that is so close to us that it could cause a crash, if that vehicle is dangerously close a max braking condition is applied by decreasing vehicle speed in 0.4 increments. This yields close to a 10m/s jerk, or the stopping limits of the vehicle.

* If the vehicle ahead is within a normal following distance, we just maintain a safe distance using the second and third options (increase/decrease speed). The speed is adjusted proportionately to the difference in car speed and followed vehicle speed. This allows for more matched acceleration and braking when needed, a small delta distance is also factored in as this helps keep the following distance closer to the specified buffer.

* When following another vehicle and also traveling slower then the speed limit a flag is set on line 76 of `datavariables.cpp`. This flag signals the PrepareLaneChange() method to prepare to pass the slow vehicle.

### 2. PrepareLaneChange()

* The prepare for lane change method is located on lines 117-175 of `datavariables.cpp` and essentially gathers information about the vehicle's surroundings. In particular, it searches for vehicles that could cause an accident if we change lanes as well as any vehicles that are not in a potential path directly but near us and may affect a lane decision.

* The lane checking is accomplished by finding all other available lanes. From there, a scan is performed in those lanes ahead of and behind the car's current position. This search identifies gaps that our car can fit into if it was to change lanes. Based on the distance from the car, cars are either "tracked" or "nearby" (Lines 162-172 `datavariables.cpp`).

* Once all the information has been gathered about the vehicle's surroundings, it is fed into the ChooseLane() method on line 178 of `datavariables.cpp`

### 3. ChooseLane()

* The choose lane method is located on lines 217-286 of `datavariables.cpp` and is responsible for choosing a lane that  is better then the slow lane we are in following another car. It receives information about the available lanes and the vehicles in them that are close to our car's position on the road.

* The first step is to calculate a cost for the lane we are currently in. This is just a simple sigmoid function based on the car's current speed (Line 225 `datavaribles.cpp`).

* To find the best lane we have to filter the lane list and weigh the pros and cons of it. To start we only want to switch into lanes adjacent to us that have no vehicles in them that could cause a collision. This check is performed on line 235 of `datavariable.cpp`, if the lane is clear of vehicles and adjacent to us we can start to look at other nearby vehicles that will be in that lane if we were to switch in to it. Its important to check these vehicles because it wouldnt make sense to switch into a lane if the vehicles in it are moving slower then we currently are.

* If there are no nearby vehicles in the lane its safe to say that this is going to be the best lane because there is no other relevant traffic, so the best lane is updated. If there are other nearby vehicles, all of the vehicle speeds are recorded so an average speed for the lane can be calculated and can be taken into account for the best lane choice. (Lines 243-270 `datavariables.cpp`).

* Once the average speed for the lane is calculated, the average is fed into another cost function that adds a penalty weight of 0.05 for traffic and a multiplier of 0.02 for average speed. The weights were chosen through trial and error as well as some experimentation on paper. I'd like to note that creating a cost function that provided the desired behavior was probably the most frustrating part of this project. Minor changes to the cost functions completely changed lane changing behavior in certain situations. The function and values used were the result of a lot of trial and error.

* Once a cost has been calculated, the cost and average speed is compared to decided if the current lane being examined is better then the lane the vehicle is presently driving in (Line 276 `datavariables.cpp`). If things look better, the best lane is updated.

* If all lanes prove to be worse then the one we are currently  driving in, we dont update the vehicle's lane and just continue driving until a better lane is available.

* Once the best lane is returned, prepare lane change passes the identified lane to LaneChange() on line 213 of `datavariables.cpp`.

### 4. Lane(Change()

* The lane change method is located on line 289 of `datavariables.cpp` and updates the car's lane variable with the lane argument provided. This begins a lane transition sequence if the lane value is changed from the previous.

---
## Lane Transitions
---

When the car's lane variable has been switched, the new lane number alters the ref_d value that is calculated in `main.cpp` on line 302. This change reflects the lane change by altering the next spline that is generated from the 5 points previously mentioned in the Waypoint Generation section above.

The 5 point vector that is used to fit a spline in `main.cpp` line 328 starts by using previous waypoint path values as a base point. Once the lane is changed the next three future points generated on lines 305-307 in `main.cpp` are now over in the new lane because ref_d has been updated.

This means that you end up with two points in the old lane and three points in the new lane which then get fit with a spline to create an "S" like curve between the old and new lane. A new path is then generated along this "S" shaped spline that takes the car into the new lane.

---
## Summary
---

In conclusion, the vehicle is incentivised to do the speed limit at all times. The only exception to that is if there is traffic in the way. If we can get around the traffic we try to do so as long as it is safe to change lanes and pass.

In retrospect I would have done a few things differently knowing what I know now, in particular, if I had more time I would have liked to make a more robust cost function for lane decisions. Overall, I found the project to be very interesting and challenging. I intend to see if I can make a more advanced algorithm in the future that could possibly brake to create more opportunity for lane changes and/or better match speeds in other lanes to avoiding cutting cars off.










