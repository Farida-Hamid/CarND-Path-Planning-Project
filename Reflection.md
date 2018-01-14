
# Path Planning
### Reflection
---

## Rubric Points


##### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.


## Compilation


### The code compiles correctly:
 I had to change line 19 in CMakeLists.txt from `link_directories(/usr/local/Cellar/libuv/1.11.0/lib)` to `link_directories(/usr/local/Cellar/libuv/1.18.0/lib)` because I tried to install the older version of libuv using brew but couldn’t. I asked if this were a problem in the forums and was told it's *not*.


## Valid Trajectories:
### Goals achieved:
- **The car is able to drive at least 4.32 miles without incident..**

- **The car drives according to the speed limit.** The car increases its speed if the car in front is far enough and the speed is 49.5 (lines 352-354), and especially increase the speed if it were turning not to hit other cars (lines 337-339). Velocity is converted and calculated (line 450), then incorporated in X and Y calculations (lines 451 - 453). 

- **Max Acceleration and Jerk are not Exceeded:** Speed is smoothly increased to the maximum if (i.e. when the flag `slow_down` is set to true) the car in front is far enough *(lines 352-353)*. 

- **Car does not have collisions:** The car slows down if the car in front are too close, and only changes lanes if the car on the new lane is far enough.

- **The car stays in its lane, except for the time between changing lanes:** This is achieved by using Frenet coordinates. Smooth calculations of the path using the spline library incorporate `lane` (lines 399-401).

- **The car is able to change lanes:** The car changes to the lane with lower cost if the car in front is far enough (lines 335-350). *I found it easier to change to the lane with the furthest car, so the the car will change to the lane with the biggest of `cost_R` and `cost_L`.*

---
## Reflections

**Overall explanation of my code:**
1. Unless we need to wait because the car just changed lanes, the code goes through each car in the `sensor_fusion` vector (starting at line 262).
     1. If the car were in the same lane, it is considered for being close enough to take action (line 267). If the front car was closer than 50 S units, the `too_close` flag is set to true. If the car was even closer than 15 S units, the `slow_sown` flag is set to true (lines 278-284).
     2. If the car were in a neighboring lane, the distance between our car and the closest car in the possible lane/s is calculated (lines 268-309).
     3. Set the cost of turning left and turning right to the distance of the car in that lane. Set the cost of a lane to zero if it were not recommended, otherwise the cost will be maximum (lines 314-327).
     4. Adjust the `two_cars_not_close`, flag if needed (lines 324-327).
     5. Check if the `too_close` flag is true to change lanes or slow down (line 329).
     6. slow down, if needed (lines 332-333).
     7. If needed change lanes (lines 335-350).
     8. Speed up to the maximum speed, if needed (lines 352-354).
2. If the lane had just changes, wait for 77 frames for the car to settle in the new lane (lines 357-360).
3. Build the path using the spline library as mentioned in the walkthrough video (lines 363-469).

### How the main problems were adressed:


**Accelerating/Decelerating with minimum jerk:**

- When the car needed to change lane after it slowed down it hit the neighboring car. So, it needed to accelerate before changing lanes if it wasn't fast enough *(lines 337-340)*.

- The `slow_down` flag is set to true when the car in front is less than 15 S units close *(lines 282-283)*. The car will slow down if the cars in all possible lanes are less than 10 S units far from each-other, otherwise the car will change between lanes too frequently and face the same problem in the new lane.

**Changing lanes too quickly:**
- In some cases the car would move to a lane with a lower cost then find that the next lane has an even lower cost. This means that the car could decide to go from lane 0 to lane 1 and then to lane 2 in only 2 frames. In this case the car will decide to move to lane 2 before it gets to lane 1 which increases jerk to more than the maximum allowed.
- To avoid this, a counter, `waiting` is added to wait for 77 frames until the car is settled in the new lane before making any calculations that may end up changing the lane (lines 357-360).

**When the cars in front are too close to each other that they have close costs:**
- If the car was in a side lane (i.e. lanes 0 or 2) and the neighboring car is too close, the car will give priority to being in lane 1 since it would give more options. This way we only need to consider the neighboring possible lane/s but not the furthest lane.
- The flag `two_cars_not_close = false` only if the cars in the only 2 possible lanes are less than 12 S units close from each other and the car is in lane 1 *(lines 324-327)*, in such case the car will slow down until it can move to another lane, otherwise it will move to lane 1 for better options. This is achieved by checking the `two_cars_not_close` flag when slowing down (line 332) or changing lanes (line 335).

---
###### *Note:*  Almost all the suggestions in the walkthrough video were used in building this project.
