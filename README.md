# q_learning_project


Sam Nitkin and Matthias Ling

## Implementation Plan

### Q-learning algorithm
#### Executing the Q-learning algorithm



#### Determining when the Q-matrix has converged

For each action, see if Q changed. If Q does not change for 2 * number of actions iterations in a row, we will determine that the matrix has converged and conclude the algorithm.

#### Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward

We will use a greedy approach, the robot will take the actions with the highest Q in sequence until the task is complete.  In our Q matrix, we use the current state to 
determine the row and then pick the action by following the column with the highest Q value. 

### Robot perception
#### Determining the identities and locations of the three colored objects

Using HSV values, we will determine the relative locations of the three objects, i.e. is pink left of blue. The exact locations will then be determined using LIDAR, seeking the closest object in that direction.  

#### Determining the identities and locations of the three AR tags

### Robot manipulation & movement

#### Picking up and putting down the colored objects with the OpenMANIPULATOR arm
We will approach the objects, grip them lightly and then angle the arm up. 

#### Navigating to the appropriate locations to pick up and put down the colored objects

### Timeline

April 27 - Implementation Plan Due
May 3 - Q matrix intermediate deliverable complete
May 10 - Project complete

## Writeup
