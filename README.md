# q_learning_project


Sam Nitkin and Matthias Ling

## Implementation Plan

### Q-learning algorithm
#### Executing the Q-learning algorithm

We'll implement the Q learning algorithm by implementing the formula that we got in class.  We'll set a variable for convergence, which we'll initialize to either 0 or infinity and implement a while loop that'll run as long as it's beyond a certain threshold.  Because the matrix is easily acessible by rows/columns, we can reference the subsequent values for the reward function.  We'll also set a constant learning rate.

To test this, we can simply print out one value for one action and do the math out ourselves to see if it corresponds.  This is the simple but most straightforward way to test.  We can also put the values into a dataframe so that it'll print out in a visually more readable way.

#### Determining when the Q-matrix has converged

For each action, see if Q changed. If Q does not change for 2 * number of actions iterations in a row, we will determine that the matrix has converged and conclude the algorithm.

To test this, we can look at how individual rows impact the overall convergence score and print the corresponding columns out to see if their values in fact match those that we'd expect.

#### Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward

We will use a greedy approach, the robot will take the actions with the highest Q in sequence until the task is complete.  In our Q matrix, we use the current state to determine the row and then pick the action by following the column with the highest Q value.  A greedy approach is the simplest one in our opinion.  

By printing out the matrix as a dataframe, we can see the indices of the maximum value per row.  We can also save the values into a dictionary or similar data structure to see whether or not we're actually moving along the most optimal path. 

### Robot perception
#### Determining the identities and locations of the three colored objects

Using HSV values, we will determine the relative locations of the three objects, i.e. is pink left of blue. The exact locations will then be determined using LIDAR, seeking the closest object in that direction.  

This will be mostly tested either during lab or on Gazebo.  We'll use the debugging functions provided as well.  We can see the coordinates of the objects in gazebo and see whether or not those are accurately being saved into our code.

#### Determining the identities and locations of the three AR tags
Similar to the previous step, we can use lidar to identify the locations of the tags and the camera to visually identify each tag.

We'll test this using Gazebo and the debugging functions.  Since there are only three tags, it'll be straightforward to see whether or not the robot is accurately approaching the correct tag for each colored object.

### Robot manipulation & movement

#### Picking up and putting down the colored objects with the OpenMANIPULATOR arm
We will approach the objects, grip them lightly and then angle the arm up.  We'll use Lab F as a reference point to identify the proper angles to use.

We don't have much experience using the arm, so it'll largely be trial and error, figuring out what angles and grip are optimal for the objects.

#### Navigating to the appropriate locations to pick up and put down the colored objects
At this point, we should know which locations and tags are correct for each object.  Once we have an object picked up, we'll navigate to the corresponding coordinates on the map, minus some constant to place it in front.  We'll learn the correct angles to place it down from Lab F, and release the grip once that angle's been achieved.

We can test this in the lab, and more modularly, try just picking up and putting down one object to make sure the angles are correct.


### Timeline

April 27 - Implementation Plan Due
May 3 - Q matrix intermediate deliverable complete
May 10 - Project complete

## Writeup
