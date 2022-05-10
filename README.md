# q_learning_project


Sam Nitkin and Matthias Ling

## FINAl PRODUCT:
Because the file is too large, we shared it here on google drive:
https://drive.google.com/file/d/15-Z79cb8ayctUAH5AAwsKXrB_wpJYxxX/view?usp=sharing


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

April 28 - Work on robot in lab, complete algorithm implementation

May 3 - Q matrix intermediate deliverable complete, be able to converge, print, and save matrix

May 7 - Have Robot preceptions steps complete

May 10 - Project complete, meaning robot movement and manipulation are finalized and all project elemtents are integrated.

## Writeup

### Objective Description

The goal of this project is to properly implement the Q-learning algoirthm to understand reward based learning.

### High-level description

From the action matrix, we generate a list of viable candidate actions. We then choose an action randomly from this list and get the expected reward from the node. We then update the corresponding value of our Q-matrix based on this reward and the other parameters. When the Q-matrix has gone through 32n iterations (where n is the number of possible actions) with no change, we say it has converged and save the q matrix. 

### Q-learning algorithm

#### Selecting Actions

This code is located in a for loop in q_learning_algorithm(). Taking the current state, we loop through the possible actions and save the candidates if the action if greater than or equal to 0 in the action matrix. If there are no possible actions, we set the state back to 0. If there is a possible action, we select a new state randomly from the list of candidates.

#### Updating Q-matrix

This code is in a while loop in q_learning_algorithm(). We publish the action and wait for the reward to be plublished. Given the reward, we calculate a new q value using the Q-learning rule based on our discount and learning rate parameters and the previous q value. We take the calculated q value and updated our matrix accordingly.

#### Determining Convergence

For each new q, we check if the q-value changed. If there was no change, we iterate our ticker. If there is a change, we reset the value to of our ticker to 0. When our ticker has gotten sufficiently high (currently 2048) we say the matrix has converged and save it.

#### Executing Path

The path is determined in get_policy() after the csv is read in. Starting in state 0, the highest value is determined for the row corresponding to the state in the q matrix. The column of that value is the corresponding action. If the value is 0, it means that there are no new optimal policies and the loop ends. If instead the value is greater than 0, the action is appended to the policy and the new state is determined based on the action. When the policy is complete, the actions in the policy are looped through in do_actions()

### Robot Perception
 
 The robot precieves using the LaserScan and Image sensors. In a callback function for each sensor, we intialized the laser ranges and image and made sure not to run the rest of our code until these were intialized. find_and_face_color() and find_and_face_ar() are structured very similarly. First, they process the image and search for any pixels of the color for color or the corners of the appropriate ar tag for ar and average the Xs values. This code is based on lab 2, line follower. For the colors, HSV values were selected and tested using broad ranges. For the AR tags, is is based on the dictionary provided to us in the lab instructions. If the robot cannot see what it is looking for, it spins until it does.
 
 ### Robot Movement
  
  Once the color tube or tag's center of Xs has been identified, the robot uses proportional control to face the object based on the difference between the center of the object and the center of the image on the X-axis.
  
  
  ### Challenegs
  
  One challenge with this project was testing was quite difficult. Whether in Gazebo or with the physical turtlebot, it took a while to set up a single run, and sometimes the robot would stop working unexpectly and everything would have to be restarted. This limitation severly slow downed the debugging process and makes selecting and adjusting parameters such as movement speed more difficult.
  
 
 ### Future Work

 We could improve our color estimation - right now we're averaging all the pixels but it could be more precise.  We could be differentiating by shape, which could help because there was a lot of ambient color that affected the robot's readings.  In the future we could also refine our parameters more, making the robot move faster or navigate to directly in front of the AR tags, instead of to the wall.  We also could imcorporate ros messages.
 
 ### Takeaways
  
  This project involved working on two distinct steps. One thing that was helpful for this was organizing our code into seperate files and using messages, nodes, and launch files to properly access the different actions we wanted to do. This high level organization, along with intermediate deliverables, made this large task much more manageable. 
