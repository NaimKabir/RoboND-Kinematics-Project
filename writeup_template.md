## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[jointorigins]: ./misc_images/jointorigins.png
[DHorigins]: ./misc_images/DHorigins.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Looking at the kr210.urdf.xacro file, we can start building out coordinates for the location of every joint's center in space. After doing some addition of positional changes from joint to joint, we can see that these coordinates are as shown below:

![Joint coordinates as given by the urdf.][jointorigins]

However the joint origins we want to set should mostly zero out all the Denavit-Hartenberg parameters we're dealing with. So we can make a few translations of each joint's 'origin' to make the problem easy for us (by collapsing as many link lengths and joint offsets to 0!). I do that below, by simply shifting coordinates such that they line up as much as possible.

![DH origins of each joint.][DHorigins]

With this new set of joint origins, and the **important assumption that the Z-axis for each joint's reference frame is its axis of rotation**, we can derive the following some DH parameters.

Starting with Joint 1, we can see that its Z-axis is coincident with the base axis, so twist angles between the two are 0 as are the link lengths. However, the X-axes for both are offset by 0.75. It's angle of rotation will be the variable θ<sub>1</sub>.

Joint 2: Remembering that the Z-axis is actually shooting into the page (the axis of this joint's rotation), we can see that the Z<sub>1</sub> would have had to rotate clockwise about X<sub>1</sub> by 90° to match Z<sub>2</sub>, making its twist angle -90°. The Z's are also offset by Joint 2's positional change along the GLOBAL X-axis (to the right of the image), a link length of 0.35. There is no offset between X<sub>1</sub> and X<sub>2</sub> however, so the d<sub>2</sub> is 0. The angle of the joint here is a little tricky: it starts off with a 90° offset between X<sub>1</sub> and X<sub>2</sub> (X<sub>2</sub> is shooting upward along the global Rviz Z-axis). Because of this, any additional counterclockwise turn will 'eat' into this inital 90°. Thus the angle of joint 2 can be described as the variable: θ<sub>2</sub> - 90°.

We can follow this same kind of spatial reasoning for each of the joints, finally compiling the table below:

joint | α<sub>i-1<sub> | a<sub>i-1<sub> | d<sub>i</sub> | θ<sub>i</sub>
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | θ<sub>1</sub>
2 | -90° | 0.35 | 0 | θ<sub>2</sub> - 90°
3 | 0 | 1.25 | 0 | θ<sub>3</sub> 
4 | -90° | -0.054 | 1.5 | θ<sub>4</sub>
5 |  90° |  0 |  0  | θ<sub>5</sub>
6 |  -90°   | 0 |    0   | θ<sub>6</sub>
gripper | 0 | 0 | 0.303 | 0

The last set of parameters for the gripper (not shown in the figures) was found by adding the X-offset between the gripper and link 6 (`0.11`) and the X-offset between link-6 and the DH origin of joint 6 (`0.193`).

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The generalized transformation between joints, as parametrized by DH parameters, takes this form, with *c* being a cosine function and *s* as a sine:

<sup>i-1</sup><sub>i</sub>**T** =

 .| .| . | .
--- | --- | --- | ---
cθ<sub>i</sub> | -sθ<sub>i</sub> | 0 |  a<sub>i-1</sub> 
 sθ<sub>i</sub> * cα<sub>i-1</sub> |cθ<sub>i</sub> * cα<sub>i-1</sub> |   -sα<sub>i-1</sub>   |-sα<sub>i-1</sub> * d<sub>i</sub> 
 sθ<sub>i</sub> * sα<sub>i-1</sub> | cθ<sub>i</sub> * sα<sub>i-1</sub> | cα<sub>i-1</sub>  | cα<sub>i-1</sub> * d<sub>i</sub>
 0 | 0| 0 | 1
 
To create the transformation matrices between each link, all we have to do is substitute our known values.

 <sup>0</sup><sub>1</sub>**T** =

 .| .| . | .
--- | --- | --- | ---
cθ<sub>1</sub> | -sθ<sub>1</sub> | 0 |  0 
 sθ<sub>1</sub>  |cθ<sub>1</sub>  |   0   | 0
 0 | 0 | 1 | 0.75
 0 | 0| 0 | 1
 
<sup>1</sup><sub>2</sub>**T** =

 .| .| . | .
--- | --- | --- | ---
c(θ<sub>2</sub> - 90°)  | -s(θ<sub>2</sub> - 90°) | 0 |  0.35 
 0 |0 |   1   | 0 
 -s(θ<sub>2</sub> - 90°)  | -c(θ<sub>2</sub> - 90°) | 0  | 0
 0 | 0| 0 | 1

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd




#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


