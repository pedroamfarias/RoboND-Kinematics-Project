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

[forward_kinematics_demo]: ./misc_images/forward_kinematics_DEMO.PNG
[forward_kinematics_DH-table]: ./misc_images/forward_kinematics_DH-table.PNG
[gazebo01]: ./misc_images/gazebo01.PNG
[gazebo02]: ./misc_images/gazebo02.PNG
[gazebo03]: ./misc_images/gazebo03.PNG
[gazebo04]: ./misc_images/gazebo04.PNG
[gazebo05]: ./misc_images/gazebo05.PNG
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

    Printscreen from Virtual machine running RViz and evaluate the kr210 kinematics to help to derive DH parameters.

![alt text][forward_kinematics_demo]

    Doing the same analyses made in "KR210 Forward Kinematcs 1/2/3", it's possible to extract DH parameters like image below:
![alt text][forward_kinematics_DH-table]

    and final DH parameter table:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | q2 - pi/2
2->3 | 0 | 1.25 | 0 | q3
3->4 | -p1/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Defining DH table:

```python
# Modified DH params:
DH_Table = {alpha0:      0, a0:      0, d1:  0.75, q1: q1,
            alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2.+q2,
            alpha2:      0, a2:   1.25, d3:     0, q3:        q3,
            alpha3: -pi/2., a3: -0.054, d4:  1.50, q4:        q4,
            alpha4:  pi/2., a4:      0, d5:     0, q5:        q5,
            alpha5: -pi/2., a5:      0, d6:     0, q6:        q6,
            alpha6:      0, a6:      0, d7: 0.303, q7:         0}

``` 

Creating individual transformations matrices to each joing:

```python
    # function that will return homogeneous transform matrix
    def TF_Mat(alpha, a, d, q):
    TF = Matrix([[            cos(q),           -sin(q),           0,             a],
                 [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                 [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                 [                 0,                 0,           0,             1]])
    return TF
```
Create individual transformation matrices (applying DH parameters into Transform Matrix )

```python
    # create individual transf. matrices to each joing
T0_1 = TF_Mat(alpha0, a0, d1, q1).subs(dh)
T1_2 = TF_Mat(alpha1, a1, d2, q2).subs(dh)
T2_3 = TF_Mat(alpha2, a2, d3, q3).subs(dh)
T3_4 = TF_Mat(alpha3, a3, d4, q4).subs(dh)
T4_5 = TF_Mat(alpha4, a4, d5, q5).subs(dh)
T5_6 = TF_Mat(alpha5, a5, d6, q6).subs(dh)
T6_EE = TF_Mat(alpha6, a6, d7, q7).subs(dh)

```

and Composition of all transforms:

```python
    # Composition of all transforms:

    T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE 

```

As a difference between Gripper T6_EE in URDF and DH parameter, it's need to rotate around Z and Y frame:

```python
    # Rotation Matrices in Z and Y
R_y = Matrix([[ cos(-np.pi/2),           0, sin(-np.pi/2), 0],
              [             0,           1,             0, 0],
              [-sin(-np.pi/2),           0, cos(-np.pi/2), 0],
              [             0,           0,             0, 1]])

R_z = Matrix([[    cos(np.pi), -sin(np.pi),             0, 0],
              [    sin(np.pi),  cos(np.pi),             0, 0],
              [             0,           0,             1, 0],
              [             0,           0,             0, 1]])

# Apply the correction because of URDF vs DH
R_corr = (R_z * R_y)

T_total= (T0_EE * R_corr)

```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

![alt text][image2]

```python
    # Rotation Matrices to correct Gripper in URDF vs DH Convention.
    r, p, y = symbols('r p y')

    ROT_x = Matrix([[     1,       0,       0],
                  [       0,  cos(r), -sin(r)],
                  [       0,  sin(r),  cos(r) ]])

    ROT_y = Matrix([[cos(p),       0,  sin(p)],
                  [       0,       1,       0 ],
                  [ -sin(p),       0,  cos(p) ]])

    ROT_Z = Matrix([[ cos(y), -sin(y),       0],
                  [   sin(y),  cos(y),       0 ],
                  [        0,       0,       1 ]])                  

    ROT_EE = ROT_z * ROT_y * ROT_x

    # Apply correct to EE
    # 180º in Z axis and -90º in Y axis to match gripper with DH parameters.

    Rot_Error = ROT_Z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

    ROT_EE = ROT_EE * Rot_Error
    ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

    #Get EE cartesian position in matrix
    EE = Matrix([[px],
                 [py],
                 [pz]])

    # Get wrist center from corrected EE          
    WC = EE - (0.303) * ROT_EE[:,2]


    # Calculate joint angles using Geometric IK method
    # as help: https://www.youtube.com/watch?time_continue=1&v=o9HDo3I0arE
    # and https://www.youtube.com/watch?v=Gt8DRm-REt4

    theta1 = atan2(WC[1],WC[0])

    side_a = 1.501
    side_b = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
    side_c = 1.25

    angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
    angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
    angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

    theta2 = pi/2 - a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
    theta3 = pi/2 - (b+0.036)

    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3:theta3})

    R3_6 = R0_3.transpose() * ROT_EE

    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

```


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


To fill the "IK_server.py" I used the "Kinematics Project Walkthrough" youtube video to help me. 
Additional comments are in "IK_server.py" code.
Observed that in some cases the WC do some unotimized moviments like turn 2 times the WC before go to basket. 

Github links that helped me too in this task:

https://github.com/Vicuko/RoboND-KinematicsProject/blob/master/writeup.md
https://github.com/mkhuthir/RoboND-Kinematics-Project
https://github.com/NitishPuri/RoboND-Kinematics-Project


![alt text][gazebo01]
![alt text][gazebo02]
![alt text][gazebo03]
![alt text][gazebo04]
![alt text][gazebo05]




