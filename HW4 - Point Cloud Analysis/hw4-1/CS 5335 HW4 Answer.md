# CS 5335 HW 4

### q1_a

fit a plane by calculating the sample mean and covariance matrix of the points:



![image-20230221211208223](/Users/huang/Desktop/5335 Robotic/HW4/hw4-1/assets/image-20230221211208223.png)



### q1_b

fit a plane with some outliers

![image-20230221211741474](/Users/huang/Desktop/5335 Robotic/HW4/hw4-1/assets/image-20230221211741474.png)



### q1_c

fit a plane with RANSAC algorithm

![image-20230222134908098](/Users/huang/Desktop/5335 Robotic/HW4/hw4-1/assets/image-20230222134908098.png)



### 2

Use RANSAC to fit the 3D sphere

**input**:

N：numpy array represent the point clouds of sphere 

P：numpy array represent the estimated surface normal of points

**RANSAC distance threshold: 0.01**

**Iteration: 2000**



**output:**  

<img src="/Users/huang/Desktop/5335 Robotic/HW4/hw4-1/assets/image-20230222194409320.png" alt="image-20230222194409320" style="zoom:50%;" />

<img src="/Users/huang/Desktop/5335 Robotic/HW4/hw4-1/assets/image-20230222194901769.png" alt="image-20230222194901769" style="zoom:50%;" />

<img src="/Users/huang/Desktop/5335 Robotic/HW4/hw4-1/assets/image-20230222194944400.png" alt="image-20230222194944400" style="zoom:50%;" />

### 3

Use RANSAC to fit the 3D cylinder

**input**:

N：numpy array represent the point clouds of cylinder 

P：numpy array (204728, 1) represent the estimated surface normal of points

iterations: mimimun iteration number, default to 500

inliner_threshold: minimun number of inliners that could stop the loop, , default to 10500

RANSAC threshold: 0.004



**output:**  

<img src="/Users/huang/Desktop/5335 Robotic/HW4/hw4-1/assets/image-20230228123716626.png" alt="image-20230228123716626" style="zoom:67%;" />

<img src="/Users/huang/Desktop/5335 Robotic/HW4/hw4-1/assets/image-20230228123733918.png" alt="image-20230228123733918" style="zoom:67%;" />

<img src="/Users/huang/Desktop/5335 Robotic/HW4/hw4-1/assets/image-20230228123752159.png" alt="image-20230228123752159" style="zoom:67%;" />



### 4_a

Find the transformation matrix aligns two point clouds:

<img src="/Users/huang/Desktop/5335 Robotic/HW4/hw4-1/assets/image-20230228123842869.png" alt="image-20230228123842869" style="zoom:67%;" />

### 4_b





The algorith relys on the correspondence points between the two cloud points and then minimize the distance between the two set. When gaussian noice is added, the correspondence between the two clouds does not affected, so the algorithm work with gaussian noice.

When the point clouds are shuffled, the two clouds lost its correspondence and that's why the algorithm does not work in this case.

<img src="/Users/huang/Desktop/5335 Robotic/HW4/hw4-1/assets/image-20230228174135325.png" alt="image-20230228174135325" style="zoom:67%;" />

### 4_c

Using ICP to fit the shuffed bunny: 

<img src="/Users/huang/Desktop/5335 Robotic/HW4/hw4-1/assets/image-20230228173318260.png" alt="image-20230228173318260" style="zoom:67%;" />