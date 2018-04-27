# CarND-MPC
Model Predictive Control from Self Driving Car Nanodegree of Udacity

Source code: 

# Reflection
## The model 
We consider the following Model Predictive Control model: 

![equation](./images/equation0001.png)

![equation](./images/equation0002.png)

![equation](./images/equation0003.png)

![equation](./images/equation0004.png)

![equation](./images/equation0005.png)

![equation](./images/equation0006.png)

The actuators are: 

![equation](./images/equation0008.png)

![equation](./images/equation0009.png)

The cost function is:

![equation](./images/equation0007.png)


## Timestep Length and Elapsed Duration (N & dt)
I started with the following pair of values N = 25 and dt = 0.05. These values are based on the following code provided in the lesson 19, Model Predictive Control: 

[https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp)

In this moment we don't consider latency. I think the combination of values N=25 and dt = 0.05 is good. The result of this, is shown in the video below: 

[https://www.youtube.com/watch?v=Pgh-WwLoA_w&feature=youtu.be](https://www.youtube.com/watch?v=Pgh-WwLoA_w&feature=youtu.be)



I noticed that sometimes the vehicle would likely steer off the road and possibly crash. It would be very unpleasant to take a ride in this car, would't it? In order to fix this we tune the part of the cost function affecting steering: 


```cpp
 
 // Minimize the value gap between sequential actuations.
for (int t = 0; t < N - 2; t++) {
   fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
   fg[0] += 100*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```

The result of this, is shown in the video below: 

[https://www.youtube.com/watch?v=TQxROUeTm9Y&feature=youtu.be](https://www.youtube.com/watch?v=Pgh-WwLoA_w&feature=youtu.be)

The car's behaviour looks better. 


## Polynomial Fitting and MPC Preprocessing 

We use a polynomial degree of order 3. In order to transform the waypoints to the car's coordinate system (the server returns waypoints using the map's coordinate system) we use the following C++ code. 


```cpp

   // The following two lineas were take from
   // https://stackoverflow.com/questions/17036818/initialise-eigenvector-with-stdvector
   Eigen::VectorXd ptsx2 = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());

   Eigen::VectorXd ptsy2 = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());


   for(int i=0; i < ptsx2.size(); i++)
   {
      double tempx = ptsx2[i];
      ptsx2[i] = cos(psi) * (ptsx2[i] - px) + sin(psi) * (ptsy2[i] - py);
      ptsy2[i] = -sin(psi) * (tempx - px) + cos(psi) * (ptsy2[i] - py);
   }

```

Moreover, we have taken into account that the vehicle's position is (0,0) that is why we assign to x,y, and psi the value of zero in the following C++ code. 

```cpp
   Eigen::VectorXd state(6);
   // x and y coordinates are always zero, psi is also zero.
   state << 0, 0, 0, v, cte, epsi;
```





