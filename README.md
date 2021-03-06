# CarND-MPC
Model Predictive Control from Self Driving Car Nanodegree of Udacity

Source code: [https://github.com/ricardoues/CarND-MPC/tree/master/src](https://github.com/ricardoues/CarND-MPC/tree/master/src)

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

In this moment we don't consider latency. The combination of values N=25 and dt = 0.05 is bad because the car was oscillating. The result of this, is shown in the video below: 

[https://www.youtube.com/watch?v=5dMnkuqyets&feature=youtu.be](https://www.youtube.com/watch?v=5dMnkuqyets&feature=youtu.be)


We try the following values of N: 

| 25| 22 | 19  | 16  | 13  |
|---|---|---|---|---|

The best value was 13. I noticed that sometimes the vehicle steers off the road and crashs. It would be very unpleasant to take a ride in this car, would't it? In order to fix this we tune the part of the cost function affecting steering: 


```cpp
 
 // Minimize the value gap between sequential actuations.
for (int t = 0; t < N - 2; t++) {
   fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
   fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```

We change the following code

```cpp
CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
```

to this (I take this value from a lesson and I think it is a good value to tune MPC).

```cpp
CppAD::100*pow(vars[a_start + t + 1] - vars[a_start + t], 2);
```

Unfortunately, the vehicle crashed but it performs better than the previous attempt. This result is shown in the following video: 

[https://www.youtube.com/watch?v=txyXmnzYC40&feature=youtu.be](https://www.youtube.com/watch?v=txyXmnzYC40&feature=youtu.be)

We multiply the following component of the cost function by these values: 1, 2, 3, 4, 5, 6, 7

```cpp
      fg[0] += 7*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
```


The best result was obtained with the value of 7, this result is shown below. 

[https://www.youtube.com/watch?v=Pgh-WwLoA_w](https://www.youtube.com/watch?v=Pgh-WwLoA_w)


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

## Model Predictive Control with Latency

In order to deal with latency we predict the state of the car 100ms in the future, we use the following code in order to do this.  


```cpp

          const double latency_dt = 0.1;
          const double Lf = 2.67; 

          px = px + v * cos(psi) * latency_dt;
          py = py + v * sin(psi) * latency_dt;
          psi = psi - v * delta / Lf * latency_dt;
          v = v + a * latency_dt;
```

After that we transform the data to the car's coordinate system, fit a polynomial with order 3, calculate cte and epsi, and finally we calculate the state with the solver. 

In the following video, we show the final MPC model working in the simulator.

[https://www.youtube.com/watch?v=zo1Ys-69pVo&feature=youtu.be](https://www.youtube.com/watch?v=zo1Ys-69pVo&feature=youtu.be)





