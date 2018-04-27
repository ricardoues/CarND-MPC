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

In this moment we don't consider latency. I think the combination of values N=25 and dt = 0.05 is good but I noticed that sometimes the vehicle would likely steer off the road and possibly crash. It woul










