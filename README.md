# Odometry
My Code for the Odometry Task
The program only relies on Odometry.ino for now.
md25 class may become useful/useable at some point in the distant future.

# DISCLAIMER: If you're looking to use this code for your Odometry Task in following years be aware of two things:
1. The code didn't work. It might become useable. The main issues are:
    1. It was written accross a two week period in a rushed and sleep deprived state.
    1. In the function `void DriveTo` there is an inadvertantly unstable expression which has 2nd Order Gain Function.
       If you were really keen you could probably develop it into a stable PID controller.
1. Proper attribution would have to be made for Academic Integrity Purposes.
