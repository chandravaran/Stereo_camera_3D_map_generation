# Stereo_camera_3D_map_generation
This repository contains algorithms for stereo vision and creating 3D maps from it  
The folder 2001 contains the images used for the different methods to find disparity 
The ipynb files contains cencus transform, Sum of absolute difference and the Normalized correlation method which use the multi block or the single block matching methods. 

This repo has 2 main folders, there is the python implementation of the new model we propose for and methods like census transform and Sum of absolute difference have been analysed.

There is a also a ros package that has the implementation of the method on a custom robot desinged, the methods have also been written in cpp and kept within header files so they can be easily reused.
