This folder contains multiple small sample scripts, each of them doing something very specific. In order of increasing complexity: 

  * `youbot_photo.m`: take a 2D picture (standard RGB sensor)
  * `youbot_3dpointcloud.m`: take a 3D point cloud (Hokuyo sensor)
  * `youbot_frames.m`: take a 3D point cloud (Hokuyo sensor) and move it to the global reference frame (not the one of the robot)
  * `youbot_moving.m`: move the robot around
  * `youbot_arm.m`: move the robot's arm
  
These scripts are supposed to replace `youbot.m`, in the parent folder, in that they should stand on their own: 
they start the simulation, initialise the relevant objects, create an infinite loop if required, etc. 

The following scripts also show some tricks about plotting data in MATLAB; they are not linked to the previous scripts: 

  * `plot_matrix.m`: plot a matrix (such as a map)
  * `plot_multiple.m`: deal with multiple windows
 