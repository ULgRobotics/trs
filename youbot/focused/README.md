This folder contains multiple small sample scripts, each of them doing something very specific. In order of increasing complexity: 

  * `youbot_photo.m`: take a 2D picture (standard RGB sensor)
  * `youbot_3dpointcloud.m`: take a 3D point cloud (Hokuyo sensor)
  
These scripts are supposed to replace `youbot.m`, in the parent folder, in that they should stand on their own: 
they start the simulation, initialise the relevant objects, create an infinite loop if required, etc. 
