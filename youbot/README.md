You should load `house.ttt` as a scene into the simulator. (If you are using an old version of V-REP, prior to 3.1.2, you should rather 
use `house-vrep-3.1.2-or-earlier.ttt`.) The script `youbot.m` should be loaded into MATLAB; it gives a comprehensive demo of what
you can do with the simulator (also check the `focused` folder for easier examples). 

Your robot should place the groceries in baskets following the instructions of the following files (they all have the same contents): 

  * `instructions.mat` (easy to load into MATLAB)
  * `instructions.cfg` (file text, for those not using MATLAB; `cfg-to-matlab.py` is an example of Python script that reads this file)
  
Use the images in the `pictures` folder to recognised the baskets. 
  
`launch.sh` is an automated shell script that runs MATLAB with V-REP; its argument should be your MATLAB script (plus a scene to load, 
if you want to). You must adapt the paths in the file before using it. 

`binding_test.m` and `binding_test.ttt` can be used for testing your installation, but their failure does not automatically mean 
your installation went berserk. 

Before running `youbot.m`, *don't forget to run `../matlab/startup_robot.m`* at least once! This script performs some installation 
steps, but also sets MATLAB's path so that the simulator API is available. 