function youbot_3dpointcloud()
    % youbot Illustrates the V-REP MATLAB bindings, more specifically the way to take a 3D point cloud.

    % (C) Copyright Renaud Detry 2013, Thibaut Cuvelier 2017.
    % Distributed under the GNU General Public License.
    % (See http://www.gnu.org/copyleft/gpl.html)
   
    %% Initiate the connection to the simulator. 
     
    disp('Program started');
    % Use the following line if you had to recompile remoteApi
    %vrep = remApi('remoteApi', 'extApi.h');
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    
    % If you get an error like: 
    %   Remote API function call returned with error code: 64. Explanation: simxStart was not yet called.
    % Make sure your code is within a function! You cannot call V-REP from a script. 

    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);

    % Make sure we close the connection whenever the script is interrupted.
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

    % This will only work in "continuous remote API server service". 
    % See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

    % Retrieve all handles, mostly the Hokuyo for this example.
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);

    % Make sure everything is settled before we start (wait for the simulation to start). 
    pause(2);
        
    %% Read data from the depth camera (Hokuyo)
    % Reading a 3D image costs a lot to VREP (it has to simulate the image). It also requires a lot of 
    % bandwidth, and processing a 3D point cloud (for instance, to find one of the boxes or cylinders that 
    % the robot has to grasp) will take a long time in MATLAB. In general, you will only want to capture a 3D 
    % image at specific times, for instance when you believe you're facing one of the tables.

    % Reduce the view angle to pi/8 in order to better see the objects. Do it only once. 
    % ^^^^^^     ^^^^^^^^^^    ^^^^                                     ^^^^^^^^^^^^^^^ 
    % simxSetFloatSignal                                                simx_opmode_oneshot_wait
    %            |
    %            rgbd_sensor_scan_angle
    % The depth camera has a limited number of rays that gather information. If this number is concentrated 
    % on a smaller angle, the resolution is better. pi/8 has been determined by experimentation. 
    res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res); % Check the return value from the previous V-REP call (res) and exit in case of error.

    % Ask the sensor to turn itself on, take A SINGLE POINT CLOUD, and turn itself off again. 
    % ^^^     ^^^^^^                ^^       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % simxSetIntegerSignal          1        simx_opmode_oneshot_wait
    %         |
    %         handle_xyz_sensor
    res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);

    % Then retrieve the last point cloud the depth sensor took.
    % If you were to try to capture multiple images in a row, try other values than 
    % vrep.simx_opmode_oneshot_wait. 
    fprintf('Capturing point cloud...\n');
    pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
    % Each column of pts has [x;y;z;distancetosensor]. However, plot3 does not have the same frame of reference as 
    % the output data. To get a correct plot, you should invert the y and z dimensions. 

    % Plot all the points. 
    figure;
    plot3(pts(1, :), pts(3, :), pts(2, :), '*');
    
    % Plot the points of the wall (further away than 1.87 m, which is determined either in the simulator by measuring 
    % distances or by trial and error) in a different colour. This value is only valid for this robot position, of
    % course. This simple test ignores the variation of distance along the wall (distance between a point and several
    % points on a line). 
    ptsWall = pts(1:3, pts(4, :) >= 1.87);
    hold on; 
    plot3(ptsWall(1, :), ptsWall(3, :), ptsWall(2, :), '.r');

    % Save the point cloud to pc.xyz. (This file can be displayed with http://www.meshlab.net/.)
    fileID = fopen('pc.xyz','w');
    fprintf(fileID,'%f %f %f\n', pts);
    fclose(fileID);
    fprintf('Read %i 3D points, saved to pc.xyz.\n', max(size(pts)));
    
    % Also have a look to the function youbot_hokuyo, used in the main youbot function. It is simpler to use than what
    % this script showed, but with fewer functionalities. This function is also used in the next focused example,
    % youbot_frames.m. 

end % main function
   
