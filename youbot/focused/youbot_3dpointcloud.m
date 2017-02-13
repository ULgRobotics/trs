function youbot_3dpointcloud()
    % youbot Illustrates the V-REP Matlab bindings, more specifically the way to take a 3D point cloud.

    % (C) Copyright Renaud Detry 2013, Thibaut Cuvelier 2017.
    % Distributed under the GNU General Public License.
    % (See http://www.gnu.org/copyleft/gpl.html)
   
    %% Initiate the connection to the simulator. 
    
    disp('Program started');
    % Use the following line if you had to recompile remoteApi
    %vrep = remApi('remoteApi', 'extApi.h');
    vrep=remApi('remoteApi');
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

    % Retrieve all handles, mostly the Hokuyo.
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);

    % Make sure everything is settled before we start. 
    pause(2);
        
    %% Read data from the range camera
    % Reading a 3D image costs a lot to VREP (it has to simulate the image). It
    % also requires a lot of bandwidth, and processing a 3D point cloud (for
    % instance, to find one of the boxes or cylinders that the robot has to
    % grasp) will take a long time in MATLAB. In general, you will only want to
    % capture a 3D image at specific times, for instance when you believe you're
    % facing one of the tables.

    % Reduce the view angle to better see the objects. 
    res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);

    % Ask the sensor to turn itself on, take A SINGLE 3D IMAGE, and turn itself off again. 
    res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);

    % Then use the depth sensor. 
    fprintf('Capturing point cloud...\n');
    pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
    % Each column of pts has [x;y;z;distancetosensor]. However, plot3 does not have the same frame of reference! 
    % To get a correct plot, you should invert the y and z dimensions. 

    figure;
    plot3(pts(1, :), pts(2, :), pts(3, :), '*');
    
    % Plot the points of the wall (further away than 1.87 m) in a different colour. 
    hold on; 
    ptsWall = pts(1:3, pts(4, :) >= 1.87);
    plot3(ptsWall(1, :), ptsWall(3, :), ptsWall(2, :), '.r');

    % Save the pointcloud to pc.xyz. (This file can be displayed with meshlab.sf.net.)
    fileID = fopen('pc.xyz','w');
    fprintf(fileID,'%f %f %f\n',pts);
    fclose(fileID);
    fprintf('Read %i 3D points, saved to pc.xyz.\n', max(size(pts)));

end % main function
   
