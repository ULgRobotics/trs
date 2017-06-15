function youbot_frames()
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
    % Get the position and orientation of the youBot in the world reference frame (as if with a GPS).
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer); 
    vrchk(vrep, res, true); % Check the return value from the previous V-REP call (res) and exit in case of error.
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer); 
    vrchk(vrep, res, true);
    
    % Determine the position of the Hokuyo with global coordinates (world reference frame). 
    trf = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
    worldHokuyo1 = homtrans(trf, [h.hokuyo1Pos(1); h.hokuyo1Pos(2); h.hokuyo1Pos(3)]);
    worldHokuyo2 = homtrans(trf, [h.hokuyo2Pos(1); h.hokuyo2Pos(2); h.hokuyo2Pos(3)]);
                        
    % Use the sensor to detect the visible points, within the world frame. 
    [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, trf);
    
    % Plot this data: delimit the visible area and highlight contact points. 
    plot(pts(1, contacts), pts(2, contacts), '*r',...
         [worldHokuyo1(1), pts(1, :), worldHokuyo2(1)], [worldHokuyo1(2), pts(2, :), worldHokuyo2(2)], 'r');
    axis([-10, 10, -10, 10]);
    axis equal;
    drawnow;

end % main function
