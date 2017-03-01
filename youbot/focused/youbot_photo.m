function youbot_photo()
    % youbot Illustrates the V-REP MATLAB bindings, more specifically the way to take pictures.

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

    % Retrieve all handles.
    h = youbot_init(vrep, id);

    % Make sure everything is settled before we start (wait for the simulation to start). 
    pause(2);

    %% Read data from the RGB camera. 
    % This starts the robot's camera to take a 2D picture of what the robot can see. 
    % Reading an image costs a lot to VREP (it has to simulate the image). It also requires a lot of bandwidth, 
    % and processing an image will take a long time in MATLAB. In general, you will only want to capture 
    % an image at specific times, for instance when you believe you're facing one of the tables or a basket.
    
    % Ask the sensor to turn itself on, take A SINGLE IMAGE, and turn itself off again. 
    % ^^^     ^^^^^^                ^^       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % simxSetIntegerSignal          1        simx_opmode_oneshot_wait
    %         |
    %         handle_rgb_sensor
    res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res); % Check the return value from the previous V-REP call (res) and exit in case of error. 
    
    % Then retrieve the last picture the camera took. The image must be in RGB (not gray scale). 
    %      ^^^^^^^^^^^^^^^^^^^^^^^^^     ^^^^^^                            ^^^
    %      simxGetVisionSensorImage2     h.rgbSensor                       0
    % If you were to try to capture multiple images in a row, try other values than vrep.simx_opmode_oneshot_wait. 
    fprintf('Capturing image...\n');
    [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    fprintf('Captured %i pixels (%i x %i).\n', resolution(1) * resolution(2), resolution(1), resolution(2));

    % Finally, show the image. 
    figure; 
    imshow(image);
    drawnow;
end % main function