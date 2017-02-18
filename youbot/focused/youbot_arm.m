function youbot_armMoving()
    % The aim of this code is to show small examples of controlling the arm
    % of the robot in V REP

    % (C) Copyright Renaud Detry 2013, Mathieu Baijot 2017.
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

    % Retrieve all handles, mostly the Hokuyo.
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);

    % Make sure everything is settled before we start. 
    pause(.2);

    % Youbot constants
    timestep = .05;

    %% Additional informations for the robot arm.
    
    % Minimum and maximum angles for all joints. Only useful to implement custom IK. 
    armJointRanges = [-2.9496064186096, 2.9496064186096;
                      -1.5707963705063, 1.308996796608;
                      -2.2863812446594, 2.2863812446594;
                      -1.7802357673645, 1.7802357673645;
                      -1.5707963705063, 1.5707963705063 ];
    % Definition of the starting pose of the arm.
    startingJoints = [-90, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];
    
    %% Preset values for the demo. 
    
    % Set the arm to its starting configuration. 
    res = vrep.simxPauseCommunication(id, true); % Send order to the simulator through vrep object. 
    vrchk(vrep, res); % Check the return value and exit in case of error. 
    for i = 1:5
        res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
    end
    res = vrep.simxPauseCommunication(id, false); 
    vrchk(vrep, res);

    % Make sure everything is settled before we start. 
    pause(2);

    % Get the initial position of the gripper.
    [res, homeGripperPosition] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % Initialise the state machine. 
    fsm = 'noIK';

    %% Start the demo. 
    while true
        tic % See end of loop to see why it's useful. 
        
        if vrep.simxGetConnectionId(id) == -1
          error('Lost connection to remote API.');
        end

        %% Apply the state machine. 
        
        % First example : move the arm without using the IK solver.
        if strcmp(fsm, 'noIK')
            % Define each angle of the robot arm.
            chooseAngle = [180*pi/180, -25*pi/180, 75*pi/180, -17*pi/180, 90*pi/180];
            
            % Apply the value to each articulation.
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            
            % Wait until the robot arm is in position.
            pause(5);
            fsm = 'useIK';
            
        % Second example : move the arm by using the IK solver.
        elseif strcmp(fsm, 'useIK')
            % Get the arm position. 
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            
            % Set the inverse kinematics (IK) mode to position AND orientation (KM_mode = 2). 
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
            
            % Set the new position wwanted for the gripper.
            tpos = [tpos(1) - 0.1, tpos(2) + 0.3, tpos(3) - 0.3];
            res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            
            % Wait long enough so that the tip is at the right position and go on to the next state. 
            pause(5);
            fsm = 'rotGrip';
        
        % Make only the tip to rotate :
        % Since the tip joint is one joint of the arm, you just have to specified which joint to modified.
        elseif strcmp(fsm, 'rotGrip')
            % Remove the inverse kinematics (IK) mode.
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
            % Set the new gripper angle to "0".
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(5), 0, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            
            % Wait long enough so that the tip is at the right position and go on to the next state. 
            pause(5);
            fsm = 'grasp';
            
        % Close the gripper
        elseif strcmp(fsm, 'grasp')
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            
            pause(3);
            fsm = 'release';
        
        % Open the gripper
        elseif strcmp(fsm, 'release')
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            pause(5);
            fsm = 'finished';
   
        elseif strcmp(fsm, 'finished')
            %% Demo done: exit the function. 
            pause(3);
            break;
        else
            error('Unknown state %s.', fsm);
        end

        % Make sure that we do not go faster that the simulator (each iteration must take 50 ms.)
        elapsed = toc;
        timeleft = timestep - elapsed;
        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end

end % main function
   
