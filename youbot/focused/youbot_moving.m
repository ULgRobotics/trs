function youbot()
    % The aim of this code is to show small examples of controlling the displacement of the robot in V-REP. 

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

    % Make sure everything is settled before we start (wait for the simulation to start). 
    pause(.2);

    % The time step the simulator is using (your code should run close to it). 
    timestep = .05;

    %% Preset values for the demo. 
    
    % Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels. 
    % They are adapted at each iteration by the code. 
    forwBackVel = 0; % Move straight ahead. 
    rightVel = 0; % Go sideways. 
    rotateRightVel = 0; % Rotate. 

    % Make sure everything is settled before we start. 
    pause(2);
    
    % First state of state machine
    fsm = 'forward';
    fprintf('Switching to state: %s\n', fsm);

    %% Start the demo. 
    while true
        tic % See end of loop to see why it's useful. 
        
        if vrep.simxGetConnectionId(id) == -1
          error('Lost connection to remote API.');
        end
    
        % Get the position and the orientation of the robot. 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true); % Check the return value from the previous V-REP call (res) and exit in case of error.
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);

        %% Apply the state machine. 
        if strcmp(fsm, 'forward')
            
            % Make the robot drive with a constant speed (very simple controller, likely to overshoot). 
            % The speed is - 1 m/s, the sign indicating the direction to follow. Please note that the robot has
            % limitations and cannot reach an infinite speed. 
            forwBackVel = -1;
            
            % Stop when the robot is close to y = - 6.5. The tolerance has been determined by experiments: if it is too
            % small, the condition will never be met (the robot position is updated every 50 ms); if it is too large,
            % then the robot is not close enough to the position (which may be a problem if it has to pick an object,
            % for example). 
            if abs(youbotPos(2) + 6.5) < .01
                forwBackVel = 0; % Stop the robot. 
                fsm = 'backward';
                fprintf('Switching to state: %s\n', fsm);
            end
            
        elseif strcmp(fsm, 'backward')
            % A speed which is a function of the distance to the destination can also be used. This is useful to avoid
            % overshooting: with this controller, the speed decreases when the robot approaches the goal. 
            % Here, the goal is to reach y = -4.5. 
            forwBackVel = - 2 * (youbotPos(2) + 4.5);
            %             ^^^   ^^^^^^^^^^^^^^^^^^^^
            %             |     distance to goal
            %             influences the maximum speed
            
            % Stop when the robot is close to y = 4.5. 
            if abs(youbotPos(2) + 4.5) < .01
                forwBackVel = 0; % Stop the robot. 
                fsm = 'right';
                fprintf('Switching to state: %s\n', fsm);
            end
            
        elseif strcmp(fsm, 'right')
            % Move sideways, again with a proportional controller (goal: x = - 4.5). 
            rightVel = - 2 * (youbotPos(1) + 4.5);
            
            % Stop at x = - 4.5
            if abs(youbotPos(1) + 4.5) < .01
                rightVel = 0; % Stop the robot. 
                fsm = 'rotateRight';
                fprintf('Switching to state: %s\n', fsm);
            end
            
        elseif strcmp(fsm, 'rotateRight')
            % Rotate until the robot has an angle of -pi/2 (measured with respect to the world's reference frame). 
            % Again, use a proportional controller. In case of overshoot, the angle difference will change sign, 
            % and the robot will correctly find its way back (e.g.: the angular speed is positive, the robot overshoots, 
            % the anguler speed becomes negative). 
            % youbotEuler(3) is the rotation around the vertical axis. 
            rotateRightVel = angdiff(- pi / 2, youbotEuler(3)); % angdiff ensures the difference is between -pi and pi. 
            
            % Stop when the robot is at an angle close to -pi/2. 
            if abs(angdiff(- pi / 2, youbotEuler(3))) < .002
                rotateRightVel = 0;
                fsm = 'finished';
                fprintf('Switching to state: %s\n', fsm);
            end
            
        elseif strcmp(fsm, 'finished')
            pause(3);
            break
        else
            error('Unknown state %s.', fsm)
        end

        % Update wheel velocities. 
        h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);
        
        % What happens if you do not update the velocities? The simulator always considers the last speed you gave it,
        % until you set a new velocity. If you perform computations for several seconds in a row without updating the
        % speeds, the robot will continue to move --- even if it bumps into a wall. 

        % Make sure that we do not go faster than the physics simulation (it is useless to go faster). 
        elapsed = toc;
        timeleft = timestep - elapsed;
        if (timeleft > 0)
          pause(min(timeleft, .01));
        end
    end

end 