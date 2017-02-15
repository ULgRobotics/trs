function youbot_moving()
    % The aim of this code is to show small examples of controlling the
    % displacement of the robot in V REP

    % (C) Copyright Renaud Detry 2013, Mathieu Baijot 2017.
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
    pause(.2);

    % Youbot constants
    timestep = .05;

    %% Preset values for the demo. 
    
    % Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels. 
    % They are adapted at each iteration by the code. 
    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0;
    prevOri = 0; 
    prevLoc = 0;

    % Make sure everything is settled before we start. 
    pause(2);
    
    % First state of state machine
    fsm = 'cst_forwBack';

    %% Start the demo. 
    while true
        tic % See end of loop to see why it's useful. 
        
        if vrep.simxGetConnectionId(id) == -1
          error('Lost connection to remote API.');
        end
    
        % Get the position and the orientation of the robot. 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);

        %% Apply the state machine. 
        if strcmp(fsm, 'cst_forwBack');
            
            % Make the robot drive with a constant speed 
            forwBackVel = -1;
            
            % And stop when the robot is close to the position "6.5"
            if (youbotPos(2) + 6.5 < .001)
                forwBackVel = 0;
                fsm = 'lin_forwBack';
            end
            
        elseif strcmp(fsm, 'lin_forwBack');
            % A speed which is a function of the distance to the destination
            % can also be used
            forwBackVel = -2*(youbotPos(2) + 4.5);
            % Stop when the robot is close to the position "4.5"
            if (abs(youbotPos(2) + 4.5) < .001)
                forwBackVel = 0;
                fsm = 'LeftRight';
            end
            
        elseif strcmp(fsm, 'LeftRight');
            % Move to the side
            leftRightVel = -2*(youbotPos(1) + 4.5);
            % And stop at position 4.5
            if (abs(youbotPos(1) + 4.5) < .001)
                leftRightVel = 0;
                fsm = 'Rot';
            end
            
        elseif strcmp(fsm, 'Rot');
            % Make a rotation
            rotVel = angdiff(-pi/2, youbotEuler(3));
            % And stop when the robot is at an angle close to "-pi/2"
            if (abs(angdiff(-pi/2, youbotEuler(3))) < .1/180*pi)
                rotVel = 0;
                fsm = 'finished';
            end;
            
        elseif strcmp(fsm, 'finished'),
            pause(3);
            break;
        else
            error(sprintf('Unknown state %s.', fsm));
        end

        % Update wheel velocities for each loop
        h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);

        % Make sure that we do not go faster that the simulator
        elapsed = toc;
        timeleft = timestep-elapsed;
        if (timeleft > 0),
          pause(min(timeleft, .01));
        end
    end

end % main function
   
