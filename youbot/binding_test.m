% To test that the VREP Matlab bindings work:

% 1) You must copy three files from the VREP app directory to the directory
%    where this file is located:
% - remApi.m
% - remoteApiProto.m
% - remoteApi.so (if you use Linux)
%   remoteApi.dll (if you use Windows)
%   remoteApi.dylib (if you use a Mac)
% You will probably find these files in the programming subdirectory of the
% VREP app directory, although this can change from version to version.

% 2) Open VREP, load binding_test.ttt, and hit the "Start simulation" button.
%    binding_test.ttt is located in the same directory as this file.

% 3) In Matlab, cd to the directory in which this file is located, and type
%       binding_test

% If you read "Number of objects in the scene: 19", everything is fine!
% If something is wrong, get help from http://www.forum.coppeliarobotics.com

function binding_test()
	disp('Program started');
	% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
	vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
	vrep.simxFinish(-1); % just in case, close all opened connections
	clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
	try
		if (clientID>-1)
			disp('Connected to remote API server');
			[res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait);
			if (res==vrep.simx_return_ok)
				fprintf('Number of objects in the scene: %d\n',length(objs));
				fprintf('The remote API is working!\n');
			else
				fprintf('Remote API function call returned with error code: %d\n',res);
			end
			vrep.simxFinish(clientID);
		else
			disp('Failed connecting to remote API server');
		end
		vrep.delete(); % call the destructor!
	catch err
		vrep.simxFinish(clientID); % close the line if still open
		vrep.delete(); % call the destructor!
	end;
	
	disp('Program ended');
end
