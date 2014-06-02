function handles = youbot_init(vrep, id)
% Initialize youBot

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)

% Retrieve all handles, and stream arm and wheel joints, the robot's pose,
% the Hokuyo, and the arm tip pose.

handles = struct('id', id);

wheelJoints = [-1,-1,-1,-1]; % front left, rear left, rear right, front right
[res wheelJoints(1)] = vrep.simxGetObjectHandle(id, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res wheelJoints(2)] = vrep.simxGetObjectHandle(id, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res wheelJoints(3)] = vrep.simxGetObjectHandle(id, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res wheelJoints(4)] = vrep.simxGetObjectHandle(id, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

handles.wheelJoints = wheelJoints;

% The Hokuyo sensor is implemented with two planar sensors that each cover 120 degrees:
[res hokuyo1] = vrep.simxGetObjectHandle(id, 'fastHokuyo_sensor1', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res hokuyo2] = vrep.simxGetObjectHandle(id, 'fastHokuyo_sensor2', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

handles.hokuyo1 = hokuyo1;
handles.hokuyo2 = hokuyo2;

[res xyzSensor] = vrep.simxGetObjectHandle(id, 'xyzSensor', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res rgbSensor] = vrep.simxGetObjectHandle(id, 'rgbSensor', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res rgbdCasing] = vrep.simxGetObjectHandle(id, 'rgbdSensor', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

handles.xyzSensor = xyzSensor;
handles.rgbSensor = rgbSensor;
handles.rgbdCasing = rgbdCasing;

[res ref] = vrep.simxGetObjectHandle(id, 'youBot_center', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res armRef] = vrep.simxGetObjectHandle(id, 'youBot_ref', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

handles.ref = ref;
handles.armRef = armRef;

% The project page ( http://renaud-detry.net/teaching/info0948/private/project.php )
% contains information on the different control modes of the arm. Search for
% km_mode on the project webpage to find the arm documentation. Read that documentation
% before working with the code below.

% The *position* of this object always corresponds to the position of the tip of
% the arm (the tip is somewhere between the two fingers)
[res ptip] = vrep.simxGetObjectHandle(id, 'youBot_gripperPositionTip', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
% In IK mode (km_mode set to 1 or 2), the robot will try to move the *position*
% of ptip to the *position* of ptarget.
[res ptarget] = vrep.simxGetObjectHandle(id, 'youBot_gripperPositionTarget', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
% The *orientation* of this object always corresponds to the orientation of the tip of
% the arm (the tip is somewhere between the two fingers)
[res otip] = vrep.simxGetObjectHandle(id, 'youBot_gripperOrientationTip', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
% In IK mode 2 (km_mode set to 2), the robot will try to move the *orienatation*
% of otip to the *orientation* of otarget.
[res otarget] = vrep.simxGetObjectHandle(id, 'youBot_gripperOrientationTarget', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
% Tip orientations are easier to manipulate in the reference frame of Rectangle22,
% because then the degree of freedom onto which the orientation controller acts
% corresponds to one of the three Euler angles of the tip orientation.
[res r22] = vrep.simxGetObjectHandle(id, 'Rectangle22', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

handles.ptip = ptip;
handles.ptarget = ptarget;
handles.otip = otip;
handles.otarget = otarget;
handles.r22 = r22;

armJoints = [-1,-1,-1,-1,-1];

for i = 0:4,
  [res armJoints(i+1)] = vrep.simxGetObjectHandle(id, sprintf('youBotArmJoint%d',i), vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
end

handles.armJoints = armJoints;

[res mapLooker] = vrep.simxGetObjectHandle(id, 'map', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

handles.mapLooker = mapLooker;

[res landmarks] = vrep.simxGetObjectHandle(id, 'Landmarks', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

handles.landmarks = landmarks;


% Stream wheel angles, Hokuyo data, and robot pose (see usage below)
% Wheel angles are not used in this example, but they may/will be necessary in
% your project.
for i = 1:4,
  res = vrep.simxGetJointPosition(id, wheelJoints(i), vrep.simx_opmode_streaming); vrchk(vrep, res, true);
end
res = vrep.simxGetObjectPosition(id, ref, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, ref, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
res = vrep.simxReadVisionSensor(id, hokuyo1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
res = vrep.simxReadVisionSensor(id, hokuyo2, vrep.simx_opmode_streaming); vrchk(vrep, res, true);

% Stream the arm joint angles and the tip position/orientation
res = vrep.simxGetObjectPosition(id, ptip, armRef, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, otip, r22, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
for i = 1:5,
  res = vrep.simxGetJointPosition(id, armJoints(i), vrep.simx_opmode_streaming); vrchk(vrep, res, true);
end

end
