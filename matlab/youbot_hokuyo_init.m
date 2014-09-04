function h = youbot_hokuyo_init(vrep, h)
% Initialize Hokuyo sensor in VREP
% This function starts the Hokuyo sensor, and it computes the transformations
% between the Hokuyo frame and the youBot reference frame h.ref.
% These transformations are stored in h.hokuyo1Trans and h.hokuyo2Trans

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)

  % Turn the Hokuyo on (constantly)
  res = vrep.simxSetIntegerSignal(h.id, 'handle_xy_sensor', 2, vrep.simx_opmode_oneshot);
  % Display the red laser beams of active sensors
  vrep.simxSetIntegerSignal(h.id, 'displaylasers', 1, vrep.simx_opmode_oneshot);

  % In order to build a map, you will need to transfer the data coming from the
  % Hokuyo to the world reference frame. The transformation between the frame of
  % hokuyoXHandle and the world is not accessible. However, you can access
  % the transformation between h.hokuyoX and h.ref (right below), and you
  % can access the transformation between h.ref and the world (see below, search for
  % "Stream wheel angles and robot pose"). By combining these two transformations,
  % you can transform the Hokuyo data to the world frame and build your map.
  % Do not forget that you can use the functions from the robot toolbox to compute
  % transformations, see page 23 of the book, e.g. the functions se2(), inv(),
  % h2e(), e2h(), homtrans(), ...
  [res h.hokuyo1Pos] = vrep.simxGetObjectPosition(h.id, h.hokuyo1, h.ref, vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
  [res h.hokuyo1Euler] = vrep.simxGetObjectOrientation(h.id, h.hokuyo1, h.ref, vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
  [res h.hokuyo2Pos] = vrep.simxGetObjectPosition(h.id, h.hokuyo2, h.ref, vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
  [res h.hokuyo2Euler] = vrep.simxGetObjectOrientation(h.id, h.hokuyo2, h.ref, vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

  %fixme
  %for j = [0:.1:(2*pi) 2*pi]
  %  vrep.simxSetObjectOrientation(h.id, h.rgbdCasing, h.ref, [0 0 -pi/2+j], vrep.simx_opmode_oneshot);
  %  pause(1/20);
  %end

  % Compute the transformations between the two Hokuyo subsensors and the youBot
  % ref frame h.ref
  h.hokuyo1Trans = transl(h.hokuyo1Pos) * trotx(h.hokuyo1Euler(1)) * troty(h.hokuyo1Euler(2)) * trotz(h.hokuyo1Euler(3));
  h.hokuyo2Trans = transl(h.hokuyo2Pos) * trotx(h.hokuyo2Euler(1)) * troty(h.hokuyo2Euler(2)) * trotz(h.hokuyo2Euler(3));

end
