function pts = youbot_xyz_sensor(vrep, h, opmode)
% Read from xyz sensor.

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)

  % Again, the data comes in a funny format. Use the lines below to move the
  % data to a Matlab matrix
  [res det auxData auxPacketInfo] = vrep.simxReadVisionSensor(h.id, h.xyzSensor, opmode); vrchk(vrep, res, true);
  width = auxData(auxPacketInfo(1)+1);
  height = auxData(auxPacketInfo(1)+2);
  pts = reshape(auxData((auxPacketInfo(1)+2+1):end), 4, width*height);
  % Each column of pts has [x;y;z;distancetosensor]

  pts = pts(:,pts(4,:)<4.9999);

end
