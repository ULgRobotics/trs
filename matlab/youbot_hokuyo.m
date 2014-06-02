function [scanned_points contacts] = youbot_hokuyo(vrep, h, opmode, trans)
% Reads from Hokuyo sensor.

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)

  pts1 = [];
  pts2 = [];
  obst1 = [];
  obst2 = [];

  if nargin > 3,
    t1 = trans*h.hokuyo1Trans;
    t2 = trans*h.hokuyo2Trans;
  else
    t1 = h.hokuyo1Trans;
    t2 = h.hokuyo2Trans;
  end
  

  % The Hokuyo data comes in a funny format. Use the code below to move it
  % to a Matlab matrix
  [res det auxData auxPacketInfo] = vrep.simxReadVisionSensor(h.id, h.hokuyo1, opmode);
  if res == 0,
    width = auxData(auxPacketInfo(1)+1);
    height = auxData(auxPacketInfo(1)+2);
    pts1 = reshape(auxData((auxPacketInfo(1)+2+1):end), 4, width*height);
    % Each column of pts1 has [x;y;z;distancetosensor]
    % The Hokuyo sensor has a range of 5m. If there are no obstacles, a point
    % is returned at the 5m limit. As we do not want these points, we throw
    % away all points that are 5m far from the sensor.
    obst1 = pts1(4,:)<4.9999;
    pts1 = pts1(1:3,:);
  end

  % Process the other 120 degrees      
  [res det auxData auxPacketInfo] = vrep.simxReadVisionSensor(h.id, h.hokuyo2, opmode);
  if res == 0,    
    width = auxData(auxPacketInfo(1)+1);
    height = auxData(auxPacketInfo(1)+2);
    pts2 = reshape(auxData((auxPacketInfo(1)+2+1):end), 4, width*height);
    obst2 = pts2(4,:)<4.9999;
    pts2 = pts2(1:3,:);
  end
  scanned_points = [ homtrans(t1, pts1) homtrans(t2, pts2) ];
  contacts = [obst1 obst2];

end
