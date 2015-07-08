function h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel)
% Sets the youBot wheel speed to achieve the given forward, lateral
% and rotational velocities. The velocities are normalized to say
% within the bounds of the actuator capabilities.

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)

  pParam=20;
  maxV=12;
  pParamRot=10;
  maxVRot=4;
  accelF=0.05;

  forwBackVel=forwBackVel*pParam;
  leftRightVel=leftRightVel*pParam;
  v=sqrt(forwBackVel*forwBackVel+leftRightVel*leftRightVel);
  if v>maxV,
      forwBackVel=forwBackVel*maxV/v;
      leftRightVel=leftRightVel*maxV/v;
  end;
  rotVel=rotVel*pParamRot;
  if (abs(rotVel)>maxVRot),
      rotVel=maxVRot*rotVel/abs(rotVel);
  end;

  df=forwBackVel-h.previousForwBackVel;
  ds=leftRightVel-h.previousLeftRightVel;
  dr=rotVel-h.previousRotVel;

  if (abs(df)>maxV*accelF),
      df=abs(df)*(maxV*accelF)/df;
  end;

  if (abs(ds)>maxV*accelF),
      ds=abs(ds)*(maxV*accelF)/ds;
  end;

  if (abs(dr)>maxVRot*accelF),
      dr=abs(dr)*(maxVRot*accelF)/dr;
  end;

  forwBackVel=h.previousForwBackVel+df;
  leftRightVel=h.previousLeftRightVel+ds;
  rotVel=h.previousRotVel+dr;
  h.previousForwBackVel=forwBackVel;
  h.previousLeftRightVel=leftRightVel;
  h.previousRotVel=rotVel;

  % Update wheel velocities
  res = vrep.simxPauseCommunication(h.id, true); vrchk(vrep, res);
  res = vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(1),...
                                  -forwBackVel-leftRightVel+rotVel,...
                                  vrep.simx_opmode_oneshot); vrchk(vrep, res);
  res = vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(2),...
                                  -forwBackVel+leftRightVel+rotVel,...
                                  vrep.simx_opmode_oneshot); vrchk(vrep, res);
  res = vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(3),...
                                  -forwBackVel-leftRightVel-rotVel,...
                                  vrep.simx_opmode_oneshot); vrchk(vrep, res);
  res = vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(4),...
                                  -forwBackVel+leftRightVel-rotVel,...
                                  vrep.simx_opmode_oneshot); vrchk(vrep, res);
  res = vrep.simxPauseCommunication(h.id, false); vrchk(vrep, res);

end
