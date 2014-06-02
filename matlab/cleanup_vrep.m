function cleanup_vrep(vrep, id)
  fprintf('Closing connection %d.\n', id);
  vrep.simxStopSimulation(id, vrep.simx_opmode_oneshot_wait);
  vrep.simxFinish(id);
  vrep.delete(); % You may need to comment this call if it crashed Matlab.
  disp('Program ended');
end

