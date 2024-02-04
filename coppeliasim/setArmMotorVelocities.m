function success = setArmMotorVelocities(vrep,clientID, motorHandles, velocities)

try
    for i = 1:length(motorHandles)	
        vrep.simxSetJointTargetVelocity(clientID, motorHandles(i), velocities(i), vrep.simx_opmode_oneshot);
    end
    success = true;
catch
    success = false;
end

end
