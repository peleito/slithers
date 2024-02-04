function success = setArmJointPositions(vrep,clientID, jointHandles, angles_rad)

try
    for i = 1:length(jointHandles)	
        vrep.simxSetJointTargetPosition(clientID, jointHandles(i), angles_rad(i), vrep.simx_opmode_oneshot);
    end
    success = true;
catch
    success = false;
end

end
