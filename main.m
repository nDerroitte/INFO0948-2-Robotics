function main()
    %% -------------------------- Init the project ---------------------------
    run('startup_robot.m')

    %
    disp('Program started');
    % Launching vrep 
    vrep = remApi('remoteApi');
    % End the last simulation if not already done
    vrep.simxFinish(-1);
    % Creating the id for vrep
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);

    % Make sure we close the connection whenever the script is interrupted.
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

    % Start simulation
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
    
    
    vrep.simxFinish(-1);
