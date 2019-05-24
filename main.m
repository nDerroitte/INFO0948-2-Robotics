function main()
    clc
    %% -------------------------- Init the project ---------------------------
    addpath(genpath('library/youbot/'))
    addpath(genpath('library/matlab/'))
    %run('startup_robot.m');

    % Launching vrep
    vrep = remApi('remoteApi');
    % End the last simulation if not already done
    vrep.simxFinish(-1);
    % Creating the id for vrep
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    % Connection failed
    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    % Connection succeeded
    fprintf('Connection %d to remote API server open.\n', id);

    % Make sure we close the connection whenever the script is interrupted.
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

    % Start simulation
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

    % robot info
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);

    % the time step
    timestep = .05;

    % Minimum and maximum angles for all joints. Only useful to implement custom IK.
    armJointRanges = [-2.9496064186096, 2.9496064186096;
                      -1.5707963705063, 1.308996796608;
                      -2.2863812446594, 2.2863812446594;
                      -1.7802357673645, 1.7802357673645;
                      -1.5707963705063, 1.5707963705063 ];

    % Definition of the starting pose of the arm.
    startingJoints = [0,0,0,0,0] ;

    % stop comunication to send all information at the same time
    res = vrep.simxPauseCommunication(id, true);
    vrchk(vrep, res);

    % Set the arm to its starting configuration
    for i = 1:5
        res = vrep.simxSetJointTargetPosition(id, h.armJoints(i),...
                                  startingJoints(i), vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
    end
    % Restart communication
    res = vrep.simxPauseCommunication(id, false);
    vrchk(vrep, res);
    % ensure everything is setup
    pause(2);

    % launch the exploration of the map
    [map, map_origin, init_pos] = exploration(vrep, id, h);
    % save the map to a file
    save map.mat map map_origin init_pos;
    % load the map
    load map.mat
    % find the centers in the map
    centers = findCircles(map, 4, 1.5);
    % display the map with the circles
    displayMap(map, {}, [], centers, 4);
    % grap the objects
    [c_tables, c_baskets, a_baskets] = getGrabPos(map, centers, init_pos, 9, 4);
    grabObject(vrep, id, h, map, map_origin, c_tables, c_baskets, a_baskets);

    % End simulation
    vrep.simxStopSimulation(id, vrep.simx_opmode_oneshot_wait);
    vrep.simxFinish(-1);
end
