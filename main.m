function main()
    clc

    %% -------------------------- Init the project ---------------------------
    addpath('library/youbot', 'library/matlab');
    run('startup_robot.m');

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

    % Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose.
    % The tip corresponds to the point between the two tongs of the gripper (for more details, see later or in the
    % file focused/youbot_arm.m).
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);
    % Let a few cycles pass to make sure there's a value waiting for us next time we try to get a joint angle or
    % the robot pose with the simx_opmode_buffer option.
    pause(.2);

    %% Youbot constants
    % The time step the simulator is using (your code should run close to it).
    timestep = .05;

    % Minimum and maximum angles for all joints. Only useful to implement custom IK.
    armJointRanges = [-2.9496064186096, 2.9496064186096;
                      -1.5707963705063, 1.308996796608;
                      -2.2863812446594, 2.2863812446594;
                      -1.7802357673645, 1.7802357673645;
                      -1.5707963705063, 1.5707963705063 ];

    % Definition of the starting pose of the arm (the angle to impose at each joint to be in the rest position).
    startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];

    % End simulation
    vrep.simxStopSimulation(id, vrep.simx_opmode_oneshot_wait);
    vrep.simxFinish(-1);
