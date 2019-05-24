function [sucess] = grabObject(vrep, id, h, map, map_origin, tables_pos_in, baskets_pos_in, access_pos_in)
    %% Initialise the simulation
    global round_parameter margin
    % Initial position and angle of the robot
    [res, init_robot_pos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    [res, ~] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait); %do only once
    vrchk(vrep, res);
    % Discard Z
    init_robot_pos = init_robot_pos([1;2])';

    % Hard coded (code without vision)
%     CENTRE_TABLE1 = [46, 17];
%     CENTRE_TABLE2 = [];
%     BASKET1 = [15, 48];
%     BASKET2 = [15, 145];
%     BASKET3 = [74, 74];
%     BASKET4 = [137, 107];
%     BASKET5 = [138, 144];
%     BASKET1_CENTRE = [5, 46];
%     BASKET2_CENTRE = [5, 147];
%     BASKET3_CENTRE = [65, 78];
%     BASKET4_CENTRE = [147, 106];
%     BASKET5_CENTRE = [146, 147];

    % Init variables
    path = {};
    sucess = false;
    positions_around_table = zeros(16,2);
    positions_basket = zeros(5,2);
    positions__centre_basket = zeros(5,2);

    % Constants used
    round_parameter = 0.1;
    margin = 3;
    radius = 9;
    accuracy = 0.01;
    angle_step_rotation = 2*pi/ 16; % step of rotation around table
    fsm = 'pathToTable'; % Start by goind to the table
    index_basket = 1;

    % Boolean variables
    takingObject = 0;
    goingBasket = 0;
    returningBasket = 0;
    nb_object_catch = 0;
    checkingObjectCatch = 0;
    current_angle = 0;

    % Postion variables
    CENTRE_TABLE1 = tables_pos_in{1};
    CENTRE_TABLE2 = tables_pos_in{2};
    table_pos = [CENTRE_TABLE1(1) + 9, CENTRE_TABLE1(2)];
    positions_basket(1, :) = access_pos_in{1};
    positions_basket(2, :) = access_pos_in{2};
    positions_basket(3, :) = access_pos_in{3};
    positions_basket(4, :) = access_pos_in{4};
    positions_basket(5, :) = access_pos_in{5};
    positions__centre_basket(1, :) = baskets_pos_in{1};
    positions__centre_basket(2, :) = baskets_pos_in{2};
    positions__centre_basket(3, :) = baskets_pos_in{3};
    positions__centre_basket(4, :) = baskets_pos_in{4};
    positions__centre_basket(5, :) = baskets_pos_in{5}; 

    %% Start the simulation.
    while true
        tic
        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end

        % Get the position and the orientation of the robot.
        [res, robot_pos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, robot_angle] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);

        robot_pos = robot_pos([1;2])'; % discard z
        robot_angle = robot_angle(3); % discard unneeded angles

        % Getting the absolute value
        abs_robot_pos = sim2abs(map_origin, robot_pos);
        abs_robot_angle = robot_angle + pi/2;

        % Go to the table
        if strcmp(fsm, 'pathToTable')
            % Compute the path using astar
            path = getPath(map, abs_robot_pos', table_pos, margin);
            % Path is empty
            if size(path,2) < 1
              disp('Table location too close too wall. try to put it further.')
              fsm ='stop';
            else
              % Prepare to move
              abs_next_pos = path{1};
              path(1) = [];
              fsm = 'Accurate move';
            end
        % Move with a great accuracy
        elseif strcmp(fsm, 'Accurate move')
            accuracy = 0.01;
            fsm = 'move';
        % Move with a smaller accuray
        elseif strcmp(fsm, 'Normal move')
            accuracy = 0.01; % to increase if you have a good computer
            fsm = 'move';
        % State responsible of the move
        elseif strcmp(fsm, 'move')
            % next position in the coordinates of the simulation
            next_pos = abs2sim(abs_next_pos, map_origin, round_parameter);

            % check if close enough to the position
            if (size(path,2) >= 1)
              if (norm(next_pos'-robot_pos) < accuracy)
                  abs_next_pos = path{1};
                  path(1) = [];
                  next_pos = abs2sim(abs_next_pos, map_origin, round_parameter);
              end
              % compute and apply the velocities
              [x_vel, y_vel, rot_vel] = velocity(next_pos, robot_pos, robot_angle);
              h = youbot_drive(vrep, h, x_vel, y_vel, rot_vel);
            else
              disp('Position reached. Change camera orientation.')
              h = youbot_drive(vrep, h, 0, 0, 0);
              % If going to the basket
              if goingBasket == 1
                fsm = 'droppingObjectRotate';
              % If returning from the basket
              elseif returningBasket == 1
                  returningBasket = 0;
                  fsm = 'initNextMove';
              % If moving around the table
              else
                fsm = 'reFocusCamera';
              end
            end
        % Compute the postions around the table to check
        elseif strcmp(fsm, 'rotateAroundTableComputation')
            disp('Computing all positions ...')
            i = 1;
            % Computing all 16 positions
            while current_angle <= (2*pi)
                % getting the next position
                [x, y] = rotateAroundCenter2(radius, current_angle, CENTRE_TABLE1);
                % Update indexes
                current_angle = current_angle + angle_step_rotation;
                positions_around_table(i, :) = round([x, y]);
                i = i+1;
            end
            disp('Done!')
            fsm = 'initNextMove';
        % Init the next move around the table
        elseif strcmp(fsm, 'initNextMove')
            % Getting the position to reached
            abs_new_pos = positions_around_table(1, :);
            positions_around_table(1, :) = [];
            % If we have a next pos and object to catch, we continue
            if isempty(abs_new_pos) || nb_object_catch == 5
                disp('Reached the end of the simulation. Closing')
                fsm = 'stop';
                continue;
            end
            % Getting the path to next position
            disp('Creating next path..')
            path = getPath(map, abs_robot_pos', abs_new_pos , 1);
            % Empty path
            if size(path,2) < 1
                fsm ='initNextMove';
            else
                % Initialising the move
                abs_next_pos = path{1};
                path(1) = [];
                disp('Moving..');
                fsm = 'Accurate move';
            end
        % Focus the camera on the centre of the table
        elseif strcmp(fsm, 'reFocusCamera')
            vect = abs_robot_pos'- CENTRE_TABLE1 ;
            % Compute the angle
            angleRotation = atan2(vect(2), vect(1));
            fsm = 'rotate';
        % Rotating the robot
        elseif strcmp(fsm, 'rotate')
            rotateRightVel = angdiff(angleRotation, abs_robot_angle);
            h = youbot_drive(vrep, h, 0, 0, rotateRightVel);
            % When the rotation is done (with a sufficiently high precision), move on to the next state.
            if (abs(angdiff(angleRotation, abs_robot_angle)) < .01 / 180 * pi)
                % If taking the object
                if takingObject == 1
                    takingObject = 0;
                    fsm = 'takingObject';
                % If going to the basket
                elseif goingBasket == 1
                    fsm = 'droppingObjectAction';
                % if checking if the object is well caught
                elseif checkingObjectCatch == 1
                    fsm = 'checkingObjectCatch';
                % Rotating around the table
                else
                    disp('Finish rotating. Capturing image..')
                    % If first time, compute the positions around the table
                    if positions_around_table(1, 1) == 0
                        fsm = 'rotateAroundTableComputation';
                    % Else we analyse the image
                    else
                        fsm = 'analyseImage';
                    end
                end
            end
        % Analysing the image
        elseif strcmp(fsm , 'analyseImage')
            % Stop the robot
            h = youbot_drive(vrep, h, 0, 0, 0);
            pause(1);
            % Taking a picture
            captureImage(vrep, id, h)
            disp('Doing image analysis.')
            % Checking if there is an objcet in foreground
            [res, color] = image_foreground();
            if res == 1
                disp('Object in foreground found');
                % Checking if the object is at the centre of the image
                [res_centred, dir] = object_centred(color);
                if res_centred == 1
                    disp('Getting at the good distance')
                    fsm = 'distanceWithObject';
                % Else we move a little
                else
                    disp('Moving a little..')
                    fsm = 'smallMovementY';
                end
            % No foreground object found
            else
                disp('No object in foreground found.')
                fsm = 'initNextMove';
            end
        % Checking the distance with the object
        elseif strcmp(fsm, 'distanceWithObject')
            % Stop the robot
            h = youbot_drive(vrep, h, 0, 0, 0);
            pause(1);
            % Captue an image
            captureImage(vrep, id, h)
            % Checking the distance using vision
            [res, dir] = good_distance_object(color);
            % if at good distance
            if res == 1
                % Rotate of 180 degrees
                disp('Now at the good distance of the object')
                angleRotation = abs_robot_angle + pi;
                takingObject = 1;
                fsm = 'rotate';
            else
                % Getting closer/further
                disp('Getting at the good distance..')
                fsm = 'smallMovementX';
            end
        % Returning to the init pos of the robot
        elseif strcmp(fsm,'goinitPos')
            returningBasket = 1;
            init_abs_robot_pos = sim2abs(map_origin, init_robot_pos);
            % Computing the path with a large margin.
            path = getPath(map, abs_robot_pos', init_abs_robot_pos' , 3);
            % Initialising the move
            abs_next_pos = path{1};
            path(1) = [];
            goingBasket = 1;
            disp('Moving toward initial position');
            fsm = 'Normal move';
        % Little movement along y axis
        elseif strcmp(fsm, 'smallMovementY')
            h = youbot_drive(vrep, h, 0, dir, 0);
            fsm = 'analyseImage';
        % Little movement around x axis
        elseif strcmp(fsm, 'smallMovementX')
            h = youbot_drive(vrep, h, dir, 0, 0);
            fsm = 'distanceWithObject';
        % Prerecorded sequence to catch the object
        elseif strcmp(fsm, 'takingObject')
            disp('Preparing the grab')
            % Stop the robot
            h = youbot_drive(vrep, h, 0, 0, 0);
            pause(3)
            % Intermediate position 1
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
            chooseAngle = [0, pi/4, -pi/2, 0, 0];
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            pause(3)
            % Intermediate position 2
            chooseAngle = [0, -(pi/8)*2, -pi *6/8, pi/2, 0];
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            pause(3)
            % Intermediate position 3
            chooseAngle = [0, -(pi/8)*2, -pi *4/8, pi/2- 2*pi/8, 0];
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            pause(3)
            % Intermediate position 4
            chooseAngle = [0, -(pi/16)*6, -pi *6/16, pi/2- 4*pi/16, 0];
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            fsm = 'waitAndGrab';
        % Grabbing the object
        elseif strcmp(fsm, 'waitAndGrab')
            pause(5);
            disp('Grabbing.');
            % Travel position
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            pause(3);
            chooseAngle = [0, 0, 0, 0, 0];
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            pause(3);
            % Going to the basket
            fsm = 'goToBasket';
            continue;
        % Going to the basket
        elseif strcmp(fsm, 'goToBasket')
            % Getting the postion of the basket
            current_basket_pos = positions_basket(index_basket, :);
            % Getting the path
            path = getPath(map, abs_robot_pos', current_basket_pos , 3);
            % Initialing the movement
            abs_next_pos = path{1};
            path(1) = [];
            goingBasket = 1;
            disp('Moving toward basket');
            fsm = 'Normal move';
       % Rotate before dropping the object
        elseif strcmp(fsm, 'droppingObjectRotate')
            disp('Rotating toward basket')
            % Computing the angle of rotation
            vect = abs_robot_pos'- positions__centre_basket(index_basket, :);
            angleRotation = atan2(vect(2), vect(1));
            angleRotation = angleRotation + pi;

            % > 0 modulo for matlab..
            index_basket = index_basket + 1;
            index_basket = mod(index_basket, 5);
            if index_basket == 0
                index_basket = 5;
            end

            fsm = 'rotate';
        % Checking if the object was indeed catch. Removed. State if necer called
        elseif strcmp(fsm, 'checkingObjectCatch')
            % Stop the robot
            disp('Checking if the object was indeed catch.')
            h = youbot_drive(vrep, h, 0, 0, 0);
            pause(1);
            % Capturing the image
            captureImage(vrep, id, h)
            disp('Doing image analysis.')
            % Check if object in foreground
            [res, color] = image_foreground();
            if res == 1
                fsm = 'analyseImage';
            else
                checkingObjectCatch = 0;
                fsm = 'goToBasket';
            end
        % Dropping the object
        elseif strcmp(fsm, 'droppingObjectAction')
            goingBasket = 0;
            % Stopping the robot
            h = youbot_drive(vrep, h, 0, 0, 0);
            pause(3)
            % Dropping position
            chooseAngle = [0, -(pi/8)*2, -pi *4/8, pi/2- 2*pi/8, 0];
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            pause(3)
            % Opening the gripper
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            pause(3);
            % Returning to travel pos
            chooseAngle = [0, 0, 0, 0, 0];
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            % Increasing the # of object caught
            nb_object_catch = nb_object_catch + 1;
            fsm = 'goinitPos';
        % Exit state
        elseif strcmp(fsm, 'stop')
          captureImage(vrep, id, h);
          disp('Grabing exits.')
          break;
        end
        % display the updated map
        displayMap(map,path,abs_robot_pos, {}, 0);
    end

end
%% Aux functions
function [path] = getPath(map, init_pos, dest_pos, margin)
    path = astar(map,init_pos , dest_pos, margin, 0);
end

function [pos_abs] = sim2abs(map_origin, pos_sim)
     global round_parameter
    pos_abs = round((1/round_parameter) * (pos_sim - map_origin))+1;
end

function [sim_position] = abs2sim(abs_position, map_origin, round_parameter)
  sim_position = bsxfun(@plus, round_parameter*abs_position, + map_origin');
end

%% Rotation around table.
% First idea to compute the postion around the table. Working but not
% precise enough
function [delta_x, delta_y] = rotateAroundCenter(radius, angle_step, angle)
    string = 2* radius * sin(angle_step/2);
    beta = (pi - angle_step)/2; %work in radian
    gamma = ((2*pi - pi/2 - angle_step)/2) - beta;

    theta = (angle +  gamma);

    delta_x = string * cos(theta);
    delta_y = string * sin(theta);
end

function [x, y] = rotateAroundCenter2(radius, angle, centre)
    x = centre(1) + radius * cos(angle);
    y = centre(2) + radius * sin(angle);
end

%% Capture image
function captureImage(vrep, id, h)
    res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    [res, ~, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    imwrite (image, 'image.png');
end
