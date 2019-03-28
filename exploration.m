function exploration(vrep, id, h)
    timestep = .05;

    [res, init_robot_position] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    [res, init_robot_angle] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);

    global round_parameter round_decimals
    initsize = 1;
    round_parameter = 0.1;
    round_decimals = -log10(round_parameter);
    global map map_origin map_size
    map = zeros(initsize, initsize);
    map_size = size(map);
    map_origin = init_robot_position([1;2])';

    rotate_angle = pi;
    rotation_next_pos = 0;
    i = 0;
    fsm = 'rotate';
    %% Start the exploration.
    while true
        tic
        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end

        % Get the position and the orientation of the ROBOT.
        % youbotPos : [x, y, z]
        % Get the position and the orientation of the robot.
        [res, robot_position] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, robot_angle] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        robot_position = robot_position([1;2])'; % discard z
        robot_angle = robot_angle(3); % discard not needed angle



        % pts : 3 x 684 avec [[x ,y ,z]] -> tous les points vu
        % contacts: une matrice 1x 684  -> 0 si pas de wall et 1 si wall
        [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
        pts = pts(1:2,:); % discard z
        updateMap(h, pts, contacts, robot_position, robot_angle);
        displayMap();


        i = i +1;


        if strcmp(fsm, 'rotate')
            % /!\ Velocity backward!!!
            rotateRightVel = angdiff(rotate_angle, robot_angle);

            % When the rotation is done (with a sufficiently high precision), move on to the next state.
            if abs(angdiff(rotate_angle, robot_angle)) < .1 / 180 * pi && abs(angdiff(prevOrientation, robot_angle)) < .01 / 180 * pi
                fsm = 'createTarget';
            end
            prevOrientation = robot_angle;
            h = youbot_drive(vrep, h, 0, 0, rotateRightVel);

        elseif strcmp(fsm, 'createTarget')
            absolute_robot_position = round((1/round_parameter) * (robot_position - map_origin));
            next_pos = [absolute_robot_position(1)+1, absolute_robot_position(2)];
            %target_pos = astarMedhi
            % next_pos
            rotation_next_pos = getRotationNextPos(absolute_robot_position, next_pos);
            fsm = 'rotateToNextPos';
        elseif strcmp(fsm, 'rotateToNextPos')
            % /!\ Velocity backward!!!
            rotateRightVel = angdiff(rotation_next_pos, robot_angle); % rotate slowwwwy

            % When the rotation is done (with a sufficiently high precision), move on to the next state.
            if abs(angdiff(rotation_next_pos, robot_angle)) < .1 / 180 * pi
                fsm = 'moveToNextPos';
            end
            h = youbot_drive(vrep, h, 0, 0, rotateRightVel);
        elseif strcmp(fsm, 'moveToNextPos')
            forwBackVel = - (robot_position(1) + 0.1);

            if (robot_position(1) + 0.1 < .001)
                forwBackVel = 0;
                break;
                % next_pos in list. Si pas de next pos move to createTarget
            end
            h = youbot_drive(vrep, h, forwBackVel, 0, 0);
        end

        % Make sure that we do not go faster than the physics simulation (each iteration must take roughly 50 ms).
        elapsed = toc;
        timeleft = timestep - elapsed;
        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end
    absolute_robot_position = round((1/round_parameter) * (robot_position - map_origin));
    disp('Simplifying the map');
    simplifyMap(2, absolute_robot_position);
    simplifyMap(3, absolute_robot_position);
    disp('Done');
    displayMap();
end
%% Update map
function updateMap(h, pts, contacts, robot_position, robot_angle)
    global map round_parameter map_origin
    % rotate the points and translate them to be on the map
    rotation_matrix = [cos(robot_angle), -sin(robot_angle);  sin(robot_angle), cos(robot_angle)];

    absolute_index_pts = round((1/round_parameter) * bsxfun(@plus, rotation_matrix * pts, robot_position - map_origin));

    hokuyoPositions = [h.hokuyo1Pos(1), h.hokuyo1Pos(2); h.hokuyo2Pos(1), h.hokuyo2Pos(2)];
    hokuyoPositions = round((1/round_parameter) * bsxfun(@plus, rotation_matrix * hokuyoPositions, robot_position - map_origin));
    max_x = max(absolute_index_pts(1,:));
    max_y = max(absolute_index_pts(2,:));
    min_x = min(absolute_index_pts(1,:));
    min_y = min(absolute_index_pts(2,:));

    max_x_extend = max(max_x - size(map, 1), 0);
    max_y_extend = max(max_y - size(map, 2), 0);
    min_x_extend = max(1 - min_x, 0);
    min_y_extend = max(1 - min_y, 0);

    translation = [0; 0];

    if (max_x_extend > 0) || (max_y_extend > 0) ||(min_x_extend > 0) || (min_y_extend > 0)
        [map, translation] = updateSizeMap(max_x_extend, max_y_extend, min_x_extend, min_y_extend);
        absolute_index_pts = bsxfun(@plus, absolute_index_pts, translation);
        hokuyoPositions = bsxfun(@plus, hokuyoPositions, translation);

    end
    % Create a 2D mesh of points, stored in the vectors X and Y. This will be used to display the area the robot can
    % see, by selecting the points within this mesh that are within the visibility range.
    [X, Y] = find(map==0);
    % Put 1 for the free space
    in = inpolygon(X, Y,...
                           [hokuyoPositions(1), absolute_index_pts(1, :), hokuyoPositions(3)],...
                           [hokuyoPositions(2), absolute_index_pts(2, :), hokuyoPositions(4)]);
    free_x = X(in);
    free_y = Y(in);
    free_index = [free_x'; free_y'];
    for i=1:length(free_x)
        if map(free_index(1, i), free_index(2, i)) == 0
            map(free_index(1, i), free_index(2, i)) = 1;
        end
    end
    % Put 2 to the positions of the obstacle
    obstacles = find(contacts);
    for i=1:length(obstacles)
        map(absolute_index_pts(1, obstacles(i)), absolute_index_pts(2, obstacles(i))) = 2;
    end
end
function [new_map, translation] = updateSizeMap(max_x_extend, max_y_extend, min_x_extend, min_y_extend)
    global map_size map map_origin round_parameter
    translation = [min_x_extend; min_y_extend];
    prevous_map_size = map_size;
    map_size = map_size + [max_x_extend + min_x_extend, max_y_extend + min_y_extend];

    new_map = zeros(map_size);
    new_map((translation(1) + 1) : (translation(1) + prevous_map_size(1)), (translation(2) + 1) : (translation(2) + prevous_map_size(2))) = map;

    map_origin = map_origin - (translation * round_parameter);

end
%% Plot map
function displayMap()
    global map
    figure(1)
    % plot obstacles
    [obstacle_x, obstacle_y] = find(map == 2);
    plot(obstacle_x, obstacle_y, 'xr')
    hold on
    % plot free_space
    [free_space_x, free_space_y] = find(map == 1);
    plot(free_space_x, free_space_y, 'xb')
    % plot unreacheable state
    [unreachable_x, unreachable_y] = find(map == 3);
    plot(unreachable_x, unreachable_y, 'xb')
    % plot unreacheable state
    [robot_x, robot_y] = find(map == 5);
    plot(robot_x, robot_y, 'oy')
    drawnow;
end
%% Simplify map
function simplifyMap(index, absolute_robot_position)
    global map
    map(absolute_robot_position(1), absolute_robot_position(2)) = 5;
    for i = 1:size(map,1)
        for j = 1:size(map,2)
            if map(i,j) == index
                if i-1 > 0 && j-1 > 0 && map(i-1,j-1) == 1
                    map(i-1,j-1) = 4;
                end
                if j-1 > 0 && map(i,j-1) == 1
                    map(i,j-1) = 4;
                end
                if i+1 < size(map,1) && j-1 > 0 && map(i+1,j-1) == 1
                    map(i+1,j-1) = 4;
                end
                if i-1 > 0 && map(i-1,j) == 1
                    map(i-1,j) = 4;
                end
                if i+1 < size(map,1) && map(i+1,j) == 1
                    map(i+1,j) = 4;
                end
                if i-1 > 0 && j+1< size(map,2)&& map(i-1,j+1) == 1
                    map(i-1,j+1) = 4;
                end
                if j+1< size(map,2)&& map(i,j+1) == 1
                    map(i,j+1) = 4;
                end
                if i+1 < size(map,1) && j+1< size(map,2)&&  map(i+1,j+1) == 1
                    map(i+1,j+1) = 4;
                end
            end
        end
    end
    map(map == 4) = 3;
end
%% Move function
function [rotation] = getRotationNextPos(robot_pos, next_pos)
    if next_pos(1) == robot_pos(1) + 1
        if next_pos(2) == robot_pos(2) + 1
            rotation = -pi/4;
        elseif next_pos(2) == robot_pos(2)
            rotation = -pi/2;
        elseif next_pos(2) == robot_pos(2) - 1
            rotation = -3*pi/4;
        end
    elseif next_pos(1) == robot_pos(2)
        if next_pos(2) == robot_pos(2) + 1
            rotation = 0;
        elseif next_pos(2) == robot_pos(2) - 1
            rotation = pi;
        end
    elseif next_pos(1) == robot_pos(1) - 1
        if next_pos(2) == robot_pos(2) + 1
            rotation = pi/4;
        elseif next_pos(2) == robot_pos(2)
            rotation = pi/2;
        elseif next_pos(2) == robot_pos(2) - 1
            rotation = 3*pi/4;
        end
    end
end
