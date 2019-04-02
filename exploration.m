function [map] = exploration(vrep, id, h)
    % initialise the simulation
    timestep = .05;
    [res, init_robot_pos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    [res, ~] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);

    % initialise the map
    global map map_origin map_size round_parameter;
    map = zeros(1, 1);
    map_size = size(map);
    map_origin = init_robot_pos([1;2])';
    round_parameter = 0.1;

    % impose margin between the robot trajectory and the obstacles
    margin = 4;
    traj = {};

    % set the initial rotation before exploration
    rotate_angle = pi;
    fsm = 'rotate';

    %% Start the exploration.
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
        % j'ai du rajouter le +1 mais yavait pas avant, bizarre...
        abs_robot_pos = round((1/round_parameter) * (robot_pos - map_origin))+1;
        robot_angle = robot_angle(3); % discard unneeded angles

        % read obstacle sensor data
        [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
        pts = pts(1:2,:); % discard z

        % update the map
        updateMap(h, pts, contacts, robot_pos, robot_angle);

        if strcmp(fsm, 'rotate')
            rot_vel = angdiff(rotate_angle, robot_angle);

            if abs(angdiff(rotate_angle, robot_angle)) < .1 / 180 * pi
                fsm = 'createTarget';
            end

            h = youbot_drive(vrep, h, 0, 0, rot_vel);

        elseif strcmp(fsm, 'createTarget')
            % compute the trajectory
            traj = computePath(map,abs_robot_pos',margin);

            % if the trajectory is empty, the exploration is finished
            if size(traj,2) < 1
              fsm ='finished';
            else
              abs_next_pos = traj{1};
              traj(1) = [];
              fsm = 'moveToTarget';
            end

        elseif strcmp(fsm, 'moveToTarget')
            % next position in the coordinates of the simulation
            next_pos = abs2sim(abs_next_pos, map_origin, round_parameter);

            % check if close enough to the position
            if (size(traj,2) >= 1 && continuePath(map, traj, margin))
              if (norm(next_pos'-robot_pos) < .1)
                  abs_next_pos = traj{1};
                  traj(1) = [];
                  next_pos = abs2sim(abs_next_pos, map_origin, round_parameter);
              end
              % compute and apply the velocities
              [x_vel, y_vel, rot_vel] = velocity(next_pos, robot_pos, robot_angle);
              h = youbot_drive(vrep, h, x_vel, y_vel, rot_vel);
            else
              h = youbot_drive(vrep, h, 0, 0, 0);
              fsm = 'stop';
            end

        elseif strcmp(fsm, 'stop')
          for i=1:20
            h = youbot_drive(vrep, h, 0, 0, 0);
          end
          fsm = 'createTarget';

        elseif strcmp(fsm, 'finished')
          disp('Exploration finished!')
          break;
        end

        % display the updated map
        displayMap(map,traj,abs_robot_pos);

        % each iteration must take roughly 50 ms
        elapsed = toc;
        timeleft = timestep - elapsed;
        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end
end

function [sim_position] = abs2sim(abs_position, map_origin, round_parameter)
  sim_position = bsxfun(@plus, round_parameter*abs_position, + map_origin');
end

%% Update map
function updateMap(h, pts, contacts, robot_pos, robot_angle)
    global map round_parameter map_origin
    % rotate the points and translate them to be on the map
    rotation_matrix = [cos(robot_angle), -sin(robot_angle);  sin(robot_angle), cos(robot_angle)];

    absolute_index_pts = round((1/round_parameter) * bsxfun(@plus, rotation_matrix * pts, robot_pos - map_origin));

    hokuyoPositions = [h.hokuyo1Pos(1), h.hokuyo1Pos(2); h.hokuyo2Pos(1), h.hokuyo2Pos(2)];
    hokuyoPositions = round((1/round_parameter) * bsxfun(@plus, rotation_matrix * hokuyoPositions, robot_pos - map_origin));
    max_x = max(absolute_index_pts(1,:));
    max_y = max(absolute_index_pts(2,:));
    min_x = min(absolute_index_pts(1,:));
    min_y = min(absolute_index_pts(2,:));

    max_x_extend = max(max_x - size(map, 1), 0);
    max_y_extend = max(max_y - size(map, 2), 0);
    min_x_extend = max(1 - min_x, 0);
    min_y_extend = max(1 - min_y, 0);

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
function displayMap(map, traj, robot_pos)
    % clear previous plot
    clf;

    % plot obstacles
    [obstacle_x, obstacle_y] = find(map == 2);
    plot(obstacle_x, obstacle_y, 'xr');
    hold on;

    % plot free spaces
    [free_space_x, free_space_y] = find(map == 1);
    plot(free_space_x, free_space_y, 'xb');

    % plot trajectory
    for i=1:size(traj,2)
      plot(traj{i}(1), traj{i}(2), 'xm');
    end

    % plot robot
    plot(robot_pos(1), robot_pos(2), 'oy');

    drawnow;
end

%% astar
function [path] = computePath(map, init_pos, margin)
  path ={};
  % search for the unexplored positions
  exp_array =  getExpArray(map);
  while (size(path,2) < 1)
    if (size(exp_array,1) < 1)
      break;
    end
    [exp_pos, exp_array] = popNextExp(exp_array,init_pos);
    % check if the unexplored position is valid
    if (checkExp(map,exp_pos,margin))
      % compute the path using A*
      path = astar(map,init_pos,exp_pos,margin);
    end
  end
end

function [exp_pos,exp_array] = popNextExp(exp_array, init_pos)
  [~,index] = min(manhattanDistance(init_pos,exp_array));
  exp_pos = exp_array(index,1:end);
  exp_array([index],:) = [];
end

% TODO il faut une meilleure strategie pour choisir le prochain point
function [exp_array] = getExpArray(map)
  [x,y] = find(map==0);
  exp_array = [x,y];
end

function [validity] = checkExp(map, exp_pos, margin)
  validity = 0;
  map_size = size(map);

  if ~checkMargin(map, exp_pos, margin, 2)
    return;
  end

  neighbor =  [exp_pos(1)+1, exp_pos(2)];
  if (neighbor(1) <= map_size(1))
    if (map(neighbor(1),neighbor(2)) == 1)
      validity = 1;
      return;
    end
  end

  neighbor =  [exp_pos(1), exp_pos(2)+1];
  if (neighbor(2) <= map_size(2))
    if (map(neighbor(1),neighbor(2)) == 1)
      validity = 1;
      return;
    end
  end

  neighbor =  [exp_pos(1)+1, exp_pos(2)+1];
  if (neighbor(1) <= map_size(1)) && (neighbor(2) <= map_size(2))
    if (map(neighbor(1),neighbor(2)) == 1)
      validity = 1;
      return;
    end
  end

  neighbor =  [exp_pos(1)-1, exp_pos(2)];
  if (neighbor(1) >= 1)
    if (map(neighbor(1),neighbor(2)) == 1)
      validity = 1;
      return;
    end
  end

  neighbor =  [exp_pos(1), exp_pos(2)-1];
  if (neighbor(2) >= 1)
    if (map(neighbor(1),neighbor(2)) == 1)
      validity = 1;
      return;
    end
  end

  neighbor =  [exp_pos(1)-1, exp_pos(2)-1];
  if (neighbor(1) >= 1) && (neighbor(2) >= 1)
    if (map(neighbor(1),neighbor(2)) == 1)
      validity = 1;
      return;
    end
  end

  neighbor =  [exp_pos(1)+1, exp_pos(2)-1];
  if (neighbor(1) <= map_size(1)) && (neighbor(2) >= 1)
    if (map(neighbor(1),neighbor(2)) == 1)
      validity = 1;
      return;
    end
  end

  neighbor =  [exp_pos(1)-1, exp_pos(2)+1];
  if (neighbor(1) >= 1) && (neighbor(2) <= map_size(2))
    if (map(neighbor(1),neighbor(2)) == 1)
      validity = 1;
      return;
    end
  end
end

function [validity] = continuePath(map, path, margin)
  if ((size(path,2) <= 10) && (map(path{end}(1),path{end}(2)) ~= 0))
    validity = 0;
  else
    validity = checkMargin(map,path{1},margin,2);
  end
end
