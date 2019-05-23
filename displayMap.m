%% Plot map
function displayMap(map, traj, robot_pos, centers, radius)
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
    if (size(robot_pos, 2) > 0)
      plot(robot_pos(1), robot_pos(2), 'xg');
    end
    % draw the circles
    for i=1:size(centers, 2)
      viscircles(centers{i}, radius, 'Color','b');
    end
    % display the plot
    drawnow;
end
