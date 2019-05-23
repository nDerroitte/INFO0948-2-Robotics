function [x_vel,y_vel,rot_vel] = velocity(position, robot_position, robot_angle)
  % absolute translation velocity
  abs_vel = position' - robot_position;
  % compute the rotation matrix
  if robot_angle >= 0
    alpha = robot_angle;
  else
    alpha = 2*pi + robot_angle;
  end
  theta = alpha - pi/2;
  rotation_matrix = [cos(theta), sin(theta);
                     -sin(theta), cos(theta)];
  % relative translation velocity
  robot_vel = rotation_matrix*abs_vel;

  % absolute rotation velocity
  abs_rot_vel = atan2(abs_vel(2), abs_vel(1))+ pi/2;
  % relative rotation velocity
  rot_vel = angdiff(abs_rot_vel, robot_angle);

  % apply factors to the velocities
  if (abs(rot_vel) > pi/3)
    % if the angle is too big, mainly rotate
    x_vel = -0.25*robot_vel(1);
    y_vel = 0.25*robot_vel(2);
    rot_vel = 0.6*rot_vel;
  elseif (abs(rot_vel) > pi/6)
    % if the angle is medium, rotate more
    x_vel = -1*robot_vel(1);
    y_vel = 1*robot_vel(2);
    rot_vel = 0.5*rot_vel;
  else
    % apply the normal factors
    x_vel = -1.5*robot_vel(1);
    y_vel = 1.5*robot_vel(2);
    rot_vel = 0.4*rot_vel;
  end
end
