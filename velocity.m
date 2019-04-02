function [x_vel,y_vel,rot_vel] = velocity(position, robot_position, robot_angle)
  % parameters of the velocity
  % TODO tune the parameters
  x_factor = 2.5;
  y_factor = 2.5;
  theta_factor = 0.5;

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

  % apply the factors
  x_vel = -x_factor*robot_vel(1);
  y_vel = y_factor*robot_vel(2);
  rot_vel = theta_factor*rot_vel;
end
