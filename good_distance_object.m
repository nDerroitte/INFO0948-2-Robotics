%% Good distance with the object
% Check if the robot is at the good distance to grab the object or not. 
% Color : 1 is red, 2 is green, 3 is blue and 4 is yellow. 5 and 6 is
% purple and white. 0 if fail
% Res is a boolean containing the result
% dir is 0 if facing, 1 if too close, -1 if too far 
function [res, dir] = good_distance_object(color)
    dir = 0;
    I = imread('image.png');
    if color == 1
        [x, ~] = find(I(:, :, 1) > 200  & I(:, :, 2) < 100 & I(:,:,3) < 100); 
    elseif color == 2
        [x, ~] = find(I(:, :, 1) < 100  & I(:, :, 2) > 200 & I(:,:,3) < 100);
    elseif color == 3
        [x, ~] = find(I(:, :, 1) < 100  & I(:, :, 2) < 100 & I(:,:,3) > 200);
    elseif color == 4
        [x, ~] = find(I(:, :, 1) > 200  & I(:, :, 2) > 200 & I(:,:,3) < 100);
    elseif color == 5
        [x, ~] = find(I(:, :, 1) > 200  & I(:, :, 2) < 100 & I(:,:,3) > 200);
    elseif color == 6
        [x, ~] = find(I(:, :, 1) > 200  & I(:, :, 2) > 200 & I(:,:,3) > 200);
    end
    
    max_x = max(x);
    
    if max_x < 410 && max_x > 400
        res = 1;
        return;
    else
        if max_x > 400
            res = 0;
            dir = 1;
        else 
            res = 0;
            dir = -1;
        end
    end
            
end