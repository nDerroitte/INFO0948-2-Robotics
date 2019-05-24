%% Image foreground. 
% Detects if the object is in foreground or not
% Res is a boolean containing the result
% Color : 1 is red, 2 is green, 3 is blue, 4 is yellow, 5 is purple, 6
% is white. 0 if fail
% sucessful
function [res, color] = image_foreground()
    I = imread('image.png');
    [x_rp, ~] = find(I(:, :, 1) > 200  & I(:, :, 2) < 100 & I(:,:,3) < 100);
    [x_gp, ~] = find(I(:, :, 1) < 100  & I(:, :, 2) > 200 & I(:,:,3) < 100);
    [x_bp, ~] = find(I(:, :, 1) < 100  & I(:, :, 2) < 100 & I(:,:,3) > 200);
    [x_yp, ~] = find(I(:, :, 1) > 200  & I(:, :, 2) > 200 & I(:,:,3) < 100);
    [x_pp, ~] = find(I(:, :, 1) > 200  & I(:, :, 2) < 100 & I(:,:,3) > 200);
    [x_wp, ~] = find(I(:, :, 1) > 200  & I(:, :, 2) > 200 & I(:,:,3) > 200);

    % One return by color to input it.
    max_x_rp = max(x_rp);
    if max_x_rp >= 375
        res = 1;
        color = 1;
        return;
    end
    max_x_gp = max(x_gp);
    if max_x_gp >= 375
        res = 1;
        color = 2;
        return;
    end
    max_x_bp = max(x_bp);
    if max_x_bp >= 375
        res = 1;
        color = 3;
        return;
    end
    
    max_x_yp = max(x_yp);
    if max_x_yp >= 375
        res = 1;
        color = 4;
        return;
    end
    
    max_x_pp = max(x_pp);
    if max_x_pp >= 375
        res = 1;
        color = 5;
        return;
    end
    
    max_x_wp = max(x_wp);
    if max_x_wp >= 375
        res = 1;
        color = 6;
        return;
    end
    
    % Default case
    res = 0;
    color = 0;
    return;
end