%% Object Centred
% Check if the object of given color is at the centre of the image
% Color : 1 is red, 2 is green, 3 is blue and 4 is yellow, 5 is purple, 6
% is white. 0 if fail
% Res is a boolean containing the result
% dir is 0 if centred, -1 if the object is too much on the right, 1 too
% much on the left
function [res, dir] = object_centred(color)
    I = imread('image.png');
    real_centre = 256;
    if color == 1
        [~, y] = find(I(:, :, 1) > 200  & I(:, :, 2) < 100 & I(:,:,3) < 100);
    elseif color == 2
        [~, y] = find(I(:, :, 1) < 100  & I(:, :, 2) > 200 & I(:,:,3) < 100);
    elseif color == 3
        [~, y] = find(I(:, :, 1) < 100  & I(:, :, 2) < 100 & I(:,:,3) > 200);
    elseif color == 4
        [~, y] = find(I(:, :, 1) > 200  & I(:, :, 2) > 200 & I(:,:,3) < 100);
    elseif color == 5
        [~, y] = find(I(:, :, 1) > 200  & I(:, :, 2) < 100 & I(:,:,3) > 200);
    elseif color == 6
        [~, y] = find(I(:, :, 1) > 200  & I(:, :, 2) > 200 & I(:,:,3) > 200);
    end
    max_y = max(y);
    min_y = min(y);
    centre = (max_y + min_y)/2;

    if abs(real_centre - centre) <= 5
        res = 1;
        dir = 0;
        return;
    end
    if centre > real_centre
        dir = -1;
    else
        dir = 1;
    end

    res = 0;
    return;
end
