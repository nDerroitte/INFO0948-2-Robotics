function exploration(vrep, id, h) 
    timestep = .05;
    global round_parameter round_decimals
    initsize = 1;
    round_parameter = 0.1;
    round_decimals = -log10(round_parameter);
    global map map_origin map_size
    map = zeros(initsize, initsize);
    map_size = [initsize, initsize];
    map_origin
    i = 0;
    %% Start the exploration. 
    while i < 1000
        tic  
        
        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end
    
        % Get the position and the orientation of the ROBOT. 
        % youbotPos : [x, y, z]
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        % J'imagine les 3 angles d'Euler .. 
        %[res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        
        % pts : 3 x 684 avec [[x ,y ,z]] -> tous les points vu 
        % contacts: une matrice 1x 684  -> 0 si pas de wall et 1 si wall
        [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
        updateMap(pts, contacts)        
        
        i = i +1;
        % /!\ Velocity backward!!!
        h = youbot_drive(vrep, h, 0, 0, 0);
        if youbotPos(2) > 0
            break;
        end
        % Make sure that we do not go faster than the physics simulation (each iteration must take roughly 50 ms). 
        elapsed = toc;
        timeleft = timestep - elapsed;
        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end
end
%% Update map
 function updateMap(pts, contacts)
    global map round_decimals
    
 end
  
%     sizes = size(map);
%     max_x_index = sizes(1);
%     max_y_index = sizes(2);
%     max_x = indexXToValue(max_x_index);
%     min_x = indexXToValue(1);
%     max_y = indexYToValue(max_y_index);
%     min_y = indexYToValue(1);
%     
%     maxs = max(pts');
%     new_max_x = maxs(1);
%     new_max_y = maxs(2);
%     mins = min(pts');
%     new_min_x = mins(1);
%     new_min_y = mins(2);
%     max_x_extend = round(new_max_x, round_decimals);
%     max_y_extend = round(new_max_y, round_decimals);
%     min_x_extend = round(new_min_x, round_decimals);
%     min_y_extend = round(new_min_y, round_decimals);
%     map = updateSizeMap(max_x_extend, max_x, max_x_index, 1, 1);
%     map = updateSizeMap(max_y_extend, max_y, max_y_index, 2, 1);
%     map = updateSizeMap(min_x_extend, min_x, 1, 1, 2);
%     map = updateSizeMap(min_y_extend, min_y, 1, 2, 2);
%     size(map)
%     
%  end
%  function [real_value] = indexXToValue(index)
%     global map_translate_x round_parameter
%     real_index = index - map_translate_x;
%     real_value = real_index * round_parameter;
%  end
%  function [real_value] = indexYToValue(index)
%     global map_translate_y round_parameter
%     real_index = index - map_translate_y;
%     real_value = real_index * round_parameter;
%  end
%  
%  function [new_map] = updateSizeMap(extend, optimum, optimum_index, i, m)
%     global map_size map round_parameter map_translate_x map_translate_y
%     if m == 1
%         if extend > optimum
%           real_extend = extend - optimum;
%           index_extend = real_extend / round_parameter;
%           map_size(i) = round(map_size(i) + index_extend,0);
%           new_map = zeros(map_size(1), map_size(2));
%           if i == 1
%              new_map(1:optimum_index,:) = map;
%           else
%             new_map(:,1:optimum_index) = map;
%           end
%         else 
%             new_map = map;
%         end
%     else
%         if extend < optimum
%           real_extend = extend + optimum;
%           if i == 1
%               map_translate_x = abs(extend);
%           else
%               map_translate_y = abs(extend);
%           end
%           index_extend = abs(real_extend) / round_parameter;
%           map_size(i) = round(map_size(i) + index_extend,0);
%           optimum_index = round(optimum_index + index_extend,0);
%           new_map = zeros(map_size(1), map_size(2));
%           if i == 1
%               try
%                 new_map(optimum_index:end,:) = map;
%               catch
%                   a = 9;
%               end
%           else
%               try
%                 new_map(:,optimum_index:end) = map;
%               catch
%                   a = 9;
%               end
%           end
%         else
%             new_map = map;
%         end
%     end
%  end