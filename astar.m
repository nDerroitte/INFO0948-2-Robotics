function [path] = main(map, init_pos)
  path = compute_astar(map,init_pos);
  %test();
end

function test()
  clc
  map = [2, 2, 2, 2, 2, 2, 2, 2, 2;
         2, 1, 1, 1, 1, 1, 2, 0, 2;
         2, 1, 1, 1, 1, 1, 2, 2, 2;
         2, 1, 1, 1, 1, 1, 1, 1, 2;
         2, 1, 1, 2, 2, 1, 1, 1, 2;
         2, 1, 2, 2, 2, 1, 1, 1, 2;
         2, 1, 1, 1, 2, 1, 1, 2, 2;
         2, 1, 1, 1, 2, 1, 2, 0, 2;
         2, 1, 1, 2, 2, 1, 2, 1, 2;
         2, 2, 2, 2, 2, 2, 2, 2, 2];
  disp(map)
  init_pos = [9,4];

  path = compute_astar(map,init_pos);

  map_with_path = write_path(map,path,8);
  disp(map_with_path);
end

% not used for now
function [map] = fill_no_exp(map, pos)
  map_size = size(map);
  map(pos(1),pos(2)) = 3;

  curr_pos =  [pos(1)+1, pos(2)];
  if (curr_pos(1) <= map_size(1))
    map(curr_pos(1),curr_pos(2)) = 3;
  end

  curr_pos =  [pos(1), pos(2)+1];
  if (curr_pos(2) <= map_size(2))
    map(curr_pos(1),curr_pos(2)) = 3;
  end

  curr_pos =  [pos(1)+1, pos(2)+1];
  if (curr_pos(1) <= map_size(1)) && (curr_pos(2) <= map_size(2))
    map(curr_pos(1),curr_pos(2)) = 3;
  end

  curr_pos =  [pos(1)-1, pos(2)];
  if (curr_pos(1) >= 1)
    map(curr_pos(1),curr_pos(2)) = 3;
  end

  curr_pos =  [pos(1), pos(2)-1];
  if (curr_pos(2) >= 1)
    map(curr_pos(1),curr_pos(2)) = 3;
  end

  curr_pos =  [pos(1)-1, pos(2)-1];
  if (curr_pos(1) >= 1) && (curr_pos(2) >= 1)
    map(curr_pos(1),curr_pos(2)) = 3;
  end

  curr_pos =  [pos(1)+1, pos(2)-1];
  if (curr_pos(1) <= map_size(1)) && (curr_pos(2) >= 1)
    map(curr_pos(1),curr_pos(2)) = 3;
  end

  curr_pos =  [pos(1)-1, pos(2)+1];
  if (curr_pos(1) >= 1) && (curr_pos(2) <= map_size(2))
    map(curr_pos(1),curr_pos(2)) = 3;
  end
end

function [path] = compute_astar(map, init_pos)
  path ={};
  % search for the unexplored positions
  exp_array =  get_exp_array(map);
  while (size(path) < 1)
    if (size(exp_array,1) < 1)
      break;
    end
    [exp_pos, exp_array] = pop_next_exp(exp_array,init_pos);
    path = compute_path(map,init_pos,exp_pos);
  end
end

function [man_dist] = manhattan_distance(pos1, pos2)
  man_dist = abs(pos1(1:end,1)-pos2(1:end,1))+abs(pos1(1:end,2)-pos2(1:end,2));
end

function [exp_pos,exp_array] = pop_next_exp(exp_array, init_pos)
  [min_dist,index] = min(manhattan_distance(init_pos,exp_array));
  exp_pos = exp_array(index,1:end);
  exp_array([index],:) = [];
end

function [exp_array] = get_exp_array(map)
  [x,y] = find(map==0);
  exp_array = [x,y];
end

function [cost] = cost(curr_cost, succ_pos, exp_pos)
  cost = curr_cost + 1 + manhattan_distance(succ_pos,exp_pos);
end

function [valid_pos] = check_pos(map, pos)
  valid_pos = (map(pos(1),pos(2)) ~= 2) && (map(pos(1),pos(2)) ~= 3);
end

function [queue] = append_node(queue, curr_node, succ_pos, exp_pos)
  path = curr_node{3};
  path{end+1} = succ_pos;
  queue{end+1} = {cost(curr_node{1}, succ_pos, exp_pos), succ_pos, path};
end

function [map,queue] = add_successors(map, queue, curr_node, exp_pos)
  map_size = size(map);
  curr_pos = curr_node{2};

  succ_pos =  [curr_pos(1)+1, curr_pos(2)];
  if (succ_pos(1) <= map_size(1)) && check_pos(map,succ_pos)
    map(succ_pos(1),succ_pos(2)) = 3;
    queue = append_node(queue,curr_node,succ_pos,exp_pos);
  end

  succ_pos =  [curr_pos(1), curr_pos(2)+1];
  if (succ_pos(2) <= map_size(2)) && check_pos(map,succ_pos)
    map(succ_pos(1),succ_pos(2)) = 3;
    queue = append_node(queue,curr_node,succ_pos,exp_pos);
  end

  succ_pos =  [curr_pos(1)+1, curr_pos(2)+1];
  if (succ_pos(1) <= map_size(1)) && (succ_pos(2) <= map_size(2)) && check_pos(map,succ_pos)
    map(succ_pos(1),succ_pos(2)) = 3;
    queue = append_node(queue,curr_node,succ_pos,exp_pos);
  end

  succ_pos =  [curr_pos(1)-1, curr_pos(2)];
  if (succ_pos(1) >= 1) && check_pos(map,succ_pos)
    map(succ_pos(1),succ_pos(2)) = 3;
    queue = append_node(queue,curr_node,succ_pos,exp_pos);
  end

  succ_pos =  [curr_pos(1), curr_pos(2)-1];
  if (succ_pos(2) >= 1) && check_pos(map,succ_pos)
    map(succ_pos(1),succ_pos(2)) = 3;
    queue = append_node(queue,curr_node,succ_pos,exp_pos);
  end

  succ_pos =  [curr_pos(1)-1, curr_pos(2)-1];
  if (succ_pos(1) >= 1) && (succ_pos(2) >= 1) && check_pos(map,succ_pos)
    map(succ_pos(1),succ_pos(2)) = 3;
    queue = append_node(queue,curr_node,succ_pos,exp_pos);
  end

  succ_pos =  [curr_pos(1)+1, curr_pos(2)-1];
  if (succ_pos(1) <= map_size(1)) && (succ_pos(2) >= 1) && check_pos(map,succ_pos)
    map(succ_pos(1),succ_pos(2)) = 3;
    queue = append_node(queue,curr_node,succ_pos,exp_pos);
  end

  succ_pos =  [curr_pos(1)-1, curr_pos(2)+1];
  if (succ_pos(1) >= 1) && (succ_pos(2) <= map_size(2)) && check_pos(map,succ_pos)
    map(succ_pos(1),succ_pos(2)) = 3;
    queue = append_node(queue,curr_node,succ_pos,exp_pos);
  end
end

function [node, queue] = pop_next(queue)
  min_cost = 999999;
  index = 1;
  for i=1:size(queue)
    if (queue{i}{1} <= min_cost)
      min_cost = queue{i}{1};
      index = i;
    end
  end
  node = queue{i};
  queue(i) = [];
end

function [path] = compute_path(map, init_pos, exp_pos)
  % initial node of the tree search
  curr_node = {0,init_pos,{}};

  % initialise the queue and the path
  path = {};
  queue = {};
  queue{1} = curr_node;

  while (size(queue) >= 1)
    % pop the next node with the lowest cost
    [curr_node, queue] = pop_next(queue);
    curr_pos = curr_node{2};

    % check if the targeted position has been reached
    if (curr_pos == exp_pos)
      path = curr_node{3};
    end

    % add the next nodes to explore
    [map,queue] = add_successors(map,queue,curr_node,exp_pos);
  end
end

function [map] = write_path(map,path,index)
  for i=1:max(size(path))
    map(path{i}(1),path{i}(2)) = index;
  end
end
