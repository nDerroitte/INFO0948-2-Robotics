function [path] = main(map,init_pos)
  path = compute_path(map,init_pos);
end

function test()
  clc

  map = [2, 2, 2, 2, 2, 2, 2, 2, 2;
         2, 1, 1, 1, 1, 1, 1, 1, 2;
         2, 1, 1, 1, 1, 1, 0, 1, 2;
         2, 1, 1, 1, 1, 1, 1, 1, 2;
         2, 1, 1, 2, 2, 1, 1, 1, 2;
         2, 1, 2, 2, 2, 1, 1, 1, 2;
         2, 1, 1, 1, 2, 1, 1, 1, 2;
         2, 1, 1, 1, 2, 1, 1, 0, 2;
         2, 1, 1, 2, 2, 1, 1, 1, 2;
         2, 2, 2, 2, 2, 2, 2, 2, 2];
  disp(map)

  init_pos = [9,4];
  path = compute_path(map,init_pos);

  map_with_path = write_path(map,path,8);
  disp(map_with_path)
end

function [man_dist] = manhattan_distance(pos1, pos2)
  man_dist = abs(pos1(1:end,1)-pos2(1:end,1))+abs(pos1(1:end,2)-pos2(1:end,2));
end

function [exp_pos] = get_exp_pos(map, curr_pos)
  [x,y] = find(map==0);
  zero_pos = [x,y];
  [min_dist,index] = min(manhattan_distance(curr_pos,zero_pos));
  exp_pos = zero_pos(index,1:end);
end

function [cost] = cost(curr_cost, succ_pos, exp_pos)
  cost = curr_cost + 1 + manhattan_distance(succ_pos,exp_pos);
end

function [valid_pos] = check_pos(map,pos)
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

function [path] = compute_path(map, init_pos)
  % initial node of the tree search
  curr_node = {0,init_pos,{}};

  % search for the closest unexplored position
  exp_pos =  get_exp_pos(map,curr_node{2});

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
