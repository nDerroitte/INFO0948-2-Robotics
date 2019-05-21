function [path] = astar(map, init_pos, dest_pos, margin)
  tic
  % initial node of the tree search
  curr_node = {0,init_pos,{}};

  % initialise the queue and the path
  path = {};
  queue = {};
  queue{1} = curr_node;

  while (size(queue,2) >= 1)
    % pop the next node with the lowest cost
    [curr_node, queue] = popNext(queue);
    curr_pos = curr_node{2};

    % check if the targeted position has been reached
    if (curr_pos == dest_pos)
      path = curr_node{3};
    end

    % add the next nodes to explore
    [map,queue] = addSuccessors(map,queue,curr_node,dest_pos,margin);
  end
  disp('total astar')
  toc
end

function [node, queue] = popNext(queue)
  min_cost = 999999;
  index = 1;
  for i=1:size(queue,2)
    if (queue{i}{1} <= min_cost)
      min_cost = queue{i}{1};
      index = i;
    end
  end
  node = queue{index};
  queue(index) = [];
end

function [diag_move] = isDiagMove(curr_pos, next_pos)
  diff = curr_pos - next_pos;
  if ((diff(1) == 0) || (diff(2) == 0))
    diag_move = 0;
  else
    diag_move = 1;
  end
end

function [cost] = cost(curr_cost, curr_pos, succ_pos, dest_pos)
  if (isDiagMove(curr_pos, succ_pos))
    % TODO fucking diag..
    cost = curr_cost + 50 + manhattanDistance(succ_pos,dest_pos);
    %cost = curr_cost + sqrt(2) + manhattanDistance(succ_pos,dest_pos);
  else
    cost = curr_cost + 1 + manhattanDistance(succ_pos,dest_pos);
  end
end

function [validity] = checkPos(map, pos, margin)
  if checkMargin(map,pos,margin,2) && (map(pos(1),pos(2)) ~= -1)
    validity = 1;
  else
    validity = 0;
  end
end

function [queue] = appendNode(queue, curr_node, succ_pos, dest_pos)
  path = curr_node{3};
  path{end+1} = succ_pos;
  queue{end+1} = {cost(curr_node{1}, curr_node{2}, succ_pos, dest_pos), succ_pos, path};
end

function [map,queue] = addSuccessors(map, queue, curr_node, dest_pos, margin)
  map_size = size(map);
  curr_pos = curr_node{2};

  succ_pos =  [curr_pos(1)+1, curr_pos(2)];
  if (succ_pos(1) <= map_size(1)) && checkPos(map,succ_pos,margin)
    map(succ_pos(1),succ_pos(2)) = -1;
    queue = appendNode(queue,curr_node,succ_pos,dest_pos);
  end

  succ_pos =  [curr_pos(1), curr_pos(2)+1];
  if (succ_pos(2) <= map_size(2)) && checkPos(map,succ_pos,margin)
    map(succ_pos(1),succ_pos(2)) = -1;
    queue = appendNode(queue,curr_node,succ_pos,dest_pos);
  end

  succ_pos =  [curr_pos(1)+1, curr_pos(2)+1];
  if (succ_pos(1) <= map_size(1)) && (succ_pos(2) <= map_size(2)) && checkPos(map,succ_pos,margin)
    map(succ_pos(1),succ_pos(2)) = -1;
    queue = appendNode(queue,curr_node,succ_pos,dest_pos);
  end

  succ_pos =  [curr_pos(1)-1, curr_pos(2)];
  if (succ_pos(1) >= 1) && checkPos(map,succ_pos,margin)
    map(succ_pos(1),succ_pos(2)) = -1;
    queue = appendNode(queue,curr_node,succ_pos,dest_pos);
  end

  succ_pos =  [curr_pos(1), curr_pos(2)-1];
  if (succ_pos(2) >= 1) && checkPos(map,succ_pos,margin)
    map(succ_pos(1),succ_pos(2)) = -1;
    queue = appendNode(queue,curr_node,succ_pos,dest_pos);
  end

  succ_pos =  [curr_pos(1)-1, curr_pos(2)-1];
  if (succ_pos(1) >= 1) && (succ_pos(2) >= 1) && checkPos(map,succ_pos,margin)
    map(succ_pos(1),succ_pos(2)) = -1;
    queue = appendNode(queue,curr_node,succ_pos,dest_pos);
  end

  succ_pos =  [curr_pos(1)+1, curr_pos(2)-1];
  if (succ_pos(1) <= map_size(1)) && (succ_pos(2) >= 1) && checkPos(map,succ_pos,margin)
    map(succ_pos(1),succ_pos(2)) = -1;
    queue = appendNode(queue,curr_node,succ_pos,dest_pos);
  end

  succ_pos =  [curr_pos(1)-1, curr_pos(2)+1];
  if (succ_pos(1) >= 1) && (succ_pos(2) <= map_size(2)) && checkPos(map,succ_pos,margin)
    map(succ_pos(1),succ_pos(2)) = -1;
    queue = appendNode(queue,curr_node,succ_pos,dest_pos);
  end
end
