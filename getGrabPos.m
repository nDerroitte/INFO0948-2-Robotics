function [c_tables, c_baskets, a_baskets] = getGrabPos(map, centers, init_pos, a_dist, margin)
  [c_tables, c_baskets] = splitCenters(map, centers, init_pos);
  a_baskets = computeAccess(map, centers, margin);
end

function [c_tables, c_baskets] = splitCenters(map, centers, init_pos)
  % initialise the variables
  c_tables = {};
  c_baskets = centers;
  nb_tables = 2;

  % the tables are supposed to be the closest circles to robot
  min_dist = Inf;
  for i=1:nb_tables
    for j=1:size(centers, 2)
      dist = norm(centers{j} - init_pos);
      if (dist < min_dist)
        min_dist = dist;
        index = j;
      end
    end
    min_dist = Inf;
    % append the table center
    c_tables{end+1} = centers{j};
    % remove the center from the basket list
    c_baskets(j) = [];
  end
end

function [a_baskets] = computeAccess(map, centers, a_dist, margin)
  a_baskets = {};
  % retrieve the closest neighbor for each center
  for i=1:size(centers, 2)
    a_baskets{end+1} = getClosestNeighbor(map, center, a_dist, margin);
  end
end

function [c_neighbor] = getClosestNeighbor(map, center, a_dist, margin)
  % compute the edge
  edge = centers{i} - a_dist;
  edge2edge = 3+2*(a_dist-1)-1;

  % adjust to edge to the map bounds
  if (edge(1) <= 1)
    edge(1) = 1;
  end
  if (edge(2) <= 1)
    edge(2) = 1;
  end
  if ((edge(1) + edge2edge) >= size(map,1))
    edge(1) = size(map,1);
  end
  if ((edge(2) + edge2edge) >= size(map,2))
    edge(2) = size(map,2);
  end

  % return the sub map containing the neighbors of the circle border
  [x, y] = find(map([edge(1):edge(1)+edge2edge],[edge(2):edge(2)+edge2edge]) == 1);
  neighbors = [x, y];
  % convert back in the map coordinates
  % TODO pas sur..
  neighbors = neighbors + edge;

  % keep the closest valid neighbor
  min_dist = a_dist;
  c_neighbor = [];
  for i=1:size(neighbors, 1)
    if (checkMargin(map, neighbors(i,:), margin, 2))
      dist = norm(neighbors(i,:) - center));
      if (dist <= min_dist)
        min_dist = dist;
        c_neighbor = neighbors(i,:);
      end
    end
  end
end