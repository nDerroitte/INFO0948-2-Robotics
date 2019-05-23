function [centers] = findCircles(map, radius, c_factor)
  % initialize the candidates
  [x, y] = find(map == 0);
  candidates = [x, y];

  % compute the clusters of the points
  clusters = computeClusters(candidates, radius, c_factor);

  centers = {};
  for i=1:size(clusters, 2)
    % initialize the variables
    best_candidate = [];
    best_score = 0;

    for j=1:size(clusters{i}, 2)
      % current center candidate
      candidate = clusters{i}{j};

      % edge of the sub map
      edge = candidate - radius;
      edge2edge = 3+2*(radius-1)-1;

      % try to built the sub map
      if (checkBounds(map, edge, edge2edge))
        sub_map = map([edge(1):edge(1)+edge2edge],[edge(2):edge(2)+edge2edge]);
        % possible area of the circle
        [x, y] = find(sub_map == 2);
        borders = [x, y];
        % compute the score of the center candidate
        score = computeScore([radius+1 radius+1], borders, radius);
        % keep the best center so far for the current cluster
        if (score > best_score)
          best_score = score;
          best_candidate = candidate;
        end
      end
    end

    % add center if one has been found
    if (size(best_candidate) ~= 0)
      centers{end+1} = best_candidate;
    end
  end
end

function [score] = computeScore(pos, borders, radius)
  dists = sqrt(sum((pos - borders).^2, 2));
  % we want to minimize the difference between the distance and the radius
  diff_radius = abs(dists - radius);
  score = 1/(mean(diff_radius));
end

function [matrix] = cell2matrix(cell_array)
  array_size = size(cell_array, 2);
  matrix = zeros(array_size, 2);

  for i=1:array_size
    matrix(i,:) = cell_array{i};
  end
end

function [clusters] = computeClusters(points, radius, c_factor)
  clusters = {};
  c_means = {};

  % initial cluster
  if (size(size(points, 1)))
    clusters{end+1} = {points(1,:)};
    c_means{end+1} = points(1,:);
  end

  for i=2:size(points, 1)
    p = points(i,:);
    % index of the closest cluster
    [dist, index] = min(sqrt(sum((points(i,:) - cell2matrix(c_means)).^2, 2)));

    % check if create a new cluster
    if ((dist > radius*c_factor) || size(clusters, 2) < 1)
      clusters{end+1} = {};
      clusters{end}{end+1} = points(i,:);
      c_means{end+1} = points(i,:);
    % put the point in an existing cluster
    else
      clusters{index}{end+1} = points(i,:);
      % recompute the mean of the cluser
      c_size = size(clusters{index}, 2);
      c_mean = [0 0];
      for j=1:c_size
        c_mean = c_mean + clusters{index}{j};
      end
        c_means{index} = c_mean/c_size;
    end
  end
end
