function [validity] = checkBounds(map, edge, edge2edge)
  validity = 1;
  if ~(edge(1) >= 1 && edge(2) >= 1)
    validity = 0;
    return;
  end

  if ~((edge(1) + edge2edge) <= size(map,1))
    validity = 0;
    return;
  end

  if ~((edge(2) + edge2edge) <= size(map,2))
    validity = 0;
    return;
  end
end
