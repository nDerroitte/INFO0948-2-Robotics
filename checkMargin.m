function [validity] = checkMargin(map, pos, margin, value)
  edge = pos - margin;
  edge2edge = 3+2*(margin-1)-1;
  if (checkBounds(map, edge, edge2edge))
    sub_map = map([edge(1):edge(1)+edge2edge],[edge(2):edge(2)+edge2edge]);
    if (size(find(sub_map == value),1) > 0)
      validity = 0;
    else
      validity = 1;
    end
  else
    validity = 0;
  end
end
