function [man_dist] = manhattanDistance(pos1, pos2)
  man_dist = abs(pos1(1:end,1)-pos2(1:end,1))+abs(pos1(1:end,2)-pos2(1:end,2));
end
