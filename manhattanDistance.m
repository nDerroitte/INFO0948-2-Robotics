function [man_dist] = manhattanDistance(pos1, pos2)
  man_dist = sum(abs(pos1 - pos2));
end
