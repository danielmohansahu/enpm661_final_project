function distance = getDistance(path)
% Returns the total cumulative rotations travelled 
% by the given path (degrees).
distance = 0;
for i = 1 : size(path,2)
    distance = distance + sum(abs(rotm2eul(path{i}.rotation)));
end
end % function