function path = backtrack(nodes)

% search variable initialization
current = nodes{end};
path = {};

while current.parent ~= 0
    path = [path, current];
    current = nodes{current.parent};
end

% reverse the path
ridx = size(path,2):-1:1;
path = path(ridx);

end % function