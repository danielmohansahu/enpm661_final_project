function [path, rRotateMatrix] = backtrack(nodes, goal_node)

search = true;
path = [];
parent = nodes.nodes(:,:,end);
rotateMatrix = {};
rotateMatrix{end+1} = nodes.rotationM{end};

while (search)
    index = 0;
    for i = 1 : size(nodes.nodes, 3)
        if all(parent == nodes.nodes(:,:,i))
            index = i;
            break;
        end
    end
    parent = nodes.parents(:,:,index);
    path(:,:,end+1) = parent;
    
    nodes.parents(:,:,index) = [];
    nodes.nodes(:,:,index) = [];
    nodes.cost(:, index) = [];
    
    rotateMatrix{:,end+1} = nodes.rotationM{index};
    nodes.rotationM(index) = [];
    if all(parent == goal_node)
        search = false;
        break;
    end
end
%% Reverse the path
rPath = path;
path = [];
rRotateMatrix = {};
for i = 1 : size(rPath, 3)
    path(:,:,end+1) = rPath(:,:,end+1-i);
    rRotateMatrix{end+1} = rotateMatrix{end+1-i};
end

end % function