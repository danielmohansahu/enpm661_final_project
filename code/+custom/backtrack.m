function [path, rRotateMatrix, rHeights] = backtrack(nodes, goal_node)

search = true;
path = [];
parent = nodes.nodes(:,:,end);
parent_height = nodes.height(end);
heights = {};
heights{end+1} = nodes.height(end);
rotateMatrix = {};
rotateMatrix{end+1} = nodes.rotationM{end};

while (search)
    index = 0;
    for i = 1 : size(nodes.nodes, 3)
        if (all(parent == nodes.nodes(:,:,i)) & parent_height == nodes.height(i))
            index = i;
            break;
        end
    end    
    if index == 0
        break;
    end
    
    parent = nodes.parents(:,:,index);
    parent_height = nodes.parent_height(index);
    path(:,:,end+1) = parent;
    
    nodes.parents(:,:,index) = [];
    nodes.parent_height(index) = [];
    nodes.nodes(:,:,index) = [];
    nodes.cost(:, index) = [];
    
    rotateMatrix{:,end+1} = nodes.rotationM{index};
    nodes.rotationM(index) = [];
    heights{:,end+1} = nodes.height(index);
    nodes.height(index) = [];
    if all(parent == goal_node)
        search = false;
        break;
    end
end
%% Reverse the path
rPath = path;
path = [];
rRotateMatrix = {};
rHeights = {};
for i = 1 : size(rPath, 3)
    path(:,:,end+1) = rPath(:,:,end+1-i);
    rRotateMatrix{end+1} = rotateMatrix{end+1-i};
    rHeights{end+1} = heights{end+1-i};
end

end % function