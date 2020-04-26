function [pathMid, rRotateMatrix] = backtrack(nodes, vecMid2, vecLeft2, vecRight2)

search = true;
pathMid = [];
parentMid = nodes.nodeMid(:, end);
pathMid(:, end+1) = parentMid;
pathLeft = [];
parentLeft = nodes.nodeLeft(:, end);
pathLeft(:, end+1) = parentLeft;
pathRight = [];
parentRight = nodes.nodeRight(:, end);
pathRight(:, end+1) = parentRight;
rotateMatrix = {};
rotateMatrix{end+1} = nodes.rotationM{end};
while (search)
    found = false;
    index = 0;
    for i = 1 : size(nodes.nodeMid, 2)
        if (parentMid == nodes.nodeMid(:, i))
            if (parentLeft == nodes.nodeLeft(:, i))
                if (parentRight == nodes.nodeRight(:, i))
                    found = true;
                    index = i;
                    break;
                end
            end
        end
    end
    parentMid = nodes.parentMid(:, index);
    pathMid(:, end+1) = parentMid;
    nodes.parentMid(:, index) = [];
    nodes.nodeMid(:, index) = [];
    parentLeft = nodes.parentLeft(:, index);
    pathLeft(:, end+1) = parentLeft;
    nodes.parentLeft(:, index) = [];
    nodes.nodeLeft(:, index) = [];
    parentRight = nodes.parentRight(:, index);
    pathRight(:, end+1) = parentRight;
    nodes.parentRight(:, index) = [];
    nodes.nodeRight(:, index) = [];
    nodes.cost(:, index) = [];
    rotateMatrix{end+1} = nodes.rotationM{index};
    nodes.rotationM(index) = [];
    if (parentMid == vecMid2)
        if (parentLeft == vecLeft2)
            if (parentRight == vecRight2)
                search = false;
                break;
            end
        end
    end
end
%% Reverse the path
rPathMid = pathMid;
pathMid = [];
rPathLeft = pathLeft;
pathLeft = [];
rPathRight = pathRight;
pathRight = [];
rRotateMatrix = {};
for i = 1 : size(rPathMid, 2)
    pathMid(:, end+1) = rPathMid(:, end+1-i);
    pathLeft(:, end+1) = rPathLeft(:, end+1-i);
    pathRight(:, end+1) = rPathRight(:, end+1-i);
    rRotateMatrix{end+1} = rotateMatrix{end+1-i};
end

end % function