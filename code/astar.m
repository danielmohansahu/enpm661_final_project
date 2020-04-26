function [search, nodes] = astar(vecMid2, vecLeft2, vecRight2, vecMid1_n, vecLeft1_n, vecRight1_n)
search = true;
% Minimum value of norm
minCost = 1e-02;
noOfSteps = 1;
stepFactor = 10;
maxStepSize = 1000;
step = [0.1; 0.1; 0.1];
queueMid = [];
queueLeft = [];
queueRight = [];
qCost = [];
nodes.nodeMid = vecMid2;
nodes.nodeLeft = vecLeft2;
nodes.nodeRight = vecRight2;
nodes.cost = 0;
nodes.parentMid = [0; 0; 0];
nodes.parentLeft = [0; 0; 0];
nodes.parentRight = [0; 0; 0];
nodes.rotationM = {[0, 0, 0; 0, 0, 0; 0, 0, 0]};
while(search && noOfSteps <= maxStepSize)
    % newNode 1    
    angles = step .* [1; 0; 0];
    rotateX = [1,      0,                 0;
                0, cosd(angles(1)),  -sind(angles(1));
                0, sind(angles(1)),  cosd(angles(1))];
    rotateY = [cosd(angles(2)),  0, sind(angles(2));
                      0,          1,       0;
                -sind(angles(2)),        0, cosd(angles(2))];
    rotateZ = [cosd(angles(3)), -sind(angles(3)), 0;
                sind(angles(3)),  cosd(angles(3)), 0;
                        0,               0,        1];
    rotate = rotateX * rotateY * rotateZ;
    newNodeMid = rotate * vecMid2;
    newNodeLeft = rotate * vecLeft2;
    newNodeRight = rotate * vecRight2;
    found = false;
    for i = 1 : size(nodes.nodeMid, 2)
        if (newNodeMid == nodes.nodeMid(:, i))
            if (newNodeLeft == nodes.nodeLeft(:, i))
                if(newNodeRight == nodes.nodeRight(:, i))
                    found = true;
                    break;
                end
            end
        end
    end
    if (found == false)
        queueMid(:, end+1) = newNodeMid;
        queueLeft(:, end+1) = newNodeLeft;
        queueRight(:, end+1) = newNodeRight;
        % Find the cost
        costMid = norm(cross(vecMid1_n, newNodeMid));
        costLeft = norm(cross(vecLeft1_n, newNodeLeft));
        costRight = norm(cross(vecRight1_n, newNodeRight));
        cost = costMid + costLeft + costRight;
        qCost(:, end+1) = cost;
        nodes.nodeMid(:, end+1) = newNodeMid;
        nodes.parentMid(:, end+1) = vecMid2;
        nodes.nodeLeft(:, end+1) = newNodeLeft;
        nodes.parentLeft(:, end+1) = vecLeft2;
        nodes.nodeRight(:, end+1) = newNodeRight;
        nodes.parentRight(:, end+1) = vecRight2;
        nodes.cost(:, end+1) = cost;
        nodes.rotationM{end+1} = rotate;
        % If cost is less or equal to min cost then stop the search
        if (cost <= minCost)
            search = false;
            break;
        end
    end
    % newNode 2
    angles = step .* [0; 1; 0];
    rotateX = [1,      0,                 0;
                0, cosd(angles(1)),  -sind(angles(1));
                0, sind(angles(1)),  cosd(angles(1))];
    rotateY = [cosd(angles(2)),  0, sind(angles(2));
                      0,          1,       0;
                -sind(angles(2)),        0, cosd(angles(2))];
    rotateZ = [cosd(angles(3)), -sind(angles(3)), 0;
                sind(angles(3)),  cosd(angles(3)), 0;
                        0,               0,        1];
    rotate = rotateX * rotateY * rotateZ;
    newNodeMid = rotate * vecMid2;
    newNodeLeft = rotate * vecLeft2;
    newNodeRight = rotate * vecRight2;
    found = false;
    for i = 1 : size(nodes.nodeMid, 2)
        if (newNodeMid == nodes.nodeMid(:, i))
            if (newNodeLeft == nodes.nodeLeft(:, i))
                if(newNodeRight == nodes.nodeRight(:, i))
                    found = true;
                    break;
                end
            end
        end
    end
    if (found == false)
        queueMid(:, end+1) = newNodeMid;
        queueLeft(:, end+1) = newNodeLeft;
        queueRight(:, end+1) = newNodeRight;
        % Find the cost
        costMid = norm(cross(vecMid1_n, newNodeMid));
        costLeft = norm(cross(vecLeft1_n, newNodeLeft));
        costRight = norm(cross(vecRight1_n, newNodeRight));
        cost = costMid + costLeft + costRight;
        qCost(:, end+1) = cost;
        nodes.nodeMid(:, end+1) = newNodeMid;
        nodes.parentMid(:, end+1) = vecMid2;
        nodes.nodeLeft(:, end+1) = newNodeLeft;
        nodes.parentLeft(:, end+1) = vecLeft2;
        nodes.nodeRight(:, end+1) = newNodeRight;
        nodes.parentRight(:, end+1) = vecRight2;
        nodes.cost(:, end+1) = cost;
        nodes.rotationM{end+1} = rotate;
        % If cost is less or equal to min cost then stop the search
        if (cost <= minCost)
            search = false;
            break;
        end
    end
    % newNode 3
    angles = step .* [0; 0; 1];
    rotateX = [1,      0,                 0;
                0, cosd(angles(1)),  -sind(angles(1));
                0, sind(angles(1)),  cosd(angles(1))];
    rotateY = [cosd(angles(2)),  0, sind(angles(2));
                      0,          1,       0;
                -sind(angles(2)),        0, cosd(angles(2))];
    rotateZ = [cosd(angles(3)), -sind(angles(3)), 0;
                sind(angles(3)),  cosd(angles(3)), 0;
                        0,               0,        1];
    rotate = rotateX * rotateY * rotateZ;
    newNodeMid = rotate * vecMid2;
    newNodeLeft = rotate * vecLeft2;
    newNodeRight = rotate * vecRight2;
    found = false;
    for i = 1 : size(nodes.nodeMid, 2)
        if (newNodeMid == nodes.nodeMid(:, i))
            if (newNodeLeft == nodes.nodeLeft(:, i))
                if(newNodeRight == nodes.nodeRight(:, i))
                    found = true;
                    break;
                end
            end
        end
    end
    if (found == false)
        queueMid(:, end+1) = newNodeMid;
        queueLeft(:, end+1) = newNodeLeft;
        queueRight(:, end+1) = newNodeRight;
        % Find the cost
        costMid = norm(cross(vecMid1_n, newNodeMid));
        costLeft = norm(cross(vecLeft1_n, newNodeLeft));
        costRight = norm(cross(vecRight1_n, newNodeRight));
        cost = costMid + costLeft + costRight;
        qCost(:, end+1) = cost;
        nodes.nodeMid(:, end+1) = newNodeMid;
        nodes.parentMid(:, end+1) = vecMid2;
        nodes.nodeLeft(:, end+1) = newNodeLeft;
        nodes.parentLeft(:, end+1) = vecLeft2;
        nodes.nodeRight(:, end+1) = newNodeRight;
        nodes.parentRight(:, end+1) = vecRight2;
        nodes.cost(:, end+1) = cost;
        nodes.rotationM{end+1} = rotate;
        % If cost is less or equal to min cost then stop the search
        if (cost <= minCost)
            search = false;
            break;
        end
    end
    % newNode 4
    angles = step .* [-1; 0; 0];
    rotateX = [1,      0,                 0;
                0, cosd(angles(1)),  -sind(angles(1));
                0, sind(angles(1)),  cosd(angles(1))];
    rotateY = [cosd(angles(2)),  0, sind(angles(2));
                      0,          1,       0;
                -sind(angles(2)),        0, cosd(angles(2))];
    rotateZ = [cosd(angles(3)), -sind(angles(3)), 0;
                sind(angles(3)),  cosd(angles(3)), 0;
                        0,               0,        1];
    rotate = rotateX * rotateY * rotateZ;
    newNodeMid = rotate * vecMid2;
    newNodeLeft = rotate * vecLeft2;
    newNodeRight = rotate * vecRight2;
    found = false;
    for i = 1 : size(nodes.nodeMid, 2)
        if (newNodeMid == nodes.nodeMid(:, i))
            if (newNodeLeft == nodes.nodeLeft(:, i))
                if(newNodeRight == nodes.nodeRight(:, i))
                    found = true;
                    break;
                end
            end
        end
    end
    if (found == false)
        queueMid(:, end+1) = newNodeMid;
        queueLeft(:, end+1) = newNodeLeft;
        queueRight(:, end+1) = newNodeRight;
        % Find the cost
        costMid = norm(cross(vecMid1_n, newNodeMid));
        costLeft = norm(cross(vecLeft1_n, newNodeLeft));
        costRight = norm(cross(vecRight1_n, newNodeRight));
        cost = costMid + costLeft + costRight;
        qCost(:, end+1) = cost;
        nodes.nodeMid(:, end+1) = newNodeMid;
        nodes.parentMid(:, end+1) = vecMid2;
        nodes.nodeLeft(:, end+1) = newNodeLeft;
        nodes.parentLeft(:, end+1) = vecLeft2;
        nodes.nodeRight(:, end+1) = newNodeRight;
        nodes.parentRight(:, end+1) = vecRight2;
        nodes.cost(:, end+1) = cost;
        nodes.rotationM{end+1} = rotate;
        % If cost is less or equal to min cost then stop the search
        if (cost <= minCost)
            search = false;
            break;
        end
    end
    % newNode 5
    angles = step .* [0; -1; 0];
    rotateX = [1,      0,                 0;
                0, cosd(angles(1)),  -sind(angles(1));
                0, sind(angles(1)),  cosd(angles(1))];
    rotateY = [cosd(angles(2)),  0, sind(angles(2));
                      0,          1,       0;
                -sind(angles(2)),        0, cosd(angles(2))];
    rotateZ = [cosd(angles(3)), -sind(angles(3)), 0;
                sind(angles(3)),  cosd(angles(3)), 0;
                        0,               0,        1];
    rotate = rotateX * rotateY * rotateZ;
    newNodeMid = rotate * vecMid2;
    newNodeLeft = rotate * vecLeft2;
    newNodeRight = rotate * vecRight2;
    found = false;
    for i = 1 : size(nodes.nodeMid, 2)
        if (newNodeMid == nodes.nodeMid(:, i))
            if (newNodeLeft == nodes.nodeLeft(:, i))
                if(newNodeRight == nodes.nodeRight(:, i))
                    found = true;
                    break;
                end
            end
        end
    end
    if (found == false)
        queueMid(:, end+1) = newNodeMid;
        queueLeft(:, end+1) = newNodeLeft;
        queueRight(:, end+1) = newNodeRight;
        % Find the cost
        costMid = norm(cross(vecMid1_n, newNodeMid));
        costLeft = norm(cross(vecLeft1_n, newNodeLeft));
        costRight = norm(cross(vecRight1_n, newNodeRight));
        cost = costMid + costLeft + costRight;
        qCost(:, end+1) = cost;
        nodes.nodeMid(:, end+1) = newNodeMid;
        nodes.parentMid(:, end+1) = vecMid2;
        nodes.nodeLeft(:, end+1) = newNodeLeft;
        nodes.parentLeft(:, end+1) = vecLeft2;
        nodes.nodeRight(:, end+1) = newNodeRight;
        nodes.parentRight(:, end+1) = vecRight2;
        nodes.cost(:, end+1) = cost;
        nodes.rotationM{end+1} = rotate;
        % If cost is less or equal to min cost then stop the search
        if (cost <= minCost)
            search = false;
            break;
        end
    end
    % newNode 6
    angles = step .* [0; 0; -1];
    rotateX = [1,      0,                 0;
                0, cosd(angles(1)),  -sind(angles(1));
                0, sind(angles(1)),  cosd(angles(1))];
    rotateY = [cosd(angles(2)),  0, sind(angles(2));
                      0,          1,       0;
                -sind(angles(2)),        0, cosd(angles(2))];
    rotateZ = [cosd(angles(3)), -sind(angles(3)), 0;
                sind(angles(3)),  cosd(angles(3)), 0;
                        0,               0,        1];
    rotate = rotateX * rotateY * rotateZ;
    newNodeMid = rotate * vecMid2;
    newNodeLeft = rotate * vecLeft2;
    newNodeRight = rotate * vecRight2;
    found = false;
    for i = 1 : size(nodes.nodeMid, 2)
        if (newNodeMid == nodes.nodeMid(:, i))
            if (newNodeLeft == nodes.nodeLeft(:, i))
                if(newNodeRight == nodes.nodeRight(:, i))
                    found = true;
                    break;
                end
            end
        end
    end
    if (found == false)
        queueMid(:, end+1) = newNodeMid;
        queueLeft(:, end+1) = newNodeLeft;
        queueRight(:, end+1) = newNodeRight;
        % Find the cost
        costMid = norm(cross(vecMid1_n, newNodeMid));
        costLeft = norm(cross(vecLeft1_n, newNodeLeft));
        costRight = norm(cross(vecRight1_n, newNodeRight));
        cost = costMid + costLeft + costRight;
        qCost(:, end+1) = cost;
        nodes.nodeMid(:, end+1) = newNodeMid;
        nodes.parentMid(:, end+1) = vecMid2;
        nodes.nodeLeft(:, end+1) = newNodeLeft;
        nodes.parentLeft(:, end+1) = vecLeft2;
        nodes.nodeRight(:, end+1) = newNodeRight;
        nodes.parentRight(:, end+1) = vecRight2;
        nodes.cost(:, end+1) = cost;
        nodes.rotationM{end+1} = rotate;
        % If cost is less or equal to min cost then stop the search
        if (cost <= minCost)
            search = false;
            break;
        end
    end
    [~, I] = min(qCost);
    vecMid2 = queueMid(:, I);
    queueMid(:, I) = [];
    vecLeft2 = queueLeft(:, I);
    queueLeft(:, I) = [];
    vecRight2 = queueRight(:, I);
    queueRight(:, I) = [];
    qCost(:, I) = [];
    noOfSteps = noOfSteps + 1;
end

end % function