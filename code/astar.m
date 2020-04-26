function [search, nodes] = astar(vecMid2, vecLeft2, vecRight2, vecMid1_n, vecLeft1_n, vecRight1_n)

% A* costs and parameters
minCost = 1e-02;
step = [0.1; 0.1; 0.1];
maxStepSize = 1000;

% other loop variables
search = true;
noOfSteps = 1;

% initialize queues and nodes (graph structure)
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

% perform A* search
while(search && noOfSteps <= maxStepSize)
    % try each permutation of actions
    permutations = [[1;0;0],[0;1;0],[0;0;1],[-1;0;0],[0;-1;0],[0;0;-1]];
    for k = 1:size(permutations,2)
        % calculate this action's rotation matrix
        angles = step .* permutations(:,k);
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
        
        % check if this new node is in a state of collision
        if isCollision(newNodeMid, newNodeLeft, newNodeRight)
            break;
        end
        
        % check if this new node is our solution
        found = false;
        for i = 1 : size(nodes.nodeMid, 2)
            if all( (newNodeMid == nodes.nodeMid(:, i)) & (newNodeLeft == nodes.nodeLeft(:, i)) & (newNodeRight == nodes.nodeRight(:, i)) )
                found = true;
                break;
            end
        end
        
        % if this isn't our goal node calculate its costs and add it
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
    end
    
    % update loop variables
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