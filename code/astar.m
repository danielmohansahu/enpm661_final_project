function [success, nodes] = astar(start_nodes, target_nodes)

% A* costs and parameters
minCost = 1e-02;
step = [0.1; 0.1; 0.1];
maxStepSize = 1000;

% other loop variables
success = false;
noOfSteps = 1;

% initialize queues and nodes (graph structure)
queue = [];
qCost = [];
nodes.nodes = start_nodes;
current_node = start_nodes;
nodes.cost = 0;
nodes.parents = zeros(size(start_nodes));
nodes.rotationM = {[0, 0, 0; 0, 0, 0; 0, 0, 0]};

% perform A* search
while(~success && noOfSteps <= maxStepSize)
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
        newNode = rotate * current_node;
        
        % check if this new node is in a state of collision
        if isCollision(newNode)
            break;
        end
        
        % check if this new node is already found
        already_found = false;
        for i = 1 : size(nodes.nodes, 3)
            if all( (newNode == nodes.nodes(:,:,i)) )
                already_found = true;
                break;
            end
        end
        
        % if this isn't our goal node calculate its costs and add it
        if ~already_found
            % calculate cost; add to queue and nodes list
            dim = size(qCost,2) + 1;
            queue(:,:,dim) = newNode;
            cost = norm(cross(target_nodes(:,1), newNode(:,1))) + norm(cross(target_nodes(:,2), newNode(:,2))) + norm(cross(target_nodes(:,3), newNode(:,3)));
            qCost(:,dim) = cost;
            
            nodes.nodes(:,:,dim) = newNode;
            nodes.parents(:,:,dim) = current_node;
            nodes.cost(:,dim) = cost;
            nodes.rotationM{dim} = rotate;
            
            
            % If cost is less or equal to min cost then stop the search
            if (cost <= minCost)
                success = true;
                break;
            end
        end
    end
    
    % update loop variables
    [~, I] = min(qCost);
    current_node = queue(:,:,I);
    current_node(:,1);
    queue(:,:,I) = [];
    qCost(:, I) = [];
    noOfSteps = noOfSteps + 1;
end

end % function