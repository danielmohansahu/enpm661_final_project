function [success, nodes] = astar(start_nodes, target_nodes, start_height, mesh, features)

% A* costs and parameters
minCost = 1e-02;
step = [0.1; 0.1; 0.1];
zstep = 0.1;
zcost = 0.0001;
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
current_cost = 0;
nodes.parents = zeros(size(start_nodes));
nodes.parent_height = start_height;
nodes.height = start_height;
current_height = start_height;
nodes.rotationM = {[1, 0, 0; 0, 1, 0; 0, 0, 1]};
nodes.rotationC = {[1, 0, 0; 0, 1, 0; 0, 0, 1]};
current_rot = nodes.rotationM{1};
current_rot_c = nodes.rotationC{1};

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
        rot_c = rotate * current_rot_c;
        
        % check if this new node is in a state of collision
        if custom.isCollision(mesh, (rot_c*(features+[0,0,current_height])')')
            continue;
        end
        
        % check if this new node is already found
        for i = 1 : size(nodes.nodes, 3)
            if all( (newNode == nodes.nodes(:,:,i)) & current_height == nodes.height(i) )
                continue;
            end
        end
        
        % if this isn't our goal node calculate its costs and add it
        dimq = size(qCost,2) + 1;
        queue(:,:,dimq) = newNode;
        cost = norm(cross(target_nodes(:,1), newNode(:,1))) + norm(cross(target_nodes(:,2), newNode(:,2))) + norm(cross(target_nodes(:,3), newNode(:,3)));
        qCost(:,dimq) = cost;

        dimn = size(nodes.cost,2) + 1;
        nodes.nodes(:,:,dimn) = newNode;
        nodes.parents(:,:,dimn) = current_node;
        nodes.parent_height(dimn) = current_height;
        nodes.cost(:,dimn) = cost;
        nodes.height(:,dimn) = current_height;
        nodes.rotationM{dimn} = rotate;
        nodes.rotationC{dimn} = rot_c;

    end
    
    % we can also move in Z to avoid collisions
    for dz = zstep:zstep
        new_height = current_height + dz;
        
        % check if this new node is already found
        for i = 1 : size(nodes.nodes, 3)
            if all( (current_node == nodes.nodes(:,:,i)) & new_height == nodes.height(i) )
                continue;
            end
        end
        
        % check if this would bring us into collision
        if custom.isCollision(mesh, (current_rot_c*(features+[0,0,new_height])')')
            continue;
        end
        
        dimq = size(qCost,2) + 1;
        cost = norm(cross(target_nodes(:,1), current_node(:,1))) + norm(cross(target_nodes(:,2), current_node(:,2))) + norm(cross(target_nodes(:,3), current_node(:,3)));
        queue(:,:,dimq) = current_node;
        qCost(:,dimq) = cost;

        dimn = size(nodes.cost,2) + 1;
        nodes.nodes(:,:,dimn) = current_node;
        nodes.parents(:,:,dimn) = current_node;
        nodes.parent_height(dimn) = current_height;
        nodes.cost(:,dimn) = cost;
        nodes.height(:,dimn) = new_height;
        nodes.rotationM{dimn} = current_rot;
        nodes.rotationC{dimn} = current_rot_c;
    end
    
    % update loop variables
    [current_cost, I] = min(qCost);
    current_node = queue(:,:,I);
    current_height = nodes.height(I);
    current_rot = nodes.rotationM{I};
    current_rot_c = nodes.rotationC{I};
    queue(:,:,I) = [];
    qCost(:, I) = [];
    noOfSteps = noOfSteps + 1;

    % If cost is less or equal to min cost then stop the search
    if (current_cost <= minCost)
        success = true;
    end
end

% if we've succeeded; try to correct for any Z motion
% if success
%     node = nodes.nodes(:,:,end);
%     cost = nodes.cost(end);
%     parent_height = nodes.height(end);
%     rot = nodes.rotationM{end};
% 
%     height = parent_height - zstep;
%     while (height > 0) && ~custom.isCollision(mesh, (node*(features+[0,0,height])')')
%         dimn = size(nodes.cost,2) + 1;
%         cost = cost - zstep;
%         
%         nodes.nodes(:,:,dimn) = node;
%         nodes.parents(:,:,dimn) = node;
%         nodes.parent_height(dimn) = parent_height;
%         nodes.cost(:,dimn) = cost;
%         nodes.height(:,dimn) = height;
%         nodes.rotationM{dimn} = rot;
%         
%         parent_height = height;
%         height = parent_height - zstep;
%     end
% end

end % function