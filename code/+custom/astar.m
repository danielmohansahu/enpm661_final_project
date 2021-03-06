function [success, nodes] = astar(start_nodes, target, start_height, mesh, features)

% A* costs and parameters
minCost = 1.0e-01;
step = [0.25; 0.25; 0.25];
zstep = 0.25;
maxzcorrection = 100;
same_node_threshold = 0.001;
maxStepSize = 10000;
permutations = [[1;0;0],[0;1;0],[0;0;1],[-1;0;0],[0;-1;0],[0;0;-1]];
    
% other loop variables
success = false;
noOfSteps = 1;

% initialize queue (costs and node indexes)
Q = [];
parent_idx = 1;

% initialize starting node
current = custom.getNode();
current.node = start_nodes;
current.height = start_height;
nodes = {current};

% convenience wrapper
check_collision = @(R,z) ...
    custom.isCollision(mesh, (R\(features+[0,0,z])')');

% perform A* search
while(~success && noOfSteps <= maxStepSize)
    % try each permutation of actions
    for k = 1:size(permutations,2)+2
        if k > size(permutations,2)
            % this is a pure translation action
            if k == size(permutations,2)+1
                new_height = current.height + zstep;
            elseif k == size(permutations,2)+2
                new_height = current.height - zstep;
            end
            R = custom.constructRotationMatrix([0,0,0]);
        else
            % this is a pure rotation action
            new_height = current.height;

            % calculate this action's rotation matrix
            angles = step .* permutations(:,k);
            R = custom.constructRotationMatrix(angles);
        end

        % check if this new node is in a state of collision
        if check_collision(R*current.cumulative,new_height)
            continue;
        end

        % construct a new node from this rotation
        new_node = custom.getNode(current);
        new_node.rotation = R;
        new_node.cumulative = R * current.cumulative;
        new_node.node = R * current.node;
        new_node.height = new_height;

        % check if this new node is already found
        isfound = @(x) (norm(x.node-new_node.node) ...
                        + abs(x.height-new_node.height)) ...
                        < same_node_threshold;
        if any(cellfun(isfound,nodes))
            continue;
        end

        % calculate its costs and add it to the list        
        cost = custom.getCost(target, new_node);
        Q = [Q; [cost, size(nodes,2) + 1]];
        new_node.cost = cost;
        new_node.parent = parent_idx;
        nodes = [nodes, new_node];
    end
    
    % get the next lowest cost node
    [~,qidx] = min(Q(:,1));
    current_cost = Q(qidx,1);
    parent_idx = Q(qidx,2);
    current = nodes{parent_idx};
    
    % remove this node from the queue
    Q(qidx,:) = [];

    % If cost is less or equal to min cost then stop the search
    noOfSteps = noOfSteps + 1;
    if (current_cost <= minCost)
        success = true;
    end
    % fprintf("COST: %d\tHEIGHT: %d\tQ: %d\n",current_cost, current.height, size(Q,1));
end

% if we've succeeded try to move in negative Z (i.e. close the gap)
if success
    new_node = custom.getNode(current);
    new_node.rotation = [1,0,0;0,1,0;0,0,1];
    new_node.height = new_node.height - zstep;
    cumulative_translation = 0;
    correction_nodes = [];
    use_correction = true;

    % check if this new node is in a state of collision
    while ~check_collision(new_node.cumulative,new_node.height)
        % add it to the node list
        new_node.parent = size(nodes,2);
        correction_nodes = [correction_nodes, new_node];
        
        % update for the next iteration
        new_node = custom.getNode(new_node);
        new_node.height = new_node.height - zstep;
        cumulative_translation = cumulative_translation + zstep;
        
        % check if we're going too far (something's wrong)
        if cumulative_translation > maxzcorrection
            fprintf('Unable to reconnect with parent; path search returned an odd result.\n');
            use_correction = false;
            break;
        end
    end
    if use_correction
        % append our correction nodes
        for i=1:size(correction_nodes,2)
            nodes = [nodes, correction_nodes(1,i)];
        end
    end
end

end % function