% Perform RRT search of the node workspace.
function [success, nodes] = rrt(start_nodes, target, start_height, mesh, features)

% RRT costs and parameters
minCost = 5e-03;
bounds = [2*pi, 2*pi, 2*pi, 10];
step = [0.1; 0.1; 0.1];
zstep = 0.1;
maxStepSize = 1000;
    
% other loop variables
success = false;
noOfSteps = 1;

% initialize starting node
current = custom.getNode();
current.node = start_nodes;
current.height = start_height;

% initialize graph
G = {current};

% convenience functions
check_collision = @(R,z) ...
    custom.isCollision(mesh, (inv(R)*(features+[0,0,z])')');
get_random = @() ...
    [custom.constructRotationMatrix(rand(3,1).*bounds(1:3)), ...
    rand()*bounds(4)];

% perform search
while(~success && noOfSteps <= maxStepSize)
    % get a random node in the workspace
    [R,z] = get_random();
    while custom.isCollision(mesh, (inv(R)*(features+[0,0,z])')')
        % continue until we get a collision free node
        [R,z] = get_random();
    end
            
    % find closest node
    random_node = custom.getNode();
    random_node.cumulative = R;
    random_node.height = z;
    random_node.node = R*start_nodes;

    distance = @(n) norm(n.node-random_node.node) ...
                    + abs(n.height-random_node.height);
    distances = cellfun(distance, G);
    [dist,idx] = min(distances);
    dist
    closest_node = G{idx};
    
    % add nodes along this path (incrementally) until we reach it or a
    % collision (check cost and return if any are close enough)
    while 
    
    
    
    % try each permutation of actions
    for k = 0:size(permutations,2)
        if k == 0
            % this is a pure translation action
            new_height = current.height + zstep;
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
                        < 1e5*eps;
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
end

% if we've succeeded try to move in negative Z (i.e. close the gap)
if success
    new_node = custom.getNode(current);
    new_node.rotation = [1,0,0;0,1,0;0,0,1];
    new_node.height = new_node.height - zstep;

    % check if this new node is in a state of collision
    while ~check_collision(new_node.cumulative,new_node.height)
        % add it to the node list
        new_node.parent = size(nodes,2);
        nodes = [nodes, new_node];
        
        % update for the next iteration
        new_node = custom.getNode(new_node);
        new_node.height = new_node.height - zstep;
    end
end

end % function