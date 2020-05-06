function [success, nodes] = astar(start_nodes, target, start_height, mesh, features)

% A* costs and parameters
minCost = 1e-02;
step = [0.1; 0.1; 0.1];
zstep = 0.1;
zcost = 0.0001;
maxStepSize = 1000;
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

% perform A* search
while(~success && noOfSteps <= maxStepSize)
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
        transformed_features = R*current.cumulative*...
            (features+[0,0,new_height])';
        if custom.isCollision(mesh, transformed_features')
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
    current_cost = Q(qidx,1)
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