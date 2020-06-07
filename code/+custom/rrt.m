% Perform RRT search of the node workspace.
function [success, G] = rrt(start_nodes, target, start_height, mesh, features)

% RRT costs and parameters
minCost = 1e-1;
% bounds are (start, range) for each degree of freedom
bounds = [...
    [-30,60];...
    [-30,60];...
    [-30,60];
    [start_height,20]];

% steps are how much to increment from closest to random node (deg,m)
steps = [1, 1, 1, 5];
zstep = 0.25;
minDist = 0.0;
maxNodes = 7500;

% other loop variables
success = false;
mincost = 1000;

% initialize starting node
current = custom.getNode();
current.node = start_nodes;
current.height = start_height;

% initialize graph
G = {current};
N = [current.node];

% convenience functions
check_collision = @(R,z) ...
    custom.isCollision(mesh, (R'*(features+[0,0,z])')');
get_random_R = @() custom.constructRotationMatrix(...
    rand(3,1).*bounds(1:3,2) + bounds(1:3,1));
get_random_z = @() rand()*bounds(4,2) + bounds(4,1);

% perform search
while(~success & size(G) <= maxNodes)
        % get a random node in the workspace
    R = get_random_R();
    z = get_random_z();
    while check_collision(R,z)
        % continue until we get a collision free node
        R = get_random_R();
        z = get_random_z();
    end
            
    % find closest node
    random_node = custom.getNode();
    random_node.cumulative = R;
    random_node.height = z;
    random_node.node = R*start_nodes;

    % distance = @(n) norm(n.node-random_node.node) ...
    %                  + abs(n.height-random_node.height);
    % distances = cellfun(distance, G);
    distances = sum((N-random_node.node).*(N-random_node.node),1:2);
    
    % make sure we're not too close to an existing node
    [dist,parent_idx] = min(distances);
    if dist < minDist
        continue;
    end
    closest_node = G{parent_idx};
    
    % add nodes along this path (incrementally) until we reach it or a
    % collision (check cost and return if any are close enough)
    diff_r = rotm2eul(R'*closest_node.cumulative)*180/pi;
    diff_z = z - closest_node.height;
    
    num_nodes = floor(max(abs([diff_r, diff_z]./steps)));
    actions = [...
        linspace(0, diff_r(1), num_nodes);...
        linspace(0, diff_r(2), num_nodes);...
        linspace(0, diff_r(3), num_nodes);...
        linspace(0, diff_z, num_nodes)];
    
    % construct and add each new node
    new_node = custom.getNode(closest_node);
    for i = 2:num_nodes
        % convert actions to matrices
        dR = eul2rotm((actions(1:3,i)-actions(1:3,i-1))'*pi/180)';
        dz = actions(4,i) - actions(4,i-1);
    
        % if this is a collision, break
        if check_collision(dR*new_node.cumulative, dz+new_node.height)
            break;
        end
        
        % otherwise add it to the list
        new_node = custom.getNode(new_node);
        new_node.node = dR*new_node.node;
        new_node.parent = parent_idx;
        new_node.rotation = dR;
        new_node.cumulative = dR*new_node.cumulative;
        new_node.height = dz + new_node.height;
        
        G = [G, new_node];
        N(:,:,size(N,3)+1) = new_node.node;
        parent_idx = size(G,2);

        % check if this is our goal node:
        cost = custom.getCost(target, new_node);
        if cost < minCost
            success = true;
            current = new_node;
            break;
        else
            norm(new_node.node - target);
            mincost = min(mincost,cost);
        end
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
        new_node.parent = size(G,2);
        G = [G, new_node];
        
        % update for the next iteration
        new_node = custom.getNode(new_node);
        new_node.height = new_node.height - zstep;
    end
end

end % function