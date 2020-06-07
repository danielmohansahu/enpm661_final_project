function [success, nodes] = rrt(start_nodes, target_nodes, mesh, features)
compare = 40; % The Value used to restrict the movement within certain nodes
Max_num_Nodes = 1600; 

success = false;
noOfSteps = 1;

% Boundary for the map or reach space of robot 
x_max = 2*pi ;
y_max = 2*pi;
z_max = 2*pi;

% initialize queues and nodes (graph structure)
queue = [];
qCost = [];
nodes.nodes = start_nodes;
current_node = start_nodes;
nodes.cost = 0;
current_cost = 0;
nodes.parents = zeros(size(start_nodes));
nodes.height = 0;
current_height = 0;
nodes.rotationM = {[0, 0, 0; 0, 0, 0; 0, 0, 0]};
current_rot = nodes.rotationM{1};


% Finding the shortest distance path by forming a vector with distances 
D = [];
for j = 1:1:length(nodes)
    tmp_dist = euc_dist_3d(nodes(j).config, q_goal.config);
    D = [D tmp_dist];
end

% perform RRT search

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


while(~success && noOfSteps <= Max_num_Nodes)
    permutations = [[1;0;0],[0;1;0],[0;0;1],[-1;0;0],[0;-1;0],[0;0;-1]];
    for i = 1:1:Max_num_Nodes
    q_rand = [rand(1)*x_max rand(1)*y_max rand(1)*z_max]
    
    
    if custom.isCollision(mesh, (rotate*(features+[0,0,current_height])')')
            break;
    end
        
    % check if this new node is already found
        already_found = false;
        for i = 1 : size(nodes.nodes, 3)
            if all( (newNode == nodes.nodes(:,:,i)) & current_height == nodes.height(i) )
                already_found = true;
                break;
            end
        end
        
    % Comparing distance between current nodes and new nodes to find
    % nearest node out of a group of random nodes
    n_dist = [];
    for j = 1:1:length(nodes.nodes)
        n = nodes.nodes(j);
        dist_tmp = euc_dist_3d(n.config, q_rand);
        n_dist = [n_dist dist_tmp];
    end    
    [val, idx] = min(n_dist);
    q_near = nodes.nodes(idx);
    q_new.config = move(q_rand, q_near.config, val, compare);
    
    % Comparing the elements of vector formed above and finding the minimum
    % distance out of it to get final path
    % If in case, the last node selected is not goal node, we replace it and
    % add at the end, goal node, in nodes list

    [val, idx] = min(D);
    q_goal.parent = idx;
    q_end = q_goal;
    nodes = [nodes q_goal];