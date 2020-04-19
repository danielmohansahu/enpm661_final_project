close all
clear all
clc
%% Import the stl file
tic
model1 = createpde;
gd1 = importGeometry(model1,'../models/Part1.STL');
generateMesh(model1);
% pdegplot(model1,'FaceLabels','on','EdgeLabels','on','FaceAlpha',0.5);
% hold on
model2 = createpde;
gd2 = importGeometry(model2,'../models/Part2.STL');
generateMesh(model2);
% pdegplot(model2,'FaceLabels','on','EdgeLabels','on','FaceAlpha',0.5);
% figure;scatter3(model2.Mesh.Nodes(1,:), model2.Mesh.Nodes(2,:), model2.Mesh.Nodes(3,:),'b');
% hold on
% axis equal
% scatter3(model1.Mesh.Nodes(1,:), model1.Mesh.Nodes(2,:), model1.Mesh.Nodes(3,:),'r');
% hold off
%% Finding a boundary
% Part1
Part1 = [model1.Mesh.Nodes(1,:)', model1.Mesh.Nodes(2,:)', model1.Mesh.Nodes(3,:)'];
k = boundary(Part1, 1);
TR1 = triangulation(k,Part1);
F1 = faceNormal(TR1);
P1 = incenter(TR1);
origin = [38.6; 33.7960; 141.5215];
P1 = P1 - origin';
% Part2
Part2 = [model2.Mesh.Nodes(1,:)', model2.Mesh.Nodes(2,:)', model2.Mesh.Nodes(3,:)'];
k = boundary(Part2, 1);
TR2 = triangulation(k,Part2);
F2 = faceNormal(TR2);
P2 = incenter(TR2);
P2 = P2 - origin';
%% Chossing the vectors on surface
pointMid1 = [33.09; 38.55; 136.3]-origin; vecMid1 = [0.8192; -1.979e-07; -0.5736];
pointMid2 = [34.25; 36.75; 137.9]-origin; vecMid2 = [-0.8192; 6.604e-11; 0.5736];
pointLeft1 = [15.38; 23.39; 129.7]-origin; vecLeft1 = [-0.766; -4.023e-07; -0.6428];
pointLeft2 = [15.12; 22.76; 130]-origin; vecLeft2 = [0.766; -4.795e-08; 0.6428];
pointRight1 = [62.68; 26.88; 87.83]-origin; vecRight1 = [-0.9397; 2.238e-08; -0.342];
pointRight2 = [62.85; 26.13; 87.37]-origin; vecRight2 = [0.9397; -2.121e-08; 0.342];
Point1 = [pointMid1, pointLeft1, pointRight1]; Vector1 = [vecMid1, vecLeft1, vecRight1];
Point2 = [pointMid2, pointLeft2, pointRight2]; Vector2 = [vecMid2, vecLeft2, vecRight2];
% Taking cross product of the vectors
C = cross(Vector1, -Vector2);
n = [norm(C(:,1)), norm(C(:,2)), norm(C(:,3))];
% Rotating the vector around Z-axis
angles = [5 10 30];
rotateX = [1,      0,                 0;
            0, cosd(angles(1)),  -sind(angles(1));
            0, sind(angles(1)),  cosd(angles(1))];
rotateY = [cosd(angles(2)),  0, sind(angles(2));
                  0,          1,       0;
            -sind(angles(2)),        0, cosd(angles(2))];
rotateZ = [cosd(angles(3)), -sind(angles(3)), 0;
            sind(angles(3)),  cosd(angles(3)), 0;
                    0,               0,        1];
rotateModel1 = rotateX * rotateY * rotateZ;
rotate = rotateX * rotateY * rotateZ;
rotateP1 = rotate * P1';
vecMid1_n = rotate * vecMid1;
vecLeft1_n = rotate * vecLeft1;
vecRight1_n = rotate * vecRight1;
Vector1_n = rotate * Vector1;
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
search
%% Tracing back the path
vecMid2 = [-0.8192; 6.604e-11; 0.5736];
vecLeft2 = [0.766; -4.795e-08; 0.6428];
vecRight2 = [0.9397; -2.121e-08; 0.342];
if (search == false) 
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
end
toc
%% Display the path
% % Display the rotated part1
% rotateP1 = rotateZ * P1';
% subplot(2,1,1);
% scatter3(P1(:,1),P1(:,2),P1(:,3), 'g');
% hold on
% scatter3(P2(:,1),P2(:,2),P2(:,3), 'r');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% hold off
% subplot(2,1,2);
% scatter3(rotateP1(1,:),rotateP1(2,:),rotateP1(3,:), 'g');
% hold on
% scatter3(0, 0, 0, 'k', 'filled');
% scatter3(rotateP2(1,:),rotateP2(2,:),rotateP2(3,:), 'r');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% hold off

% scatter3(rotateP1(1,:),rotateP1(2,:),rotateP1(3,:), 'g');
% hold on
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% % axis equal
fv1 = stlread('../models/Part1.STL');
rotateV1 = rotateModel1 * fv1.Points';
fv1 = triangulation(fv1.ConnectivityList, rotateV1');
patch('Faces', fv1.ConnectivityList, 'Vertices', fv1.Points, ...
      'FaceColor', [0.8 0.8 1.0], 'EdgeColor', 'k', ...
      'EdgeAlpha', 0.2, 'FaceLighting', 'gouraud','AmbientStrength', 0.15);

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');

% Fix the axes scaling, and set a nice view angle
axis('image');
view([-135 35]);
hold on
fv2 = stlread('../models/Part2.STL');
rotateV2 = rRotateMatrix{1} * fv2.Points';
fv2 = triangulation(fv2.ConnectivityList, rotateV2');
rotateP2 = rRotateMatrix{1} * P2';
P2 = rotateP2';
plot_handle = patch('Faces', fv2.ConnectivityList, ...
                    'Vertices', fv2.Points, ...
                    'FaceColor', [0.8 0.8 1.0], ...
                    'EdgeColor', 'k', ...
                    'EdgeAlpha', 0.2, ...
                    'FaceLighting', 'gouraud', ...
                    'AmbientStrength', 0.15);

% Add a camera light, and tone down the specular highlighting
% camlight('headlight');
% material('dull');

% Fix the axes scaling, and set a nice view angle
% axis('image');
% view([-135 35]);
% hold on
% plot_handle = scatter3(rotateP2(1,:),rotateP2(2,:),rotateP2(3,:), 'r');
i = 2;
while (i <= size(pathMid,2))
    rotateP2 = rRotateMatrix{i} * P2';
    P2 = rotateP2';
%     set(plot_handle,'XData',rotateP2(1,:),'YData',rotateP2(2,:),'ZData',rotateP2(3,:));
    rotateV2 = rRotateMatrix{i} * fv2.Points';
    fv2 = triangulation(fv2.ConnectivityList, rotateV2');
    set(plot_handle, 'Faces',fv2.ConnectivityList, 'Vertices',fv2.Points);
%     patch(fv2,'FaceColor',       [0.8 0.8 1.0], ...
%              'EdgeColor',       'black',        ...
%              'FaceLighting',    'gouraud',     ...
%              'AmbientStrength', 0.15);

    % Add a camera light, and tone down the specular highlighting
%     camlight('headlight');
%     material('dull');

    % Fix the axes scaling, and set a nice view angle
%     axis('image');
%     view([-135 35]);
    pause(0.05)
    i = i + 1;
end
hold off