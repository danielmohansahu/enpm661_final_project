close all
clear all
clc
% turn off point warnings
warning('off','MATLAB:triangulation:PtsNotInTriWarnId');

%% Import the stl file
tic
model1 = createpde;
gd1 = importGeometry(model1,'../models/Part1.STL');
generateMesh(model1);

model2 = createpde;
gd2 = importGeometry(model2,'../models/Part2.STL');
generateMesh(model2);

%% Finding a boundary
% Part1
Part1 = [model1.Mesh.Nodes(1,:)', model1.Mesh.Nodes(2,:)', model1.Mesh.Nodes(3,:)'];
k = boundary(Part1, 1);
TR1 = triangulation(k,Part1);
origin = [38.6; 33.7960; 141.5215];

% Part2
Part2 = [model2.Mesh.Nodes(1,:)', model2.Mesh.Nodes(2,:)', model2.Mesh.Nodes(3,:)'];
k = boundary(Part2, 1);
TR2 = triangulation(k,Part2);
P2 = incenter(TR2);
P2 = P2 - origin';

%% Collision Detection Logic

% 3D collision detection mesh
od_mesh = alphaShape(TR2.Points);

% pertinent features in the first mesh to perform collision detection
od_features = TR1.Points(featureEdges(TR1,pi/2),:);

offset = 0;
while custom.isCollision(od_mesh, od_features + [0,0,offset])
    offset = offset + 0.1;
end

fprintf("Starting with a Z offset of %d\n",offset);
od_features = od_features + [0,0,offset];
TR1 = triangulation(TR1.ConnectivityList, TR1.Points + [0,0,offset]);

%% Choosing the vectors on surface (hardcoded) and "breaking" bones.

vecMid1 = [0.8192; -1.979e-07; -0.5736];
vecMid2 = [-0.8192; 6.604e-11; 0.5736];
vecLeft1 = [-0.766; -4.023e-07; -0.6428];
vecLeft2 = [0.766; -4.795e-08; 0.6428];
vecRight1 = [-0.9397; 2.238e-08; -0.342];
vecRight2 = [0.9397; -2.121e-08; 0.342];
Vector1 = [vecMid1, vecLeft1, vecRight1];
Vector2 = [vecMid2, vecLeft2, vecRight2];

offset = [0,0,30];
angles = [5 10 20];
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
Vector1_n = rotate * Vector1;

% update current state variables
od_features = (rotate * (od_features+offset)')';
TR1 = triangulation(TR1.ConnectivityList, TR1.Points + offset);

%% A* Search
% sanity check that we're not starting in collision
if custom.isCollision(od_mesh, od_features)
    error("Starting in a collision; this isn't allowed.");
end

% perform A* path search
[success, nodes] = custom.astar(Vector2,Vector1_n,od_mesh,od_features);

%% Tracing back the path

% goal node (should not be hardcoded...)
vecMid2 = [-0.8192; 6.604e-11; 0.5736];
vecLeft2 = [0.766; -4.795e-08; 0.6428];
vecRight2 = [0.9397; -2.121e-08; 0.342];
if ~success
    error("Path search failed.");
end

[path, rRotateMatrix] = custom.backtrack(nodes, [vecMid2, vecLeft2, vecRight2]);

toc
%% Display the path

rotateV1 = rotateModel1 * (TR1.Points)';
fv1 = triangulation(TR1.ConnectivityList, rotateV1');
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
rotateV2 = rRotateMatrix{1} * TR2.Points';
fv2 = triangulation(TR2.ConnectivityList, rotateV2');
rotateP2 = rRotateMatrix{1} * P2';
P2 = rotateP2';
plot_handle = patch('Faces', fv2.ConnectivityList, ...
                    'Vertices', fv2.Points, ...
                    'FaceColor', [0.8 0.8 1.0], ...
                    'EdgeColor', 'k', ...
                    'EdgeAlpha', 0.2, ...
                    'FaceLighting', 'gouraud', ...
                    'AmbientStrength', 0.15);

i = 2;
while (i ~= size(path,3))
    rotateP2 = rRotateMatrix{i} * P2';
    P2 = rotateP2';

    rotateV2 = rRotateMatrix{i} * fv2.Points';
    fv2 = triangulation(fv2.ConnectivityList, rotateV2');
    set(plot_handle, 'Faces',fv2.ConnectivityList, 'Vertices',fv2.Points);

    pause(0.1)
    i = i + 1;
end
hold off