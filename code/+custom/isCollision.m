function [collision, I] = isCollision(TR1, TR2)
% Return a boolean indicating whether or not any of the given edges 
% and faces/normals intersect.

collision = false;

% edges / points on the first mesh
edges1 = edges(TR1);
points1 = TR1.Points;

% faces / normals on the second mesh
faces = TR2.ConnectivityList;
normals = faceNormal(TR2);
points2 = TR2.Points;

for i = 1:size(edges1,1)
    edge = edges1(i,:);
    p1_1 = points1(edge(1),:);
    p1_2 = points2(edge(2),:);

    for j = 1:size(faces)
        n = normals(j,:);
        p2 = points2(faces(j,1),:);
        [I,check] = custom.plane_line_intersect(n,p2,p1_1,p1_2);
        if check == 2 % ignore if the intersection is at the given point (e.g.1) 
            collision = true;
            check
            i
            j
            return;
    end
end



end % function