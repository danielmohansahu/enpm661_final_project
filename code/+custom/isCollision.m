function collision = isCollision(mesh, points)
% Return a boolean indicating whether or not any of the given edges 
% and faces/normals intersect.

inMesh = points(inShape(mesh, points));
collision = size(inMesh,1) ~= 0;

end % function