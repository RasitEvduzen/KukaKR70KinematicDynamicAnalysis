function [transPoints] = TransformGeo(points, Tmatrix)
[sze, ~] = size(points);
points(:, 4) = 1;
transPoints = zeros(sze, 4);
for i = 1:sze
    transPoints(i, :) = Tmatrix*points(i, :)';
end
end