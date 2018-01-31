function plane = createPlane(p10, p1end, p20, p2end, scale, extension, transparency, color)
% create plane from two vectors

% scaled vectors
dv1 = (p1end - p10).*(extension);
dv2 = (p2end - p20).*(extension);
v1 = (p1end - p10).*(scale + extension);
v2 = (p2end - p20).*(scale + extension);

points = [v1 + v2, v1 - dv2, -dv1 - dv2, -dv1 + v2];
X = points(1, :);
Y = points(2, :);
Z = points(3, :);
plane = patch(X,Y,Z,color);
set(plane, 'facealpha', transparency);
end