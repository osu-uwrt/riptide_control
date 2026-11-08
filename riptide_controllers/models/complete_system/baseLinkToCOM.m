function xCOM = baseLinkToCOM(xBase, baseLinkOffset)
% Convert a 13-state base_link vector to COM, with aligned body axes.
% baseLinkOffset points from COM to base_link, expressed in body axes.
% Position is expressed in world axes; linear/angular velocity in body axes.
% Quaternion order is [w x y z]. Preserve its raw components for the dynamics.

xCOM = reshape(xBase, 13, 1);
offset = reshape(baseLinkOffset, 3, 1);
quat = xCOM(4:7);
assert(all(isfinite(quat)) && norm(quat) > 0, ...
    'base_link orientation must be a finite, nonzero quaternion.');
q = quat / norm(quat);
R = quat2rotm(q.');
xCOM(1:3) = xCOM(1:3) - R * offset;
xCOM(8:10) = xCOM(8:10) - cross(xCOM(11:13), offset);
end
