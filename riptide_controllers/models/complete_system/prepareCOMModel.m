function [M, MA, DQ, DL, B, mass, buoyancy, com, cob, I, flipCost, ...
          xCOM, refCOM] = prepareCOMModel(quadratic_damping, ...
          linear_damping_matrix, added_mass_matrix, wrench_mat, m, bouy, ...
          center_of_mass, center_of_buoyancy, inertia_matrix, flip_cost, ...
          base_link, xBase, refBase)
% Build the COM-based Fossen model and convert its base_link boundary signals.
% center_of_mass and base_link are positions from the shared vehicle datum.
% All physical coefficients and wrench_mat already use COM; do not shift them.
% center_of_buoyancy is an offset from COM, not from the shared datum.
% xBase is a 13-state vector; refBase is the active 1-by-13 pose/velocity target.
%#codegen
DQ = diag(reshape(quadratic_damping, 6, 1));
DL = reshape(linear_damping_matrix, 6, 6);
MA = reshape(added_mass_matrix, 6, 6);
B = reshape(wrench_mat, 6, 8);
mass = m;
buoyancy = bouy;
I = reshape(inertia_matrix, 3, 3);
cob = reshape(center_of_buoyancy, 3, 1);
flipCost = flip_cost;

% At the COM there is no gravity moment arm or rigid-body coupling offset.
com = zeros(3, 1);
M = blkdiag(mass * eye(3), I) + MA;

offset = reshape(base_link, 3, 1) - reshape(center_of_mass, 3, 1);
xCOM = baseLinkToCOM(xBase, offset);
% referenceBase = reshape(refBase, 13, 1);
% The angular command subscriber has an all-zero quaternion until the first
% command arrives after enable. Use the measured attitude during that gap;
% preserve the desired position and velocities instead of inventing a pose.
if all(refBase(4:7) == 0)
    refBase(4:7) = xCOM(4:7);
end

reference = zeros(10, 13);
for i = 1:10
    reference(i,:) = baseLinkToCOM(refBase(i, :), offset);
end

% q and -q describe the same attitude. Track the measured representation.
reference(4:7) = reference(4:7) / norm(reference(4:7));
if dot(reference(4:7), xCOM(4:7)) < 0
    reference(4:7) = -reference(4:7);
end
refCOM = reference;
end
