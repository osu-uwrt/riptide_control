function [M, MA, DQ, DL, B, mass, buoyancy, com, cob, I, flipCost, ...
          xCOM, refCOM] = prepareCOMModel(quadratic_damping, ...
          linear_damping_matrix, added_mass_matrix, wrench_mat, m, bouy, ...
          center_of_mass, center_of_buoyancy, inertia_matrix, flip_cost, ...
          base_link, xBase, refBase)

% Build the COM-based Fossen model and convert its base_link boundary signals.
%
% center_of_mass and base_link are positions from the shared vehicle datum.
% All physical coefficients and wrench_mat already use COM.
% center_of_buoyancy is an offset from COM.
%
% xBase is a 13-state vector.
% refBase is the active reference trajectory.
%
% Quaternion order: [w x y z].
%
%#codegen

%% Physical parameters

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


%% COM offset

offset = reshape(base_link, 3, 1) ...
       - reshape(center_of_mass, 3, 1);


%% Convert measured state

xCOM = baseLinkToCOM(xBase, offset);


%% Reference trajectory

% Ensure reference is a 10-by-13 matrix.

reference = reshape(refBase, 10, 13);


% The reference quaternion may be zero until the first command arrives.
% In this case, use the measured COM attitude.

measuredQuat = xCOM(4:7);

referenceQuat = reference(1, 4:7).';


if all(referenceQuat == 0) || ...
        ~all(isfinite(referenceQuat)) || ...
        norm(referenceQuat) < 1e-8

    reference(1, 4:7) = measuredQuat.';

end


%% Convert reference trajectory to COM

for i = 1:10

    reference(i,:) = baseLinkToCOM(reference(i,:), offset);

end


%% Safeguard reference quaternion

% Use measured attitude as fallback.

for i = 1:10

    q = reference(i, 4:7).';

    if ~all(isfinite(q)) || norm(q) < 1e-8

        q = measuredQuat;

    else

        q = q / norm(q);

    end

    reference(i, 4:7) = q.';

end


%% Quaternion sign alignment

% q and -q describe the same physical orientation.
% Align each reference quaternion with the measured attitude.

for i = 1:10

    q = reference(i, 4:7).';

    if dot(q, measuredQuat) < 0

        q = -q;

    end

    reference(i, 4:7) = q.';

end


%% Output

refCOM = reference;

end