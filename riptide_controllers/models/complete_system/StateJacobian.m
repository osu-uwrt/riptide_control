function [A, Bmv] = StateJacobian(x, ~, M, MA, DQ, DL, B, mass, buoyancy, com, cob, I, ~)
% Local sensitivities: delta(x_dot) = A*delta(x) + Bmv*delta(u).
% State position and body linear velocity are based at COM.
% Output A: this gives the optimizer an idea of how a change in the state
% will effect the RATE of change of the state

% Output B: this gives the optimizer the matrix for how changes in the
% control outputs will effect the RATE of change in the state
quat = x(4:7);
nu = x(8:13);
v = nu(1:3);
omega = nu(4:6);

% FossenStateFcn normalizes the quaternion before using its rotation matrix.
nq = norm(quat);
qhat = quat / nq;
qw = qhat(1);
qx = qhat(2);
qy = qhat(3);
qz = qhat(4);
Jnormalize = (eye(4) - qhat * qhat.') / nq;

R = bodyToWorldRotation(quat);

%Finding positional derivatives based on orientation and velocities
%
dR_dqhat = rotationPartials(qhat);
dR_dq = zeros(3, 3, 4);
for j = 1:4
    for k = 1:4
        dR_dq(:, :, j) = dR_dq(:, :, j) + dR_dqhat(:, :, k) * Jnormalize(k, j);
    end
end
dp_dq = zeros(3, 4);
for j = 1:4
    dp_dq(:, j) = dR_dq(:, :, j) * v;
end

A = zeros(13, 13);
A(1:3, 4:7) = dp_dq;
A(1:3, 8:10) = R;

% Quaternion kinematics use the raw quaternion in FossenStateFcn, so their
% derivatives must not include Jnormalize.
A(4:7, 4:7) = 0.5 * [0, -omega.';
                       omega, -skew(omega)];
% These rates use the raw state quaternion, unlike rotation/restoring terms.
w = quat(1);
t = quat(2:4);
A(4:7, 11:13) = 0.5 * [-t.';
                         w * eye(3) + skew(t)];

% Derivative of the physical restoring wrench used by FossenStateFcn.
dup_dqhat = [-2*qy,  2*qz, -2*qw,  2*qx;
              2*qx,  2*qw,  2*qz,  2*qy;
              0,    -4*qx, -4*qy,  0];
dup_dq = dup_dqhat * Jnormalize;
rCombined = buoyancy * cob - mass * 9.80665 * com;
restoringJacobian = [(buoyancy - mass * 9.80665) * dup_dq;
                     skew(rCombined) * dup_dq];
A(8:13, 4:7) = M \ restoringJacobian;

% C(nu)*nu is quadratic in velocity. Differentiate both C and nu:
% d(C*nu)/dnu_j = C(:,j) + (dC/dnu_j)*nu.
C = getCoriolisMatrix(nu, MA, mass, com, I);
Jcoriolis = C;
unit = eye(6);
for j = 1:6
    Cj = getCoriolisMatrix(unit(:,j), MA, mass, com, I);
    Jcoriolis(:,j) = Jcoriolis(:,j) + Cj*nu;
end
Jdamping = DL + DQ * diag(2 * abs(nu));
A(8:13, 8:13) = -(M \ (Jcoriolis + Jdamping));

% The input wrench is B*u; M and B are fixed parameters during a solve.
Bmv = [zeros(7, size(B, 2)); M \ B];
end

function C = getCoriolisMatrix(nu, MA, mass, com, I)
omega = nu(4:6);
MA_v = MA * nu;
a = MA_v(1:3);
b = MA_v(4:6);
CA = [zeros(3), -skew(a); -skew(a), -skew(b)];
Sg = skew(com);
Sw = skew(omega);
CRB = [mass*Sw, -mass*Sw*Sg; mass*Sg*Sw, -skew(I*omega)];
C = CRB + CA;
end

function dR = rotationPartials(quat)
% Partials of the homogeneous quaternion rotation polynomial at qhat.
w = quat(1);
t = quat(2:4);
e = eye(3);
dR = zeros(3, 3, 4);
dR(:, :, 1) = 2*w*eye(3) + 2*skew(t);
for j = 1:3
    dR(:, :, j + 1) = -2*t(j)*eye(3) + 2*(e(:,j)*t.' + t*e(:,j).') + 2*w*skew(e(:,j));
end
end

function R = bodyToWorldRotation(quat)
q = quat / norm(quat);
R = quat2rotm(q.');
end

function S = skew(v)
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end
