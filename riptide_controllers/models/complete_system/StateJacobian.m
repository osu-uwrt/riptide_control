function [A, Bmv] = StateJacobian(x, thrusterForces, M, MA, DQ, DL, B, mass, buoyancy, com, cob, I, flipCost) 

%states
quat = x(4:7);
nu = x(8:13);

%from states
qw = quat(1);
qx = quat(2);
qy = quat(3);
qz = quat(4);

vx = nu(1);
vy = nu(2);
vz = nu(3);
p = nu(4);   
q = nu(5);    
r = nu(6);

R = bodyToWorldRotation(quat);


S = 0.5 * [-qx, -qy, -qz;
             qw, qy, -qz;
             qw, qz, -qx;
             qw, qx, -qy];
    
dqd_dq = 0.5  * [0, -p, -q, -r;
                  p, 0, r, -q;
                  q, -r, 0, p;
                  r, q, -p, 0];


dpd_dq = 2.0 * [
    qy * vz - qz * vy,   qy * vy + qz * vz,   qw * vz + qx * vy - 2.0 * qy * vx,   -qw * vy + qx * vz - 2.0 * qz * vx;
    -qx * vz + qz * vx,  -qw * vz - 2.0 * qx * vy + qy * vx,  qx * vx + qz * vz,   qw * vx + qy * vz - 2.0 * qz * vy;
    qx * vy - qy * vx,   qw * vy - 2.0 * qx * vz + qz * vx,   -qw * vx - 2.0 * qy * vz + qz * vy,  qx * vx + qy * vy
];

%%%%%%
%MAYBE COULD WORK TO GET ANNOYING DERIVATIVES
dvd_dv = jacobian()

syms qw qx qy qz
q = [qw, qx, qy, qz]
dvd_dq = jacobian(nu_dot, q)
%this is like generally how it works I think but not sure
%%%%%%


pdot_rows = [zeros(3,3), dpd_dq, R, zeros(3,3)]; %DONE?          
qdot_rows = [zeros(4,3), dqd_dq, zeros(4,3), S]; %DONE?
vdot_rows = [zeros(6,3), dvd_dq, dvd_dv]; %NOT DONE

A = [pdot_rows;
     qdot_rows
     vdot_rows];

Bmv = [zeros(7,8); M\B];



function R = bodyToWorldRotation(quat)
    %normalize quat
    qnorm = sqrt(quat.' * quat);
    q = quat ./ qnorm;
    %make rotm
    R = quat2rotm(q');
end