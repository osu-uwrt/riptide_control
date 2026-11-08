function dxdt = FossenStateFcn(x, thrusterForces, M, MA, DQ, DL, B, mass, buoyancy, com, cob, I, flipCost)
    % x contains world COM position, wxyz attitude, and body velocity at COM.
    % M, MA, damping, inertia and thruster moments are all based at COM.
    % The parameter builder supplies com=zeros(3,1), with cob relative to COM.
    quat = x(4:7);
    nu = x(8:13); %linear and angular velos
    
    R = bodyToWorldRotation(quat);
    g = getRestoringVector(R, mass, buoyancy, com, cob);

    tau = B * thrusterForces;

    % Compute C(nu)*nu without constructing the full Coriolis matrix.
    v = nu(1:3);
    omega = nu(4:6);
    MA_v = MA * nu;
    a = MA_v(1:3);
    b = MA_v(4:6);
    coriolis = [mass * cross(omega, v) - mass * cross(omega, cross(com, omega)) + cross(omega, a);
                mass * cross(com, cross(omega, v)) + cross(omega, I * omega) ...
                - cross(a, v) + cross(omega, b)];

    damping = DL * nu + DQ * (abs(nu) .* nu);
    nu_dot = M \ (tau - coriolis - damping - g);
    position_dot = R * nu(1:3);
    
    quat_dot = getQuaternionDerivative(quat, nu);
    
    dxdt = [position_dot; quat_dot; nu_dot];
end

function quat_dot = getQuaternionDerivative(quat, nu)
    
    %get quat
    qw = quat(1);
    qx = quat(2);
    qy = quat(3);
    qz = quat(4);

    % Angular velocities
    p = nu(4);   
    q = nu(5);    
    r = nu(6);    

    %q derivative calc
    quat_dot = 0.5 * [
        -qx*p - qy*q - qz*r;
         qw*p + qy*r - qz*q;
         qw*q + qz*p - qx*r;
         qw*r + qx*q - qy*p
    ];
end

function g = getRestoringVector(R, mass, buoyancy, com, cob)
   
    %rotate gravity and bouyancy into body
    weightBody = R.' * [0; 0; -mass * 9.80665];
    buoyancyBody = R.' * [0; 0; buoyancy];

    %get forces and moments
    forceBody = weightBody + buoyancyBody;
    momentBody = cross(com, weightBody) + cross(cob, buoyancyBody);
    
    g = -[forceBody; momentBody];
end

function R = bodyToWorldRotation(quat)
    %normalize quat
    qnorm = sqrt(quat.' * quat);
    q = quat ./ qnorm;
    %make rotm
    R = quat2rotm(q');
end
