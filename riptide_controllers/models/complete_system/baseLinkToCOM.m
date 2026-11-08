function xCOM = baseLinkToCOM(xBase, baseLinkOffset)

% Convert a 13-state base_link vector to COM.
%
% baseLinkOffset points from COM to base_link, expressed in body axes.
%
% Position is expressed in world axes.
% Linear/angular velocity is expressed in body axes.
%
% Quaternion order: [w x y z].
%
% Invalid or zero quaternions are replaced with identity orientation.
%
%#codegen


%% State initialization

xCOM = reshape(xBase, 13, 1);

offset = reshape(baseLinkOffset, 3, 1);


%% Quaternion

quat = xCOM(4:7);


% Default orientation:
% Identity quaternion [1 0 0 0].

if ~all(isfinite(quat)) || norm(quat) < 1e-8

    quat = [1; 0; 0; 0];

else

    quat = quat / norm(quat);

end


%% Rotation matrix

R = quat2rotm(quat.');


%% Position conversion

% Convert base_link position to COM position.

xCOM(1:3) = xCOM(1:3) - R * offset;


%% Linear velocity conversion

% v_COM = v_base_link - omega x r_COM_to_base

omega = xCOM(11:13);

xCOM(8:10) = xCOM(8:10) - cross(omega, offset);


%% Store normalized quaternion

xCOM(4:7) = quat;


end