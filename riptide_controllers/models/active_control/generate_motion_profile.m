%
% Generates a motion profile between the current state and the desired
% state. curr_state and desired_state are both input as vectors in the
% order: [x, y, z, r, p, w, vx, vy, vz, vr, vp, vw]
%
function profiles = generate_motion_profile(curr_state, desired_state)
    if coder.target("MATLAB")
        profiles = generate_with_mex(curr_state, desired_state);
    else
        profiles = generate_with_ceval(curr_state, desired_state);
    end
end


%
% Uses a MEX function to invoke the C++ code that generates the motion
% profile. Because MEX cannot be compiled to C++, this function would only
% be used when debugging IN LOOP
% 
function profiles = generate_with_mex(curr_state, desired_state)

end


%
% Uses ceval to invoke the C++ code that generates the motion profile.
% Because ceval cannot be used to debug in loop, this function will only be
% used when running generated code.
%
function profiles = generate_with_ceval(curr_state, desired_state)

end
