%
% Generates a motion profile between the current state and the desired
% state. curr_state and desired_state are both input as vectors in the
% order: [x, y, z, r, p, w, vx, vy, vz, vr, vp, vw]
%
function profiles = generate_motion_profile( ...
    curr_state, desired_state, vmax, amax, jmax, time_step)
    if coder.target("MATLAB")
        profiles = generate_with_mex( ...
            curr_state, desired_state, vmax, amax, jmax, time_step);
    else
        profiles = generate_with_ceval( ...
            curr_state, desired_state, vmax, amax, jmax, time_step);
    end 
end


%
% Uses a MEX function to invoke the C++ code that generates the motion
% profile. Because MEX cannot be compiled to C++, this function would only
% be used when debugging IN LOOP
% 
function profiles = generate_with_mex( ...
    curr_state, desired_state, vmax, amax, jmax, time_step)
    profiles = zeros(4, 4);
    fprintf("mex not supported yet\n");
end


%
% Uses ceval to invoke the C++ code that generates the motion profile.
% Because ceval cannot be used to debug in loop, this function will only be
% used when running generated code.
%
function profiles = generate_with_ceval( ...
    curr_state, desired_state, vmax, amax, jmax, time_step)
    
    DEGREES_OF_FREEDOM = 6;
    profile_length = 0;

    coder.updateBuildInfo("addSourceFiles", "profiler_matlab_interface.cpp")
    
    profile_length = coder.ceval("profiler_invoke", ...
        curr_state, desired_state, vmax, amax, jmax, time_step)

    profiles = zeros(DEGREES_OF_FREEDOM, profile_length);

    for i = 1 : DEGREES_OF_FREEDOM
        coder.ceval("profiler_get", i, coder.wref(profiles(i)));
    end

    profiles
end
