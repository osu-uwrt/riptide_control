function test_profiler_int()
    vmax = [1, 2, 3, 4, 5, 6];
    amax = [2, 4, 6, 8, 10, 12];
    jmax = [3, 6, 9, 12, 15, 18];
    time_step = 0.02;

    curr_state = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];
    desired_state = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24];

    generate_motion_profile(curr_state, desired_state, vmax, amax, jmax, time_step);
end
