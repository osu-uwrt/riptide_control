function test_profiler_int()
    res = zeros(3, 3);
    coder.updateBuildInfo("addSourceFiles", "profiler_matlab_interface.cpp");
    test = [1,2,3,4,5,6,7,8,9];
    a = int64(3);
    b = int64(3);
    coder.ceval("call_profiler_hello", coder.ref(test), a, b, coder.wref(res));
    disp(res);
end
