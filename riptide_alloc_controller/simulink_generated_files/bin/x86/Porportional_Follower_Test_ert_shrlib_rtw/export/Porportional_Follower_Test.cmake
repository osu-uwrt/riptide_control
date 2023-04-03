include("${CMAKE_CURRENT_LIST_DIR}/Porportional_Follower_TestTargets.cmake")
target_include_directories(Porportional_Follower_Test::Porportional_Follower_Test INTERFACE 
    ${MATLAB_ROOT}/extern/include
    ${MATLAB_ROOT}/simulink/include
    ${MATLAB_ROOT}/rtw/c/src
    ${MATLAB_ROOT}/rtw/c/src/ext_mode/common
    ${MATLAB_ROOT}/rtw/c/ert)
