include("${CMAKE_CURRENT_LIST_DIR}/FaultControllerModelTargets.cmake")
target_include_directories(FaultControllerModel::FaultControllerModel INTERFACE 
    ${MATLAB_ROOT}/extern/include
    ${MATLAB_ROOT}/simulink/include
    ${MATLAB_ROOT}/rtw/c/src
    ${MATLAB_ROOT}/rtw/c/src/ext_mode/common
    ${MATLAB_ROOT}/rtw/c/ert)
