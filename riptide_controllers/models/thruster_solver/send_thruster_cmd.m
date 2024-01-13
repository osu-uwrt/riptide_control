%input: 8-long array of thruster rpm values
function send_thruster_cmd(cmds)
    SELECTED_COMMAND_FUNCTION_NAME = "send_thruster_cmd_canbus";
    errno = 0;

    if ~coder.target("MATLAB") %only execute this on generated c++ code
        coder.updateBuildInfo("addSourceFiles", ...
            SELECTED_COMMAND_FUNCTION_NAME + ".c");

        coder.updateBuildInfo('addIncludePaths','thruster_solver');
        coder.cinclude('send_thruster_cmd.h');
        
        errno = coder.ceval(SELECTED_COMMAND_FUNCTION_NAME, int16(cmds));
        if errno ~= 0
            error("Could not invoke %s. Failed with errno %d", ...
                SELECTED_COMMAND_FUNCTION_NAME, errno);
        end
    end
end
