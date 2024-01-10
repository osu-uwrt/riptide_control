%
% This script runs MATLAB message discovery on the riptide_msgs package so
% simulink models can use the custom definitions.
%

format compact

UWRT_ROOT_DIR = "~/osu-uwrt";
WORK_DIR = fullfile(UWRT_ROOT_DIR, 'matlab', 'custom_msgs');

MSG_PKG_NAME = "riptide_msgs2";
MSG_PKG_LOCATION = fullfile(UWRT_ROOT_DIR, ...
    'development', 'software', 'src', 'riptide_core', 'riptide_msgs');

% prepare the work dir by deleting it, re-creating it, then copying the
% messages package into it

fprintf("Preparing work directory\n");
if exist(WORK_DIR, 'file') == 7
    rmdir(WORK_DIR, 's');
end

% mkdir WORKDIR/MSG_PKG_NAME
msg_work_dir = fullfile(WORK_DIR, MSG_PKG_NAME);
mkdir(msg_work_dir);

% copy msgs into the work directory
copyfile(MSG_PKG_LOCATION, msg_work_dir);

% generate messages
ros2genmsg(WORK_DIR);
