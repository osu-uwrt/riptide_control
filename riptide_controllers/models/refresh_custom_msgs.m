%
% This script runs MATLAB message discovery on the riptide_msgs package so
% simulink models can use the custom definitions.
%

format compact

UWRT_DIR_NAME = 'osu-uwrt';
UWRT_ROOT_DIR = fullfile('~', UWRT_DIR_NAME);
WORK_DIR = fullfile(UWRT_ROOT_DIR, 'matlab', 'custom_msgs');

current_directory = pwd;
if contains(current_directory, UWRT_DIR_NAME)
    pwd_uwrt_index = strfind(current_directory, UWRT_DIR_NAME);
    pwd_from_uwrt_root = current_directory(length(UWRT_ROOT_DIR) + ...
        pwd_uwrt_index - 1: end);

    uwrt_work_dir = pwd_from_uwrt_root(1 : strfind(pwd_from_uwrt_root, ...
        '/src') - 1);
else
    error("Cannot find the osu-uwrt directory!");
end

MSG_PKG_NAME = "riptide_msgs2";
MSG_PKG_LOCATION = fullfile(UWRT_ROOT_DIR, uwrt_work_dir, ...
    'src', 'riptide_core', 'riptide_msgs');

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
