#! /usr/bin/env python3

import argparse
import os
import sys
import subprocess
import platform
import shutil

#
# Figure out location of the models in the src tree. This may be weird because we may be running out of the install tree
#
FILE_LOC = os.path.abspath(__file__)
CONTROLLERS_ROOT_LOCATION = os.path.join(FILE_LOC[0 : FILE_LOC.find("/riptide_controllers/")], 
                                         "riptide_control", "riptide_controllers")

# check for install directory
if CONTROLLERS_ROOT_LOCATION.find("/install/") >= 0:
    CONTROLLERS_ROOT_LOCATION = os.path.join(CONTROLLERS_ROOT_LOCATION[0 : CONTROLLERS_ROOT_LOCATION.find("/install/")],
                                             "src", "riptide_control", "riptide_controllers")

UWRT_ROOT   = CONTROLLERS_ROOT_LOCATION[0 : CONTROLLERS_ROOT_LOCATION.find("/development/software/src/")]
MODELS_ROOT = os.path.join(CONTROLLERS_ROOT_LOCATION, "models")

#check that paths exist
def assert_path_exists(name, pth):
    if not os.path.exists(pth):
        print(f"FATAL: Could not verify the existence of the {name} in the source tree. " + \
            "Please ensure that the source tree exists, otherwise this script serves no purpose", file=sys.stderr)
        exit()

assert_path_exists("uwrt root", UWRT_ROOT)
print(f"Detected uwrt root as {UWRT_ROOT}")
assert_path_exists("models path", MODELS_ROOT)
print(f"Detected models root as {MODELS_ROOT}")

#
# MACHINE IDENTIFICATION
#

ARCHITECTURE_CONFIGS = {
    "x86" : os.path.join(MODELS_ROOT, "x86_cfg.m"),
    "arm" : os.path.join(MODELS_ROOT, "arm_cfg.m")
}

def determine_system_cfg(machine: str):
    for key in ARCHITECTURE_CONFIGS.keys():
        if key in machine:
            return ARCHITECTURE_CONFIGS[key]

#
# CONSTS
#

DEFAULT_LOCAL_CONFIG = determine_system_cfg(platform.uname().machine)
print(f"Selecting local config {DEFAULT_LOCAL_CONFIG}")
print()
DEFAULT_DEPLOY_CONFIG = ARCHITECTURE_CONFIGS["arm"]
DEFAULT_ROBOT_NAME = "orin"
DEFAULT_ARCHIVE_DIR = os.path.join(UWRT_ROOT, "controller_model_archives")
DEFAULT_LOCAL_DIR = os.path.join(UWRT_ROOT, "development", "software", "src", "controller_models")
DEFAULT_DEPLOY_DIR = os.path.join(UWRT_ROOT, "release", "src", "controller_models")


def execute_command(cmd: 'list[str]', cwd: str):
    proc = subprocess.run(cmd, cwd=cwd)
    if proc.returncode != 0:
        cmdstr = " ".join(cmd)
        raise RuntimeError(f"Command {cmdstr} returned with non-zero exit code {proc.returncode}")


def generate_packges(models_select: 'list[str]', models_ignore: 'list[str]', configs_select: 'list[str]', configs_ignore: 'list[str]'):
    matlab_cmd_args = []    
        
    if len(models_select) > 0:
        matlab_cmd_args += ["'--models-select'"] + wrap_entries(get_abs_paths(models_select), "'")
    elif len(models_ignore) > 0:
        matlab_cmd_args += ["'--models-ignore'"] + wrap_entries(get_abs_paths(models_ignore), "'")
    
    if len(configs_select) > 0:
        matlab_cmd_args += ["'--configs-select'"] + wrap_entries(get_abs_paths(configs_select), "'")
    elif len(configs_ignore) > 0:
        matlab_cmd_args += ["'--configs-ignore'"] + wrap_entries(get_abs_paths(configs_ignore), "'")
    
    matlab_cmd = ",".join(matlab_cmd_args)
    bash_cmd = ["matlab", "-batch", f"generate_cpp_packages({matlab_cmd})"]

    cmdstr = " ".join(bash_cmd)
    print(f"Running command {cmdstr}")
    execute_command(bash_cmd, cwd=MODELS_ROOT)


def archive_packages(archives_dir: str):
    print(f"Archiving packages in directory {archives_dir}")
    generated_output_directory = os.path.join(MODELS_ROOT, "generated_models")
    ensure_not_directory_exists(archives_dir)
    shutil.copytree(generated_output_directory, archives_dir)


def handle_local_packages(local_config: str, local_dir: str, no_build: bool):
    print(f"Storing local packages ({os.path.basename(local_config)}) in {local_dir} and " + \
        f"{'not building' if no_build else 'building'}")
    
    handle_packages_generic(local_config, local_dir)
    
    if not no_build:
        # figure out where to run the build. If the directory is within UWRT_ROOT/development/software/src, then 
        # the build will be run in UWRT_ROOT/developement/software. Otherwise, the build will happen in place
        build_dir = local_dir
        software_dir = os.path.join(UWRT_ROOT, "development", "software")
        if software_dir in local_dir:
            print(f"Local directory detected in software directory. Building in software directory")
            build_dir = software_dir
            
        print(f"Building models in directory {build_dir}")
        execute_command(["colcon", "build"], build_dir)


def handle_deploy_packages(deploy_config: str, deploy_dir: str, no_deploy: bool, deploy_target: str):
    print(f"Storing deploy packages ({os.path.basename(deploy_config)}) in {deploy_dir} and " + \
        f"{'not deploying' if no_deploy else 'deploying'} to {deploy_target}")
    
    handle_packages_generic(deploy_config, deploy_dir)
    
    if not no_deploy:
        # figure out where to run the deploy. If the directory is within UWRT_ROOT/release/src, then 
        # the build will be run in UWRT_ROOT/developement/software. Otherwise, the build will happen in place
        cmd_dir = deploy_dir
        rel_dir = os.path.join(UWRT_ROOT, "release")
        if rel_dir in deploy_dir:
            print(f"Local directory detected in release directory. Deploying from release directory")
            cmd_dir = rel_dir
            
        print(f"Building models in directory {cmd_dir}")
        execute_command(["colcon", "deploy", deploy_target], cmd_dir)


def handle_packages_generic(config, dir):
    generated_output_directory = os.path.join(MODELS_ROOT, "generated_models")
    config_name = get_object_name_from_file(config)
    archive_names = filter_list(os.listdir(generated_output_directory), config_name)
    
    ensure_not_directory_exists(dir) #clear directory if it exists
    ensure_directory_exists(dir)
    unpack_archives(generated_output_directory, archive_names, dir)
    

def ensure_directory_exists(dir: str):
    if not os.path.exists(dir):
        os.mkdir(dir)


def ensure_not_directory_exists(dir: str):
    if os.path.exists(dir):
        shutil.rmtree(dir)


def get_object_name_from_file(file):
    file_name = os.path.basename(file)
    object_name = file_name[0 : file_name.find('.')]
    return object_name


def filter_list(lis, filt):
    filtered = []
    for item in lis:
        if filt in item:
            filtered.append(item)
    
    return filtered


def unpack_archives(src: str, files: 'list[str]', dst: str):    
    for file_name in files:
        print(f"Unpacking archive {file_name}")
        src_name = os.path.join(src, file_name)
        dst_name = os.path.join(dst, file_name)
        shutil.copyfile(src_name, dst_name)
        
        # invoke tar to unpack archive
        tar_cmd = ["tar", "-xf", dst_name]
        execute_command(tar_cmd, cwd=dst)
        
        #remove archive
        os.remove(dst_name)
    

def find_files(files: 'list[str]'):
    paths = files
    for i in range(0, len(paths)):
        if not os.path.exists(paths[i]):
            model_relative_path = os.path.join(MODELS_ROOT, paths[i])
            if os.path.exists(model_relative_path):
                paths[i] = model_relative_path
            else:
                msg = f"File {files[i]} does not name an existing file relative to the current directory, the models directory, or root."
                print(msg, file=sys.stderr)
                raise ValueError(msg)
    
    return paths
                

def get_abs_paths(paths: 'list[str]'):
    abs_paths = []
    for i in range(0, len(paths)):
        abs_paths.append(os.path.abspath(paths[i]))

    return abs_paths


def wrap(thing: str, wrapper: str):
    return wrapper + thing + wrapper


def wrap_entries(arr: str, wrapper: str):
    wrapped = []
    for i in range(0, len(arr)):
        wrapped.append(wrap(arr[i], wrapper))
    
    return wrapped


def parse_args():
    parser = argparse.ArgumentParser(
        prog = "generate_cpp_packages.py",
        description = "Generates c++ packages from Simulink models in the riptide_controllers source tree. This program will, " + \
                            "unless otherwise specified using the arguments below, invoke MATLAB to detect all available models " + \
                            "and build them with every available config. The number of generated packages will be equal to the " + \
                            "number of actionable models times the number of actionable configs. After generating the packages, " + \
                            "this program will automatically move the archives into the directory specified by --archives-dir, " + \
                            "the local packages (runnable by the local machine) into the directory specified by --local-dir, " + \
                            "and the deployable packages (runnable by the robot) into the directory specified by --deploy-dir. " + \
                            "Then, unless otherwise specified by the --no-build and --no-deploy flags, the program will " + \
                            "invoke Colcon to build the local packages and deploy the deployable ones."
    )
    
    parser.add_argument("--models-select", action="store", default=[], nargs="+",
                        help="Select specific models to build. Models not specified after this flag will not be built. " + \
                            "This flag takes precedence over --models-ignore. If neither --models-ignore or --models-select " + \
                            "are used, then all packages will be built")
    
    parser.add_argument("--models-ignore", action="store", default=[], nargs="+",
                        help="Select specific models NOT to build. If --models-select is used, this flag will be ignored")
    
    parser.add_argument("--configs-select", action="store", default=[], nargs="+",
                        help="Select specific confurations to use when building the models. Configs not specified after " + \
                            "this flag will not be used. This flag takes precedence over --configs-ignore. If neither " + \
                            "--configs-ignore or --configs-select are used, then all configs will be used")
    
    parser.add_argument("--configs-ignore", action="store", default=[],  nargs="+",
                        help="Select specific configs NOT to build. If --configs-select is used, this flag will be ignored")
    
    parser.add_argument("--local-config", action="store", default=DEFAULT_LOCAL_CONFIG,
                        help="Sets the config that generates code runnable by the local machine. If allowed, code generated " + \
                            "with this config will be automatically transferred to the local-dir and built.")
    
    parser.add_argument("--deploy-config", action = "store", default=DEFAULT_DEPLOY_CONFIG,
                        help="Sets the config that generates code runnable by the robot. If allowed, code generated with) " + \
                            "this config will be automatically transferred to the deploy-dir and built")

    parser.add_argument("--no-archive", action="store_true",
                        help="If specified, the generated archives will not be stored")

    parser.add_argument("--no-process-local", action="store_true",
                        help="If specified, local packages will not be unpacked and built")
    
    parser.add_argument("--no-process-deploy", action="store_true",
                        help="If specified, deploy packages will not be unpacked and built")
    
    parser.add_argument("--no-build", action="store_true",
                        help="If specified, the program will not attempt to build the local packages")
    
    parser.add_argument("--no-deploy", action="store_true",
                        help="If specified, the program will not attempt to deploy the deployable packages")
    
    parser.add_argument("--archives-dir", action="store", default=DEFAULT_ARCHIVE_DIR,
                        help="The path to the directory in which to store the tar archives containing the generated packages")
    
    parser.add_argument("--local-dir", action="store", default=DEFAULT_LOCAL_DIR,
                        help="The path to the directory in which to store the local packages (packages which are runnable by the " + \
                            "local machine)")

    parser.add_argument("--deploy-dir", action="store", default=DEFAULT_DEPLOY_DIR,
                        help="The path to the directory in which to store the deployable packages (packages which are runnable by " + \
                            "the robot)")
    
    parser.add_argument("--deploy-target", action="store", default=DEFAULT_ROBOT_NAME,
                        help="Specifies the name of the target to deploy the deployable packages to")
        
    return parser.parse_args()


def main():
    args = parse_args()
    
    models_select = find_files(args.models_select)
    models_ignore = find_files(args.models_ignore)
    configs_select = find_files(args.configs_select)
    configs_ignore = find_files(args.configs_ignore)
    
    generate_packges(models_select, models_ignore, configs_select, configs_ignore)
    
    if not args.no_archive:
        archive_packages(os.path.abspath(args.archives_dir))
    
    if not args.no_process_local:
        handle_local_packages(
            args.local_config,
            os.path.abspath(args.local_dir),
            args.no_build)
    
    if not args.no_process_deploy:
        handle_deploy_packages(
            args.deploy_config,
            os.path.abspath(args.deploy_dir),
            args.no_deploy,
            args.deploy_target)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
