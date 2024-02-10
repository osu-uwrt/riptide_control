#! /usr/bin/env python3

#
# UWRT controller MATLAB model manager. 
# handles code generation, download, and other general management of matlab and all simulink models in all configurations.
#

import argparse
import os
import sys
import subprocess
import platform
import shutil
import glob
import re

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

UWRT_ROOT   = os.path.expanduser("~/osu-uwrt")
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
    "x86" : "x86_cfg",
    "arm" : "arm_cfg"
}

def determine_system_cfg(machine: str):
    for key in ARCHITECTURE_CONFIGS.keys():
        if key in machine:
            return ARCHITECTURE_CONFIGS[key]
    
    return ""

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

DEFAULT_DOWNLOAD_LATEST_URL = "https://github.com/osu-uwrt/riptide_control/releases/latest/download/"
DEFAULT_DOWNLOAD_VERSION_URL = "https://github.com/osu-uwrt/riptide_control/releases/download/"

#list of file patterns that are okay to delete when cleaning the workspace
#these should all be in the gitignore.
#all paths are evaluated relative to MODELS_ROOT
CLEANABLE_FILE_PATTERNS = [
    "**/+bus_conv_fcns",
    "**/*_ert_rtw",
    "**/slprj",
    "**/src",
    "**/*.slxc",
    "**/build_ros2_model.sh",
    "**/generated_models",
    "**/*.tgz",
    "**/codegen",
    "**/*.mex*"
]

SIMULINK_PROJECT_NAME = "Riptide_control_models.prj"

#cmdline names
GENERATE_PACKAGES_TASK_NAME = "generate_packages"
REFRESH_MSGS_TASK_NAME      = "refresh_custom_msg_support"
DOWNLOAD_PACKAGES_TASK_NAME = "download_packages"
DELETE_PACKAGES_TASK_NAME   = "delete_packages"
CLEAN_WORKSPACE_TASK_NAME   = "clean_workspace"
OPEN_PROJECT_TASK_NAME    = "open_project"
PROCESS_CACHED_TASK_NAME    = "process_cached"


def execute_command(cmd: 'list[str]', cwd: str):
    proc = subprocess.run(cmd, cwd=cwd)
    if proc.returncode != 0:
        cmdstr = " ".join(cmd)
        raise RuntimeError(f"Command {cmdstr} returned with non-zero exit code {proc.returncode}")


def generate_packages(models: 'list[str]', configs: 'list[str]'):
    # "wrap" model and config names with quotation marks for matlab
    wrapped_models = [f"\"{model}\"" for model in models]
    wrapped_configs = [f"\"{config}\"" for config in configs]
    
    matlab_cmd_args = [f"[{','.join(wrapped_models)}]", f"[{','.join(wrapped_configs)}]"]
    matlab_cmd = ",".join(matlab_cmd_args)
    bash_cmd = ["matlab", "-batch", f"generate_cpp_packages({matlab_cmd})"]

    cmdstr = " ".join(bash_cmd)
    print(f"Running command {cmdstr}")
    execute_command(bash_cmd, cwd=MODELS_ROOT)


def refresh_custom_messages():
    cmd = ["matlab", "-batch", "refresh_custom_msgs"]
    cmdstr = " ".join(cmd)
    print(f"Running command {cmdstr}")
    execute_command(cmd, cwd=MODELS_ROOT)


def download_packages(
    url: str, 
    local_config: str, 
    deploy_config: str,
    models: 'list[str]',
    configs: 'list[str]',
    assume_yes: bool
):
    local_config_name = get_object_name_from_file(local_config)
    deploy_config_name = get_object_name_from_file(deploy_config)
    print(f"Downloading packages from url {url} using local config {local_config_name} and deploy config {deploy_config_name}")
    
    # find and clear generated output dir
    generated_output_directory = os.path.join(MODELS_ROOT, "generated_models")
    ensure_not_directory_exists(generated_output_directory)
    ensure_directory_exists(generated_output_directory)
    
    if not yesNoPrompt("Proceed with download?", assume_yes):
        print("Not proceeding with operation.")
        exit(0)
        
    # perform download
    for model in models:
        for cfg in configs:
            try:
                download_single_package(url, model, cfg, generated_output_directory)
            except KeyboardInterrupt: #needed otherwise wget might go on a rampage because it becomes uninterruptable
                exit()
            except:
                print(f"Failed to Download model: {model} with config {cfg}. Please ensure the model exists!" + \
                      f"Tt should be listed as an asset at the link {url}", file=sys.stderr)


def delete_packages(
    packages: 'list[str]', 
    exclude_archive: bool,
    archives_dir: str, 
    exclude_local: bool,
    local_dir: str, 
    exclude_deploy: bool,
    deploy_dir: str, 
    assume_yes: bool
):
    def print_delete_summary(to_delete: 'list[str]', category: str):
        print(f"Deleting from {category}:")
        print_list(to_delete)
        print()
    
    (delete_archive, delete_local, delete_deploy) = resolve_archives_to_delete(
        packages,
        exclude_archive,
        archives_dir,
        exclude_local,
        local_dir,
        exclude_deploy,
        deploy_dir
    )
    
    print_delete_summary(delete_archive, "archive")
    print_delete_summary(delete_local, "local")
    print_delete_summary(delete_deploy, "deploy")
        
    #remove if user consents
    if yesNoPrompt("Continue?", assume_yes):
        print("Deleting")
        for name in delete_archive + delete_local + delete_deploy:
            #expect file
            remove_file_or_directory(os.path.join(archives_dir, name))
    else:
        print("Not deleting")


def clean_workspace(archives_dir: str, local_dir: str, deploy_dir: str, full_clean: bool, no_delete_packages: bool, assume_yes: bool):
    # list will keep track of absolute path of items to delete
    to_delete = []
    
    #change into models directory to avoid hitting items outside of there
    os.chdir(MODELS_ROOT)
    
    #full clean with user consent if necessary
    if full_clean:
        print("Specifying --full-clean will result in custom message support being deleted. This can be regenerated with the command: \n" + \
              f"  ros2 run riptide_controllers2 model_manager.py {REFRESH_MSGS_TASK_NAME}.\n")
        if yesNoPrompt("Include custom message support files in clean?", assume_yes):
            to_delete.append(os.path.expanduser("~/osu-uwrt/matlab/custom_msgs"))
        else:
            print("Custom message support will NOT be cleaned as a result of this operation.")
        
        print()
    
    # add package items if not specified otherwise
    if not no_delete_packages:
        (delete_archive, delete_local, delete_deploy) = resolve_archives_to_delete(
            [],
            False,
            archives_dir,
            False,
            local_dir,
            False,
            deploy_dir)

        to_delete += delete_archive + delete_local + delete_deploy
    
    #add glob files
    for pattern in CLEANABLE_FILE_PATTERNS:
        items = glob.glob(pattern, recursive=True)
        for item in items:
            to_delete.append(os.path.abspath(item))
    
    #print a summary of what we will do
    print("The clean operation will result in the following items being deleted:")
    print_list(to_delete)
    print()
    
    if yesNoPrompt("Continue?", assume_yes):
        print("Deleting")
        for item in to_delete:
            if os.path.exists(item):
                remove_file_or_directory(item)
    else:
        print("Not deleting")


def open_project():
    cmd = ["matlab", "-r", f"openProject(\"{SIMULINK_PROJECT_NAME}\")"]
    cmdstr = " ".join(cmd)
    print(f"Running command {cmdstr}")
    execute_command(cmd, cwd=MODELS_ROOT)


def archive_packages(archives_dir: str):
    print(f"Archiving packages in directory {archives_dir}")
    generated_output_directory = os.path.join(MODELS_ROOT, "generated_models")
    ensure_not_directory_exists(archives_dir)
    shutil.copytree(generated_output_directory, archives_dir)
    

def download_single_package(url, model, config, output_dir):
    asset_name = model + "_" + config + ".tgz"
    asset_url = os.path.join(url, asset_name)
    execute_command(["wget", asset_url], cwd=output_dir)


def handle_local_packages(local_config: str, local_dir: str, with_build: bool):
    print(f"Storing local packages ({os.path.basename(local_config)}) in {local_dir} and " + \
        f"{'building' if with_build else 'not building'}")
    
    local_dir_container = os.path.dirname(local_dir)
    if not os.path.exists(local_dir_container):
        print(f"No local path {local_dir_container}. Skipping handling of local (development) packages.")
        return
    
    handle_packages_generic(local_config, local_dir)
    
    if with_build:
        # figure out where to run the build. If the directory is within UWRT_ROOT/development/software/src, then 
        # the build will be run in UWRT_ROOT/developement/software. Otherwise, the build will happen in place
        build_dir = local_dir
        software_dir = os.path.join(UWRT_ROOT, "development", "software")
        if software_dir in local_dir:
            print(f"Local directory detected in software directory. Building in software directory")
            build_dir = software_dir
        
        #read off directory names to figure out the names of the packages to build
        pkgs = os.listdir(local_dir)
        cmd = ["colcon", "build"]
        if len(pkgs) > 0:
            cmd += ["--packages-select"] + pkgs
            
        print(f"Building models in directory {build_dir}")
        execute_command(cmd, build_dir)


def handle_deploy_packages(deploy_config: str, deploy_dir: str, with_deploy: bool, deploy_target: str):
    print(f"Storing deploy packages ({os.path.basename(deploy_config)}) in {deploy_dir} and " + \
        f"{'deploying' if with_deploy else 'not deploying'} to {deploy_target}")

    deploy_dir_container = os.path.dirname(deploy_dir)
    if not os.path.exists(deploy_dir_container):
        print(f"No deploy path {deploy_dir_container}. Skipping handling of deploy (release) packages.")
        return
    
    handle_packages_generic(deploy_config, deploy_dir)
    
    if with_deploy:
        # figure out where to run the deploy. If the directory is within UWRT_ROOT/release/src, then 
        # the build will be run in UWRT_ROOT/developement/software. Otherwise, the build will happen in place
        
        if ping_device(deploy_target):
            cmd_dir = deploy_dir
            rel_dir = os.path.join(UWRT_ROOT, "release")
            if rel_dir in deploy_dir:
                print(f"Local directory detected in release directory. Deploying from release directory")
                cmd_dir = rel_dir
            
            #read off directory names to figure out which packages to deploy
            pkgs = os.listdir(deploy_dir)
            cmd = ["colcon", "deploy", deploy_target]
            if len(pkgs) > 0:
                cmd += ["--packages-select"] + pkgs
            
            print(f"Building models in directory {cmd_dir}")
            execute_command(cmd, cmd_dir)
        else:
            print(f"{deploy_target} not detected on the local network. Not deploying")


def handle_packages_generic(config: str, dir: str):
    generated_output_directory = os.path.join(MODELS_ROOT, "generated_models")
    config_name = get_object_name_from_file(config)
    archive_names = filter_list(os.listdir(generated_output_directory), config_name)
    
    ensure_directory_exists(dir)
    unpack_archives(generated_output_directory, archive_names, dir)


def resolve_archives_to_delete(
    packages: 'list[str]', 
    exclude_archive: bool,
    archives_dir: str, 
    exclude_local: bool,
    local_dir: str, 
    exclude_deploy: bool,
    deploy_dir: str, 
):
    packages_snake = [title_to_snake(package) for package in packages]
    
    def resolve_files_to_delete(dir: str, packages: 'list[str]', exclude: bool, include_build_install: bool, ws_base_dir: str):
        to_delete = [] #contains absolute paths of items to be deleted
        if not exclude:
            dir_contents = os.listdir(dir)
            for file_name in dir_contents:
                if len(packages) > 0:
                    for package in packages:
                            if file_name.startswith(package):
                                to_delete.append(os.path.join(dir, file_name))
                else:
                    to_delete.append(os.path.join(dir, file_name))
            
            # delete the package from the build and install directories if necessary
            if include_build_install:
                delete_tmp = to_delete.copy() #avoid infinite loop
                for path in delete_tmp:
                    file_name = os.path.basename(path)
                    build_path = os.path.join(ws_base_dir, "build", file_name)
                    install_path = os.path.join(ws_base_dir, "install", file_name)
                    
                    if os.path.exists(build_path):
                        to_delete.append(build_path)
                    
                    if os.path.exists(install_path):
                        to_delete.append(install_path)
            
        return to_delete
    
    #figure out where workspace bases are for local and deploy
    local_ws_dir = local_dir
    software_dir = os.path.join(UWRT_ROOT, "development", "software")
    if software_dir in local_dir:
        local_ws_dir = software_dir
    
    deploy_ws_dir = deploy_dir
    release_dir = os.path.join(UWRT_ROOT, "release")
    if release_dir in deploy_dir:
        deploy_ws_dir = release_dir
    
    #figure out what to delete out of archive directory
    delete_from_archive = resolve_files_to_delete(archives_dir, packages, exclude_archive, False, "")
    delete_from_local = resolve_files_to_delete(local_dir, packages_snake, exclude_local, True, local_ws_dir)
    delete_from_deploy = resolve_files_to_delete(deploy_dir, packages_snake, exclude_deploy, True, deploy_ws_dir)
    
    return (delete_from_archive, delete_from_local, delete_from_deploy)


# function taken from this answer and adapted a bit:
# https://stackoverflow.com/questions/1175208/elegant-python-function-to-convert-camelcase-to-snake-case
#
# this is helpful for learning regexes:
# https://docs.python.org/3/library/re.html
def title_to_snake(title: str):
    return re.sub(r'(?<!^)(?<=[a-z])(?=[A-Z]|[0-9])', '_', title).lower()


def ping_device(name: str):
    try:
        execute_command(["ping", "-W0.25", "-i0.25", "-c4", name], os.getcwd())
        return True
    except RuntimeError:
        return False


def yesNoPrompt(question: str, assume_yes: bool):
    if assume_yes:
        return True
    
    yes_no = input(question + " [y/n]: ")
    return yes_no.lower() == "y" or len(yes_no) == 0


def ensure_directory_exists(dir: str):
    if not os.path.exists(dir):
        os.mkdir(dir)


def ensure_not_directory_exists(dir: str):
    if os.path.exists(dir) and os.path.isdir(dir):
        shutil.rmtree(dir)


def remove_file_or_directory(path: str):
    if os.path.isdir(path):
        shutil.rmtree(path)
    else:
        os.remove(path)


def get_object_name_from_file(file):
    file_name = os.path.basename(file)
    object_name = file_name[0 : file_name.find('.')]
    return object_name


def get_object_names_from_files(files):
    names = []
    for file in files:
        names.append(get_object_name_from_file(file))
    
    return names


def filter_list(lis, filt):
    filtered = []
    for item in lis:
        if filt in item:
            filtered.append(item)
    
    return filtered


def list_intersection(l1, l2):
    return [item for item in l1 if item in l2]

#returns l1 - l2
def list_difference(l1, l2):
    return [item for item in l1 if item not in l2]


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
        remove_file_or_directory(dst_name)


def print_list(lis: list):
    for item in lis:
        print(f"  {item}")
           

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
                            "unless otherwise specified using the arguments below, download all available models or " + \
                            "invoke MATLAB to build them with every available config. The number of " + \
                            "generated/downloaded packages will be equal to the " + \
                            "number of actionable models times the number of actionable configs. After generating the packages, " + \
                            "this program will automatically move the archives into the directory specified by --archives-dir, " + \
                            "the local packages (runnable by the local machine) into the directory specified by --local-dir, " + \
                            "and the deployable packages (runnable by the robot) into the directory specified by --deploy-dir. " + \
                            "Then, unless otherwise specified by the --no-build and --no-deploy flags, the program will " + \
                            "invoke Colcon to build the local packages and deploy the deployable ones."
    )
    
    parser.add_argument("--test", action="store_true")
    
    parser.add_argument("-y", "--assume-yes", action="store_true",
                        help="Assume yes on all prompts")
    
    select_parser = argparse.ArgumentParser(add_help=False)
    
    select_parser.add_argument("--models-select", action="store", default=[], nargs="+",
                        help="Select specific models to build. Models not specified after this flag will not be built. " + \
                            "This flag takes precedence over --models-ignore. If neither --models-ignore or --models-select " + \
                            "are used, then all packages will be built")
    
    select_parser.add_argument("--models-ignore", action="store", default=[], nargs="+",
                        help="Select specific models NOT to build. If --models-select is used, this flag will be ignored")
    
    select_parser.add_argument("--configs-select", action="store", default=[], nargs="+",
                        help="Select specific confurations to use when building the models. Configs not specified after " + \
                            "this flag will not be used. This flag takes precedence over --configs-ignore. If neither " + \
                            "--configs-ignore or --configs-select are used, then all configs will be used")
    
    select_parser.add_argument("--configs-ignore", action="store", default=[],  nargs="+",
                        help="Select specific configs NOT to build. If --configs-select is used, this flag will be ignored")
    
    select_parser.add_argument("--local-config", action="store", default=DEFAULT_LOCAL_CONFIG,
                        help="Sets the config that generates code runnable by the local machine. If allowed, code generated " + \
                            "with this config will be automatically transferred to the local-dir and built.")
    
    select_parser.add_argument("--deploy-config", action = "store", default=DEFAULT_DEPLOY_CONFIG,
                        help="Sets the config that generates code runnable by the robot. If allowed, code generated with " + \
                            "this config will be automatically transferred to the deploy-dir and built")
    
    dir_parser = argparse.ArgumentParser(add_help=False)
    
    dir_parser.add_argument("--archives-dir", action="store", default=DEFAULT_ARCHIVE_DIR,
                        help="The path to the directory in which to store the tar archives containing the generated packages")
    
    dir_parser.add_argument("--local-dir", action="store", default=DEFAULT_LOCAL_DIR,
                        help="The path to the directory in which to store the local packages (packages which are runnable by the " + \
                            "local machine)")

    dir_parser.add_argument("--deploy-dir", action="store", default=DEFAULT_DEPLOY_DIR,
                        help="The path to the directory in which to store the deployable packages (packages which are runnable by " + \
                            "the robot)")
    
    process_parser = argparse.ArgumentParser(add_help=False, parents=[ select_parser, dir_parser ])

    process_parser.add_argument("--no-process-local", action="store_true",
                        help="If specified, local packages will not be unpacked and built")
    
    process_parser.add_argument("--no-process-deploy", action="store_true",
                        help="If specified, deploy packages will not be unpacked and built")
    
    process_parser.add_argument("--build", action="store_true",
                        help="If specified, the program will unpack the local packages but it will not attempt to build them")
    
    process_parser.add_argument("--deploy", action="store_true",
                        help="If specified, the program will unpack the deploy packages but it will not attempt to deploy them")
    
    process_parser.add_argument("--deploy-target", action="store", default=DEFAULT_ROBOT_NAME,
                        help="Specifies the name of the target to deploy the deployable packages to")
    
    #
    # SUBPARSERS
    #
    subparsers = parser.add_subparsers(title="task", dest="task", help="The task to complete", required=True)
    
    #GENERATE_PACKAGES SUBPARSER
    
    generate_subparser = subparsers.add_parser(GENERATE_PACKAGES_TASK_NAME, parents=[ process_parser ], help="Generate Colcon packages from Simulink models")
    
    generate_subparser.add_argument("--no-archive", action="store_true",
                        help="If specified, the generated archives will not be stored")
    
    #REFRESH_CUSTOM_MSGS SUBPARSER
    refresh_subparser = subparsers.add_parser(REFRESH_MSGS_TASK_NAME, help="Refresh MATLAB support for custom ROS message types")
    
    #DOWNLOAD_PACKAGES SUBPARSER
    download_subparser = subparsers.add_parser(DOWNLOAD_PACKAGES_TASK_NAME, parents=[ process_parser ], help="Download released packages from the Internet")
    
    download_subparser.add_argument("--from-url", action="store", default=DEFAULT_DOWNLOAD_LATEST_URL,
                                    help="Specifies the URL to download the asset from.")
    
    download_subparser.add_argument("--from-release", action="store", default="latest",
                                    help="Specifies the github release from which to download the packages. Cannot be used with --from-url")
    
    #DELETE_PACKAGES SUBPARSER
    delete_subparser = subparsers.add_parser(DELETE_PACKAGES_TASK_NAME, parents=[ dir_parser ], help="Delete packages from the local and deploy directories")
    
    delete_subparser.add_argument("packages", action="store", nargs="*",
                                  help="specifies the packages to delete")
    
    delete_subparser.add_argument("--exclude-archive", action="store_true",
                                  help="Exclude the archive directory when finding files to delete")

    delete_subparser.add_argument("--exclude-local", action="store_true",
                                  help="Exclude the local directory when finding files to delete")

    delete_subparser.add_argument("--exclude-deploy", action="store_true",
                                  help="Exclude the deploy directory when finding files to delete")

    #CLEAN_WORKSPACE SUBPARSER
    clean_subparser = subparsers.add_parser(CLEAN_WORKSPACE_TASK_NAME, parents=[ dir_parser ], help="Clean workspace by deleting gitignored files and built packages")
    
    clean_subparser.add_argument("--full-clean", action="store_true",
                                 help="If specified, important items like custom message support files will be deleted.")
    
    clean_subparser.add_argument("--no-delete-packages", action="store_true", 
                                 help="If specified, does not delete code generated from models (what delete_packages normally does)")
    
    #OPEN_PROJECT SUBPARSER
    open_subparser = subparsers.add_parser(OPEN_PROJECT_TASK_NAME, help="Open the project in MATLAB")
    
    #PROCESS_CACHED SUBPARSER
    process_subparser = subparsers.add_parser(PROCESS_CACHED_TASK_NAME, parents=[ process_parser ], help="Process cached packages that were already downloaded or generated")
    
    return parser.parse_args()

def main():    
    args = parse_args()
    
    if args.task == REFRESH_MSGS_TASK_NAME:
        refresh_custom_messages()
    elif args.task == OPEN_PROJECT_TASK_NAME:
        open_project()
    elif args.task in [GENERATE_PACKAGES_TASK_NAME, DOWNLOAD_PACKAGES_TASK_NAME, PROCESS_CACHED_TASK_NAME, DELETE_PACKAGES_TASK_NAME, CLEAN_WORKSPACE_TASK_NAME]:
        if args.task == DELETE_PACKAGES_TASK_NAME:
            delete_packages(
                args.packages,
                args.exclude_archive,
                args.archives_dir,
                args.exclude_local,
                args.local_dir,
                args.exclude_deploy,
                args.deploy_dir,
                args.assume_yes)
            
        if args.task == CLEAN_WORKSPACE_TASK_NAME:
            clean_workspace(
                args.archives_dir,
                args.local_dir,
                args.deploy_dir,
                args.full_clean,
                args.no_delete_packages,
                args.assume_yes)
        
        if args.task in [GENERATE_PACKAGES_TASK_NAME, DOWNLOAD_PACKAGES_TASK_NAME, PROCESS_CACHED_TASK_NAME]:
            # these options use the select parser
            
            #
            # filter out models based on models_select and models_ignore
            #
            
            #glob all models not in the referenced_models directory
            # model_files = glob.glob(os.path.join(MODELS_ROOT, "[!referenced_models]**/*.slx"), recursive=True)
            model_files = glob.glob(os.path.join(MODELS_ROOT, "**/*.slx"), recursive=True)
            
            #exclude all models in referenced_models
            unreferenced_models = []
            for model_file in model_files:
                if not "referenced_models" in model_file:
                    unreferenced_models.append(model_file)
            
            model_names = get_object_names_from_files(unreferenced_models)
                        
            if len(args.models_select) > 0:
                model_names = list_intersection(model_names, args.models_select)
            elif len(args.models_ignore) > 0:
                model_names = list_difference(model_names, args.models_ignore)
            
            print(f"Processing models: {model_names}")
            
            #determine configs
            cfg_names = [args.local_config]
            if args.deploy_config != args.local_config:
                cfg_names.append(args.deploy_config)
            
            #filter out configs based on configs_select and configs_ignore
            if len(args.configs_select) > 0:
                cfg_names = list_intersection(cfg_names, args.configs_select)
            elif len(args.configs_ignore) > 0:
                cfg_names = list_difference(cfg_names, args.configs_ignore)
                
            print(f"Processing configurations: {cfg_names}")
            
            if args.task in [GENERATE_PACKAGES_TASK_NAME, DOWNLOAD_PACKAGES_TASK_NAME, PROCESS_CACHED_TASK_NAME]:
                #these options use the process parser
                if args.task == GENERATE_PACKAGES_TASK_NAME:
                    generate_packages(model_names, cfg_names)
                    
                    if not args.no_archive:
                        archive_packages(os.path.abspath(args.archives_dir))
                
                elif args.task == DOWNLOAD_PACKAGES_TASK_NAME:
                    #determine url to download from
                    if args.from_url != DEFAULT_DOWNLOAD_LATEST_URL and args.from_release != "latest":
                        #both --from-url and --from-release were specified. This is a no-no
                        print("--from-url and --from-release cannot both be specified.")
                        exit(1)
                    
                    download_url = args.from_url
                    if args.from_release != "latest":
                        download_url = os.path.join(DEFAULT_DOWNLOAD_VERSION_URL, args.from_release)
                    
                    download_packages(
                        download_url,
                        args.local_config, 
                        args.deploy_config,
                        model_names,
                        cfg_names,
                        args.assume_yes)
                
                if not args.no_process_local:
                    handle_local_packages(
                        args.local_config,
                        os.path.abspath(args.local_dir),
                        args.build)
                
                if not args.no_process_deploy:
                    handle_deploy_packages(
                        args.deploy_config,
                        os.path.abspath(args.deploy_dir),
                        args.deploy,
                        args.deploy_target)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
