%
% OSU UWRT Simulink model deployment generator script
% Generates ROS packages for multiple architectures for local system use,
% deployment to robot, and release for distribution
%
% The syntax of the command is as follows:
% generate_c_packages(["--models-select", [MODELS], "--models-ignore",
% [MODELS], "--configs-select", [CONFIGS], "--configs-ignore" [CONFIGS])
%
% Alternative form:
% generate_c_packages([PACKAGES_TO_SELECT])
%

% THIS FUNCTION RUNS WHEN THE PROGRAM IS CALLED.
% ITS DONE LIKE THIS TO GET CMD LINE ARGS
function generate_cpp_packages(varargin)
    format compact

    %consts
    MODE_ALL    = 0;
    MODE_SELECT = 1;
    MODE_IGNORE = 2;

    % read arguments
    argument_names = ["--models-select", "--models-ignore", ...
        "--configs-select", "--configs-ignore"];

    % matrix row indices correspond with argument_names:
    % row 1: selected models
    % row 2: ignored models (selected as precedent)
    % row 3: selected configs
    % row 4: ignored configs (selected has precedent)
    argument_matrix = strings(4, 1);
    current_mode = 1; % --models-select

    for i = 1 : nargin
        arg = varargin{i};
        if arg == "-h" || arg == "--help"
            print_help();
            return
        end

        arg_matches = argument_names(:) == arg;
        if any(arg_matches)
            current_mode = find(arg_matches, 1, "first");
        else
            %need to add argument to matrix. find where
            current_mode_empties = argument_matrix(current_mode, :) == "";
            idx_to_add = find(current_mode_empties, 1, "first");
            argument_matrix(current_mode, idx_to_add) = arg;

            %ensure that there is at least one empty string at the end of
            %the matrix for find() to find
            if idx_to_add == width(argument_matrix)
                argument_matrix(:, idx_to_add + 1) = strings(4, 1);
            end
        end
    end

    model_mode = MODE_ALL;
    config_mode = MODE_ALL;

    % trim_string_array removes empty strings
    selected_models = make_absolute(trim_string_array(argument_matrix(1, :)));
    ignored_models = make_absolute(trim_string_array(argument_matrix(2, :)));
    selected_configs = make_absolute(trim_string_array(argument_matrix(3, :)));
    ignored_configs = make_absolute(trim_string_array(argument_matrix(4, :)));

    if ~isempty(selected_models)
        model_mode = MODE_SELECT;        
        if ~isempty(ignored_models)
            error("Cannot use --models-select and --models-ignore in the same command");
        end
    elseif ~isempty(ignored_models)
        model_mode = MODE_IGNORE;
    end

    if ~isempty(selected_configs)
        config_mode = MODE_SELECT;
        if ~isempty(ignored_configs)
            error("Cannot use --configs-select and --configs-ignore in the same command");
        end
    elseif ~isempty(ignored_configs)
        config_mode = MODE_IGNORE;
    end

    % contextual information
    current_file = mfilename('fullpath');
    current_file_dir = get_dirname(current_file);
    
    % ensure we are in the correct directory
    cd(current_file_dir);
    fprintf("Current directory is now %s\n\n", pwd);
    
    % discover available models
    model_files_struct = dir("**/*.slx");
    model_files_list = file_structs_to_list(model_files_struct);
    
    % discover available configs
    cfg_files_struct = dir("*_cfg.m");
    config_files_list = file_structs_to_list(cfg_files_struct);
        
    % filter discovered models based on arguments
    actionable_models = model_files_list; % will hold if mode is MODE_ALL
    if model_mode == MODE_SELECT
        actionable_models = select_from_array(model_files_list, selected_models);
    elseif model_mode == MODE_IGNORE
        actionable_models = ignore_from_array(model_files_list, ignored_models);
    end

    % filter discovered configs based on arguments
    actionable_configs = config_files_list; % will hold if mode is MODE_ALL
    if config_mode == MODE_SELECT
        actionable_configs = select_from_array(config_files_list, selected_configs);
    elseif config_mode == MODE_IGNORE
        actionable_configs = ignore_from_array(config_files_list, ignored_configs);
    end
    
    % print actionable models and configs
    fprintf("Discovered actionable models:\n");
    print_string_array(actionable_models);
    fprintf("\n");

    fprintf("Discovered actionable configs:\n");
    print_string_array(actionable_configs);
    fprintf("\n");

    % check that configs are in the current directory so that we can
    % execute them
    for i = 1 : length(actionable_configs)
        if ~(get_dirname(actionable_configs(i)) == current_file_dir)
            error("The config %s is not in the current directory! Please" + ...
                "place the configs in the same directory as the script.", ...
                actionable_configs(i));
        end
    end

    fprintf("Preparing output directory\n");
    if isfolder("generated_models")
        rmdir("generated_models", 's'); %force recursively remove generated_models (the s means recursive)
    end
    mkdir("generated_models");

    for i = 1 : length(actionable_models)
        generate_all_packages_for_model(actionable_models(i), actionable_configs);
    end
end


%
% FUNCTIONS
%
function result = get_dirname(file)
    slashes = file(:) == '/';
    lastSlash = find(slashes, 1, 'last');
    result = file(1 : lastSlash);
end


function list = file_structs_to_list(structs)
    num_files = height(structs);
    list = strings(num_files, 1);
    for i = 1 : num_files
        list(i) = fullfile(structs(i).folder, structs(i).name);
    end
end


function absolute_files = make_absolute(files)
    absolute_files = files;
    for i = 1 : length(absolute_files)
        if ~startsWith(absolute_files(i), "/")
            absolute_files(i) = fullfile(pwd, absolute_files(i));
        end
    end
end


function selected = select_from_array(arr, select)
    selected = arr;
    matches = select_items(arr, select);
    selected(~matches) = [];
end


function selected = ignore_from_array(arr, ignore)
    selected = arr;
    matches = select_items(arr, ignore);
    selected(matches) = [];
end


% outputs an array with 1s wherever there is an item in arr that matches an
% item in items. If anything in items is not in arr, this function will
% generate a warning.
function arr_matches = select_items(arr, items)
    arr_matches = zeros(size(arr));
    for i = 1 : length(items)
        elem_matches = arr(:) == items(i);
        if ~any(elem_matches)
            warning("Unrecognized item %s. Please make sure that it" + ...
                " is typed correctly if it is an argument, and if it is" + ...
                " a config file, please make sure it is in the same" + ...
                " directory as the script.", items(i));
        end

        arr_matches = arr_matches | elem_matches;
    end
end


function name = get_file_name(file_path, dots)
    %convert to char array so that we can do the finding technique
    file_path_char = convertStringsToChars(file_path);

    lastSlash = find(file_path_char(:) == '/', 1, "last");
    lastPeriod = find(file_path_char(:) == '.', dots, "last");
    
    name = file_path_char(lastSlash + 1 : lastPeriod - 1);
end


function trimmed = trim_string_array(str_arr)
    trimmed = str_arr;
    empties = str_arr(:) == "";
    trimmed(empties) = [];
end


function print_string_array(str_arr)
    for i = 1 : length(str_arr)
        fprintf("  %s\n", str_arr(i))
    end
end


function print_help()
    fprintf( ...
        "Usage: generate_cpp_packages [OPTIONS]\n" + ...
        "Automatically generate code for releases of UWRT Simulink models. \n" + ...
        "\n" + ...
        "  OPTIONS:\n" + ...
        "    --models-select [MODELS]    : Select specific models to build. Specify\n" + ...
        "                                  .slx files relative to the current path\n." + ... 
        "\n" + ...
        "    --models-ignore [MODELS]    : Ignore specific models and exclude them\n" + ...
        "                                  from the build. Specify .slx files\n" + ...
        "                                  relative to the current path.\n" + ...
        "\n" + ...
        "    --configs-select [CONFIGS]  : Select specific configs to use. Specify\n" + ...
        "                                  .cfg.m files relative to the current path\n" + ...
        "\n" + ...
        "    --configs-ignore [CONFIGS]  : Ignore specific configs and exclude them\n" + ...
        "                                  from the build. Specifiy .cfg.m files\n" + ...
        "                                  relative to the current path.\n" + ...
        "\n" + ...
        "    -h, --help                  : Show this help and exit.\n");

end


function generate_all_packages_for_model(model_location, config_locations)
    %consts
    TMP_CONFIG_NAME = "codegen_cs";

    model_name = get_file_name(model_location, 1);

    fprintf("Ensuring output directory\n");
    if ~isfolder("generated_models")
        mkdir("generated_models");
    end

    fprintf("Opening model %s (in %s)\n", model_name, model_location);
    open_system(model_location, 'loadonly');

    fprintf("Configuring %s for code generation\n", model_name);
    configs = getConfigSets(model_name);
    reservedConfigs = configs == TMP_CONFIG_NAME;
    if any(reservedConfigs)
        % one of the configs for this model has the same name as the tmp
        % config. This will cause issues when we try to load configs under
        % this name. So, lets try to remove it
        if get_param(getActiveConfigSet(model_name), "Name") == TMP_CONFIG_NAME
            fprintf("Reserved config name %s is in use as the model's active config. Removing\n", TMP_CONFIG_NAME);
            
            if size(configs) <= 1
                % this call will not return
                error("FAILED because the only available config is %s. " + ...
                    "Please set the model to use something else", TMP_CONFIG_NAME);
            end
            
            % find the name of a config thats not the reserved one
            good_config = find(~reservedConfigs, 1, "first");
            setActiveConfigSet(model_name, configs{good_config});
            fprintf("Setting default to non-reserved config %s\n", configs{good_config});
        end

        detachConfigSet(model_name, TMP_CONFIG_NAME);
    end

    fprintf("Model configured.\n");

    % now build the model on the different configs
    tarname = model_name + ".tgz";
    for i = 1 : length(config_locations)
        % clear src directory to ensure fresh code
        if isfolder("src")
            rmdir("src", 's');
        end

        if isfile(tarname)
            delete(tarname);
        end

        % build model
        config_name = get_file_name(config_locations(i), 1);
        fprintf("Building model %s with config %s\n", model_name, config_name);
        build_model_with_config(model_name, config_name, TMP_CONFIG_NAME);

        % copy generated code to output directory
        fprintf("Copying generated code to the output directory\n");
        new_outfile_name = fullfile(pwd, "generated_models", model_name + "_" + config_name + ".tgz");
        copyfile(tarname, new_outfile_name);
    end

    fprintf("Cleaning up\n");
    rmdir("src", 's');
    delete(tarname);
    delete("build_ros2_model.sh")

    fprintf("Closing model\n");
    bdclose(model_name);
end


function build_model_with_config(model_name, config_name, tmp_config_name)
    old_active_config = get_param(getActiveConfigSet(model_name), 'Name');

    config_builder = str2func(config_name);
    config = config_builder();
    set_param(config, 'Name', tmp_config_name);
    attachConfigSet(model_name, config);
    setActiveConfigSet(model_name, tmp_config_name);

    slbuild(model_name);

    setActiveConfigSet(model_name, old_active_config);
    detachConfigSet(model_name, tmp_config_name);
end
