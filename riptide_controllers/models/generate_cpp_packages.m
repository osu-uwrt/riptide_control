%
% OSU UWRT Simulink model deployment generator script
% Generates ROS packages for multiple architectures for local system use,
% deployment to robot, and release for distribution
%
% Args:
%   models: list of model names (ex. "PID", "SMC"). No extension.
%   configs: list of config names (ex. "arm_cfg"). No extension.
%
function generate_cpp_packages(models, configs)
    format compact

    % contextual information
    current_file = mfilename('fullpath');
    current_file_dir = get_dirname(current_file);
    
    % ensure we are in the correct directory
    fprintf("Changing current directory to %s\n", current_file_dir)
    cd(current_file_dir);
    
    %open project to add all needed resources to the path
    fprintf("Opening Project\n");
    proj = openProject("Riptide_control_models.prj");
    
    %ensure generated_models directory exists
    fprintf("Preparing output directory\n");
    if isfolder("generated_models")
        rmdir("generated_models", 's'); %force recursively remove generated_models (the s means recursive)
    end
    mkdir("generated_models");

    %generate ROS packages from simulink models
    open_all_models(models);
    
    for i = 1 : length(configs)
        generate_all_packages_for_config(configs(i), models);
    end
    close_all_models(models);
    
    fprintf("Closing Project\n");
    close(proj);
end


%
% FUNCTIONS
%
function result = get_dirname(file)
    slashes = file(:) == '/';
    lastSlash = find(slashes, 1, 'last');
    result = file(1 : lastSlash);
end


function open_all_models(names)
    for i = 1 : length(names)
        name = names(i);
        fprintf("Opening %s\n", name);
        if exist(name, 'file') == 4 %4 means its a simulink model
            open_system(name, 'loadonly')
        else
            fprintf("FAILED to open file with name %s because it is " + ...
                "not a Simulink model.\n", name);
        end
    end
end


function close_all_models(names)
    for i = 1 : length(names)
        name = names(i);
        fprintf("Closing %s\n", name);
        close_system(name, 0); %if system not open, this simply returns
    end
end


function configs = get_active_configs(models)
    configs = strings(length(models), 1);
    for i = 1 : length(models)
        configs(i) = getActiveConfigSet(models(i)).name;
    end
end


function unload_temp_config_from_models(models, old_configs, temp_name)
    for i = 1 : length(models)
        model = models(i);
        model_configs = getConfigSets(model);
        configs_reserved = model_configs == temp_name;
        if any(configs_reserved)
            %one of the configs for this model has the same name as the
            %temp config. Unload it. First, make sure its not the active
            %config
            fprintf("Removing temporary config from %s\n", model);

            %set active config set back to the old one
            setActiveConfigSet(model, old_configs(i));

            if old_configs(i) == temp_name
                %config in use as active config, so set model to use
                %something else
                if size(model_configs) == 1
                    error("  FAILED because the temporary config is " + ...
                          "the only one available. Please add " + ...
                          "another config to the model.\n");
                end

                good_config_index = find(~configs_reserved, 1); %find first nonzero index of ~configs_reserved
                good_config = model_configs{good_config_index};
                setActiveConfigSet(model, good_config);
                fprintf("  Set new default config to %s\n", good_config);
            end

            detachConfigSet(model, temp_name);
        end
    end
end


function load_config_onto_models(models, config_name, temp_name)
    config_builder = str2func(config_name);
    config = config_builder();
    config.name = temp_name;
    
    for i = 1 : length(models)
        model = models(i);
        fprintf("Attaching %s to %s\n", config_name, model);
        config = config_builder();
        config.name = temp_name;
        attachConfigSet(model, config);
        setActiveConfigSet(model, temp_name);
    end
end


function generate_all_packages_for_config(config, models)
    TEMP_CONFIG_NAME = "CODEGEN_CS";

    %get old configs
    old_configs = get_active_configs(models);
    
    %unload temp config from models in case it was already loaded
    unload_temp_config_from_models(models, old_configs, TEMP_CONFIG_NAME);
    old_configs = get_active_configs(models);

    %load config and build model
    load_config_onto_models(models, config, TEMP_CONFIG_NAME);

    for i = 1 : length(models)
        model = models(i);
        slbuild(model);
        
        % copy generated code to output directory
        fprintf("Copying generated code to the output directory\n");
        tarname = model + ".tgz"; % expected name of matlabs output archive
        new_outfile_name = fullfile(pwd, "generated_models", model + "_" + config + ".tgz"); % new, more descriptive name
        copyfile(tarname, new_outfile_name);
    end

    %unload config
    unload_temp_config_from_models(models, old_configs, TEMP_CONFIG_NAME);
end
