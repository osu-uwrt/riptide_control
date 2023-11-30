%
% This script attaches the x86_cfg and arm_cfg configs to an open simulink
% model whose name is passed in through the function's parameter. 
%

function attach_configs_to_model(model_name)
    fprintf("Attaching configs to %s\n", model_name)

    cfg_files_struct = dir("*_cfg.m");
    config_files_list = file_structs_to_list(cfg_files_struct);
        
    for i = 1 : length(config_files_list)
        cfg = config_files_list(i);
        cfg_name = get_file_name(cfg, 1);
        fprintf("Attaching config %s from file %s\n", cfg_name, cfg);
        config_builder = str2func(cfg_name);
        config = config_builder();
        attachConfigSet(model_name, config);
    end
end


%
% HELPER FUNCTIONS
%
function list = file_structs_to_list(structs)
    num_files = height(structs);
    list = strings(num_files, 1);
    for i = 1 : num_files
        list(i) = fullfile(structs(i).folder, structs(i).name);
    end
end

function name = get_file_name(file_path, dots)
    %convert to char array so that we can do the finding technique
    file_path_char = convertStringsToChars(file_path);

    lastSlash = find(file_path_char(:) == '/', 1, "last");
    lastPeriod = find(file_path_char(:) == '.', dots, "last");
    
    name = file_path_char(lastSlash + 1 : lastPeriod - 1);
end

