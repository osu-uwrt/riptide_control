function outputDir = build_development(outputDir)
% Generate the complete controller using a temporary desktop configuration.
% Do not build, deploy, or run a node on a ROS device.
if nargin == 0
    outputDir = fullfile(tempdir, 'riptide_nmpc_development');
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end
originalDir = pwd;
originalPath = path;
originalFileGen = Simulink.fileGenControl('getConfig');
environmentCleanup = onCleanup(@() restoreEnvironment(originalDir, originalPath, originalFileGen)); %#ok<NASGU>
cd(outputDir);
outputDir = pwd;
modelDir = fileparts(mfilename('fullpath'));
addpath(modelDir, fileparts(modelDir), fullfile(fileparts(modelDir), 'referenced_models'));
Simulink.fileGenControl('set', 'CacheFolder', fullfile(outputDir, 'cache'), ...
    'CodeGenFolder', outputDir, 'createDir', true);

model = 'complete_controller';
load_system(model);
originalConfig = getActiveConfigSet(model);
originalConfig = originalConfig.Name;
originalDirty = get_param(model, 'Dirty');
config = copy(getConfigSet(model, 'x86_cfg'));
config.Name = matlab.lang.makeUniqueStrings('nmpc_development', getConfigSets(model));
config.set_param('GenCodeOnly', 'on');
target = config.get_param('CoderTargetData');
target.Runtime.BuildAction = 'None';
config.set_param('CoderTargetData', target);
attachConfigSet(model, config);
configCleanup = onCleanup(@() restoreConfiguration(model, originalConfig, config.Name, originalDirty)); %#ok<NASGU>
setActiveConfigSet(model, config.Name);
slbuild(model);
fprintf('Development code generated in %s\n', outputDir);
end

function restoreConfiguration(model, originalConfig, temporaryConfig, originalDirty)
setActiveConfigSet(model, originalConfig);
detachConfigSet(model, temporaryConfig);
set_param(model, 'Dirty', originalDirty);
end

function restoreEnvironment(originalDir, originalPath, originalFileGen)
cd(originalDir);
path(originalPath);
Simulink.fileGenControl('setConfig', 'config', originalFileGen);
end
