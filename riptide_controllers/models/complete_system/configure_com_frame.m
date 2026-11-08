function configure_com_frame()
% Wire the active complete_controller prediction model to COM coordinates.
% Run in MATLAB with this directory on the path. Existing base_link pose and
% command interfaces remain unchanged; only the internal MPC signals use COM.
modelDir = fileparts(mfilename('fullpath'));
modelsDir = fileparts(modelDir);
originalDir = pwd;
directoryCleanup = onCleanup(@() cd(originalDir)); %#ok<NASGU>
% The model's custom source is relative to models/, not the caller's cwd.
addpath(modelDir, modelsDir, fullfile(modelsDir, 'referenced_models'));
cd(modelsDir);
model = 'complete_controller';
load_system(fullfile(modelDir, [model '.slx']));
assert(strcmp(get_param(model, 'SimulationStatus'), 'stopped'), ...
    'Stop the simulation before changing the controller frame.');

backup = [tempname '.slx'];
copyfile(get_param(model, 'FileName'), backup);
fprintf('Saved-model backup: %s\n', backup);

control = [model '/Control'];
builder = [control '/MATLAB Function'];
root = sfroot;
chart = root.find('-isa', 'Stateflow.EMChart', 'Path', builder);
assert(isscalar(chart), 'Cannot find the active Fossen parameter builder.');

% Preserve the eleven parameter outputs and append COM state/reference signals.
% The existing base_link input is number eleven.
chart.Script = sprintf([ ...
    'function [M, MA, DQ, DL, B, mass, buoyancy, com, cob, I, flipCost, xCOM, refCOM] = ' ...
    'buildFossenParameters(quadratic_damping, linear_damping_matrix, added_mass_matrix, ' ...
    'wrench_mat, m, bouy, center_of_mass, center_of_bouyancy, inertia_matrix, ' ...
    'flip_cost, base_link, xBase, refBase)\n' ...
    '[M, MA, DQ, DL, B, mass, buoyancy, com, cob, I, flipCost, xCOM, refCOM] = ...\n' ...
    '    prepareCOMModel(quadratic_damping, linear_damping_matrix, added_mass_matrix, ...\n' ...
    '        wrench_mat, m, bouy, center_of_mass, center_of_bouyancy, ...\n' ...
    '        inertia_matrix, flip_cost, base_link, xBase, refBase);\n' ...
    'end\n']);
% The MPC mask checks these dimensions before propagating sizes through the
% external helper. Specify the fixed output contracts instead of inheriting.
sizes = {'M', '[6 6]'; 'MA', '[6 6]'; 'DQ', '[6 6]'; 'DL', '[6 6]'; ...
    'B', '[6 8]'; 'mass', '1'; 'buoyancy', '1'; 'com', '[3 1]'; ...
    'cob', '[3 1]'; 'I', '[3 3]'; 'flipCost', '1'; ...
    'xCOM', '[13 1]'; 'refCOM', '[1 13]'};
for k = 1:size(sizes, 1)
    data = chart.find('-isa', 'Stateflow.Data', 'Name', sizes{k, 1});
    assert(isscalar(data), 'Missing parameter-builder output %s.', sizes{k, 1});
    data.Props.Array.Size = sizes{k, 2};
end
drawnow;

% The saved model added base_link to the selector and function signature but
% did not connect selector output eleven to function input eleven. An open
% input otherwise propagates a scalar, which cannot be reshaped to a position.
selector = get_param([control '/' sprintf('Bus\nSelector2')], 'PortHandles');
ports = get_param(builder, 'PortHandles');
baseLine = get_param(ports.Inport(11), 'Line');
if baseLine == -1
    add_line(control, selector.Outport(11), ports.Inport(11), 'autorouting', 'on');
else
    assert(get_param(baseLine, 'SrcPortHandle') == selector.Outport(11), ...
        'Unexpected source on the base_link parameter input.');
end

% Keep the existing Record blocks connected to the actual MPC input signals.
moveSignal(control, 'State Vector Compilation', builder, 12, 'COM State');
moveSignal(control, 'Pose Reference', builder, 13, 'COM Reference');

% This is a ROS MATLAB System block; it is not found by a SubSystem/Reference
% BlockType filter once loaded. Its copied length-six default controls width.
baseParameter = [model '/Subsystem3/Subsystem/mass1'];
assert(strcmp(get_param(baseParameter, 'ParameterName'), 'base_link'), ...
    'The expected base_link parameter block has changed.');
set_param(baseParameter, 'ParameterMaxArrayLength', '3', ...
    'ParameterInitialValue', '[0,0,0]');

set_param(model, 'SimulationCommand', 'update');
save_system(model);
fprintf('COM model saved. MPC state/reference and physical wrenches use COM.\n');
end

function moveSignal(system, sourceName, builder, number, label)
source = get_param([system '/' sourceName], 'PortHandles');
ports = get_param(builder, 'PortHandles');
assert(numel(ports.Inport) >= number && numel(ports.Outport) >= number, ...
    'The parameter-builder ports have not updated.');

% A repeated invocation must not insert a second conversion or duplicate wire.
inputLine = get_param(ports.Inport(number), 'Line');
if inputLine ~= -1
    assert(get_param(inputLine, 'SrcPortHandle') == source.Outport(1), ...
        'Unexpected wiring on the COM conversion input.');
    return;
end
oldLine = get_param(source.Outport(1), 'Line');
assert(oldLine ~= -1, 'Missing original state/reference signal.');
destinations = get_param(oldLine, 'DstPortHandle');
assert(all(destinations ~= -1), 'Unconnected branch on state/reference signal.');
delete_line(oldLine);
for k = 1:numel(destinations)
    newLine = add_line(system, ports.Outport(number), destinations(k), ...
        'autorouting', 'on');
    set_param(newLine, 'Name', label);
end
add_line(system, source.Outport(1), ports.Inport(number), 'autorouting', 'on');
end
