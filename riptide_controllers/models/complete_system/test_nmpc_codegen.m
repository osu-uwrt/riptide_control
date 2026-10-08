function test_nmpc_codegen()
% Compare the compatibility helpers with the installed MathWorks originals.
originalPath = path;
cleanup = onCleanup(@() path(originalPath)); %#ok<NASGU>
configure_nmpc_codegen();
baselineDir = tempname;
mkdir(baselineDir);
names = {'znlmpc_generateRuntimeData', 'znlmpc_confun', ...
    'znlmpc_objfun', 'znlmpc_getXUe', 'znlmpc_computeInfo'};
for k = 1:numel(names)
    name = names{k};
    source = fileread(fullfile(matlabroot, 'toolbox', 'mpc', 'mpcutils', [name '.m']));
    source = strrep(source, [name '('], ['baseline_' name '(']);
    fid = fopen(fullfile(baselineDir, ['baseline_' name '.m']), 'w');
    assert(fid ~= -1);
    fileCleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '%s', source);
    clear fileCleanup;
end
addpath(baselineDir);

x = [0; 0; 0; 1; 0; 0; 0; .1; -.05; .02; .01; -.02; .03];
u = .1 * ones(8, 1);
parameters = {1.2 * eye(6); .2 * eye(6); .1 * eye(6); .2 * eye(6); ...
    [eye(6), zeros(6, 2)]; 1; 9.80665; zeros(3, 1); zeros(3, 1); eye(3); 0};
fields = {'hStateFcn', 'hOutputFcn', 'hCostFcn', 'hEqConFcn', 'hIneqConFcn', ...
    'hJacobianStateFcn', 'hJacobianOutputFcn', 'hJacobianCostFcn', ...
    'hJacobianEqConFcn', 'hJacobianIneqConFcn', 'hPassivityInputFcn', ...
    'hPassivityOutputFcn', 'hPassivityInputJacobianFcn', 'hPassivityOutputJacobianFcn'};
handles = cell2struct(repmat({[]}, size(fields)), fields, 2);
handles.hStateFcn = @FossenStateFcn;

for scaled = [false, true]
    cfg = struct('PH', 3, 'CH', 2, 'Ts', .05);
    object = initialize(cfg);
    if scaled
        for j = 1:13
            object.States(j).ScaleFactor = 1 + j / 10;
        end
    end
    validateFcns(object, x, u, [], parameters);
    core = getCodeGenerationData(object, x, u, parameters);
    ref = x';
    ref(1) = .2;
    args = {core, x, u, ref, [], [], [], [], [], [], [], [], [], [], ...
        [], [], [], [], parameters, [], [], 0};
    [runtime, userdata, z] = znlmpc_generateRuntimeData(args{:});
    [expectedRuntime, expectedUserdata, expectedZ] = baseline_znlmpc_generateRuntimeData(args{:});
    assert(isequaln(runtime, expectedRuntime));
    assert(isequaln(userdata, expectedUserdata));
    assert(isequaln(z, expectedZ));

    [X, U, e] = znlmpc_getXUe(core, z, x, []);
    [expectedX, expectedU, expectedE] = baseline_znlmpc_getXUe(core, z, x, []);
    assert(isequaln(X, expectedX) && isequaln(U, expectedU) && isequaln(e, expectedE));
    [cost, gradient] = znlmpc_objfun(z, core, runtime, userdata, handles);
    [expectedCost, expectedGradient] = baseline_znlmpc_objfun(z, core, runtime, userdata, handles);
    assert(isequaln(cost, expectedCost) && isequaln(gradient, expectedGradient));
    [c, ceq, Jc, Jceq] = znlmpc_confun(z, core, runtime, userdata, handles);
    [expectedC, expectedCeq, expectedJc, expectedJceq] = baseline_znlmpc_confun(z, core, runtime, userdata, handles);
    assert(isequaln(c, expectedC) && isequaln(ceq, expectedCeq));
    assert(isequaln(Jc, expectedJc) && isequaln(Jceq, expectedJceq));
    args = {core, X, U, e, cost, 1, 1, [], parameters};
    [X0, MV0, slack, info] = znlmpc_computeInfo(args{:});
    [expectedX0, expectedMV0, expectedSlack, expectedInfo] = baseline_znlmpc_computeInfo(args{:});
    assert(isequaln(X0, expectedX0) && isequaln(MV0, expectedMV0));
    assert(isequaln(slack, expectedSlack) && isequaln(info, expectedInfo));
end
fprintf('NMPC compatibility helpers match the MathWorks originals with and without state scaling.\n');
end
