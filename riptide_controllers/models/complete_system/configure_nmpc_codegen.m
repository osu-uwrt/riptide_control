function configure_nmpc_codegen()
% R2023a compatibility fixes for the NMPC library's code-generation helpers.
% Generate patched copies in tempdir; never modify the MATLAB installation.
% The NMPC library block and the prediction equations remain unchanged.
if ~strcmp(version('-release'), '2023a')
    return;
end

sourceDir = fullfile(matlabroot, 'toolbox', 'mpc', 'mpcutils');
patchDir = fullfile(tempdir, 'riptide_nmpc_codegen', version('-release'));
if ~isfolder(patchDir)
    mkdir(patchDir);
end

% Preserve fixed parameter counts, initialize scaling temporaries on every
% path, and infer trajectory sizes from fixed array dimensions. The passivity
% guard prevents analyzing absent callbacks when passivity is disabled.
patches = {
    'znlmpc_generateRuntimeData.m', {
        'Parameters = Parameters0;', ...
        'Parameters = reshape(Parameters0, coder.const(coredata.npara), 1);'
    };
    'znlmpc_confun.m', {
        '% only allocate/compute when scaling is requested', ...
        sprintf('Sx = eye(nx);\nTx = eye(nx);\n%% only allocate/compute when scaling is requested');
        'if coredata.enforcePassivity', ...
        ['if coredata.enforcePassivity && ' ...
         'isa(handles.hPassivityOutputFcn,''function_handle'') && ' ...
         'isa(handles.hPassivityInputFcn,''function_handle'')']
    };
    'znlmpc_objfun.m', {
        sprintf('pnmv = p*nmv;\nif coredata.hasxscale'), ...
        sprintf('pnmv = p*nmv;\nxsInv = ones(1,nx);\nif coredata.hasxscale')
    };
    'znlmpc_getXUe.m', {
        'p1 = coredata.p + 1;', 'p1 = size(coredata.MVMin,1) + 1;';
        'nx = coredata.nx;', 'nx = numel(coredata.Xscale);';
        'nu = coredata.nu;', 'nu = numel(coredata.Uscale);';
        'nmv = coredata.nmv;', 'nmv = numel(coredata.imv);';
        'nmd = coredata.nmd;', 'nmd = numel(coredata.imd);'
    };
    'znlmpc_computeInfo.m', {
        'p = coredata.p;', 'p = size(X,1)-1;';
        'ny = coredata.ny;', 'ny = numel(coredata.Yscale);'
    }
};

for k = 1:size(patches, 1)
    name = patches{k, 1};
    text = fileread(fullfile(sourceDir, name));
    changes = patches{k, 2};
    for j = 1:size(changes, 1)
        assert(numel(strfind(text, changes{j, 1})) == 1, ...
            'riptide:NMPCCodegen:UnsupportedSource', ...
            'Unexpected R2023a source in %s. Review the NMPC compatibility fixes.', name);
        text = strrep(text, changes{j, 1}, changes{j, 2});
    end
    target = fullfile(patchDir, name);
    if ~isfile(target) || ~strcmp(fileread(target), text)
        fid = fopen(target, 'w');
        assert(fid ~= -1, 'riptide:NMPCCodegen:WriteFailed', ...
            'Cannot write NMPC compatibility helper: %s', target);
        cleanup = onCleanup(@() fclose(fid));
        fprintf(fid, '%s', text);
        clear cleanup;
    end
end
addpath(patchDir, '-begin');
end
