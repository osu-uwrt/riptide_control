# NMPC development builds

From this directory in MATLAB R2023a:

```matlab
test_nmpc_codegen
outputDir = build_development;
```

`build_development` temporarily clones the existing `x86_cfg`, enables code
generation only, and sets the hardware build action to `None`. It adds the
model dependencies to the path and writes generated files under
`fullfile(tempdir, 'riptide_nmpc_development')`. It restores the previous model
configuration, directory, MATLAB path, and file-generation settings afterward.
An optional output-directory argument selects another destination. No ROS
device connection is needed.

The original failure is the expansion of `parameters{:}` in
`znlmpc_confun.m`: the parameter cell array loses its fixed size during code
generation. Subsequent analysis also encounters conditionally initialized
scaling variables, absent passivity callbacks, and variable trajectory sizes.

`initialize` calls `configure_nmpc_codegen`, which creates temporary copies of
five installed MathWorks helpers and puts them on the session's MATLAB path.
The small substitutions preserve the compile-time parameter count, initialize
scaling temporaries on all paths, guard absent passivity callbacks, and derive
trajectory dimensions from existing fixed-size arrays. The installed toolbox
files are never edited. The workaround is restricted to R2023a and checks each
expected source fragment before applying it; review it when updating MATLAB.

The model changes restore the existing parameter-processing blocks, enable and
connect the NMPC parameter input, and preserve the reference constant as a
1-by-13 matrix. The NMPC library link, state equations, horizons, weights, and
physical parameter types are retained.

`test_nmpc_codegen` compares runtime data, objective values and gradients,
constraints and Jacobians, and trajectory outputs against the original
MathWorks helpers, with state scaling both enabled and disabled.
