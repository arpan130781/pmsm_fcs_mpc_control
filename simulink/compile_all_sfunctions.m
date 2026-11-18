% =========================================================================
% compile_all_sfunctions.m
%
% Compiles Level-2 C MEX S-Functions for the Finite-Control-Set Model
% Predictive Control (FCS-MPC) PMSM Drive in MATLAB/Simulink:
%   1. sfun_mpc_controller: 2-step ahead FCS-MPC with Multi-Objective Loss/CMV
%   2. sfun_pmsm_plant: Continuous-state 3-Phase PMSM & 2L-VSI Inverter
%
% Author: Arpan Kailash Gupta (Roll No: 241030009)
% Department of Electrical Engineering, VJTI Mumbai
% =========================================================================

function compile_all_sfunctions()
    here = fileparts(mfilename('fullpath'));
    src_dir = fullfile(here, '..', 'src');
    
    fprintf('==================================================================\n');
    fprintf('Compiling Simulink C-MEX S-Functions for FCS-MPC PMSM Drive...\n');
    fprintf('Directory: %s\n', here);
    fprintf('==================================================================\n');
    
    mpc_sources = { ...
        fullfile(src_dir, 'inverter_2lvsi.c'), ...
        fullfile(src_dir, 'mpc_core.c'), ...
        fullfile(src_dir, 'speed_controller.c'), ...
        fullfile(src_dir, 'mpc_api.c') ...
    };

    % Check platform
    if ismac
        matlab_root = matlabroot;
        % Clang direct build for macOS (handles architecture and SDK automatically)
        arch = computer('arch');
        if strcmp(arch, 'maci64')
            target_arch = 'x86_64';
            lib_sub = 'maci64';
            ext = 'mexmaci64';
        else
            target_arch = 'arm64';
            lib_sub = 'maca64';
            ext = 'mexmaca64';
        end
        
        fprintf('Targeting macOS (%s) with clang...\n', target_arch);
        
        % 1. Compile sfun_mpc_controller
        mpc_src_str = sprintf('"%s" ', mpc_sources{:});
        clang_cmd_mpc = sprintf(['clang -arch %s -bundle -fPIC -O3 ' ...
            '-I"%s/extern/include" -I"%s/simulink/include" -I"%s" ' ...
            '-DMATLAB_MEX_FILE -L"%s/bin/%s" -lmx -lmex -lmat -lm ' ...
            '-Wl,-exported_symbols_list,"%s/extern/lib/%s/mexFunction.map" ' ...
            '"%s/sfun_mpc_controller.c" %s -o "%s/sfun_mpc_controller.%s"'], ...
            target_arch, matlab_root, matlab_root, src_dir, ...
            matlab_root, lib_sub, matlab_root, lib_sub, ...
            here, mpc_src_str, here, ext);
        
        status1 = system(clang_cmd_mpc);
        if status1 == 0
            fprintf('  [SUCCESS] sfun_mpc_controller.%s built successfully.\n', ext);
        else
            fprintf('  [FALLBACK] Attempting mex command for sfun_mpc_controller...\n');
            mex(['-I' src_dir], fullfile(here, 'sfun_mpc_controller.c'), mpc_sources{:}, '-outdir', here);
        end
        
        % 2. Compile sfun_pmsm_plant
        clang_cmd_plant = sprintf(['clang -arch %s -bundle -fPIC -O3 ' ...
            '-I"%s/extern/include" -I"%s/simulink/include" ' ...
            '-DMATLAB_MEX_FILE -L"%s/bin/%s" -lmx -lmex -lmat -lm ' ...
            '-Wl,-exported_symbols_list,"%s/extern/lib/%s/mexFunction.map" ' ...
            '"%s/sfun_pmsm_plant.c" -o "%s/sfun_pmsm_plant.%s"'], ...
            target_arch, matlab_root, matlab_root, ...
            matlab_root, lib_sub, matlab_root, lib_sub, ...
            here, here, ext);
        
        status2 = system(clang_cmd_plant);
        if status2 == 0
            fprintf('  [SUCCESS] sfun_pmsm_plant.%s built successfully.\n', ext);
        else
            fprintf('  [FALLBACK] Attempting mex command for sfun_pmsm_plant...\n');
            mex(fullfile(here, 'sfun_pmsm_plant.c'), '-outdir', here);
        end
    else
        % Linux / Windows
        mex(['-I' src_dir], fullfile(here, 'sfun_mpc_controller.c'), mpc_sources{:}, '-outdir', here);
        mex(fullfile(here, 'sfun_pmsm_plant.c'), '-outdir', here);
    end
    
    fprintf('==================================================================\n');
    fprintf('All MEX S-Function binaries are compiled and ready.\n');
    fprintf('==================================================================\n');
end
