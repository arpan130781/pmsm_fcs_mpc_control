% =========================================================================
% create_pmsm_mpc_model.m
%
% Programmatically creates and wires the complete Simulink model:
%   pmsm_mpc_simulink.slx
%
% Complete Closed-Loop Finite-Control-Set Model Predictive Control Architecture:
%   - Speed Reference & DC Bus
%   - Controller S-Function (Level-2 C-MEX sfun_mpc_controller)
%   - 3-Phase PMSM Motor & 2L-VSI Inverter S-Function (sfun_pmsm_plant)
%   - Multi-Objective Power Electronics Inverter Telemetry
%   - ToWorkspace Logging Blocks & Interactive Scopes
%
% Author: Arpan Kailash Gupta (Roll No: 241030009)
% Department of Electrical Engineering, VJTI Mumbai
% =========================================================================

function create_pmsm_mpc_model()
    here = fileparts(mfilename('fullpath'));
    addpath(here);
    
    % Initialize variables in Base Workspace
    init_pmsm_mpc;
    
    mdl = 'pmsm_mpc_simulink';
    slx_path = fullfile(here, [mdl, '.slx']);
    
    fprintf('Creating Simulink FCS-MPC Model: %s...\n', mdl);
    
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    if exist(slx_path, 'file')
        delete(slx_path);
    end
    
    new_system(mdl);
    load_system('simulink');
    
    % Solver Configuration: Fixed-step ode4 (Runge-Kutta 4th order) at 10 us
    set_param(mdl, ...
        'SolverType', 'Fixed-step', ...
        'Solver', 'ode4', ...
        'FixedStep', num2str(sim_cfg.Ts_plant), ...
        'StopTime', num2str(sim_cfg.T_final), ...
        'SaveTime', 'on', ...
        'TimeSaveName', 'tout', ...
        'SaveOutput', 'off');

    %% --- 1. Sources & Operating Demands ---
    % Speed Reference (Constant)
    add_block('simulink/Sources/Constant', [mdl, '/Speed_Ref'], ...
        'Value', 'profile.speed_ref_rpm', ...
        'Position', [40, 60, 110, 80]);
    
    % DC Bus Supply Voltage (Constant)
    add_block('simulink/Sources/Constant', [mdl, '/V_bus'], ...
        'Value', 'motor.Vbus', ...
        'Position', [40, 110, 110, 130]);
    
    % External Load Torque Step
    add_block('simulink/Sources/Step', [mdl, '/Load_Torque'], ...
        'Time', 'profile.load_step_time', ...
        'Before', '0.0', ...
        'After', 'profile.load_torque_Nm', ...
        'SampleTime', '0', ...
        'Position', [40, 360, 110, 380]);

    % Sensor Fault Injection Sources
    add_block('simulink/Sources/Step', [mdl, '/Fault_Step_A'], ...
        'Time', 'fault.onset_time', ...
        'Before', '0.0', ...
        'After', 'fault.offset_val * fault.enable', ...
        'SampleTime', '0', ...
        'Position', [40, 220, 110, 240]);
    
    add_block('simulink/Sources/Constant', [mdl, '/Zero_BC'], ...
        'Value', '[0.0; 0.0]', ...
        'Position', [40, 260, 110, 280]);

    %% --- 2. Fault Injection & Signal Routing ---
    add_block('simulink/Signal Routing/Mux', [mdl, '/Mux_Fault'], ...
        'Inputs', '2', ...
        'Position', [140, 225, 145, 275]);

    add_block('simulink/Math Operations/Sum', [mdl, '/Sum_Fault_Inject'], ...
        'Inputs', '++', ...
        'Position', [180, 175, 200, 205]);

    % Mux for FCS-MPC Controller Input (Width 7):
    % [ia, ib, ic, theta_e, speed_rpm, speed_ref_rpm, v_dc]
    add_block('simulink/Signal Routing/Mux', [mdl, '/Mux_MPC_In'], ...
        'Inputs', '5', ...
        'Position', [250, 65, 255, 225]);

    %% --- 3. FCS-MPC Controller S-Function Block ---
    add_block('simulink/User-Defined Functions/S-Function', [mdl, '/FCS_MPC_Controller'], ...
        'FunctionName', 'sfun_mpc_controller', ...
        'Parameters', 'sim_cfg.Ts_ctrl', ...
        'Position', [290, 115, 460, 175]);

    % Demux for FCS-MPC Controller Output (Width 14):
    % [3:Sa,Sb,Sc, 1:opt_vec, 2:id_iq, 2:id_iq_ref, 1:Te, 1:Vcm, 1:Ploss, 1:Eff, 1:Fsw, 1:Jcost]
    add_block('simulink/Signal Routing/Demux', [mdl, '/Demux_MPC_Out'], ...
        'Outputs', '[3, 1, 2, 2, 1, 1, 1, 1, 1, 1]', ...
        'Position', [490, 45, 495, 245]);

    %% --- 4. PMSM Motor Plant Block ---
    % Mux for Plant Input (Width 5): [Sa, Sb, Sc, T_load, Vbus]
    add_block('simulink/Signal Routing/Mux', [mdl, '/Mux_Plant_In'], ...
        'Inputs', '3', ...
        'Position', [550, 80, 555, 190]);

    add_block('simulink/User-Defined Functions/S-Function', [mdl, '/PMSM_Motor_Plant'], ...
        'FunctionName', 'sfun_pmsm_plant', ...
        'Parameters', 'MotorParamVec, motor.Tcog', ...
        'Position', [590, 105, 750, 165]);

    % Demux for Plant Output (Width 9):
    % [3:ia_true, 1:theta_e, 1:spd_rpm, 1:omega_e, 1:Te, 2:dq_true]
    add_block('simulink/Signal Routing/Demux', [mdl, '/Demux_Plant_Out'], ...
        'Outputs', '[3, 1, 1, 1, 1, 2]', ...
        'Position', [780, 75, 785, 195]);

    %% --- 5. Data Logging Muxes ---
    % Speed Log Mux: [Ref, True]
    add_block('simulink/Signal Routing/Mux', [mdl, '/Mux_Speed_Log'], ...
        'Inputs', '2', ...
        'Position', [880, 45, 885, 95]);

    % Torque Log Mux: [Te_actual, T_load]
    add_block('simulink/Signal Routing/Mux', [mdl, '/Mux_Torque_Log'], ...
        'Inputs', '2', ...
        'Position', [880, 120, 885, 160]);

    % d-q Current Log Mux: [id_true, iq_true, id_ref, iq_ref]
    add_block('simulink/Signal Routing/Mux', [mdl, '/Mux_DQ_Log'], ...
        'Inputs', '2', ...
        'Position', [880, 180, 885, 230]);

    % Switching & Vector Log Mux: [Sa, Sb, Sc, opt_vector]
    add_block('simulink/Signal Routing/Mux', [mdl, '/Mux_Switch_Log'], ...
        'Inputs', '2', ...
        'Position', [600, 270, 605, 320]);

    % Power Electronics Telemetry Mux: [Vcm, Ploss, Eff, Fsw, Jcost]
    add_block('simulink/Signal Routing/Mux', [mdl, '/Mux_PE_Log'], ...
        'Inputs', '5', ...
        'Position', [600, 340, 605, 410]);

    %% --- 6. To Workspace Logging Blocks ---
    add_block('simulink/Sinks/To Workspace', [mdl, '/To_WS_Speed'], ...
        'VariableName', 'sim_speed_rpm', ...
        'SaveFormat', 'Array', ...
        'Position', [920, 55, 1010, 85]);

    add_block('simulink/Sinks/To Workspace', [mdl, '/To_WS_Torque'], ...
        'VariableName', 'sim_torque', ...
        'SaveFormat', 'Array', ...
        'Position', [920, 125, 1010, 155]);

    add_block('simulink/Sinks/To Workspace', [mdl, '/To_WS_Currents_ABC'], ...
        'VariableName', 'sim_currents_abc', ...
        'SaveFormat', 'Array', ...
        'Position', [380, 330, 470, 360]);

    add_block('simulink/Sinks/To Workspace', [mdl, '/To_WS_Currents_DQ'], ...
        'VariableName', 'sim_currents_dq', ...
        'SaveFormat', 'Array', ...
        'Position', [920, 190, 1010, 220]);

    add_block('simulink/Sinks/To Workspace', [mdl, '/To_WS_Switching'], ...
        'VariableName', 'sim_switching', ...
        'SaveFormat', 'Array', ...
        'Position', [640, 280, 730, 310]);

    add_block('simulink/Sinks/To Workspace', [mdl, '/To_WS_PE_Metrics'], ...
        'VariableName', 'sim_pe_metrics', ...
        'SaveFormat', 'Array', ...
        'Position', [640, 360, 730, 390]);

    add_block('simulink/Sinks/To Workspace', [mdl, '/To_WS_Vbus'], ...
        'VariableName', 'sim_vbus', ...
        'SaveFormat', 'Array', ...
        'Position', [220, 280, 270, 305]);

    %% --- 7. Scopes for Real-Time Visualization ---
    add_block('simulink/Sinks/Scope', [mdl, '/Scope_Speed_and_Torque'], ...
        'NumInputPorts', '2', ...
        'Position', [1040, 70, 1080, 130]);

    add_block('simulink/Sinks/Scope', [mdl, '/Scope_Currents'], ...
        'NumInputPorts', '2', ...
        'Position', [1040, 180, 1080, 240]);

    add_block('simulink/Sinks/Scope', [mdl, '/Scope_Power_Electronics'], ...
        'NumInputPorts', '2', ...
        'Position', [760, 310, 800, 370]);

    %% --- 8. Wiring & Interconnections ---
    % Fault Mux
    add_line(mdl, 'Fault_Step_A/1', 'Mux_Fault/1', 'autorouting', 'on');
    add_line(mdl, 'Zero_BC/1', 'Mux_Fault/2', 'autorouting', 'on');

    % Fault Sum: Plant True Currents + Fault Offset
    add_line(mdl, 'Demux_Plant_Out/1', 'Sum_Fault_Inject/1', 'autorouting', 'on');
    add_line(mdl, 'Mux_Fault/1', 'Sum_Fault_Inject/2', 'autorouting', 'on');

    % Connect to Controller Input Mux: [I_meas, theta_e, speed_rpm, Speed_Ref, V_bus]
    add_line(mdl, 'Sum_Fault_Inject/1', 'Mux_MPC_In/1', 'autorouting', 'on');
    add_line(mdl, 'Demux_Plant_Out/2', 'Mux_MPC_In/2', 'autorouting', 'on');
    add_line(mdl, 'Demux_Plant_Out/3', 'Mux_MPC_In/3', 'autorouting', 'on');
    add_line(mdl, 'Speed_Ref/1', 'Mux_MPC_In/4', 'autorouting', 'on');
    add_line(mdl, 'V_bus/1', 'Mux_MPC_In/5', 'autorouting', 'on');

    % Connect Controller S-Function
    add_line(mdl, 'Mux_MPC_In/1', 'FCS_MPC_Controller/1', 'autorouting', 'on');
    add_line(mdl, 'FCS_MPC_Controller/1', 'Demux_MPC_Out/1', 'autorouting', 'on');

    % Plant Input Mux: [Sa_Sb_Sc, T_load, V_bus]
    add_line(mdl, 'Demux_MPC_Out/1', 'Mux_Plant_In/1', 'autorouting', 'on');
    add_line(mdl, 'Load_Torque/1', 'Mux_Plant_In/2', 'autorouting', 'on');
    add_line(mdl, 'V_bus/1', 'Mux_Plant_In/3', 'autorouting', 'on');

    % Connect Plant S-Function
    add_line(mdl, 'Mux_Plant_In/1', 'PMSM_Motor_Plant/1', 'autorouting', 'on');
    add_line(mdl, 'PMSM_Motor_Plant/1', 'Demux_Plant_Out/1', 'autorouting', 'on');

    % Speed Mux & Log: [Ref, True]
    add_line(mdl, 'Speed_Ref/1', 'Mux_Speed_Log/1', 'autorouting', 'on');
    add_line(mdl, 'Demux_Plant_Out/3', 'Mux_Speed_Log/2', 'autorouting', 'on');
    add_line(mdl, 'Mux_Speed_Log/1', 'To_WS_Speed/1', 'autorouting', 'on');
    add_line(mdl, 'Mux_Speed_Log/1', 'Scope_Speed_and_Torque/1', 'autorouting', 'on');

    % Torque Mux & Log: [Te, T_load]
    add_line(mdl, 'Demux_Plant_Out/5', 'Mux_Torque_Log/1', 'autorouting', 'on');
    add_line(mdl, 'Load_Torque/1', 'Mux_Torque_Log/2', 'autorouting', 'on');
    add_line(mdl, 'Mux_Torque_Log/1', 'To_WS_Torque/1', 'autorouting', 'on');
    add_line(mdl, 'Mux_Torque_Log/1', 'Scope_Speed_and_Torque/2', 'autorouting', 'on');

    % 3-Phase Currents Log
    add_line(mdl, 'Sum_Fault_Inject/1', 'To_WS_Currents_ABC/1', 'autorouting', 'on');
    add_line(mdl, 'Sum_Fault_Inject/1', 'Scope_Currents/1', 'autorouting', 'on');

    % d-q Currents Mux & Log: [dq_true, dq_ref]
    add_line(mdl, 'Demux_Plant_Out/6', 'Mux_DQ_Log/1', 'autorouting', 'on');
    add_line(mdl, 'Demux_MPC_Out/4', 'Mux_DQ_Log/2', 'autorouting', 'on');
    add_line(mdl, 'Mux_DQ_Log/1', 'To_WS_Currents_DQ/1', 'autorouting', 'on');
    add_line(mdl, 'Mux_DQ_Log/1', 'Scope_Currents/2', 'autorouting', 'on');

    % Switching & Vector Log: [Sa_Sb_Sc, opt_vec]
    add_line(mdl, 'Demux_MPC_Out/1', 'Mux_Switch_Log/1', 'autorouting', 'on');
    add_line(mdl, 'Demux_MPC_Out/2', 'Mux_Switch_Log/2', 'autorouting', 'on');
    add_line(mdl, 'Mux_Switch_Log/1', 'To_WS_Switching/1', 'autorouting', 'on');
    add_line(mdl, 'Mux_Switch_Log/1', 'Scope_Power_Electronics/1', 'autorouting', 'on');

    % Power Electronics Telemetry Log: [Vcm, Ploss, Eff, Fsw, Jcost]
    add_line(mdl, 'Demux_MPC_Out/6', 'Mux_PE_Log/1', 'autorouting', 'on');
    add_line(mdl, 'Demux_MPC_Out/7', 'Mux_PE_Log/2', 'autorouting', 'on');
    add_line(mdl, 'Demux_MPC_Out/8', 'Mux_PE_Log/3', 'autorouting', 'on');
    add_line(mdl, 'Demux_MPC_Out/9', 'Mux_PE_Log/4', 'autorouting', 'on');
    add_line(mdl, 'Demux_MPC_Out/10', 'Mux_PE_Log/5', 'autorouting', 'on');
    add_line(mdl, 'Mux_PE_Log/1', 'To_WS_PE_Metrics/1', 'autorouting', 'on');
    add_line(mdl, 'Mux_PE_Log/1', 'Scope_Power_Electronics/2', 'autorouting', 'on');

    % V_bus Log
    add_line(mdl, 'V_bus/1', 'To_WS_Vbus/1', 'autorouting', 'on');

    % Terminators for unused outputs
    add_block('simulink/Sinks/Terminator', [mdl, '/Term_Plant_OmegaE'], 'Position', [830, 140, 850, 150]);
    add_line(mdl, 'Demux_Plant_Out/4', 'Term_Plant_OmegaE/1', 'autorouting', 'on');

    add_block('simulink/Sinks/Terminator', [mdl, '/Term_MPC_idq_meas'], 'Position', [540, 100, 560, 110]);
    add_line(mdl, 'Demux_MPC_Out/3', 'Term_MPC_idq_meas/1', 'autorouting', 'on');

    add_block('simulink/Sinks/Terminator', [mdl, '/Term_MPC_Torque'], 'Position', [540, 140, 560, 150]);
    add_line(mdl, 'Demux_MPC_Out/5', 'Term_MPC_Torque/1', 'autorouting', 'on');

    % Save and close
    save_system(mdl, slx_path);
    close_system(mdl);
    
    fprintf('==================================================================\n');
    fprintf('Successfully built and saved %s!\n', slx_path);
    fprintf('==================================================================\n');
end
