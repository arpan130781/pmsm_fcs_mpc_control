% =========================================================================
% run_simulink_mpc_simulation.m
%
% Runs the Finite-Control-Set Model Predictive Control (FCS-MPC) 3-Phase
% PMSM Drive Simulink Simulation, calculates quantitative control and power
% electronics metrics, and renders publication-grade response figures.
%
% Author: Arpan Kailash Gupta (Roll No: 241030009)
% Department of Electrical Engineering, VJTI Mumbai
% =========================================================================

function run_simulink_mpc_simulation()
    here = fileparts(mfilename('fullpath'));
    addpath(here);
    results_dir = fullfile(here, 'results');
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end

    fprintf('==================================================================\n');
    fprintf('  FCS-MPC PMSM Drive MATLAB/Simulink Co-Simulation Suite\n');
    fprintf('  Author: Arpan Kailash Gupta | Roll No: 241030009\n');
    fprintf('  Department of Electrical Engineering, VJTI Mumbai\n');
    fprintf('==================================================================\n\n');

    % 1. Load Parameters into Base Workspace
    evalin('base', 'init_pmsm_mpc');
    
    sim_cfg = evalin('base', 'sim_cfg');
    profile = evalin('base', 'profile');
    motor   = evalin('base', 'motor');

    mdl = 'pmsm_mpc_simulink';
    slx_path = fullfile(here, [mdl, '.slx']);
    
    if ~exist(slx_path, 'file')
        fprintf('Model %s.slx not found. Generating model...\n', mdl);
        create_pmsm_mpc_model;
    end

    fprintf('\nExecuting Simulink Simulation: %s (Duration = %.1f s)...\n', mdl, sim_cfg.T_final);
    tic;
    simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    sim_wall_time = toc;
    fprintf('Simulink simulation completed successfully in %.2f seconds wall-clock time.\n', sim_wall_time);

    %% --- 2. Extract Simulation Signals ---
    t = simOut.get('tout');
    speed_data    = simOut.get('sim_speed_rpm');       % [Ref, True]
    torque_data   = simOut.get('sim_torque');          % [Te, T_load]
    currents_abc  = simOut.get('sim_currents_abc');    % [ia, ib, ic]
    currents_dq   = simOut.get('sim_currents_dq');     % [id_meas, iq_meas, id_ref, iq_ref]
    switching     = simOut.get('sim_switching');       % [Sa, Sb, Sc, opt_vector]
    pe_metrics    = simOut.get('sim_pe_metrics');      % [v_cm, p_loss, efficiency, f_sw, min_cost]
    vbus_data     = simOut.get('sim_vbus');

    spd_ref  = speed_data(:, 1);
    spd_true = speed_data(:, 2);

    Te_true  = torque_data(:, 1);
    T_load   = torque_data(:, 2);

    ia = currents_abc(:, 1);
    ib = currents_abc(:, 2);
    ic = currents_abc(:, 3);

    id_true = currents_dq(:, 1);
    iq_true = currents_dq(:, 2);
    id_ref  = currents_dq(:, 3);
    iq_ref  = currents_dq(:, 4);

    % Resample control-rate signals to plant timebase if dimensions differ
    t_ctrl = linspace(0, t(end), size(switching, 1))';
    if size(switching, 1) ~= length(t)
        sa         = interp1(t_ctrl, switching(:, 1), t, 'previous', 'extrap');
        sb         = interp1(t_ctrl, switching(:, 2), t, 'previous', 'extrap');
        sc         = interp1(t_ctrl, switching(:, 3), t, 'previous', 'extrap');
        opt_vec    = interp1(t_ctrl, switching(:, 4), t, 'previous', 'extrap');
        v_cm       = interp1(t_ctrl, pe_metrics(:, 1), t, 'linear', 'extrap');
        p_loss     = interp1(t_ctrl, pe_metrics(:, 2), t, 'linear', 'extrap');
        efficiency = interp1(t_ctrl, pe_metrics(:, 3), t, 'linear', 'extrap');
        f_sw       = interp1(t_ctrl, pe_metrics(:, 4), t, 'linear', 'extrap');
        min_cost   = interp1(t_ctrl, pe_metrics(:, 5), t, 'linear', 'extrap');
    else
        sa         = switching(:, 1);
        sb         = switching(:, 2);
        sc         = switching(:, 3);
        opt_vec    = switching(:, 4);
        v_cm       = pe_metrics(:, 1);
        p_loss     = pe_metrics(:, 2);
        efficiency = pe_metrics(:, 3);
        f_sw       = pe_metrics(:, 4);
        min_cost   = pe_metrics(:, 5);
    end

    %% --- 3. Compute Quantitative Metrics ---
    % Steady-state window: 0.5s to 0.75s (after initial startup, before load step)
    idx_ss1 = (t >= 0.5) & (t <= 0.75);
    mean_spd_ss1 = mean(spd_true(idx_ss1));
    spd_err_ss1  = abs(mean_spd_ss1 - profile.speed_ref_rpm);
    spd_pct_err  = (spd_err_ss1 / profile.speed_ref_rpm) * 100.0;

    % Post-load steady state: 1.0s to 1.25s (with 0.10 N*m load)
    idx_load = (t >= 1.0) & (t <= 1.25);
    mean_Te_load  = mean(Te_true(idx_load));
    ripple_Te     = max(Te_true(idx_load)) - min(Te_true(idx_load));
    pct_Te_ripple = (ripple_Te / mean_Te_load) * 100.0;

    % Power Electronics Inverter Metrics
    mean_fsw_khz  = mean(f_sw(idx_load)) / 1000.0;
    mean_loss_w   = mean(p_loss(idx_load));
    mean_eff_pct  = mean(efficiency(idx_load));
    max_cmv_v     = max(abs(v_cm(idx_load)));

    fprintf('\n------------------------------------------------------------------\n');
    fprintf('  Simulink FCS-MPC Quantitative Performance Metrics:\n');
    fprintf('------------------------------------------------------------------\n');
    fprintf('  Speed Reference Tracking:      %.1f RPM (Error: %.2f RPM, %.3f%%)\n', ...
        mean_spd_ss1, spd_err_ss1, spd_pct_err);
    fprintf('  Electromagnetic Torque (Mean): %.3f N*m (Target Load: %.2f N*m)\n', ...
        mean_Te_load, profile.load_torque_Nm);
    fprintf('  Torque Peak-to-Peak Ripple:    %.4f N*m (%.1f%%)\n', ...
        ripple_Te, pct_Te_ripple);
    fprintf('  Average Inverter Switching:    %.2f kHz (constrained <= 20 kHz)\n', ...
        mean_fsw_khz);
    fprintf('  Mean Inverter Total Losses:    %.3f W\n', mean_loss_w);
    fprintf('  Inverter Electrical Efficiency:%.2f%%\n', mean_eff_pct);
    fprintf('  Peak Common-Mode Voltage (CMV):%.2f V\n', max_cmv_v);
    fprintf('------------------------------------------------------------------\n');

    %% --- 4. Publication Figure 1: Dynamic Drive Response ---
    fig1 = figure('Name', 'Simulink FCS-MPC Dynamic Response', ...
        'Color', 'w', 'Position', [100, 100, 1100, 850], 'Visible', 'off');

    % Subplot 1: Speed Tracking & Load Rejection
    subplot(4, 1, 1);
    plot(t, spd_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference \omega_{ref}'); hold on;
    plot(t, spd_true, 'Color', [0.0, 0.45, 0.74], 'LineWidth', 1.8, 'DisplayName', 'FCS-MPC Actual \omega_m');
    grid on; box on;
    ylabel('Speed (RPM)', 'FontWeight', 'bold');
    title('FCS-MPC PMSM Closed-Loop Dynamic Response (Simulink Co-Simulation)', 'FontWeight', 'bold', 'FontSize', 12);
    legend('Location', 'southeast', 'FontSize', 9);
    xlim([0, sim_cfg.T_final]);

    % Subplot 2: Torque Response
    subplot(4, 1, 2);
    plot(t, T_load, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Load Torque T_L'); hold on;
    plot(t, Te_true, 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.2, 'DisplayName', 'Electromagnetic Torque T_e');
    grid on; box on;
    ylabel('Torque (N\cdotm)', 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 9);
    xlim([0, sim_cfg.T_final]);

    % Subplot 3: Stator 3-Phase Currents (abc)
    subplot(4, 1, 3);
    plot(t, ia, 'Color', [0.0, 0.45, 0.74], 'LineWidth', 1.1, 'DisplayName', 'i_a (Phase A)'); hold on;
    plot(t, ib, 'Color', [0.47, 0.67, 0.19], 'LineWidth', 1.1, 'DisplayName', 'i_b (Phase B)');
    plot(t, ic, 'Color', [0.49, 0.18, 0.56], 'LineWidth', 1.1, 'DisplayName', 'i_c (Phase C)');
    grid on; box on;
    ylabel('Current (A)', 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 9);
    xlim([0, sim_cfg.T_final]);

    % Subplot 4: Synchronous d-q Stator Currents
    subplot(4, 1, 4);
    plot(t, id_ref, 'k--', 'LineWidth', 1.2, 'DisplayName', 'i_d^* (0 A MTPA)'); hold on;
    plot(t, id_true, 'Color', [0.0, 0.45, 0.74], 'LineWidth', 1.2, 'DisplayName', 'i_d Actual');
    plot(t, iq_ref, 'r--', 'LineWidth', 1.2, 'DisplayName', 'i_q^* Reference');
    plot(t, iq_true, 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.2, 'DisplayName', 'i_q Actual');
    grid on; box on;
    xlabel('Time (seconds)', 'FontWeight', 'bold');
    ylabel('d-q Current (A)', 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 9);
    xlim([0, sim_cfg.T_final]);

    plot1_path = fullfile(results_dir, 'simulink_mpc_dynamic_response.png');
    saveas(fig1, plot1_path);
    fprintf('\n[SAVED FIGURE] %s\n', plot1_path);
    close(fig1);

    %% --- 5. Publication Figure 2: Power Electronics Telemetry ---
    fig2 = figure('Name', 'Simulink FCS-MPC Power Electronics Metrics', ...
        'Color', 'w', 'Position', [150, 100, 1100, 850], 'Visible', 'off');

    % Subplot 1: Discrete Switching States & Optimal Vector
    subplot(4, 1, 1);
    stairs(t, opt_vec, 'Color', [0.1, 0.2, 0.6], 'LineWidth', 1.2, 'DisplayName', 'Active Vector V_0..V_7');
    grid on; box on;
    ylabel('Vector (0..7)', 'FontWeight', 'bold');
    title('FCS-MPC 2L-VSI Power Electronics & Efficiency Telemetry', 'FontWeight', 'bold', 'FontSize', 12);
    xlim([0.8, 0.85]); % Zoomed view showing discrete switching transitions
    xlabel('Time (s) [Zoomed Window: 0.8s - 0.85s]', 'FontSize', 8);

    % Subplot 2: Common-Mode Voltage (CMV)
    subplot(4, 1, 2);
    plot(t, v_cm, 'Color', [0.64, 0.08, 0.18], 'LineWidth', 1.2);
    grid on; box on;
    ylabel('CMV V_{cm} (V)', 'FontWeight', 'bold');
    title('Inverter Common-Mode Voltage Reduction (w_{cmv} = 0.02)', 'FontWeight', 'bold', 'FontSize', 10);
    xlim([0, sim_cfg.T_final]);

    % Subplot 3: Inverter Power Loss & Efficiency
    subplot(4, 1, 3);
    yyaxis left;
    plot(t, p_loss, 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.2);
    ylabel('Loss P_{tot} (W)', 'FontWeight', 'bold');
    yyaxis right;
    plot(t, efficiency, 'Color', [0.47, 0.67, 0.19], 'LineWidth', 1.4);
    ylabel('Efficiency \eta (%)', 'FontWeight', 'bold');
    grid on; box on;
    title('Inverter Conduction + Switching Losses and Stage Efficiency', 'FontWeight', 'bold', 'FontSize', 10);
    xlim([0, sim_cfg.T_final]);

    % Subplot 4: Average Switching Frequency & Cost Trajectory
    subplot(4, 1, 4);
    yyaxis left;
    plot(t, f_sw / 1000.0, 'Color', [0.0, 0.45, 0.74], 'LineWidth', 1.2);
    ylabel('f_{sw} (kHz)', 'FontWeight', 'bold');
    ylim([0, 22]);
    yyaxis right;
    plot(t, min_cost, 'Color', [0.49, 0.18, 0.56], 'LineWidth', 1.0);
    ylabel('Min Cost J_{opt}', 'FontWeight', 'bold');
    grid on; box on;
    xlabel('Time (seconds)', 'FontWeight', 'bold');
    title('Inverter Switching Frequency Regulation & Multi-Objective Cost', 'FontWeight', 'bold', 'FontSize', 10);
    xlim([0, sim_cfg.T_final]);

    plot2_path = fullfile(results_dir, 'simulink_mpc_power_electronics_metrics.png');
    saveas(fig2, plot2_path);
    fprintf('[SAVED FIGURE] %s\n', plot2_path);
    close(fig2);

    fprintf('\n==================================================================\n');
    fprintf('  All FCS-MPC Simulink Co-Simulations Completed Successfully!\n');
    fprintf('  Figures saved in: %s\n', results_dir);
    fprintf('==================================================================\n');
end
