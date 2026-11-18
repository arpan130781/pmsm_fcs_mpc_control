% =========================================================================
% init_pmsm_mpc.m
% 
% Initialization script for Finite-Control-Set Model Predictive Control
% (FCS-MPC) 3-Phase PMSM Drive Simulink Model.
%
% Author: Arpan Kailash Gupta (Roll No: 241030009)
% Department of Electrical Engineering, VJTI Mumbai
% =========================================================================

fprintf('==================================================================\n');
fprintf('Initializing FCS-MPC PMSM Drive Parameters for MATLAB/Simulink...\n');
fprintf('Author: Arpan Kailash Gupta (Roll No: 241030009), VJTI Mumbai\n');
fprintf('==================================================================\n');

%% --- 1. PMSM Motor Parameters (100W Surface PMSM) ---
motor.Rs    = 0.5;         % Stator phase resistance (Ohms)
motor.Ls    = 1.0e-3;      % Stator inductance Ld = Lq (Henries)
motor.psi_f = 0.015;       % Permanent magnet flux linkage (Webers)
motor.P     = 4;           % Number of pole pairs
motor.J     = 0.0001;      % Rotor moment of inertia (kg*m^2)
motor.B     = 0.001;       % Viscous friction coefficient (N*m*s/rad)
motor.Vbus  = 24.0;        % Nominal DC link bus voltage (Volts)
motor.In    = 5.0;         % Rated peak phase current (Amperes)
motor.Tcog  = 0.005;       % 6th harmonic cogging torque magnitude (N*m)

% Vector passed to C-MEX S-Function sfun_pmsm_plant
MotorParamVec = [ ...
    motor.Rs, motor.Ls, motor.psi_f, motor.P, ...
    motor.J,  motor.B,  motor.Vbus,  motor.In ...
];

%% --- 2. FCS-MPC Controller & Multi-Objective Weights ---
ctrl.fs       = 20000.0;   % Control sampling & switching frequency (20 kHz)
ctrl.Ts       = 1.0 / ctrl.fs; % 50 microseconds sampling period
ctrl.w_id     = 1.0;       % d-axis current tracking penalty weight
ctrl.w_iq     = 1.0;       % q-axis torque current tracking penalty weight
ctrl.w_sw     = 0.05;      % Inverter switching frequency reduction weight
ctrl.w_cmv    = 0.02;      % Common-mode voltage reduction weight
ctrl.spd_kp   = 0.194;     % Outer speed loop PI proportional gain
ctrl.spd_ki   = 17.4;      % Outer speed loop PI integral gain
ctrl.iq_max   = 8.0;       % Maximum torque current limit (Amperes)

%% --- 3. Simulation Timing Configuration ---
sim_cfg.Ts_ctrl  = ctrl.Ts; % 50 us discrete control execution rate
sim_cfg.Ts_plant = 1.0e-5;  % 10 us continuous integration solver step (ode4 Runge-Kutta)
sim_cfg.T_final  = 2.0;     % 2.0 seconds total simulation duration

%% --- 4. Operating Profiles & Dynamic Demands ---
% Speed Reference Command
profile.speed_ref_rpm  = 1800.0; % Target mechanical rotor speed (RPM)
profile.speed_ramp_end = 0.15;   % Soft-start ramp duration (s)

% External Load Torque Disturbance
profile.load_step_time = 0.8;    % Load step onset time (s)
profile.load_torque_Nm  = 0.10;   % Load step amplitude (N*m) (~22% continuous rated torque)

% Inverter DC Link Supply & Disturbance Profile
profile.v_dc_nominal   = 24.0;   % Nominal supply voltage (V)
profile.dc_sag_time    = 1.3;    % DC supply dip onset time (s)
profile.dc_sag_depth   = 6.0;    % Supply dip depth (24V -> 18V, 25% sag)
profile.dc_sag_dur     = 0.3;    % Dip duration (s)

% Current Sensor Fault Configuration
fault.enable       = 0;          % 0 = Nominal, 1 = Injected sensor fault
fault.onset_time   = 1.5;        % Fault onset time (s)
fault.channel      = 1;          % Phase A current sensor
fault.offset_val   = 2.0;        % +2.0 A DC offset error

fprintf('Configuration complete:\n');
fprintf('  Motor: 4 Pole Pairs, Rs = %.2f Ohm, Ls = %.3f mH, Vbus = %.1f V\n', ...
    motor.Rs, motor.Ls * 1e3, motor.Vbus);
fprintf('  FCS-MPC: Ts = %.1f us (20 kHz), Wid = %.2f, Wiq = %.2f, Wsw = %.3f, Wcmv = %.3f\n', ...
    ctrl.Ts * 1e6, ctrl.w_id, ctrl.w_iq, ctrl.w_sw, ctrl.w_cmv);
fprintf('  Speed Target: %.1f RPM | Load Step: %.2f N*m at t = %.2f s\n', ...
    profile.speed_ref_rpm, profile.load_torque_Nm, profile.load_step_time);
