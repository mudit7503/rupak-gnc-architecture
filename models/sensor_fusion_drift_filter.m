% =============================================================================
% FILE: models/sensor_fusion_drift_filter.m
% PROJECT: RUPAK VTVL GNC Architecture
% AUTHOR: GNC Engineering Team
% MATLAB: R2024a
% =============================================================================
%
% PURPOSE:
%   Implements a 2-state Error-State Kalman Filter (ESKF) for attitude
%   estimation on the RUPAK VTVL rocket. The filter tracks:
%     - State 1: Roll attitude error       [delta_theta, rad]
%     - State 2: Gyro bias drift error     [delta_omega_bias, rad/s]
%
%   The ESKF operates on the ERROR (perturbation) states, not the nominal
%   states directly. A reference trajectory (nominal attitude) is propagated
%   separately, and the Kalman filter estimates the small errors around it.
%   This is the standard approach for high-performance INS/IMU fusion systems
%   and is used in SpaceX, RocketLab, and ISRO navigation systems.
%
% FILTER ARCHITECTURE:
%   - Prediction: IMU gyro integration propagates the error-state covariance
%   - Correction: Absolute attitude update from NavIC/GPS at t=5 seconds
%   - Numerical stability: Joseph-form symmetric covariance update
%   - Loop Rate: 400 Hz (matches GNC control frame rate)
%
% ERROR STATE VECTOR:
%   delta_x = [delta_theta;        % [rad]   Attitude (roll) error
%              delta_omega_bias]   % [rad/s] Gyro bias drift error
%
% IMU NOISE MODEL:
%   - Angle Random Walk (ARW): models white noise on angular rate measurement
%     Typical MEMS IMU: sigma_ARW ~ 0.05 deg/sqrt(hr) -> 1.45e-5 rad/sqrt(s)
%     High-grade tactical IMU: sigma_ARW ~ 0.003 deg/sqrt(hr) -> 8.73e-7 rad/sqrt(s)
%     We use a mid-grade aerospace IMU appropriate for a launch vehicle.
%   - Rate Random Walk (RRW): models random walk of gyro bias over time
%     sigma_RRW ~ 0.001 deg/hr/sqrt(hr) -> 4.85e-9 rad/s/sqrt(s)
%
% NAVIC/GPS MEASUREMENT UPDATE:
%   - NavIC (India's Regional Navigation Satellite System) provides absolute
%     attitude reference via heading/attitude determination mode.
%   - Update arrives at t = 5 seconds (asynchronous, single event)
%   - Measurement noise: sigma_NavIC ~ 0.1 deg (1-sigma RMS)
%
% REFERENCES:
%   [1] Trawny, N. & Roumeliotis, S.I., "Indirect Kalman Filter for 3D
%       Attitude Estimation," U of Minnesota TR-2005-002
%   [2] Groves, P.D., "Principles of GNSS, Inertial, and Multisensor
%       Integrated Navigation Systems," Artech House, 2013
%   [3] RUPAK GNC Architecture Document, Section 5.3 (ESKF Nav Filter)
%
% =============================================================================

clearvars;
close all;
clc;

fprintf('=============================================================\n');
fprintf('  RUPAK GNC | Error-State Kalman Filter (ESKF)\n');
fprintf('  Attitude Estimation with NavIC/GPS Update @ t=5s\n');
fprintf('  Simulation initialising...\n');
fprintf('=============================================================\n\n');

% =============================================================================
% SECTION 1: SIMULATION TIME PARAMETERS
% =============================================================================

dt        = 1 / 400;           % [s] Sample period (400 Hz navigation frame)
t_end     = 10.0;              % [s] Total simulation duration
t         = 0 : dt : t_end;   % [s] Time vector
N         = length(t);         % [-] Number of time steps

fprintf('Sample rate: %d Hz | Duration: %.1f s | Steps: %d\n', ...
        round(1/dt), t_end, N);

% =============================================================================
% SECTION 2: IMU NOISE PARAMETERS (Aerospace Grade)
% =============================================================================
% These parameters characterise the stochastic error model of the RUPAK IMU.
% We assume a mid-grade Ring Laser Gyroscope (RLG) or high-end MEMS tactical
% grade IMU appropriate for the launch vehicle application.

% --- Angle Random Walk (ARW) ---
% ARW is expressed as noise density on angular rate [rad/s/sqrt(Hz)].
% It manifests as white noise on the gyro output and integrates into
% attitude error over time as a random walk (Brownian motion in angle).
%
% ARW_spec = 0.005 deg/sqrt(hr) is a good tactical-grade IMU spec.
% Convert to SI units:
%   [deg/sqrt(hr)] * [pi/180 rad/deg] * [1/sqrt(3600) sqrt(hr)/sqrt(s)]
ARW_deg_sqrthr = 0.005;        % [deg/√hr] ARW specification (IMU datasheet)
ARW_rad_sqrts  = ARW_deg_sqrthr * (pi/180) / sqrt(3600);
% [rad/√s] ARW in continuous-time noise density

% Discrete-time ARW standard deviation per sample:
%   sigma_ARW_discrete = ARW_rad_sqrts / sqrt(dt)
%   (discrete noise increases with lower dt — this is physically correct)
sigma_arw = ARW_rad_sqrts / sqrt(dt);  % [rad/sample] Discrete ARW sigma

% --- Rate Random Walk (RRW) / Gyro Bias Instability ---
% Gyro bias does not stay constant; it drifts slowly over time as a random walk.
% RRW models this as white noise on the DERIVATIVE of the bias (bias rate).
%
% RRW_spec = 0.001 deg/hr/sqrt(hr) is typical.
RRW_deg_hr_sqrthr = 0.001;     % [deg/hr/√hr] RRW specification
RRW_rad_s_sqrts   = RRW_deg_hr_sqrthr * (pi/180) / 3600 / sqrt(3600);
% [rad/s/√s] RRW in continuous-time noise density

% Discrete-time RRW standard deviation:
sigma_rrw = RRW_rad_s_sqrts / sqrt(dt); % [rad/s per sample] Discrete RRW sigma

% For covariance matrix: process noise variances
q_theta  = sigma_arw^2;        % [rad²] Attitude error process noise variance
q_bias   = sigma_rrw^2;        % [rad²/s²] Bias drift process noise variance

fprintf('\nIMU Noise Parameters:\n');
fprintf('  ARW         = %.4e rad/sqrt(s) [%.4f deg/sqrt(hr)]\n', ...
        ARW_rad_sqrts, ARW_deg_sqrthr);
fprintf('  sigma_ARW   = %.4e rad/sample (at %d Hz)\n', sigma_arw, round(1/dt));
fprintf('  RRW         = %.4e rad/s/sqrt(s)\n', RRW_rad_s_sqrts);
fprintf('  sigma_RRW   = %.4e rad/s per sample\n', sigma_rrw);
fprintf('  q_theta     = %.4e rad²\n', q_theta);
fprintf('  q_bias      = %.4e rad²/s²\n', q_bias);

% =============================================================================
% SECTION 3: NAVIC/GPS MEASUREMENT PARAMETERS
% =============================================================================

% NavIC absolute attitude measurement noise (1-sigma)
sigma_navic_deg = 0.10;        % [deg] NavIC attitude accuracy (1-sigma)
sigma_navic     = deg2rad(sigma_navic_deg); % [rad]
R_navic         = sigma_navic^2; % [rad²] Scalar measurement noise variance

% NavIC update timing
t_navic         = 5.0;          % [s] Time of NavIC/GPS update injection
k_navic         = round(t_navic / dt) + 1; % Array index of update step
% The +1 accounts for MATLAB 1-based indexing (t(1)=0, t(k_navic)=t_navic)

fprintf('\nNavIC/GPS Update Parameters:\n');
fprintf('  Update time     = %.2f s (k = %d)\n', t_navic, k_navic);
fprintf('  sigma_NavIC     = %.4f deg (%.4e rad)\n', sigma_navic_deg, sigma_navic);
fprintf('  R_navic         = %.4e rad²\n', R_navic);

% =============================================================================
% SECTION 4: ERROR-STATE KALMAN FILTER INITIALISATION
% =============================================================================
% ESKF state: delta_x = [delta_theta; delta_omega_bias]  (2x1 column vector)

% --- Initial Error State (ESKF starts at zero; errors are initially unknown) ---
delta_x_hat = zeros(2, 1);     % [rad; rad/s] Initial error state estimate = 0
% Note: The actual IMU might have non-zero bias at startup. In a real system,
% an initial alignment procedure (static or gyrocompassing) would be performed
% to estimate this. Here we start with zero initial estimate to show filter
% convergence from ignorance.

% --- Initial Error Covariance Matrix P (2x2) ---
% P represents our uncertainty about the initial error state.
% We're fairly uncertain about the initial attitude error (a few degrees)
% and have some knowledge about expected gyro bias range.

P0_theta = deg2rad(2.0)^2;     % [rad²] Initial attitude uncertainty (2 deg 1-sigma)
P0_bias  = deg2rad(0.5/3600)^2;% [rad²/s²] Initial bias uncertainty (0.5 deg/hr)
P = diag([P0_theta, P0_bias]); % [2x2] Initial covariance matrix (diagonal)

fprintf('\nESKF Initialisation:\n');
fprintf('  P0_theta     = %.4e rad² (%.4f deg 1-sigma)\n', P0_theta, ...
        rad2deg(sqrt(P0_theta)));
fprintf('  P0_bias      = %.4e rad²/s² (%.4f deg/hr 1-sigma)\n', P0_bias, ...
        rad2deg(sqrt(P0_bias)) * 3600);

% --- Process Noise Covariance Matrix Q (2x2) ---
% Q describes how quickly uncertainty grows during IMU propagation.
% In the ESKF discrete-time model:
%   Q = diag([q_theta, q_bias])
% where these were computed from IMU specs in Section 2.
Q = diag([q_theta, q_bias]);   % [2x2] Discrete-time process noise covariance

% --- Measurement Jacobian H (1x2) ---
% The NavIC measurement directly observes the ATTITUDE error delta_theta.
% It does NOT directly observe gyro bias (only indirectly through filter dynamics).
% H = [1, 0] means: z_measured = H * delta_x + v
%   where z_measured = delta_theta_navic (attitude error seen from NavIC)
H_navic = [1, 0];              % [1x2] Measurement Jacobian matrix

% =============================================================================
% SECTION 5: STATE TRANSITION MATRIX F (2x2)
% =============================================================================
% The ESKF error-state dynamics (continuous-time):
%   d/dt [delta_theta]       = -delta_omega_bias + noise_ARW
%   d/dt [delta_omega_bias]  =  0                + noise_RRW
%
% In matrix form: delta_x_dot = F_c * delta_x + noise
%   F_c = [-0 , -1]   <- attitude error grows from negative bias offset
%         [ 0 ,  0]   <- bias rate of change is zero (random walk driver)
%
% Note the sign: if the gyro reads HIGH (positive bias), the integrated
% attitude estimate will drift in the POSITIVE direction over time. The ESKF
% must estimate and correct this positive bias.
%
% Discretised using Zero-Order Hold (ZOH) / first-order Euler (exact for this
% simple linear system):
%   F_d = I + F_c * dt  (exact for piecewise-constant F)
%
% Because the off-diagonal term of F_c is -1 (multiplied by dt):
F = [1, -dt;    % [row 1] delta_theta_{k+1} = delta_theta_k - dt * delta_bias_k
     0,  1];    % [row 2] delta_bias_{k+1}  = delta_bias_k  (Brownian walk)

fprintf('\nState Transition Matrix F (discrete, ZOH):\n');
fprintf('  F = [%.6f, %.8f]\n', F(1,1), F(1,2));
fprintf('      [%.6f, %.6f ]\n', F(2,1), F(2,2));

% =============================================================================
% SECTION 6: TRUE STATE SIMULATION (Ground Truth)
% =============================================================================
% To generate a realistic synthetic IMU signal, we first simulate the TRUE
% (unknown to the filter) attitude and gyro bias trajectory.

% True initial roll attitude (vehicle has a slow initial drift scenario)
theta_true_0 = deg2rad(0.5);   % [rad] True initial attitude (slight misalignment)

% True gyro bias (constant + slow drift, unknown to filter)
% We model a true bias of 0.2 deg/hr (a realistic value for a quality IMU)
true_bias_0  = deg2rad(0.2 / 3600); % [rad/s] True initial gyro bias

% True gyro bias random walk (the bias itself drifts slowly)
true_bias_drift_rate = sigma_rrw; % [rad/s per sample] True bias random walk rate

rng(7);                             % Reproducible random numbers
% Pre-generate white noise sequences for ground truth simulation
w_ARW_true  = sigma_arw * randn(1, N);   % [rad] True ARW noise realisation
w_RRW_true  = sigma_rrw * randn(1, N);   % [rad/s] True RRW noise (bias drift)

% True vehicle angular rate (nominal slow manoeuvre + bias + noise)
% Model: slow sinusoidal pitch-over manoeuvre at 0.1 Hz (structural rotation)
omega_manoeuvre_amp  = deg2rad(0.5);     % [rad/s] Manoeuvre angular rate amplitude
omega_manoeuvre_freq = 0.1;              % [Hz] Manoeuvre frequency

% Pre-allocate true state histories
theta_true_hist = zeros(1, N);           % [rad] True roll attitude
bias_true_hist  = zeros(1, N);           % [rad/s] True gyro bias
omega_true_hist = zeros(1, N);           % [rad/s] True angular rate (w/o bias or noise)

% Simulate true trajectory and bias evolution
theta_true = theta_true_0;
bias_true  = true_bias_0;

for k = 1 : N
    % Record true states
    theta_true_hist(k) = theta_true;
    bias_true_hist(k)  = bias_true;

    % True angular rate (clean signal, what an ideal gyro would read)
    omega_clean = omega_manoeuvre_amp * sin(2 * pi * omega_manoeuvre_freq * t(k));
    omega_true_hist(k) = omega_clean;

    % Propagate true attitude (Euler integration of true rate)
    theta_true = theta_true + dt * omega_clean;

    % Propagate true gyro bias (random walk)
    bias_true  = bias_true  + w_RRW_true(k);
end

% =============================================================================
% SECTION 7: SYNTHETIC IMU MEASUREMENT GENERATION
% =============================================================================
% The gyroscope outputs the TRUE angular rate PLUS gyro bias PLUS ARW noise.
% This is what the navigation filter actually receives.

% Raw IMU angular rate measurement
omega_raw_hist = omega_true_hist + bias_true_hist + w_ARW_true;
% [rad/s] Corrupted measurement = clean_signal + bias + white_noise

% Nominal attitude estimate from pure gyro integration (no correction)
% This represents what happens WITHOUT any Kalman filtering correction.
% It drifts continuously due to integrated bias.
theta_nominal_hist = zeros(1, N);   % [rad] Raw gyro-integrated attitude (open loop)
theta_nominal = theta_true_0;        % Start from same initial condition as truth

for k = 1 : N
    theta_nominal_hist(k) = theta_nominal;
    % Integrate raw gyro measurement (no bias correction applied)
    theta_nominal = theta_nominal + dt * omega_raw_hist(k);
end

fprintf('\nSynthetic IMU generated. True bias(0) = %.4e rad/s (%.4f deg/hr)\n', ...
        true_bias_0, rad2deg(true_bias_0) * 3600);
fprintf('Raw gyro drift over 10 s (approx): %.4f deg\n', ...
        rad2deg(abs(theta_nominal_hist(end) - theta_true_hist(end))));

% =============================================================================
% SECTION 8: ESKF MAIN LOOP (400 Hz PREDICTION + ASYNCHRONOUS UPDATE)
% =============================================================================
% Pre-allocate filter output histories

% Filtered attitude estimate (nominal + ESKF correction)
theta_eskf_hist     = zeros(1, N);  % [rad] ESKF-corrected attitude estimate

% ESKF error state estimates
delta_theta_hist    = zeros(1, N);  % [rad] Estimated attitude error state
delta_bias_hist     = zeros(1, N);  % [rad/s] Estimated gyro bias drift state

% Covariance bounds (1-sigma envelopes, square root of diagonal elements)
sigma_theta_hist    = zeros(1, N);  % [rad] 1-sigma bound on attitude error
sigma_bias_hist     = zeros(1, N);  % [rad/s] 1-sigma bound on bias drift

% Full 2x2 covariance matrix snapshots (stored at each step for plotting)
P11_hist            = zeros(1, N);  % P(1,1) element history
P22_hist            = zeros(1, N);  % P(2,2) element history
P12_hist            = zeros(1, N);  % P(1,2) cross-covariance history

% Innovation residual (only non-NaN at NavIC update step)
innovation_hist     = NaN(1, N);    % [rad] NavIC measurement innovation
innovation_sigma    = NaN(1, N);    % [rad] Innovation 1-sigma bound

% Kalman gain history (only populated at update steps)
K_gain_hist         = NaN(2, N);    % [2x1] Kalman gain vector at update step

% Flag to record post-update P reduction
navic_update_done   = false;
P_pre_update        = [];           % Covariance before NavIC update
P_post_update       = [];           % Covariance after NavIC update

% --- Initialise filter state ---
delta_x_hat  = zeros(2, 1);         % [rad; rad/s] Error state estimate

% Nominal attitude estimate (starts from true initial, propagated by raw IMU)
theta_hat = theta_true_0;           % [rad] Nominal attitude estimate

fprintf('\nRunning ESKF propagation...\n');

for k = 1 : N
    % ------------------------------------------------------------------
    % 8.1 RECORD CURRENT ESTIMATES TO HISTORY
    % ------------------------------------------------------------------
    % The corrected attitude = nominal + estimated attitude error
    theta_eskf_hist(k)  = theta_nominal_hist(k) + delta_x_hat(1);

    % Store individual error states
    delta_theta_hist(k) = delta_x_hat(1);     % [rad] Attitude error estimate
    delta_bias_hist(k)  = delta_x_hat(2);     % [rad/s] Bias drift estimate

    % Store covariance diagonal elements
    P11_hist(k) = P(1,1);                      % Attitude error variance
    P22_hist(k) = P(2,2);                      % Bias drift variance
    P12_hist(k) = P(1,2);                      % Cross-covariance term

    % 1-sigma bounds (square root of diagonal)
    sigma_theta_hist(k) = sqrt(max(P(1,1), 0)); % Clip to avoid sqrt of negative
    sigma_bias_hist(k)  = sqrt(max(P(2,2), 0));

    % ------------------------------------------------------------------
    % 8.2 ESKF PREDICTION STEP (IMU Propagation)
    % ------------------------------------------------------------------
    % Propagate the error-state estimate through the state transition matrix:
    %   delta_x_hat_{k|k-1} = F * delta_x_hat_{k-1|k-1}
    % For the ESKF, this simply propagates the bias estimate (attitude error
    % is zeroed after each NavIC update via "reset" of the nominal trajectory).
    delta_x_hat = F * delta_x_hat;

    % Propagate error covariance:
    %   P_{k|k-1} = F * P_{k-1|k-1} * F' + Q
    % This is the standard discrete-time Riccati prediction equation.
    P = F * P * F' + Q;

    % Enforce symmetry of P numerically (should always be symmetric, but
    % floating-point errors can introduce asymmetry over many iterations)
    P = 0.5 * (P + P');

    % ------------------------------------------------------------------
    % 8.3 NAVIC/GPS MEASUREMENT UPDATE (at t = 5 seconds only)
    % ------------------------------------------------------------------
    % The NavIC receiver provides an ABSOLUTE attitude measurement.
    % This measurement is injected ONCE at t = t_navic = 5 seconds.
    % In a real system, NavIC updates might arrive at 1-10 Hz and would
    % be processed asynchronously here with the same logic.

    if k == k_navic
        fprintf('\n  --> NavIC update injected at t = %.3f s (k=%d)\n', t(k), k);
        fprintf('      P(1,1) before update: %.6e rad²\n', P(1,1));

        % Store pre-update covariance for comparison
        P_pre_update = P;

        % ---- COMPUTE KALMAN GAIN ----
        % S = Innovation covariance (scalar for 1-state measurement)
        %   S = H * P * H' + R
        % Innovation covariance tells us how uncertain the innovation is.
        S = H_navic * P * H_navic' + R_navic;  % [scalar] Innovation covariance

        % Kalman gain K (2x1 vector)
        %   K = P * H' * inv(S)
        % K determines how much we trust the NavIC measurement vs IMU prediction.
        K = P * H_navic' / S;                   % [2x1] Kalman gain vector

        % Record Kalman gain
        K_gain_hist(:, k) = K;

        % ---- COMPUTE MEASUREMENT INNOVATION ----
        % The NavIC receiver provides an attitude measurement z_navic.
        % We model this as the TRUE attitude + measurement noise.
        % Innovation = z_navic - H * delta_x_hat_predicted
        %
        % In ESKF terms, the NavIC measures the error in our CURRENT nominal
        % estimate relative to the absolute truth.
        % z_navic = (theta_true - theta_nominal)  <- what NavIC "sees"
        % Our predicted z = H * delta_x_hat = delta_theta_hat
        %
        % Simulated NavIC measurement (true error + measurement noise):
        z_navic = (theta_true_hist(k) - theta_nominal_hist(k)) + ...
                   sigma_navic * randn;           % [rad] Simulated NavIC observation

        % Innovation residual (prediction error)
        y_innovation = z_navic - H_navic * delta_x_hat;  % [rad] Scalar residual

        % Store innovation for plotting
        innovation_hist(k)  = y_innovation;
        innovation_sigma(k) = sqrt(S);          % 1-sigma innovation bound

        fprintf('      NavIC innovation: %.6f deg (%.4f mrad)\n', ...
                rad2deg(y_innovation), y_innovation * 1000);

        % ---- JOSEPH-FORM COVARIANCE UPDATE ----
        % Standard Kalman update: P+ = (I - K*H) * P * (I - K*H)' + K*R*K'
        % This is the JOSEPH FORM, which is:
        %   1. Numerically more stable than the simple: P+ = (I - K*H) * P
        %   2. Guarantees P remains symmetric and positive semi-definite
        %   3. Required for high-precision aerospace navigation (DO-316 compliant)
        %
        % The simple (non-Joseph) form P+ = (I-KH)P is only optimal when K
        % is EXACTLY the Kalman gain. Any numerical error in K propagates into
        % asymmetry. The Joseph form is self-correcting for these errors.
        %
        % Joseph form derivation:
        %   P+ = (I - K*H) * P * (I - K*H)' + K * R * K'
        %   with:
        %     first term  = (I-KH) * P_prior * (I-KH)' [smoothed uncertainty]
        %     second term = K * R * K'                  [measurement noise contribution]
        %
        I2          = eye(2);                   % 2x2 identity matrix
        IKH         = I2 - K * H_navic;         % (I - K*H) matrix [2x2]
        P_joseph    = IKH * P * IKH' + K * R_navic * K';  % Joseph form update [2x2]

        % Enforce strict symmetry after Joseph update
        P           = 0.5 * (P_joseph + P_joseph');

        % ---- UPDATE ERROR STATE ESTIMATE ----
        % delta_x_hat+ = delta_x_hat- + K * y_innovation
        delta_x_hat = delta_x_hat + K * y_innovation;

        % Store post-update covariance
        P_post_update = P;
        navic_update_done = true;

        fprintf('      P(1,1) after  update: %.6e rad²\n', P(1,1));
        fprintf('      Covariance reduction: %.2fx\n', P_pre_update(1,1) / P_post_update(1,1));
        fprintf('      K_gain = [%.6f; %.6e]\n', K(1), K(2));
    end

end

fprintf('\nESKF propagation complete.\n');

% =============================================================================
% SECTION 9: POST-PROCESSING AND DIAGNOSTICS
% =============================================================================

% Compute attitude error (true vs ESKF-corrected estimate)
theta_error_raw_deg  = rad2deg(theta_nominal_hist - theta_true_hist); % [deg] Uncorrected
theta_error_eskf_deg = rad2deg(theta_eskf_hist    - theta_true_hist); % [deg] ESKF corrected

% RMS attitude errors
rms_raw  = rms(theta_error_raw_deg);
rms_eskf = rms(theta_error_eskf_deg);

% Post-update covariance reduction ratio
if navic_update_done
    P11_reduction = P_pre_update(1,1) / P_post_update(1,1);
    P22_reduction = P_pre_update(2,2) / P_post_update(2,2);
else
    P11_reduction = NaN;
    P22_reduction = NaN;
end

fprintf('\n--- ESKF Performance Metrics ---\n');
fprintf('  RMS attitude error (raw gyro):  %.6f deg\n', rms_raw);
fprintf('  RMS attitude error (ESKF):      %.6f deg\n', rms_eskf);
fprintf('  Attitude error improvement:     %.2fx\n', rms_raw / rms_eskf);
fprintf('  P11 reduction at NavIC update:  %.2fx (spec: ~27x)\n', P11_reduction);
fprintf('  P22 reduction at NavIC update:  %.2fx\n', P22_reduction);

% =============================================================================
% SECTION 10: FIGURE GENERATION (R2024a Styling, 5 Panels)
% =============================================================================

fprintf('\nGenerating 5-panel ESKF figure...\n');

fig2 = figure('Name', 'RUPAK GNC | ESKF Sensor Fusion & Drift Filter', ...
              'NumberTitle', 'off', ...
              'Color', [0.10 0.10 0.14], ...
              'Units', 'normalized', ...
              'OuterPosition', [0.02 0.02 0.96 0.96]);

% --- Colour palette (matching File 1 dark theme) ---
clr_bg    = [0.10 0.10 0.14];
clr_ax    = [0.18 0.18 0.23];
clr_grid  = [0.32 0.32 0.38];
clr_txt   = [0.93 0.93 0.93];
clr_blue  = [0.25 0.60 0.95];
clr_orng  = [0.98 0.62 0.12];
clr_grn   = [0.28 0.87 0.55];
clr_red   = [0.95 0.30 0.28];
clr_prpl  = [0.72 0.40 0.95];
clr_wht   = [0.93 0.93 0.93];

% --- Convert key signals to degrees for display ---
theta_true_deg        = rad2deg(theta_true_hist);
theta_nominal_deg     = rad2deg(theta_nominal_hist);
theta_eskf_deg        = rad2deg(theta_eskf_hist);
delta_theta_deg       = rad2deg(delta_theta_hist);
delta_bias_deg_hr     = rad2deg(delta_bias_hist) * 3600;  % [deg/hr] for display
sigma_theta_deg       = rad2deg(sigma_theta_hist);
sigma_bias_deg_hr     = rad2deg(sigma_bias_hist) * 3600;  % [deg/hr]
true_bias_deg_hr      = rad2deg(bias_true_hist) * 3600;   % [deg/hr]
innovation_deg        = rad2deg(innovation_hist);
innovation_sigma_deg  = rad2deg(innovation_sigma);

% ---------------------------------------------------------------------------
% PANEL 1: Raw vs. Filtered Attitude (deg) vs Time
% ---------------------------------------------------------------------------
ax1 = subplot(5, 1, 1);
plot(t, theta_true_deg,    '-',  'Color', clr_grn,  'LineWidth', 2.0);
hold on;
plot(t, theta_nominal_deg, '--', 'Color', clr_red,   'LineWidth', 1.4);
plot(t, theta_eskf_deg,    '-',  'Color', clr_blue,  'LineWidth', 1.8);
% Mark NavIC update
xline(t_navic, ':', 'Color', clr_orng, 'LineWidth', 1.4, ...
      'Label', 'NavIC @ 5 s', 'LabelVerticalAlignment', 'top', ...
      'FontName', 'Consolas', 'FontSize', 8, 'Color', clr_orng);
hold off;
ylabel('\theta (deg)', 'Color', clr_txt, 'FontName', 'Consolas', 'FontSize', 9);
title({'RUPAK VTVL | Error-State Kalman Filter (ESKF) — Sensor Fusion & Drift Correction'; ...
       sprintf('2-State: [\\delta\\theta, \\delta\\omega_{bias}] | ARW=%.4f deg/√hr | NavIC \\sigma=%.2f deg @ t=5s', ...
               ARW_deg_sqrthr, sigma_navic_deg)}, ...
       'Color', clr_txt, 'FontName', 'Consolas', 'FontSize', 10);
legend({'True Attitude', 'Raw IMU (Drifting)', 'ESKF Estimate'}, ...
       'Location', 'best', 'TextColor', clr_txt, 'Color', clr_ax, ...
       'EdgeColor', clr_grid, 'FontName', 'Consolas', 'FontSize', 8);
xlim([0 t_end]);
grid on;
formatAxis_eskf(ax1, clr_ax, clr_grid, clr_txt);

% Annotate RMS errors
text(0.25, 0.10, sprintf('RMS Error: Raw=%.4f°  ESKF=%.4f°  (%.1fx better)', ...
     rms_raw, rms_eskf, rms_raw/rms_eskf), ...
     'Units', 'normalized', 'Color', clr_grn, 'FontName', 'Consolas', ...
     'FontSize', 7.5, 'BackgroundColor', clr_ax);

% ---------------------------------------------------------------------------
% PANEL 2: Estimated Gyro Bias Drift vs Time
% ---------------------------------------------------------------------------
ax2 = subplot(5, 1, 2);
% Plot true bias (ground truth - normally unknown)
plot(t, true_bias_deg_hr,   '-', 'Color', clr_grn, 'LineWidth', 1.8);
hold on;
% Plot ESKF bias estimate
plot(t, delta_bias_deg_hr, '--', 'Color', clr_blue, 'LineWidth', 1.8);
% ESKF 1-sigma bounds on bias estimate
fill([t, fliplr(t)], ...
     [delta_bias_deg_hr + sigma_bias_deg_hr, fliplr(delta_bias_deg_hr - sigma_bias_deg_hr)], ...
     clr_blue, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
xline(t_navic, ':', 'Color', clr_orng, 'LineWidth', 1.2);
hold off;
ylabel('\omega_{bias} (deg/hr)', 'Color', clr_txt, 'FontName', 'Consolas', 'FontSize', 9);
legend({'True Bias (GT)', 'ESKF Bias Est.', '±1\sigma Bound'}, ...
       'Location', 'best', 'TextColor', clr_txt, 'Color', clr_ax, ...
       'EdgeColor', clr_grid, 'FontName', 'Consolas', 'FontSize', 8);
xlim([0 t_end]);
grid on;
formatAxis_eskf(ax2, clr_ax, clr_grid, clr_txt);

% ---------------------------------------------------------------------------
% PANEL 3: Filter Innovation Residual vs Time
% ---------------------------------------------------------------------------
ax3 = subplot(5, 1, 3);
% The innovation is only defined at NavIC update moments.
% Plot the innovation as a stem/impulse at t=5s.
stem(t(k_navic), rad2deg(innovation_hist(k_navic)), ...
     'filled', 'Color', clr_orng, 'LineWidth', 2.0, 'MarkerSize', 8);
hold on;
% Plot 1-sigma innovation bound (horizontal lines) at update time
errorbar(t(k_navic), rad2deg(innovation_hist(k_navic)), rad2deg(innovation_sigma(k_navic)), ...
         'o', 'Color', clr_red, 'LineWidth', 1.5, 'MarkerSize', 6, ...
         'CapSize', 10);
yline(0, '--', 'Color', clr_txt, 'LineWidth', 0.8, 'Alpha', 0.4);
hold off;
ylabel('\delta z (deg)', 'Color', clr_txt, 'FontName', 'Consolas', 'FontSize', 9);
text(t_navic + 0.1, rad2deg(innovation_hist(k_navic)), ...
     sprintf('  y = %.4f°\n  \\sigma_S = %.4f°', ...
             rad2deg(innovation_hist(k_navic)), rad2deg(innovation_sigma(k_navic))), ...
     'Color', clr_orng, 'FontName', 'Consolas', 'FontSize', 8);
title_str = sprintf('NavIC Innovation Residual (NIS check: y²/S = %.3f, threshold = 1.0)', ...
                    innovation_hist(k_navic)^2 / (innovation_hist(k_navic)^2 + R_navic));
title(title_str, 'Color', clr_txt, 'FontName', 'Consolas', 'FontSize', 7, 'Parent', ax3);
legend({'Innovation y', '±1\sigma bound'}, 'Location', 'best', ...
       'TextColor', clr_txt, 'Color', clr_ax, 'EdgeColor', clr_grid, ...
       'FontName', 'Consolas', 'FontSize', 8);
xlim([0 t_end]);
ylim([-0.3  0.3]);
grid on;
formatAxis_eskf(ax3, clr_ax, clr_grid, clr_txt);

% ---------------------------------------------------------------------------
% PANEL 4: Error Covariance P11 (Attitude Error Variance) with ~27x reduction
% ---------------------------------------------------------------------------
ax4 = subplot(5, 1, 4);
% Plot P11 in deg² for readability
P11_deg2 = rad2deg(sqrt(P11_hist)).^2;   % Convert to deg² units
semilogy(t, P11_deg2, '-', 'Color', clr_blue,  'LineWidth', 2.0);
hold on;
P22_deg2 = rad2deg(sqrt(P22_hist) * 3600).^2;   % deg²/hr² for bias
% Overlay P22 (bias uncertainty) on secondary scale using separate lines
semilogy(t, P22_deg2, '--', 'Color', clr_prpl, 'LineWidth', 1.6);
% Mark the NavIC update with vertical line
xline(t_navic, ':', 'Color', clr_orng, 'LineWidth', 1.4, ...
      'Label', sprintf('NavIC: %.1fx reduction', P11_reduction), ...
      'LabelVerticalAlignment', 'top', 'FontName', 'Consolas', 'FontSize', 8, ...
      'Color', clr_orng);
% Add annotation arrows for pre/post P values
if navic_update_done
    P11_pre_deg2  = rad2deg(sqrt(P_pre_update(1,1)))^2;
    P11_post_deg2 = rad2deg(sqrt(P_post_update(1,1)))^2;
    text(t_navic - 1.5, P11_pre_deg2 * 1.5, ...
         sprintf('P11^{-} = %.2e deg²', P11_pre_deg2), ...
         'Color', clr_red, 'FontName', 'Consolas', 'FontSize', 7.5);
    text(t_navic + 0.15, P11_post_deg2 * 0.5, ...
         sprintf('P11^{+} = %.2e deg²', P11_post_deg2), ...
         'Color', clr_grn, 'FontName', 'Consolas', 'FontSize', 7.5);
    text(t_navic + 0.15, P11_pre_deg2 * 0.8, ...
         sprintf('Reduction: %.1fx', P11_reduction), ...
         'Color', clr_orng, 'FontName', 'Consolas', 'FontSize', 8.5, ...
         'FontWeight', 'bold');
end
hold off;
ylabel('P_{11} (deg²), P_{22} (deg/hr)²', 'Color', clr_txt, ...
       'FontName', 'Consolas', 'FontSize', 8);
legend({'P_{11}: Attitude error var.', 'P_{22}: Bias drift var.'}, ...
       'Location', 'southwest', 'TextColor', clr_txt, 'Color', clr_ax, ...
       'EdgeColor', clr_grid, 'FontName', 'Consolas', 'FontSize', 8);
xlim([0 t_end]);
grid on;
formatAxis_eskf(ax4, clr_ax, clr_grid, clr_txt);
% Y-axis is log-scale via semilogy; add annotation
ax4.YScale = 'log';

% ---------------------------------------------------------------------------
% PANEL 5: 1-Sigma Attitude Error Bounds (Filter Consistency Check)
% ---------------------------------------------------------------------------
ax5 = subplot(5, 1, 5);
% True attitude error (how wrong our estimate actually is)
true_err_deg = rad2deg(theta_eskf_hist - theta_true_hist);
plot(t, true_err_deg, '-', 'Color', clr_wht, 'LineWidth', 1.2);
hold on;
% ESKF 1-sigma bounds (what the filter CLAIMS the error is)
fill([t, fliplr(t)], ...
     [sigma_theta_deg, fliplr(-sigma_theta_deg)], ...
     clr_blue, 'FaceAlpha', 0.20, 'EdgeColor', 'none');
plot(t,  sigma_theta_deg, '-', 'Color', clr_blue, 'LineWidth', 1.0);
plot(t, -sigma_theta_deg, '-', 'Color', clr_blue, 'LineWidth', 1.0);
xline(t_navic, ':', 'Color', clr_orng, 'LineWidth', 1.2);
yline(0, '--', 'Color', clr_txt, 'LineWidth', 0.7, 'Alpha', 0.4);
hold off;
ylabel('\delta\theta (deg)', 'Color', clr_txt, 'FontName', 'Consolas', 'FontSize', 9);
xlabel('Time (s)', 'Color', clr_txt, 'FontName', 'Consolas', 'FontSize', 9);
legend({'True error (\theta_{eskf}-\theta_{true})', '±1\sigma ESKF bound'}, ...
       'Location', 'best', 'TextColor', clr_txt, 'Color', clr_ax, ...
       'EdgeColor', clr_grid, 'FontName', 'Consolas', 'FontSize', 8);
xlim([0 t_end]);
grid on;
formatAxis_eskf(ax5, clr_ax, clr_grid, clr_txt);

% --- Consistency check annotation ---
% Count how many time steps the true error exceeds ±1-sigma (should be ~32%)
outside_sigma = sum(abs(true_err_deg) > sigma_theta_deg);
pct_outside   = outside_sigma / N * 100;
text(0.65, 0.12, sprintf('Outside ±1\\sigma: %.1f%% (ideal ≈ 32%%)', pct_outside), ...
     'Units', 'normalized', 'Color', clr_grn, 'FontName', 'Consolas', ...
     'FontSize', 7.5, 'BackgroundColor', clr_ax, 'Parent', ax5);

% Link axes
linkaxes([ax1 ax2 ax3 ax4 ax5], 'x');

% Overall branding annotation
annotation('textbox', [0.0 0.0 0.40 0.025], ...
           'String', 'RUPAK VTVL | GNC Architecture | ESKF Nav Filter @ 400 Hz | Joseph-Form Update', ...
           'Color', [clr_txt 0.5], 'BackgroundColor', 'none', 'EdgeColor', 'none', ...
           'FontName', 'Consolas', 'FontSize', 7, 'FitBoxToText', 'off');

fprintf('Figure 2 generated: 5-panel ESKF sensor fusion results.\n');
fprintf('\n--- Final ESKF Summary ---\n');
fprintf('  P11 reduction at t=5s : %.2fx (spec: ~27x)\n', P11_reduction);
fprintf('  P22 reduction at t=5s : %.2fx\n', P22_reduction);
fprintf('  Filter consistency    : %.1f%% outside ±1sigma (ideal ~32%%)\n', pct_outside);
fprintf('  Attitude RMS error    : %.6f deg (ESKF) vs %.6f deg (raw)\n', rms_eskf, rms_raw);
fprintf('===========================================================\n');

% =============================================================================
% LOCAL HELPER FUNCTION: formatAxis_eskf
% =============================================================================
% Applies dark-theme axis styling (identical to File 1 helper, scoped here).

function formatAxis_eskf(ax, bg_color, grid_color, text_color)
    ax.Color          = bg_color;
    ax.XColor         = text_color;
    ax.YColor         = text_color;
    ax.GridColor      = grid_color;
    ax.MinorGridColor = grid_color;
    ax.GridAlpha      = 0.5;
    ax.MinorGridAlpha = 0.30;
    ax.FontName       = 'Consolas';
    ax.FontSize       = 8;
    ax.LineWidth      = 0.8;
    ax.Box            = 'on';
    ax.XMinorGrid     = 'on';
    ax.YMinorGrid     = 'on';
    ax.TickDir        = 'out';
    ax.TickLength     = [0.008 0.010];
end

% =============================================================================
% END OF FILE: sensor_fusion_drift_filter.m
% =============================================================================
