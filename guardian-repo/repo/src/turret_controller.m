function [V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret) %#ok<INUSD>
%TURRET_CONTROLLER  Predictive elevation controller and firing supervisor.
%
%   [V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, ...
%                                       theta_turret, theta_dot_turret)
%
%   Signal flow (called once per simulation step):
%     1. ESTIMATE : 6-state constant-acceleration Kalman filter (Cartesian),
%                   updated only when a NEW radar sample is available, using
%                   the actual elapsed time (blackouts need no special case).
%     2. PREDICT  : extrapolate the estimate to t + tp and convert to an
%                   elevation reference theta_ref.
%     3. CONTROL  : P-only (type-1) compensator with voltage saturation.
%                   Kp is chosen so the ramp lag 1/Kv equals tp, hence the
%                   turret points at the CURRENT target position.
%     4. FIRE     : single-step pulse when the estimated beam miss is small.
%
%   State is kept in a persistent struct and re-initialised whenever time
%   restarts (t <= 0 or t decreases), so consecutive simulations are independent.
%   All design constants live in controller_config() below.

persistent st
cfg = controller_config();

if isempty(st) || t <= 0 || t < st.t_prev
    st = initial_state();
end
st.t_prev = t;

% ---- 1. Estimation ------------------------------------------------------
new_sample = (meas_valid == 1) && (t - st.last_meas_t) >= cfg.Ts - 1e-9;
if new_sample
    st = kalman_step(st, cfg, t, r_meas, theta_meas);
end

if ~st.kf_init                       % nothing to aim at yet
    V_m = 0.0; fire_cmd = 0; st.prev_fire = 0;
    return;
end

% ---- 2. Prediction ------------------------------------------------------
[p_now,  ~] = extrapolate(st, t, 0);          % estimated target now
[p_pred, ~] = extrapolate(st, t, cfg.tp);     % estimated target at t + tp
theta_ref = min(max(atan2(p_pred(2), p_pred(1)), cfg.th_min), cfg.th_max);

% ---- 3. Feedback control ------------------------------------------------
e   = theta_ref - theta_turret;
V_m = min(max(cfg.Kp * e, -cfg.V_max), cfg.V_max);

% ---- 4. Firing supervisor -----------------------------------------------
[fire_cmd, st] = firing_supervisor(st, cfg, t, p_now, theta_turret);
end


%% ========================================================================
%  Configuration and state
%  ========================================================================
function cfg = controller_config()
% Single source of truth for every design constant.
cfg.Ts     = 0.05;              % radar sample interval [s]
cfg.tp     = 0.50;              % prediction horizon [s]
cfg.Kp     = 2000.0;            % P gain [V/rad]; Kv = Kp*Kdc ~ 2 1/s -> lag 0.5 s
cfg.V_max  = 300.0;             % voltage saturation [V]
cfg.th_min = deg2rad(-10.0);    % elevation hard stops [rad]
cfg.th_max = deg2rad(85.0);

% Kalman filter
cfg.q      = 50.0;              % white-jerk spectral density [m^2/s^5]
cfg.sig_r  = 1.0;               % radar range noise std [m]
cfg.sig_th = deg2rad(0.02);     % radar bearing noise std [rad]
cfg.P0     = diag([4, 75^2, 15^2]);   % per-axis initial covariance (|v|<=150, |a|<=20)

% Firing supervisor
cfg.gate     = 1.0;             % max estimated beam miss to fire [m]
cfg.n_min    = 8;               % min fused measurements before firing
cfg.t_rech   = 1.0;             % laser recharge time [s]
end

function st = initial_state()
st.x_hat      = zeros(6, 1);    % [x vx ax y vy ay]'
st.P          = eye(6);
st.kf_init    = false;
st.n_upd      = 0;              % number of fused measurements
st.last_meas_t = -inf;          % time of last accepted measurement
st.t_prev     = 0;
st.t_fire     = -inf;           % time of last issued pulse
st.prev_fire  = 0;
end


%% ========================================================================
%  Estimation
%  ========================================================================
function st = kalman_step(st, cfg, t, r_meas, theta_meas)
% One Kalman cycle for a newly accepted radar sample.
z = [r_meas*cos(theta_meas); r_meas*sin(theta_meas)];
R = measurement_covariance(r_meas, theta_meas, cfg);

if ~st.kf_init
    % First sample: position from measurement, velocity/acceleration unknown.
    st.x_hat   = [z(1); 0; 0; z(2); 0; 0];
    st.P       = blkdiag(cfg.P0, cfg.P0);
    st.kf_init = true;
    st.n_upd   = 1;
else
    d = t - st.last_meas_t;                    % actual elapsed time (handles blackouts)
    F1 = [1 d d^2/2; 0 1 d; 0 0 1];
    F  = blkdiag(F1, F1);
    Q1 = cfg.q * [d^5/20, d^4/8, d^3/6;
                  d^4/8,  d^3/3, d^2/2;
                  d^3/6,  d^2/2, d    ];
    Q  = blkdiag(Q1, Q1);

    % Predict
    st.x_hat = F * st.x_hat;
    st.P     = F * st.P * F' + Q;

    % Update (Joseph form keeps P symmetric positive definite)
    H = [1 0 0 0 0 0;
         0 0 0 1 0 0];
    S = H * st.P * H' + R;
    K = (st.P * H') / S;
    st.x_hat = st.x_hat + K * (z - H * st.x_hat);
    IKH  = eye(6) - K * H;
    st.P = IKH * st.P * IKH' + K * R * K';
    st.P = 0.5 * (st.P + st.P');
    st.n_upd = st.n_upd + 1;
end
st.last_meas_t = t;
end

function R = measurement_covariance(r, th, cfg)
% Linearised polar-to-Cartesian measurement covariance.
c = cos(th); s = sin(th);
var_r  = cfg.sig_r^2;
var_t  = (r * cfg.sig_th)^2;      % cross-range variance from bearing noise
R = [var_r*c^2 + var_t*s^2,  (var_r - var_t)*c*s;
     (var_r - var_t)*c*s,    var_r*s^2 + var_t*c^2];
end


%% ========================================================================
%  Prediction
%  ========================================================================
function [pos, vel] = extrapolate(st, t, horizon)
% Constant-acceleration extrapolation of the estimate to time t + horizon.
dt = (t - st.last_meas_t) + horizon;
x = st.x_hat;
pos = [x(1) + x(2)*dt + 0.5*x(3)*dt^2;
       x(4) + x(5)*dt + 0.5*x(6)*dt^2];
vel = [x(2) + x(3)*dt;
       x(5) + x(6)*dt];
end


%% ========================================================================
%  Firing supervisor
%  ========================================================================
function [fire_cmd, st] = firing_supervisor(st, cfg, t, p_now, theta_turret)
% Fire when the beam (along theta_turret) passes within cfg.gate of the
% estimated current target. fire_cmd is high for ONE step so every pulse is a
% clean 0->1 edge; it can re-fire once the recharge time has elapsed.
along = p_now(1)*cos(theta_turret) + p_now(2)*sin(theta_turret);
miss  = abs(p_now(1)*sin(theta_turret) - p_now(2)*cos(theta_turret));

aligned  = along > 0 && miss <= cfg.gate;
settled  = st.n_upd >= cfg.n_min;
recharged = (t - st.t_fire) >= cfg.t_rech - 1e-9;

if aligned && settled && recharged && st.prev_fire == 0
    fire_cmd  = 1;
    st.t_fire = t;
else
    fire_cmd = 0;
end
st.prev_fire = fire_cmd;
end
