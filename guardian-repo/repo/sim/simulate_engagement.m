function out = simulate_engagement(target_fn, theta0, t_end, seed, dt)
%SIMULATE_ENGAGEMENT  Closed-loop RK4 simulation of one engagement.
%
%   out = simulate_engagement(target_fn, theta0, t_end, seed [, dt])
%
%   target_fn : handle  [r, theta, valid, x, y] = target_fn(t)
%   theta0    : initial turret elevation [rad]
%   t_end     : simulation length [s]
%   seed      : RNG seed for radar noise
%   dt        : integration/controller step [s] (default 5 ms)
%
%   Output fields
%     t, theta, los, err, V, x, y     time histories (err = los - theta)
%     pulses       [time, miss] for each issued pulse (recharge enforced)
%     n_pulses, n_hits, first_hit_t, min_miss, max_V, sat_frac
if nargin < 5, dt = 0.005; end
R_ENG = 2.0;                                   % engagement radius [m]
T_RECHARGE = 1.0;                              % min inter-pulse time [s]
TH_MIN = deg2rad(-10); TH_MAX = deg2rad(85);   % hard stops

rng(seed);
t_vec = 0:dt:t_end;  n = numel(t_vec);
x_state = struct('theta', theta0, 'omega', 0, 'i', 0);   % turret angle, rate, current
prev_fire = 0;

hist = struct('theta', zeros(1,n), 'los', zeros(1,n), 'V', zeros(1,n), ...
              'x', zeros(1,n), 'y', zeros(1,n));
pulses = zeros(0, 2);

for k = 1:n
    t = t_vec(k);
    [r_m, th_m, valid, xt, yt] = target_fn(t);
    [Vm, fire] = turret_controller(t, r_m, th_m, valid, x_state.theta, x_state.omega);

    hist.theta(k) = x_state.theta;  hist.los(k) = atan2(yt, xt);
    hist.V(k) = Vm;                 hist.x(k) = xt;  hist.y(k) = yt;

    if fire == 1 && prev_fire == 0
        if isempty(pulses) || (t - pulses(end,1)) >= T_RECHARGE - 1e-9
            pulses(end+1, :) = [t, beam_miss(xt, yt, x_state.theta)]; %#ok<AGROW>
        end
    end
    prev_fire = fire;

    x_state = rk4_step(t, Vm, x_state, dt, TH_MIN, TH_MAX);
end

out.t = t_vec;  out.theta = hist.theta;  out.los = hist.los;
out.err = hist.los - hist.theta;  out.V = hist.V;  out.x = hist.x;  out.y = hist.y;
out.pulses = pulses;  out.n_pulses = size(pulses, 1);
hits = pulses(:,2) <= R_ENG;
out.n_hits = sum(hits);
if out.n_hits > 0, out.first_hit_t = pulses(find(hits, 1), 1); else, out.first_hit_t = NaN; end
if isempty(pulses), out.min_miss = NaN; else, out.min_miss = min(pulses(:,2)); end
out.max_V = max(abs(hist.V));
out.sat_frac = mean(abs(hist.V) >= 299.99);
end


function miss = beam_miss(xt, yt, theta)
% Distance from target to the laser ray (origin, direction theta).
if xt*cos(theta) + yt*sin(theta) > 0
    miss = abs(xt*sin(theta) - yt*cos(theta));
else
    miss = hypot(xt, yt);                      % target behind the turret
end
end


function s = rk4_step(t, Vm, s, dt, th_min, th_max)
% One RK4 step of turret_plant with hard-stop handling (angle clamp + rate zeroing).
[k1a,k1b,k1c] = turret_plant(t,      Vm, s.theta,              s.omega,              s.i);
[k2a,k2b,k2c] = turret_plant(t+dt/2, Vm, s.theta+dt*k1a/2,     s.omega+dt*k1b/2,     s.i+dt*k1c/2);
[k3a,k3b,k3c] = turret_plant(t+dt/2, Vm, s.theta+dt*k2a/2,     s.omega+dt*k2b/2,     s.i+dt*k2c/2);
[k4a,k4b,k4c] = turret_plant(t+dt,   Vm, s.theta+dt*k3a,       s.omega+dt*k3b,       s.i+dt*k3c);

theta = s.theta + (dt/6)*(k1a + 2*k2a + 2*k3a + k4a);
s.omega = s.omega + (dt/6)*(k1b + 2*k2b + 2*k3b + k4b);
s.i     = s.i     + (dt/6)*(k1c + 2*k2c + 2*k3c + k4c);
s.theta = min(max(theta, th_min), th_max);
if (s.theta <= th_min && s.omega < 0) || (s.theta >= th_max && s.omega > 0)
    s.omega = 0;                               % hard stop absorbs velocity
end
end
