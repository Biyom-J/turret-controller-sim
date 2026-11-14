function [r_meas, theta_meas, meas_valid, x, y] = mc_target(t, p)
%MC_TARGET  Radar model for a Monte Carlo target (same interface as
%   target_motion_generator): noisy range/bearing plus blackout flag.
[x, y] = mc_kin(t, p);
meas_valid = ~(t >= p.tb && t <= p.tb + p.dur);
r_meas     = hypot(x, y) + 1.0*randn;
theta_meas = atan2(y, x) + deg2rad(0.02)*randn;
end
