function [r_meas, theta_meas, meas_valid, x_true, y_true] = target_motion_generator(t, scenario)
x0 = 1400.0; y0 = 500.0; vx0 = -60.0; vy0 = 40.0;
if strcmp(scenario, 'A') || t < 1.0
    x_true = x0 + vx0*t; y_true = y0 + vy0*t;
else
    tau = t - 1.0; w = 1.5;
    x_true = x0 + vx0*t + (8.0/w^2)*(w*tau - sin(w*tau));
    y_true = y0 + vy0*t + (10.0/w^2)*(1.0 - cos(w*tau));
end
r_true = hypot(x_true, y_true); th_true = atan2(y_true, x_true);
meas_valid = ~(strcmp(scenario, 'C') && t >= 1.0 && t <= 1.5);
r_meas = r_true + 1.0*randn;
theta_meas = th_true + deg2rad(0.02)*randn;
end
