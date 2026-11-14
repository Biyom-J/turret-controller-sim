function [theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i) %#ok<INUSL>
J_eq = 3200.0; N = 100.0; b = 1500.0; Kt = 10.0; Ke = 10.0; Rm = 1.0; Lm = 0.10;
th_min = deg2rad(-10.0); th_max = deg2rad(85.0);
i_dot = (V_m - Rm*i - N*Ke*theta_dot_turret) / Lm;
theta_ddot_turret = (N*Kt*i - b*theta_dot_turret) / J_eq;
if (theta_turret <= th_min && theta_dot_turret < 0) || (theta_turret >= th_max && theta_dot_turret > 0)
    theta_dot_turret = 0.0;
    if (theta_turret <= th_min && theta_ddot_turret < 0) || (theta_turret >= th_max && theta_ddot_turret > 0)
        theta_ddot_turret = 0.0;
    end
end
end
