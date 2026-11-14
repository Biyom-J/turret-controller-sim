function [x, y, vx, vy, ax, ay] = mc_kin(t, p)
%MC_KIN  True kinematics of a Monte Carlo target: constant velocity plus a
%   smooth sinusoidal-acceleration maneuver starting at t = p.tm.
x = p.x0 + p.vx0*t;  y = p.y0 + p.vy0*t;
vx = p.vx0;          vy = p.vy0;
ax = 0;              ay = 0;
if t > p.tm
    tau = t - p.tm;  w = p.w;
    x  = x  + (p.ax/w^2)*(w*tau - sin(w*tau));
    y  = y  + (p.ay/w^2)*(w*tau - sin(w*tau));
    vx = vx + (p.ax/w)*(1 - cos(w*tau));
    vy = vy + (p.ay/w)*(1 - cos(w*tau));
    ax = p.ax*sin(w*tau);
    ay = p.ay*sin(w*tau);
end
end
