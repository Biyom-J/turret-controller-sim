function p = mc_params(seed)
%MC_PARAMS  Draw one random admissible target for the Monte Carlo study.
%   Rejection-samples until range (800-2000 m), speed (<=150 m/s),
%   acceleration (<=20 m/s^2) and elevation (-10..85 deg) limits hold on
%   0 <= t <= 10 s. Also draws blackout timing and the initial turret angle.
rng(seed);
t_check = 0:0.01:10;
while true
    % Initial geometry and velocity
    r0 = 900 + 1000*rand;   el0 = deg2rad(5 + 55*rand);
    p.x0 = r0*cos(el0);     p.y0 = r0*sin(el0);
    speed = 100*rand;       heading = 2*pi*rand;
    p.vx0 = speed*cos(heading);  p.vy0 = speed*sin(heading);

    % Smooth maneuver: a(t) = A*sin(w*(t - tm)) along a random direction
    A = 20*rand;            dirn = 2*pi*rand;
    p.ax = A*cos(dirn);     p.ay = A*sin(dirn);
    p.w  = 0.5 + 2.5*rand;  p.tm = 1 + 2*rand;

    % Radar blackout (70 % of cases)
    if rand < 0.7, p.tb = 1 + 2*rand; p.dur = 0.5*rand;
    else,          p.tb = inf;        p.dur = 0;  end

    if mc_is_admissible(p, t_check), break; end
end
p.th0 = deg2rad(-10 + 95*rand);              % random initial turret angle
end


function ok = mc_is_admissible(p, t_check)
ok = true;
for k = 1:numel(t_check)
    [x, y, vx, vy, ax, ay] = mc_kin(t_check(k), p);
    r = hypot(x, y);  el = atan2(y, x);
    if r < 800 || r > 2000 || hypot(vx, vy) > 150 || hypot(ax, ay) > 20 ...
            || el < deg2rad(-10) || el > deg2rad(85)
        ok = false; return;
    end
end
end
