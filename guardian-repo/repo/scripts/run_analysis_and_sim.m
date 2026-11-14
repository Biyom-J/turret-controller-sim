%% RUN_ANALYSIS_AND_SIM  Loop analysis + standard scenarios A, B, C
%  Turret starts at 0 deg (NOT aligned with the target). Plots go to results/.

%% Paths
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'src'), fullfile(root, 'sim'));
res = fullfile(root, 'results');
if ~exist(res, 'dir'), mkdir(res); end

%% Frequency-domain analysis (needs Control System Toolbox / Octave control pkg)
Kp = 2000;
if exist('tf', 'file')
    Gp = tf(1000, [320 3350 1001500 0]);
    L  = Kp * Gp;
    [Gm, Pm, Wcg, Wgc] = margin(L);
    fprintf('Wgc=%.2f rad/s, PM=%.1f deg, GM=%.1f dB (phase crossover %.1f rad/s)\n', ...
            Wgc, Pm, 20*log10(Gm), Wcg);
    disp('Closed-loop poles:');  disp(pole(feedback(L, 1)));
end

%% Standard scenarios
theta0    = 0;                 % initial turret angle [rad]
seed      = 42;
scenarios = {'A', 'B', 'C'};

for j = 1:numel(scenarios)
    sc  = scenarios{j};
    out = simulate_engagement(@(t) target_motion_generator(t, sc), theta0, 10.0, seed);

    fprintf('Scenario %s: Pulses=%d, Hits=%d, FirstHit=%.2f s, MinMiss=%.3f m, max|V|=%.0f V, sat=%.1f%%\n', ...
            sc, out.n_pulses, out.n_hits, out.first_hit_t, out.min_miss, out.max_V, 100*out.sat_frac);
    disp(out.pulses);

    figure(j);
    subplot(3,1,1);
    plot(out.t, rad2deg(out.los), 'k--', out.t, rad2deg(out.theta), 'b');
    grid on; ylabel('Elev. [deg]'); legend('Target LOS', 'Turret'); title(['Scenario ' sc]);
    subplot(3,1,2);
    plot(out.t, 1000*out.err); grid on; ylabel('LOS err [mrad]');
    subplot(3,1,3);
    plot(out.t, out.V); grid on; ylabel('V_m [V]'); xlabel('t [s]'); hold on;
    for k = 1:out.n_pulses
        plot(out.pulses(k,1)*[1 1], ylim, 'r:');     % pulse times
    end
    print(fullfile(res, ['scenario_' sc '.png']), '-dpng');
end
