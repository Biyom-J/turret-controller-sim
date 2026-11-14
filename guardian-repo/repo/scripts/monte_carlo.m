%% MONTE_CARLO  Generalisation study over random admissible engagements
%  Random maneuvers, blackout timing, initial turret angles and noise seeds,
%  all with the same fixed controller. Results go to results/.

%% Paths
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'src'), fullfile(root, 'sim'));
res = fullfile(root, 'results');
if ~exist(res, 'dir'), mkdir(res); end

%% Run cases
N = 200;
success = false(N,1);   t_first = nan(N,1);   min_miss = nan(N,1);
n_hits  = zeros(N,1);   n_pulses = zeros(N,1);  sat_frac = zeros(N,1);

for i = 1:N
    p   = mc_params(i);                                   % target for case i
    out = simulate_engagement(@(t) mc_target(t, p), p.th0, 10.0, 1e6 + i);
    success(i)  = out.n_hits > 0;
    t_first(i)  = out.first_hit_t;     min_miss(i)  = out.min_miss;
    n_hits(i)   = out.n_hits;          n_pulses(i)  = out.n_pulses;
    sat_frac(i) = out.sat_frac;
end

%% Summary
fprintf('Success rate        : %.1f %% (%d/%d)\n', 100*mean(success), sum(success), N);
fprintf('First-hit time  [s] : median %.2f, 90th pct %.2f, max %.2f\n', ...
        percentile_nearest(t_first, 50), percentile_nearest(t_first, 90), max(t_first));
fprintf('Min miss (cases w/ pulse) [m]: median %.2f, 90th pct %.2f\n', ...
        percentile_nearest(min_miss, 50), percentile_nearest(min_miss, 90));
fprintf('Mean pulses/run %.2f, mean hits/run %.2f, mean sat. fraction %.3f\n', ...
        mean(n_pulses), mean(n_hits), mean(sat_frac));

%% Figure and data
figure;
subplot(1,2,1); hist(t_first(~isnan(t_first)), 20);
xlabel('First-hit time [s]'); ylabel('Cases'); grid on;
subplot(1,2,2); hist(min_miss(~isnan(min_miss)), 20);
xlabel('Min miss distance [m]'); ylabel('Cases'); grid on;
print(fullfile(res, 'mc_hist.png'), '-dpng');
save(fullfile(res, 'mc_results.mat'), 'success', 't_first', 'min_miss', 'n_hits', 'n_pulses', 'sat_frac');
