function v = percentile_nearest(x, p)
%PERCENTILE_NEAREST  p-th percentile (nearest rank) ignoring NaNs; no toolbox needed.
x = sort(x(~isnan(x)));
if isempty(x), v = NaN; else, v = x(max(1, ceil(p/100*numel(x)))); end
end
