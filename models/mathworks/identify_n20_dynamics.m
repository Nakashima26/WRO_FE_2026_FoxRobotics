function identify_n20_dynamics(outputDir)
% Identify a first-order N20 drive from timestamped forward bench runs.
arguments
    outputDir (1,1) string = fullfile(fileparts(mfilename('fullpath')), 'output')
end
if ~isfolder(outputDir), mkdir(outputDir); end
repo = fileparts(fileparts(fileparts(mfilename('fullpath'))));
folder = fullfile(repo, 'logs_tof', 'calib_20261021', 'p5_escalones');
files = dir(fullfile(folder, 'step_p*.csv'));
assert(numel(files) == 9, 'Expected the nine forward N20 step recordings.');
sampleTime = 0.01;
deadZone = 45.5; % Existing N20 calibration, fixed for both models.

data = cell(numel(files),1);
heldOut = false(numel(files),1);
for k = 1:numel(files)
    raw = readtable(fullfile(folder, files(k).name));
    [time, uniqueRows] = unique(raw.t_ms/1000, 'stable');
    assert(all(diff(time) > 0), 'Non-monotonic timestamps in %s.', files(k).name);
    sample = (0:sampleTime:time(end)-time(1))';
    pwm = interp1(time-time(1), raw.pwm(uniqueRows), sample, 'previous');
    speed = interp1(time-time(1), raw.v_mm_s(uniqueRows), sample, 'linear');
    % Prepend zeros to represent the measured motor starting from rest.
    data{k} = iddata([zeros(10,1); speed], ...
        [zeros(10,1); max(pwm-deadZone,0)], sampleTime, 'InterSample', 'zoh');
    heldOut(k) = endsWith(files(k).name, '_r03.csv');
end

train = merge(data{~heldOut});
opts = procestOptions('InitialCondition','zero');
noDelay = procest(train, 'P1', opts);
withDelay = procest(train, 'P1D', opts);
save(fullfile(outputDir, 'n20_identified_models.mat'), 'noDelay', 'withDelay', 'deadZone');
scores = nan(nnz(heldOut),2);
indices = find(heldOut);
fig = figure('Visible','off');
cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
layout = tiledlayout(fig, numel(indices), 1);
for k = 1:numel(indices)
    test = data{indices(k)};
    ax = nexttile(layout);
    time = test.SamplingInstants;
    measured = test.OutputData;
    opts = compareOptions('InitialCondition','z');
    [first, scores(k,1)] = compare(test, noDelay, opts);
    [delayed, scores(k,2)] = compare(test, withDelay, opts);
    plot(ax, time, [measured, first.OutputData, delayed.OutputData]);
    ylabel(ax, 'mm/s');
    title(ax, files(indices(k)).name, 'Interpreter','none');
end
xlabel(ax, 'Time (s)');
legend(ax, 'Measured','First order','First order + delay');
writetable(table(string({files(indices).name})', scores(:,1), scores(:,2), ...
    'VariableNames', {'test_run','first_order_fit_pct','delay_fit_pct'}), ...
    fullfile(outputDir, 'n20_identification_holdout.csv'));
exportgraphics(layout, fullfile(outputDir, 'n20_identification.png'), 'Resolution', 150);
end
