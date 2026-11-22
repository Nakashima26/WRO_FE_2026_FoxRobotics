function fit_n30_dynamics(outputDir)
% Fit bounded N30 forward-drive parameters using measured step summaries.
arguments
    outputDir (1,1) string = fullfile(fileparts(mfilename('fullpath')), 'output')
end
if ~isfolder(outputDir), mkdir(outputDir); end
data = readtable(fullfile(fileparts(mfilename('fullpath')), 'n30_drive_summary.csv'), ...
    'TextType', 'string');

% Reserve the mid-PWM observation and the shorter maximum-PWM run.
train = ismember(data.run, ["short100","short180","short255"]);
baseline = [6.51, 22.9, 0.28, 0]; % Existing twin values plus zero delay.
options = optimoptions('lsqnonlin', 'Display', 'off');
[p, ~, ~, exitFlag] = lsqnonlin(@(x) residual(x, data(train,:)), ...
    baseline, [0.1 0 0.02 0], [20 95 1 0.25], options);
assert(exitFlag > 0 && all(isfinite(p)), 'N30 estimation did not converge.');

predicted = prediction(p, data.pwm);
reference = prediction(baseline, data.pwm);
data.split = repmat("holdout", height(data), 1);
data.split(train) = "fit";
data.baseline_speed_mm_s = reference(:,1);
data.pred_speed_mm_s = predicted(:,1);
data.pred_t63_s = predicted(:,2);
data.pred_t90_s = predicted(:,3);
data.speed_error_mm_s = predicted(:,1) - data.speed_mm_s;
data.baseline_t63_s = reference(:,2);
data.baseline_t90_s = reference(:,3);
writetable(data, fullfile(outputDir, 'n30_fit_results.csv'));
writetable(array2table(p, 'VariableNames', ...
    {'gain_mm_s_per_pwm','pwm_intercept','tau_s','delay_s'}), ...
    fullfile(outputDir, 'n30_fit_parameters.csv'));
disp(data)
end

function r = residual(params, data)
% Normalize the heterogeneous residuals with explicit engineering scales.
error = prediction(params, data.pwm) - [data.speed_mm_s, data.t63_s, data.t90_s];
scaled = error ./ [50 0.03 0.05];
r = scaled(:);
end

function response = prediction(params, pwm)
gain = params(1);
intercept = params(2);
tau = params(3);
delay = params(4);
speed = gain .* max(pwm - intercept, 0);
crossingTimes = delay - tau .* log(1 - [0.63 0.90]);
response = [speed, repmat(crossingTimes, numel(pwm), 1)];
end
