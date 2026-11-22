function run_braking_study(outputDir)
% Reproduce the N30 straight-line braking study in MATLAB + Simulink.
studyDir = fileparts(mfilename('fullpath'));
if nargin == 0
    outputDir = fullfile(studyDir, 'output');
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end
[ok, info] = fileattrib(outputDir);
assert(ok, 'Could not resolve the output directory.');
outputDir = info.Name;

data = readtable(fullfile(studyDir, 'n30_measurements.csv'), 'TextType', 'string');
coastFit = data.mode == "coast" & data.pwm == 255;
coastDecel = data.initial_mm_s(coastFit)^2 / (2 * data.encoder_stop_mm(coastFit));
shortDecel = 5000; % Existing N30 twin parameter (low-speed, non-locking regime).
profileDecel = 3000; % Commanded planning assumption, not a measured response.
highSpeed = data.mode == "short" & data.pwm == 255;
slipGap = data.tape_stop_mm(highSpeed) - data.encoder_stop_mm(highSpeed);
fprintf('High-speed short brake: approximately %.0f mm absent from encoder odometry.\n', slipGap);

model = 'fox_n30_braking';
build_model(model);
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
save_system(model, fullfile(outputDir, [model '.slx']));
lowShort = data.mode == "short" & data.pwm == 100;
coastHoldout = data.mode == "coast" & data.pwm == 150;
role = repmat("outside_model", height(data), 1);
role(lowShort) = "reference_low_speed";
role(coastHoldout) = "holdout";
role(coastFit) = "calibration";
data.role = role;
validation = data(role ~= "outside_model", :);
validation.decel_mm_s2 = repmat(coastDecel, height(validation), 1);
validation.decel_mm_s2(validation.mode == "short") = shortDecel;
validation.simulated_mm = zeros(height(validation), 1);
for k = 1:height(validation)
    validation.simulated_mm(k) = simulate_distance(model, validation.initial_mm_s(k), ...
        0, validation.decel_mm_s2(k));
end
validation.error_mm = validation.simulated_mm - validation.encoder_stop_mm;
writetable(validation, fullfile(outputDir, 'bench_comparison.csv'));
disp(validation);

% Scenario sweep for reducing straight-line speed to a nominal curve entry.
% Reaction distance assumes unchanged speed during the delay.
[start, delay, modeIndex] = ndgrid([1000 1250 1500], [0 0.10 0.20], 1:2);
strategy = ["coast"; "profile_3000"];
rates = [coastDecel; profileDecel];
sweep = table(strategy(modeIndex(:)), start(:), repmat(500, numel(start), 1), ...
    delay(:), rates(modeIndex(:)), 'VariableNames', ...
    {'strategy','initial_mm_s','entry_mm_s','delay_s','decel_mm_s2'});
sweep.active_mm = zeros(height(sweep), 1);
for k = 1:height(sweep)
    activeDistance = simulate_distance(model, sweep.initial_mm_s(k), ...
        sweep.entry_mm_s(k), sweep.decel_mm_s2(k));
    sweep.active_mm(k) = activeDistance;
end
sweep.total_mm = sweep.active_mm + sweep.initial_mm_s .* sweep.delay_s;
writetable(sweep, fullfile(outputDir, 'curve_entry_sweep.csv'));

fig = figure('Visible', 'off', 'Color', 'white', 'Position', [100 100 850 470]);
figCleanup = onCleanup(@() close(fig)); %#ok<NASGU>
ax = axes(fig);
hold(ax, 'on');
for k = 1:numel(strategy)
    rows = sweep.strategy == strategy(k) & sweep.delay_s == 0.10;
    plot(ax, sweep.initial_mm_s(rows), sweep.total_mm(rows), '-o', ...
        'LineWidth', 2, 'DisplayName', strategy(k));
end
xlabel(ax, 'Approach speed (mm/s)');
ylabel(ax, 'Distance to reach 500 mm/s (mm)');
title(ax, 'N30 braking scenarios with 100 ms assumed delay');
legend(ax, 'Location', 'northwest', 'Interpreter', 'none');
grid(ax, 'on');
exportgraphics(ax, fullfile(outputDir, 'curve_entry.png'), 'Resolution', 180);
end

function build_model(model)
new_system(model);
add_block('simulink/Sources/Constant', [model '/Negative acceleration'], ...
    'Value', '-deceleration', 'Position', [40 71 105 99]);
add_block('simulink/Continuous/Integrator', [model '/Speed mm_s'], ...
    'InitialCondition', 'initialSpeed', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'inf', 'LowerSaturationLimit', 'entrySpeed', ...
    'Position', [160 65 200 105]);
add_block('simulink/Continuous/Integrator', [model '/Distance mm'], ...
    'Position', [265 65 305 105]);
add_block('simulink/Sinks/To Workspace', [model '/Distance log'], ...
    'VariableName', 'travel', 'SaveFormat', 'Timeseries', ...
    'Position', [365 70 450 100]);
add_line(model, 'Negative acceleration/1', 'Speed mm_s/1');
add_line(model, 'Speed mm_s/1', 'Distance mm/1');
add_line(model, 'Distance mm/1', 'Distance log/1');
workspace = get_param(model, 'ModelWorkspace');
assignin(workspace, 'initialSpeed', 1500);
assignin(workspace, 'entrySpeed', 500);
assignin(workspace, 'deceleration', 3000);
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', '0.001', 'StopTime', '0.334', ...
    'ReturnWorkspaceOutputs', 'on');
end

function distance = simulate_distance(model, initialSpeed, entrySpeed, deceleration)
duration = (initialSpeed - entrySpeed) / deceleration;
input = Simulink.SimulationInput(model);
input = input.setVariable('initialSpeed', initialSpeed, 'Workspace', model);
input = input.setVariable('entrySpeed', entrySpeed, 'Workspace', model);
input = input.setVariable('deceleration', deceleration, 'Workspace', model);
step = duration / ceil(duration / 0.001);
input = input.setModelParameter('StopTime', num2str(duration, 17), ...
    'FixedStep', num2str(step, 17));
result = sim(input);
distance = result.travel.Data(end);
expected = (initialSpeed^2 - entrySpeed^2) / (2 * deceleration);
assert(abs(distance - expected) < 3, 'Simulink and analytic distances differ by 3 mm or more.');
end
