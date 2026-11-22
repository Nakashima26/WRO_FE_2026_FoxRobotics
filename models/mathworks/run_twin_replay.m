function run_twin_replay(caseFile, outputDir)
% Build a discrete Simulink replay and compare it to the Python twin.
studyDir = fileparts(mfilename('fullpath'));
repo = fileparts(fileparts(studyDir));
if nargin < 2
    outputDir = fullfile(studyDir, 'output');
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end
[ok, info] = fileattrib(outputDir);
assert(ok, 'Could not resolve the output directory.');
outputDir = info.Name;
addpath(studyDir, outputDir);
sources = fullfile(repo, 'src', 'RASPI', 'cam', 'pure_pursuit', 'native', 'src');
mex('-outdir', outputDir, '-output', 'fox_vehicle_step_mex', ...
    fullfile(studyDir, 'fox_vehicle_step_mex.c'), fullfile(sources, 'twin_vehicle.c'));

c = jsondecode(fileread(caseFile));
time = (0:size(c.commands,1)-1)' * c.sample_s;
model = 'fox_twin_replay';
new_system(model);
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
workspace = get_param(model, 'ModelWorkspace');
assignin(workspace, 'actuators', c.commands);
assignin(workspace, 'plantParameters', c.plant_parameters);
assignin(workspace, 'encoderParameters', c.encoder_parameters);
assignin(workspace, 'initialState', c.states(1,:)');
assignin(workspace, 'sampleTime', c.sample_s);
assignin(workspace, 'fieldSegments', c.segments);
assignin(workspace, 'sensorMounts', c.mounts);

add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [model '/Servo PWM direction'], 'FunctionName', 'fox_twin_commands', ...
    'Parameters', 'actuators, sampleTime', 'Position', [35 85 140 125]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [model '/Twin plant and encoder'], 'FunctionName', 'fox_twin_plant', ...
    'Parameters', 'plantParameters, encoderParameters, initialState, sampleTime', ...
    'Position', [215 75 345 135]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [model '/Field center rays'], 'FunctionName', 'fox_twin_ranges', ...
    'Parameters', 'fieldSegments, sensorMounts', 'Position', [390 155 505 210]);
add_block('simulink/Sinks/To Workspace', [model '/State log'], ...
    'VariableName', 'vehicleLog', 'SaveFormat', 'Structure With Time', ...
    'Position', [560 85 645 125]);
add_block('simulink/Sinks/To Workspace', [model '/Range log'], ...
    'VariableName', 'rangeLog', 'SaveFormat', 'Structure With Time', ...
    'Position', [560 163 645 203]);
add_line(model, 'Servo PWM direction/1', 'Twin plant and encoder/1');
add_line(model, 'Twin plant and encoder/1', 'State log/1', 'autorouting', 'on');
add_line(model, 'Twin plant and encoder/1', 'Field center rays/1', 'autorouting', 'on');
add_line(model, 'Field center rays/1', 'Range log/1');
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(c.sample_s, 17), 'StopTime', num2str(time(end), 17), ...
    'ReturnWorkspaceOutputs', 'on');

result = sim(model);
state = result.vehicleLog.signals.values;
ranges = result.rangeLog.signals.values;
assert(isequal(size(state),size(c.states)) && isequal(size(ranges),size(c.ranges)), ...
    'Simulink and Python logs have different dimensions.');
assert(max(abs(result.vehicleLog.time(:) - time)) < 1e-10 && ...
       max(abs(result.rangeLog.time(:) - time)) < 1e-10, ...
    'Simulink and Python logs are not aligned.');
stateError = max(abs(state-c.states), [], 1);
rangeError = max(abs(ranges-c.ranges), [], 1);
assert(all(stateError(1:11) < 1e-6) && stateError(12) == 0, ...
    'Twin plant or encoder has diverged from the Python reference.');
assert(all(rangeError < 1e-6), 'Field ranges have diverged from the Python reference.');
fprintf('Seed %d, %d samples, max continuous error %.3g, max range error %.3g mm.\n', ...
    c.field_seed, numel(time), max(stateError(1:11)), max(rangeError));
save_system(model, fullfile(outputDir, [model '.slx']));

names = {'time_s','x_mm','y_mm','heading_deg','speed_mm_s','servo_deg', ...
    'effective_servo_deg','wheel_mm_s','sliding','lagged_servo_deg','wheel_deg', ...
    'encoder_fraction','encoder_count','yaw_rate_ccw_deg_s', ...
    'left_mm','right_mm','rear_mm','front_mm'};
yawRate = [0; -diff(state(:,3))/c.sample_s];
writetable(array2table([time state yawRate ranges], 'VariableNames', names), ...
    fullfile(outputDir, 'twin_replay.csv'));

fig = figure('Visible', 'off', 'Color', 'white', 'Position', [100 100 900 550]);
figCleanup = onCleanup(@() close(fig)); %#ok<NASGU>
layout = tiledlayout(fig, 1, 2);
ax = nexttile(layout);
hold(ax, 'on');
for k = 1:size(c.segments,1)
    plot(ax, c.segments(k,[1 3]), c.segments(k,[2 4]), 'k-');
end
plot(ax, state(:,1), state(:,2), 'b-', 'LineWidth', 2);
axis(ax, 'equal');
xlabel(ax, 'X (mm)');
ylabel(ax, 'Y (mm)');
title(ax, 'Field and replay trajectory');
ax = nexttile(layout);
plot(ax, time, state(:,[4 7]), 'LineWidth', 1.3);
legend(ax, 'Chassis', 'Wheel');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Speed (mm/s)');
title(ax, 'Encoder divergence during wheel lock');
grid(ax, 'on');
exportgraphics(layout, fullfile(outputDir, 'twin_replay.png'), 'Resolution', 160);
end
