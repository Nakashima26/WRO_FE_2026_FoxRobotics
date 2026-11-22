function run_n30_electrical(outputDir)
% Simulate battery sag and motor transients for a sensitivity study.
arguments
    outputDir (1,1) string = fullfile(fileparts(mfilename('fullpath')), 'output')
end
if ~isfolder(outputDir), mkdir(outputDir); end
[ok, info] = fileattrib(outputDir);
assert(ok, 'Could not resolve the output folder.');
outputDir = string(info.Name);
source = string(fileparts(mfilename('fullpath')));
original = pwd;
restore = onCleanup(@() cd(original)); %#ok<NASGU>
cd(source)
sscbuild('foxdrive', '-output', char(fullfile(outputDir, 'foxdrive_lib.slx')));
addpath(source, outputDir);

model = 'fox_n30_supply';
new_system(model);
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
block = @(name) [model '/' name];
add_block('fl_lib/Electrical/Electrical Sources/DC Voltage Source', block('Pack EMF'), ...
    'v0', '11.88', 'Position', [40 175 90 225]);
add_block('ee_lib/Passive/Resistor', block('Pack resistance'), ...
    'R', '0.12', 'Position', [150 145 210 195]);
add_block('fl_lib/Electrical/Electrical Elements/Electrical Reference', block('Ground'), ...
    'Position', [100 270 135 305]);
add_block('nesl_utility/Solver Configuration', block('Solver'), ...
    'Position', [172 260 218 302]);
add_block('foxdrive_lib/Averaged Drive', block('N30 drive'), ...
    'Position', [330 138 460 248]);
add_block('simulink/Sources/From Workspace', block('Command'), ...
    'VariableName', 'driveCommand', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value', 'Position', [140 42 235 75]);
add_block('nesl_utility/Simulink-PS Converter', block('Duty converter'), ...
    'Position', [255 45 292 75]);
add_line(model, 'Command/1', 'Duty converter/1');

simscape.addConnection(block('Pack EMF'), 'p', block('Pack resistance'), 'p');
simscape.addConnection(block('Pack resistance'), 'n', block('N30 drive'), 'p');
simscape.addConnection(block('Pack EMF'), 'n', block('N30 drive'), 'n');
simscape.addConnection(block('Pack EMF'), 'n', block('Ground'), 'V');
simscape.addConnection(block('Pack EMF'), 'n', block('Solver'), 'port');
simscape.addConnection(block('Duty converter'), 'output', block('N30 drive'), 'command');

signals = {'pack_v','pack_a','speed'};
for k = 1:numel(signals)
    converter = ['Convert ' signals{k}];
    logger = ['Log ' signals{k}];
    y = 64 + 70*k;
    add_block('nesl_utility/PS-Simulink Converter', block(converter), ...
        'Position', [500 y 535 y+28]);
    add_block('simulink/Sinks/To Workspace', block(logger), ...
        'VariableName', signals{k}, 'SaveFormat','Timeseries', ...
        'Position', [590 y 660 y+28]);
    simscape.addConnection(block('N30 drive'), signals{k}, block(converter), 'input');
    add_line(model, [converter '/1'], [logger '/1']);
end
workspace = get_param(model, 'ModelWorkspace');
assignin(workspace, 'driveCommand', [0 0; .1 1; 1.4 -1; 2 -1]);
set_param(model, 'Solver', 'ode23t', 'MaxStep', '0.005', 'StopTime', '2', ...
    'ReturnWorkspaceOutputs', 'on');
save_system(model, fullfile(outputDir, [model '.slx']));

fig = figure('Visible','off');
figCleanup = onCleanup(@() close(fig)); %#ok<NASGU>
layout = tiledlayout(fig,3,1);
axesList = [nexttile(layout), nexttile(layout), nexttile(layout)];
rows = zeros(4,7);
index = 0;
for voltage = [11.88, 12.4]
    for stopMode = [-1, 0]
        index = index + 1;
        input = Simulink.SimulationInput(model);
        input = input.setBlockParameter(block('Pack EMF'), 'v0', num2str(voltage));
        input = input.setVariable('driveCommand', ...
            [0 0; .1 1; 1.4 stopMode; 2 stopMode], 'Workspace', model);
        run = sim(input);
        assert(all(isfinite(run.pack_v.Data)) && max(run.pack_a.Data) > 0.5);
        assert(max(abs(run.pack_v.Data + 0.12*run.pack_a.Data - voltage)) < 1e-4, ...
            'Pack voltage and current failed the circuit balance.');
        speed = run.speed.Data;
        [speedTime, uniqueSamples] = unique(run.speed.Time, 'last');
        speedAt200ms = interp1(speedTime, speed(uniqueSamples), 1.6);
        afterStopCommand = speedTime >= 1.4;
        travelAfterCommand = trapz(speedTime(afterStopCommand), speed(uniqueSamples(afterStopCommand)));
        rows(index,:) = [voltage, stopMode, max(run.pack_a.Data), min(run.pack_v.Data), ...
            max(speed), speedAt200ms, travelAfterCommand];
        label = sprintf('%.2f V, %s', voltage, string(modeName(stopMode)));
        for k = 1:numel(signals)
            trace = run.get(signals{k});
            hold(axesList(k), 'on');
            plot(axesList(k), trace.Time, trace.Data, 'DisplayName', label);
        end
    end
end
ylabel(axesList(1), 'Pack voltage (V)');
ylabel(axesList(2), 'Pack current (A)');
ylabel(axesList(3), 'Speed (m/s)');
xlabel(axesList(3), 'Time (s)');
legend(axesList(1), 'Location','best');
writetable(array2table(rows, 'VariableNames', ...
    {'pack_ocv_v','stop_command','peak_pack_a','min_pack_v','peak_speed_m_s', ...
    'speed_200ms_after_command_m_s','travel_600ms_after_command_m'}), ...
    fullfile(outputDir,'n30_electrical_sensitivity.csv'));
exportgraphics(layout, fullfile(outputDir,'n30_electrical.png'), 'Resolution', 150);
end

function mode = modeName(command)
if command == 0
    mode = "short brake";
else
    mode = "coast";
end
end
