function view_twin_replay(folder)
% Open the FoxRobotics replay exported from Simulink.
if nargin == 0
    folder = fileparts(mfilename('fullpath'));
end
manifestPath = fullfile(folder, 'replay_manifest.json');
assert(isfile(manifestPath), 'Extract fox_twin_viewer.zip before opening the replay.');
data = jsondecode(fileread(manifestPath));
times = data.times_s(:);
titleText = 'Replay de Simulink';
if isfield(data, 'title')
    titleText = data.title;
end
assert(numel(times) >= 2 && all(diff(times) > 0), 'Invalid replay timeline.');
frame = 1;
started = [];
startTime = 0;
player = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.05, ...
    'TimerFcn', @tick);

bg = [0.08 0.10 0.13];
fg = [0.95 0.96 0.98];
fig = uifigure('Name', ['FoxRobotics | ' titleText], ...
    'Position', [50 50 1120 740], 'Color', bg, 'CloseRequestFcn', @closeViewer);
grid = uigridlayout(fig, [4 2], 'RowHeight', {34,'1x','1x',48}, ...
    'ColumnWidth', {'2x','1x'}, 'BackgroundColor', bg, 'Padding', [16 12 16 12]);
header = uilabel(grid, 'Text', sprintf('FoxRobotics   /   %s   /   Pista %d', titleText, data.seed), ...
    'FontSize', 18, 'FontWeight', 'bold', 'FontColor', fg);
header.Layout.Row = 1;
header.Layout.Column = [1 2];
map = panel('map', 'Pista');
map.Layout.Row = [2 3];
map.Layout.Column = 1;
camera = panel('camera', 'Cámara simulada');
camera.Layout.Row = 2;
camera.Layout.Column = 2;
bev = panel('bev', 'Vista cenital de la cámara (BEV)');
bev.Layout.Row = 3;
bev.Layout.Column = 2;

controls = uigridlayout(grid, [1 5], 'ColumnWidth', {105,82,82,'1x',145}, ...
    'BackgroundColor', bg, 'Padding', [0 0 8 0]);
controls.Layout.Row = 4;
controls.Layout.Column = [1 2];
play = uibutton(controls, 'Text', 'Reproducir', 'ButtonPushedFcn', @toggle);
uibutton(controls, 'Text', 'Anterior', 'ButtonPushedFcn', @(~,~) seek(frame-1));
uibutton(controls, 'Text', 'Siguiente', 'ButtonPushedFcn', @(~,~) seek(frame+1));
slider = uislider(controls, 'Limits', [1 numel(times)], 'Value', 1, ...
    'MajorTicks', [], 'MinorTicks', [], ...
    'ValueChangedFcn', @(~,event) seek(round(event.Value)));
status = uilabel(controls, 'FontColor', fg, 'HorizontalAlignment', 'right');
showFrame();

    function holder = panel(key, titleText)
        holder = uigridlayout(grid, [2 1], 'RowHeight', {24,'1x'}, ...
            'BackgroundColor', bg, 'Padding', [0 0 0 0], 'RowSpacing', 2);
        uilabel(holder, 'Text', titleText, 'FontColor', fg, 'FontWeight', 'bold');
        picture = uiimage(holder, 'ScaleMethod', 'fit');
        holder.UserData = struct('Picture', picture, 'Key', key);
    end

    function showFrame()
        for holder = [map, camera, bev]
            item = holder.UserData;
            item.Picture.ImageSource = fullfile(folder, 'frames', ...
                sprintf('%s_%04d.jpg', item.Key, frame));
        end
        slider.Value = frame;
        status.Text = sprintf('%.2f / %.2f s', times(frame), times(end));
    end

    function stopPlayback()
        stop(player);
        play.Text = 'Reproducir';
    end

    function seek(index)
        stopPlayback();
        frame = min(numel(times), max(1,index));
        showFrame();
    end

    function toggle(~,~)
        if strcmp(player.Running, 'on')
            stopPlayback();
            return
        end
        if frame == numel(times)
            frame = 1;
            showFrame();
        end
        startTime = times(frame);
        started = tic;
        play.Text = 'Pausar';
        start(player);
    end

    function tick(~,~)
        next = find(times <= startTime + toc(started), 1, 'last');
        if next ~= frame
            frame = next;
            showFrame();
        end
        if frame == numel(times)
            stopPlayback();
        end
    end

    function closeViewer(~,~)
        stop(player);
        delete(player);
        delete(fig);
    end
end
