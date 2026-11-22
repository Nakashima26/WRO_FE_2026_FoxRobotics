function fox_twin_ranges(block)
% Deterministic center-ray distances in left, right, rear, front order.
block.NumDialogPrms = 2; % Field segments and validated mounts.
block.DialogPrmsTunable = {'Nontunable','Nontunable'};
block.NumInputPorts = 1;
block.NumOutputPorts = 1;
block.InputPort(1).Dimensions = 12;
block.InputPort(1).DatatypeID = 0;
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).SamplingMode = 'Sample';
block.InputPort(1).DirectFeedthrough = true;
block.OutputPort(1).Dimensions = 4;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';
block.SampleTimes = [-1, 0];
block.RegBlockMethod('Outputs', @measure);
end

function measure(block)
pose = block.InputPort(1).Data;
segments = block.DialogPrm(1).Data;
mounts = block.DialogPrm(2).Data;
start = segments(:,1:2);
edges = segments(:,3:4) - start;
h = deg2rad(pose(3));
ranges = mounts(:,4);
for sensor = 1:size(mounts,1)
    right = mounts(sensor,1);
    forward = mounts(sensor,2);
    origin = [pose(1) + right*cos(h) + forward*sin(h), ...
              pose(2) - right*sin(h) + forward*cos(h)];
    angle = h + deg2rad(mounts(sensor,3));
    perpendicular = [-cos(angle); sin(angle)];
    denom = edges * perpendicular;
    delta = origin - start;
    valid = abs(denom) > 1e-12;
    ray = (edges(valid,1).*delta(valid,2) - edges(valid,2).*delta(valid,1)) ./ denom(valid);
    fraction = (delta(valid,:) * perpendicular) ./ denom(valid);
    hits = ray >= 0 & ray <= ranges(sensor) & fraction >= 0 & fraction <= 1;
    ranges(sensor) = min([ranges(sensor); ray(hits)]);
end
block.OutputPort(1).Data = ranges;
end
