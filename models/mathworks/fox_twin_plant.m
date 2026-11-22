function fox_twin_plant(block)
% Ten native vehicle states, followed by encoder fractional and integer counts.
block.NumDialogPrms = 4; % Plant parameters, encoder parameters, initial state, sample time.
block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable','Nontunable'};
block.NumInputPorts = 1;
block.NumOutputPorts = 1;
block.InputPort(1).Dimensions = 3;
block.InputPort(1).DatatypeID = 0;
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).SamplingMode = 'Sample';
block.InputPort(1).DirectFeedthrough = false;
block.OutputPort(1).Dimensions = 12;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';
block.SampleTimes = [block.DialogPrm(4).Data, 0];
block.RegBlockMethod('PostPropagationSetup', @configure_state);
block.RegBlockMethod('InitializeConditions', @initialize);
block.RegBlockMethod('Outputs', @outputs);
block.RegBlockMethod('Update', @advance);
end

function configure_state(block)
block.NumDworks = 1;
block.Dwork(1).Name = 'vehicleAndEncoder';
block.Dwork(1).Dimensions = 12;
block.Dwork(1).DatatypeID = 0;
block.Dwork(1).Complexity = 'Real';
block.Dwork(1).UsedAsDiscState = true;
end

function initialize(block)
block.Dwork(1).Data = block.DialogPrm(3).Data(:);
end

function outputs(block)
block.OutputPort(1).Data = block.Dwork(1).Data;
end

function advance(block)
state = block.Dwork(1).Data;
dt = block.DialogPrm(4).Data;
next = fox_vehicle_step_mex(state(1:10), [dt; block.InputPort(1).Data(:)], ...
    block.DialogPrm(1).Data);
p = block.DialogPrm(2).Data;
acceleration = (next(4) - state(4)) / dt;
slip = p(2) + min(p(3), p(4)*abs(acceleration));
if p(6) > 1e-6
    slip = slip + p(5)*(abs(next(10))/p(6))^2;
end
fraction = state(11) + next(7)*dt*p(1)*max(0, slip);
counts = fix(fraction);
block.Dwork(1).Data = [next; fraction-counts; state(12)+counts];
end
