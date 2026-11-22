function fox_twin_commands(block)
% Replay one command per fixed plant sample.
block.NumDialogPrms = 2;
block.DialogPrmsTunable = {'Nontunable','Nontunable'};
block.NumInputPorts = 0;
block.NumOutputPorts = 1;
block.OutputPort(1).Dimensions = 3;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';
block.SampleTimes = [block.DialogPrm(2).Data, 0];
block.RegBlockMethod('Outputs', @command);
end

function command(block)
commands = block.DialogPrm(1).Data;
sample = round(block.CurrentTime / block.DialogPrm(2).Data) + 1;
block.OutputPort(1).Data = commands(min(sample, size(commands,1)),:)';
end
