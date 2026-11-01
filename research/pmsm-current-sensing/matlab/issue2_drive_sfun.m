function issue2_drive_sfun(block)
setup(block);
end
function setup(block)
block.NumDialogPrms=1;block.NumInputPorts=1;block.NumOutputPorts=1;
block.SetPreCompInpPortInfoToDynamic;block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=3;block.InputPort(1).DirectFeedthrough=true;block.OutputPort(1).Dimensions=5;
c=block.DialogPrm(1).Data;block.SampleTimes=[c.Ts 0];block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('PostPropagationSetup',@post);block.RegBlockMethod('InitializeConditions',@initial);block.RegBlockMethod('Outputs',@outputs);block.RegBlockMethod('Update',@update);
end
function post(block)
block.NumDworks=1;block.Dwork(1).Name='drive_state';block.Dwork(1).Dimensions=3;block.Dwork(1).DatatypeID=0;block.Dwork(1).Complexity='Real';block.Dwork(1).UsedAsDiscState=true;
end
function initial(block)
block.Dwork(1).Data=zeros(3,1);
end
function outputs(block)
[~,y]=issue2_drive_step(block.Dwork(1).Data,block.InputPort(1).Data,block.DialogPrm(1).Data);block.OutputPort(1).Data=y;
end
function update(block)
[s,~]=issue2_drive_step(block.Dwork(1).Data,block.InputPort(1).Data,block.DialogPrm(1).Data);block.Dwork(1).Data=s;
end
