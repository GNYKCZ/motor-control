function firmware_controller_sfun(b)
b.NumDialogPrms=1;b.NumInputPorts=1;b.NumOutputPorts=2;b.SetPreCompPortInfoToDefaults;
b.InputPort(1).Dimensions=3;b.InputPort(1).DirectFeedthrough=false;b.OutputPort(1).Dimensions=4;b.OutputPort(2).Dimensions=28;
b.SampleTimes=[b.DialogPrm(1).Data.dt 0];b.SimStateCompliance='DefaultSimState';
b.RegBlockMethod('PostPropagationSetup',@post);b.RegBlockMethod('InitializeConditions',@init);b.RegBlockMethod('Outputs',@output);b.RegBlockMethod('Update',@update);
end
function post(b)
b.NumDworks=1;b.Dwork(1).Name='controller_state';b.Dwork(1).Dimensions=28;b.Dwork(1).DatatypeID=0;b.Dwork(1).Complexity='Real';b.Dwork(1).UsedAsDiscState=true;
end
function init(b),b.Dwork(1).Data=control_initial;end
function output(b),s=b.Dwork(1).Data;b.OutputPort(1).Data=[s(3:5);s(6)];b.OutputPort(2).Data=s;end
function update(b),b.Dwork(1).Data=control_tick(b.Dwork(1).Data,b.InputPort(1).Data,b.DialogPrm(1).Data);end
