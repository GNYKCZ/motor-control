function sensor_adc_sfun(b)
b.NumDialogPrms=1;b.NumInputPorts=2;b.NumOutputPorts=1;b.SetPreCompPortInfoToDefaults;
b.InputPort(1).Dimensions=7;b.InputPort(2).Dimensions=4;b.InputPort(1).DirectFeedthrough=true;b.InputPort(2).DirectFeedthrough=true;
b.OutputPort(1).Dimensions=3;b.SampleTimes=[b.DialogPrm(1).Data.dt 0];b.SimStateCompliance='DefaultSimState';b.RegBlockMethod('Outputs',@output);
end
function output(b)
x=b.InputPort(1).Data;d=b.InputPort(2).Data;c=b.DialogPrm(1).Data;b.OutputPort(1).Data=[sample_adc(x(1:2),d(1:3),b.CurrentTime,c);x(3)];
end
