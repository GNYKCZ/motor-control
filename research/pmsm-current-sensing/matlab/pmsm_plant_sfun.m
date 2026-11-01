function pmsm_plant_sfun(block)
setup(block);
end
function setup(b)
b.NumDialogPrms=1;b.NumInputPorts=1;b.NumOutputPorts=1;b.SetPreCompPortInfoToDefaults;
b.InputPort(1).Dimensions=4;b.InputPort(1).DirectFeedthrough=false;b.OutputPort(1).Dimensions=7;
b.NumContStates=4;b.SampleTimes=[0 0];b.SimStateCompliance='DefaultSimState';
b.RegBlockMethod('InitializeConditions',@init);b.RegBlockMethod('Outputs',@output);b.RegBlockMethod('Derivatives',@deriv);
end
function init(b)
c=b.DialogPrm(1).Data;b.ContStates.Data=[0;0;0;c.omega];
end
function output(b)
b.OutputPort(1).Data=pmsm_outputs(b.ContStates.Data,b.DialogPrm(1).Data);
end
function deriv(b)
u=b.InputPort(1).Data;b.Derivatives.Data=pmsm_rhs(b.ContStates.Data,u(1:3),u(4),b.DialogPrm(1).Data);
end
