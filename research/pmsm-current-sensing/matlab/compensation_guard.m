function [allow,bounds,reason]=compensation_guard(raw,biasHat,errorBound,p,valid)
% Candidate measurement-validity contract ONLY. Not installed in firmware.
% Proof obligation: |bias - biasHat + analog noise + gain error| <= errorBound, plus ADC half LSB.
% Conservative permission only when every possible phase current is in limit.
% Unknown/out-of-domain => STOP, not fallback to ambiguous raw feedback.
allow=false;bounds=inf(3,1);reason="invalid_calibration";
if ~valid || any(~isfinite([raw(:);biasHat(:);errorBound(:)])) || any(errorBound<0),return;end
if any(raw<=0|raw>=4095),reason="adc_saturation";return;end
measured=(raw*p.Vref/4096-p.ref)/(p.shunt*p.gain);
c=measured-biasHat;eps=errorBound+p.lsb/2;
bounds=[abs(c)+eps;abs(sum(c))+sum(eps)];
allow=all(bounds<=128*p.lsb);reason="interval_overlimit";if allow,reason="permitted";end
end
