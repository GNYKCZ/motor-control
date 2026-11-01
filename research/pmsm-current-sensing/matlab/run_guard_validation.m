function run_guard_validation
root=fileparts(fileparts(mfilename('fullpath')));p=parameters;rng(2810);n=50000;
% Entire population sampled before scoring. Truth used by validator only.
truth=.16*(2*rand(2,n)-1);hat=[.06;.059]+.02*(2*rand(2,n)-1);eb=.003*ones(2,n);
bias=hat+eb.*(2*rand(2,n)-1);raw=round((p.ref+truth+bias)/p.Vref*4096);
allowed=false(1,n);violation=false(1,n);for j=1:n
 allowed(j)=compensation_guard(raw(:,j),hat(:,j),eb(:,j),p,true);
 violation(j)=any(abs([truth(:,j);-sum(truth(:,j))])>128*p.lsb);
end
assert(~any(allowed&violation));assert(any(allowed));
% Exact 50% diagnostic-like zero current admitted only under certificate.
r=round((p.ref+[.06;.059])/p.Vref*4096);[a,b,~]=compensation_guard(r,[.06;.059],[.003;.003],p,true);assert(a);
assert(~compensation_guard(r,[.06;.059],[.003;.003],p,false));
% Same ADC record can conceal actual current if calibration certificate false.
% Deliberately outside assumed envelope: demonstrates limits of proof.
concealed=[.08;.08];sameBias=[.06;.059]-concealed;
assert(max(abs(concealed+sameBias-[.06;.059]))<1e-12);
assert(any(abs([concealed;-sum(concealed)])>128*p.lsb));
T=table(n,nnz(allowed),nnz(allowed&violation),true,'VariableNames',{'cases','allowed','unsafe_allowed_within_contract','unknown_certificate_stops'});
writetable(T,fullfile(root,'results','data','guard_validation.csv'));save(fullfile(root,'results','data','guard_validation.mat'),'T','b','concealed','sameBias');
disp(T);fprintf('GUARD_CONDITIONAL_VALIDATION_PASS\n');
end
