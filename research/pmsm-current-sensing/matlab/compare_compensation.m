function compare_compensation
% 只从保存的原始结果汇总，不重新运行控制器；失败/提前停机不填零误差。
root=fileparts(fileparts(mfilename('fullpath')));T=readtable(fullfile(root,'results','data','unified_closed_loop.csv'),'TextType','string');
scenarios=unique(T.scenario,'stable');methods=unique(T.algorithm,'stable');rows={};H=nan(numel(methods),numel(scenarios));
for si=1:numel(scenarios)
for mi=1:numel(methods)
 q=T(T.scenario==scenarios(si)&T.algorithm==methods(mi),:);completed=q.trip_code==8;valid=completed&q.diagnostic_envelope_breaches==0;
 rmse=mean(q.dq_error_RMSE_A,'omitnan');H(mi,si)=rmse*1000;
 rows(end+1,:)={scenarios(si),methods(mi),height(q),nnz(completed),nnz(valid),rmse,min(q.dq_error_RMSE_A),max(q.dq_error_RMSE_A),mean(q.iq_tracking_RMSE_A,'omitnan'),max(q.peak_integral_V),mean(q.saturation_fraction),sum(q.diagnostic_envelope_breaches)};
end
end
S=cell2table(rows,'VariableNames',{'scenario','algorithm','runs','completed','completed_without_observed_bound_breach','mean_dq_RMSE_A','min_dq_RMSE_A','max_dq_RMSE_A','mean_tracking_RMSE_A','peak_integral_V','mean_saturation_fraction','bound_breaches'});
writetable(S,fullfile(root,'results','data','comparison_summary.csv'));
f=figure('Visible','off','Position',[100 100 1400 650]);h=heatmap(replace(scenarios,'_',' '),replace(methods,'_',' '),H);h.Title='Active-interval dq error RMS (mA); color capped at 40';h.MissingDataLabel='Stopped';h.ColorLimits=[0 40];exportgraphics(f,fullfile(root,'results','figures','unified_comparison.png'));close(f);
fprintf('COMPARISON_SUMMARY_PASS %d rows from %d saved runs\n',height(S),height(T));
end
