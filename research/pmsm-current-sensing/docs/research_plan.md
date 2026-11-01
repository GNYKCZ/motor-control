# 执行契约与验收矩阵

## 目标与范围
比较O1在线梯度/RLS、O2测量偏置ESO/物理扰动ESO、O3离线辨识及简单基线，选出有条件有效的方案。MATLAB/Simulink主平台；不部署固件、不发布提交。

## 实施与独立验收
|包|实现|独立判据|证据|
|---|---|---|---|
|A|采样电阻、增益、REF、延迟、ADC、变换|手算123mA/60°、半LSB误差、饱和、线性延迟|run_model_a，SLX对照|
|辨识|M1/M2/M3、LS区间、留出、零序注入、RLS/梯度及门控|秩与反例、训练/留出分离、真实电流不可用于在线更新|run_identification|
|B|实际C核心、事件延迟、平均逆变器、PMSM、保护|C/MATLAB逐步对照、能量、故障复现、步长收敛|run_model_b/run_foc_simulink/run_linear_analysis|
|C|开关/死区/共模/模拟前端/孔径|平均/开关交叉、时间步长、零差模对照|run_model_c/run_switching_crosscheck|
|决策|统一指标、适用域、退出、C接口与Issue2|完整要求逐项审计|comparison_report/final_recommendation/requirements|

明确失败工况保留；有限扫描不证明全局稳定。参照用户附件第13节A-E完成审计，未完成不得标完成。

## 当前停止条件
所有可运行研究继续推进；仅真实硬件实验或外部数据必要时报告缺口。模型不能唯一辨识时仍完成算法/对照/实验设计。
