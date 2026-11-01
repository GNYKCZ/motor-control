"""Artifact/data consistency audit; supplements documented manual scope review."""
from pathlib import Path
import csv,json,subprocess
root=Path(__file__).resolve().parents[1];data=root/'results/data';checks=[]
def table(name,n):
    with (data/name).open(encoding='utf-8-sig',newline='') as f: r=list(csv.DictReader(f))
    assert len(r)==n,(name,len(r),n)
    checks.append({'artifact':name,'rows':n});return r
assert all(x['passed']=='1' for x in table('model_a_checks.csv',7))
assert [int(x['rank']) for x in table('identifiability.csv',4)]==[2,1,2,3]
table('algorithm_comparison_A.csv',189);table('eso_comparison.csv',120);table('adaptation_stress.csv',16);table('o3_mismatch.csv',120)
assert all(float(x['spectral_radius'])<1 for x in table('linear_current_loop.csv',54))
f=table('foc_simulink.csv',7);assert [int(x['trip_code']) for x in f]==[3,8,4,8,8,8,8]
u=table('unified_closed_loop.csv',351);assert len({x['algorithm'] for x in u})==9 and len({x['scenario'] for x in u})==13
table('comparison_summary.csv',117);table('o1_closed_loop.csv',78);table('control_boundaries.csv',7)
assert all(x['trip']=='8' for x in table('seed_validation.csv',20))
assert all(float(x['KCL_error_A'])<1e-12 and x['shootthrough']=='0' for x in table('model_c_sweep.csv',24))
assert all(float(x['cycle_mean_difference_A'])<.01 and float(x['step_refinement_A'])<.005 for x in table('switching_crosscheck.csv',2))
assert [x['equal50_PWM_high'] for x in table('phase_features.csv',2)]==['0','1']
it=table('issue2_interface.csv',1)[0];assert float(it['simulink_error'])<1e-12 and float(it['linear_error'])<1e-12
pc=json.loads((data/'portable_c_checks.json').read_text());assert pc['result']=='PASS' and pc['host_cases']==50000 and not pc['production_integration']
for p in ['README.md','START_HERE.m','docs/comparison_report.md','docs/final_recommendation.md','docs/experiment_design.md','docs/stm32_integration.md','docs/issue2_interface.md','models/sensor_bias_model.slx','models/pmsm_foc_model.slx','models/issue2_drive_demo.slx']:
    assert (root/p).is_file() and (root/p).stat().st_size>0,p
log=(root/'results/logs/run_all_20261101_183950.txt').read_text(encoding='utf-8',errors='replace');assert 'REQUESTED_RESEARCH_STAGE_COMPLETE: all' in log
last=(root/'results/logs/final_entry_phase.txt').read_text(encoding='utf-8',errors='replace');assert 'PHASE_FEATURES_PASS' in last and 'FOC_SIMULINK_PASS' in last and 'COMPARISON_SUMMARY_PASS' in last
subprocess.run(['git','diff','--exit-code','--','firmware','control_core'],cwd=root.parents[1],check=True,capture_output=True)
record={'result':'PASS','coverage':'Artifact consistency only; scope and scientific limitations audited in docs/requirements.md','checks':checks,'hardware_tested':False,'production_changed':False}
(data/'delivery_audit.json').write_text(json.dumps(record,indent=2),encoding='utf-8');print('DELIVERY_CONSISTENCY_AUDIT_PASS')
