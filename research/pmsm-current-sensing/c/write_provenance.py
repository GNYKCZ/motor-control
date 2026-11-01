"""Record exact local research/firmware inputs and result hashes; no commit or upload."""
from pathlib import Path
import hashlib,json,subprocess,datetime
root=Path(__file__).resolve().parents[1];repo=root.parents[1]
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
sources={str(p.relative_to(repo)):sha(p) for folder in ['matlab','c','docs','models'] for p in (root/folder).rglob('*') if p.is_file() and p.suffix in {'.m','.c','.h','.py','.md','.slx'}}
for p in root.iterdir():
    if p.is_file() and p.suffix in {'.m','.md'}: sources[str(p.relative_to(repo))]=sha(p)
for p in (repo/'control_core').glob('*'):
    if p.is_file() and p.suffix in {'.c','.h'}: sources[str(p.relative_to(repo))]=sha(p)
outputs={str(p.relative_to(root)):sha(p) for p in (root/'results').rglob('*') if p.is_file() and p.name!='provenance.json'}
record={'generated_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'base_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),'branch':subprocess.check_output(['git','branch','--show-current'],cwd=repo,text=True).strip(),'research_committed':False,'environment':'MATLAB R2024b, Windows; exact ver saved by run_all; host C bundled Clang','truth_policy':'Synthetic truth feeds plant/sensor and post-run metrics only, never estimator/controller reference during motor operation','source_sha256':sources,'output_sha256':outputs}
schematic=repo.parent/'A1资料/原理图/原理图.pdf'
if schematic.exists(): record['external_reference_sha256']={str(schematic):sha(schematic)}
(root/'results/data/provenance.json').write_text(json.dumps(record,indent=2,ensure_ascii=False),encoding='utf-8')
print(f'PROVENANCE_WRITTEN {len(sources)} inputs {len(outputs)} outputs')
