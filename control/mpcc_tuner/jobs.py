"""Single-worker job lifecycle and incremental event persistence."""
from datetime import datetime,timezone
import fcntl,json,os,signal,subprocess,threading,uuid
import yaml
from pathlib import Path
from .config import ROOT,TUNER_PROFILE,validated_request

TERMINAL={'completed','failed','cancelled'}

def save_json(path,value):
    temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False));temporary.replace(path)

class BusyError(Exception): pass

class JobManager:
    def __init__(self,root=None):
        self.root=Path(root or ROOT/'results/건지광장/mpcc_tuner');self.root.mkdir(parents=True,exist_ok=True)
        self.owner_lock=(self.root/'.manager.lock').open('a')
        try:fcntl.flock(self.owner_lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            self.owner_lock.close();raise BusyError('다른 튜너 서버 또는 이전 시뮬레이션이 실행 중입니다.')
        self.lock=threading.RLock();self.process=None;self.active=None;self.cache={}
        # A restarted server does not claim that an interrupted run completed.
        for meta in self.root.glob('*/job.json'):
            self.get(meta.parent.name,10**10)  # Recover unpolled terminal events first.
            data=json.loads(meta.read_text())
            if data['status'] not in TERMINAL:
                data.update(status='failed',error='서버가 종료되어 실행이 중단됐습니다. 로그를 확인하세요.');save_json(meta,data)

    def start(self,request):
        request=validated_request(request)
        with self.lock:
            if self.process is not None and self.process.poll() is None: raise BusyError('현재 실행을 마치거나 중지한 후 다시 실행하세요.')
            if self.active:self.get(self.active,10**10)
            stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ');ident=stamp+'_'+uuid.uuid4().hex[:6]
            job=self.root/ident;job.mkdir()
            save_json(job/'request.json',request)
            meta=dict(id=ident,created_at=datetime.now(timezone.utc).isoformat(),status='building',**request,error=None,run_path=None,summary=None,solver_regularization=TUNER_PROFILE['levenberg_marquardt'])
            save_json(job/'job.json',meta)
            env=os.environ.copy();env['OPENBLAS_NUM_THREADS']='1';env['OMP_NUM_THREADS']='1'
            try:
                with (job/'worker.log').open('w') as log:
                    self.process=subprocess.Popen([str(ROOT/'scripts/acados_python.sh'),'-m','tools.mpcc_tuner.worker',str(job)],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,pass_fds=(self.owner_lock.fileno(),))
            except Exception as e:
                meta.update(status='failed',error=str(e));save_json(job/'job.json',meta);raise
            self.active=ident
            return ident

    def _job(self,ident):
        if not ident or any(c not in '0123456789TZabcdef_' for c in ident): raise KeyError('실행을 찾을 수 없습니다.')
        job=self.root/ident
        if not (job/'job.json').is_file():raise KeyError('실행을 찾을 수 없습니다.')
        return job

    def get(self,ident,after=-1):
        with self.lock:
            job=self._job(ident);meta=json.loads((job/'job.json').read_text());old=dict(meta)
            cache=self.cache.setdefault(ident,{'offset':0,'frames':[]})
            events=job/'events.jsonl'
            if events.exists():
                with events.open() as f:
                    f.seek(cache['offset'])
                    while True:
                        line=f.readline()
                        if not line.endswith('\n'):break
                        cache['offset']=f.tell();event=json.loads(line)
                        if event['type']=='started':meta['run_path']=event['directory']
                        elif event['type']=='frame':
                            cache['frames'].append({k:v for k,v in event.items() if k!='type'})
                            if meta['status'] not in TERMINAL:meta['status']='running'
                        elif event['type']=='finished':
                            summary=event['summary'];meta['summary']=summary;meta['run_path']=event['directory']
                            meta['status']='completed' if summary['completed_lap'] else ('cancelled' if summary['status']=='cancelled' else 'failed')
                            meta['error']=(summary.get('failure') or {}).get('message')
                        elif event['type']=='error':meta.update(status='failed',error=event['error'])
            if ident==self.active and self.process is not None and self.process.poll() is not None and meta['status'] not in TERMINAL:
                meta.update(status='failed',error=f'시뮬레이션 프로세스가 종료됐습니다 (code {self.process.returncode}). worker.log를 확인하세요.')
            if 'solver_regularization' not in meta and meta.get('run_path'):
                config_path=Path(meta['run_path'])/'config.yaml'
                if config_path.exists():meta['solver_regularization']=yaml.safe_load(config_path.read_text()).get('levenberg_marquardt')
            if old!=meta:save_json(job/'job.json',meta)
            return {**meta,'frames':[f for f in cache['frames'] if f['index']>after]}

    def cancel(self,ident):
        with self.lock:
            job=self._job(ident)
            if ident!=self.active or self.process is None or self.process.poll() is not None:return
            (job/'cancel').touch()

    def history(self):
        with self.lock:
            if self.active:self.get(self.active,10**10)
            jobs=[]
            for path in sorted(self.root.glob('*/job.json'),reverse=True)[:50]:
                d=json.loads(path.read_text());jobs.append({k:d[k] for k in ['id','track','status','created_at','weights','speed_max']})
            return jobs

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.cancel(self.active)
            try:self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGTERM)
                try:self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(self.process.pid,signal.SIGKILL);self.process.wait()
        self.owner_lock.close()
