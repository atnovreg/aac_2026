"""Dedicated acados process, emitting observations without changing solver inputs."""
from pathlib import Path
import json,sys,traceback
from .config import simulation_config

def main():
    job=Path(sys.argv[1]).resolve()
    request=json.loads((job/'request.json').read_text())
    from control.acados_mpcc.simulation import run,_json_safe
    frame_index=0
    with (job/'events.jsonl').open('a',buffering=1) as events:
        def emit(event):
            nonlocal frame_index
            if event['type']=='frame':
                event['index']=frame_index;frame_index+=1
            events.write(json.dumps(_json_safe(event),allow_nan=False,separators=(',',':'))+'\n')
        try:
            run(simulation_config(request,job/'runs'),observer=emit,should_cancel=lambda:(job/'cancel').exists())
        except Exception as error:
            traceback.print_exc()
            emit({'type':'error','error':f'{type(error).__name__}: {error}'})
            raise

if __name__=='__main__':main()
