"""Loopback-only local MPCC tuning UI. Run via src/aac_2026/control/mpcc_tuner/scripts/mpcc_tuner.sh."""
import argparse,json,mimetypes,signal
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse,parse_qs
from functools import lru_cache
from .config import ROOT,catalog,TRACKS
from .assets import map_png,track_data
from .jobs import JobManager,BusyError

STATIC=Path(__file__).parent/'static'
get_track=lru_cache(maxsize=len(TRACKS))(track_data)
get_map=lru_cache(maxsize=len(TRACKS))(map_png)

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def send(self,value,status=200,mime='application/json',filename=None):
        data=json.dumps(value,ensure_ascii=False,allow_nan=False,separators=(',',':')).encode() if mime=='application/json' else value
        self.send_response(status);self.send_header('Content-Type',mime+('; charset=utf-8' if mime.startswith('text/') or mime=='application/json' else ''))
        self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff')
        if filename:self.send_header('Content-Disposition',f'attachment; filename="{filename}"')
        self.end_headers()
        try:self.wfile.write(data)
        except (BrokenPipeError,ConnectionResetError):pass

    def local_request(self):
        host=self.headers.get('Host','');parsed=urlparse('http://'+host)
        if parsed.hostname not in ['127.0.0.1','localhost']:raise PermissionError('localhost에서만 접근할 수 있습니다.')
        origin=self.headers.get('Origin')
        if origin and origin not in ['http://'+host]:raise PermissionError('동일한 로컬 화면에서 요청하세요.')

    def do_GET(self):
        try:
            self.local_request();url=urlparse(self.path);path=url.path;parts=path.strip('/').split('/')
            if path=='/api/catalog':return self.send(catalog())
            if path=='/api/history':return self.send({'jobs':self.server.manager.history()})
            if len(parts)==3 and parts[:2]==['api','track'] and parts[2] in TRACKS:return self.send(get_track(parts[2]))
            if len(parts) in [3,4] and parts[:2]==['api','jobs']:
                after=int(parse_qs(url.query).get('after',['-1'])[0]);data=self.server.manager.get(parts[2],after)
                if len(parts)==3:return self.send(data)
                if parts[3]=='download':return self.send(self.server.manager.get(parts[2],-1),filename=f'mpcc-{parts[2]}.json')
            if path=='/map.png':
                key=parse_qs(url.query).get('track',['both_inner'])[0]
                if key not in TRACKS:raise KeyError('등록된 코스를 선택하세요.')
                return self.send(get_map(key),mime='image/png')
            if path in ['/','/index.html','/app.js','/styles.css']:
                file=STATIC/('index.html' if path=='/' else path[1:]);return self.send(file.read_bytes(),mime=mimetypes.guess_type(file.name)[0] or 'application/octet-stream')
            return self.send({'error':'찾을 수 없습니다.'},404)
        except PermissionError as e:self.send({'error':str(e)},403)
        except (KeyError,FileNotFoundError) as e:self.send({'error':str(e)},404)
        except ValueError as e:self.send({'error':str(e)},400)
        except Exception as e:self.send({'error':f'{type(e).__name__}: {e}'},500)

    def do_POST(self):
        try:
            self.local_request()
            if self.headers.get('Content-Type','').split(';')[0]!='application/json':raise ValueError('JSON 요청만 지원합니다.')
            length=int(self.headers.get('Content-Length','0'))
            if not 0<length<=16384:raise ValueError('요청 크기가 잘못되었습니다.')
            data=json.loads(self.rfile.read(length))
            path=urlparse(self.path).path;parts=path.strip('/').split('/')
            if path=='/api/jobs':return self.send({'id':self.server.manager.start(data)},202)
            if len(parts)==4 and parts[:2]==['api','jobs'] and parts[3]=='cancel':
                self.server.manager.cancel(parts[2]);return self.send({'ok':True})
            return self.send({'error':'찾을 수 없습니다.'},404)
        except PermissionError as e:self.send({'error':str(e)},403)
        except BusyError as e:self.send({'error':str(e)},409)
        except KeyError as e:self.send({'error':str(e)},404)
        except (ValueError,TypeError) as e:self.send({'error':str(e)},400)
        except Exception as e:self.send({'error':f'{type(e).__name__}: {e}'},500)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8765);args=parser.parse_args()
    # Acquire port before inspecting job state, so a second launcher cannot mark an active job interrupted.
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler);server.daemon_threads=True
    try:server.manager=JobManager()
    except Exception:
        server.server_close();raise
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    print(f'MPCC tuner: http://127.0.0.1:{args.port}',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.manager.close();server.server_close()

if __name__=='__main__':main()
