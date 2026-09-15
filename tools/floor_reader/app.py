#!/usr/bin/env python3
"""Local-network floor reader prototype. Emits detection only, never motion commands."""
import argparse
import collections
import json
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
DATA = ROOT / 'data'
SEGMENTS = {'0':'abcdef', '1':'bc', '2':'abged', '3':'abgcd',
            '4':'fgbc', '5':'afgcd', '6':'afgecd', '7':'abc',
            '8':'abcdefg', '9':'abfgcd', 'B':'fgecd'}

def pattern(label):
    out = np.zeros((220, 140 * len(label) + 40, 3), np.uint8)
    boxes = {'a':(25,15,85,29), 'b':(85,29,99,95), 'c':(85,109,99,175),
             'd':(25,175,85,189), 'e':(11,109,25,175),
             'f':(11,29,25,95), 'g':(25,95,85,109)}
    for i, char in enumerate(label):
        for seg in SEGMENTS.get(char, ''):
            x1,y1,x2,y2 = boxes[seg]
            cv2.rectangle(out, (x1+20+i*140,y1+10), (x2+20+i*140,y2+10), (240,240,240), -1)
    return out

def normalize(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    if gray.size < 20 or int(gray.max()) - int(gray.min()) < 25:
        return None
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    clean = np.zeros_like(mask)
    for i in range(1, count):
        if stats[i, cv2.CC_STAT_AREA] >= max(3, mask.size * .0008):
            clean[labels == i] = 255
    pts = cv2.findNonZero(clean)
    if pts is None:
        return None
    x,y,w,h = cv2.boundingRect(pts)
    if h < 10 or w < 3:
        return None
    # Preserve aspect ratio: stretching a '1' to full width makes recognition ambiguous.
    scale = min(144 / w, 88 / h)
    small = cv2.resize(clean[y:y+h,x:x+w], (max(1,round(w*scale)),max(1,round(h*scale))), interpolation=cv2.INTER_NEAREST)
    canvas = np.zeros((96,160), np.uint8)
    oy, ox = (96-small.shape[0])//2, (160-small.shape[1])//2
    canvas[oy:oy+small.shape[0], ox:ox+small.shape[1]] = small
    return canvas

def classify(mask, templates):
    if mask is None:
        return 'UNKNOWN', 0., 0.
    scores = []
    for label, refs in templates.items():
        best = 0.
        for ref in refs:
            a, b = mask > 0, ref > 0
            union = np.count_nonzero(a | b)
            best = max(best, np.count_nonzero(a & b) / max(1, union))
        scores.append((best, label))
    scores.sort(reverse=True)
    if len(scores) < 2:
        return 'UNKNOWN', 0., 0.
    score, label = scores[0]
    margin = score - scores[1][0]
    return (label if score >= .70 and margin >= .08 else 'UNKNOWN'), score, margin

class Gate:
    def __init__(self):
        self.reset()
    def reset(self):
        self.history = collections.deque(maxlen=10)
        self.sent = False
    def update(self, label, target, now):
        self.history.append((now, label))
        recent = [v for t,v in self.history if now-t <= 1.5]
        ok = len(recent) == 10 and recent[-1] == target and recent.count(target) >= 8
        if target and ok and not self.sent:
            self.sent = True
            return True
        return False

class Reader:
    def __init__(self, source):
        DATA.mkdir(exist_ok=True)
        self.lock = threading.RLock()
        self.frame = None
        self.mask = None
        self.roi = None
        self.target = '4'
        self.gate = Gate()
        self.last_event = None
        self.seq = 0
        self.status = {'floor':'UNKNOWN', 'error':'카메라 연결 중', 'score':0}
        self.templates = {s:[normalize(pattern(s))] for s in ['B1','B2']+[str(i) for i in range(1,21)]}
        for p in DATA.glob('template_*.png'):
            label = p.stem[len('template_'):]
            ref = cv2.imread(str(p), 0)
            if ref is not None and ref.shape == (96,160):
                self.templates.setdefault(label, []).append(ref)
        cfg = DATA/'config.json'
        if cfg.exists():
            try:
                self.roi = json.loads(cfg.read_text()).get('roi')
            except (ValueError, OSError):
                pass
        self.source = source

    def crop(self, frame):
        if not self.roi:
            return None
        h,w = frame.shape[:2]
        points = np.float32([[x*w,y*h] for x,y in self.roi])
        tw = max(np.linalg.norm(points[1]-points[0]), np.linalg.norm(points[2]-points[3]))
        th = max(np.linalg.norm(points[3]-points[0]), np.linalg.norm(points[2]-points[1]))
        ow,oh = max(16,int(round(float(tw)))), max(16,int(round(float(th))))
        matrix = cv2.getPerspectiveTransform(points, np.float32([[0,0],[ow-1,0],[ow-1,oh-1],[0,oh-1]]))
        return cv2.warpPerspective(frame, matrix, (ow,oh))

    def run(self):
        pipeline = ('nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=30/1 ! '
                    'nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false')
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER) if self.source == 'jetson' else cv2.VideoCapture(self.source)
        last = 0
        while cap.isOpened():
            ok, frame = cap.read()
            if not ok:
                break
            now = time.monotonic()
            if now-last < .1:
                continue
            last = now
            with self.lock:
                self.frame = frame
                try:
                    crop = self.crop(frame)
                    self.mask = normalize(crop) if crop is not None else None
                    label, score, margin = classify(self.mask, self.templates)
                except (cv2.error, ValueError, TypeError) as exc:
                    self.mask = None
                    self.gate.reset()
                    self.status = {'floor':'UNKNOWN', 'score':0, 'updated':time.time(),
                                   'error':'영역 처리 오류: 영역을 다시 지정하세요'}
                    print('Frame processing error: '+str(exc), flush=True)
                    continue
                self.status = {'floor':label, 'score':round(score,3), 'margin':round(margin,3),
                               'error':None, 'updated':time.time()}
                if self.gate.update(label, self.target, now):
                    self.seq += 1
                    event = {'event':'TARGET_FLOOR_DETECTED', 'floor':label, 'seq':self.seq,
                             'timestamp':time.time(), 'motion_authorized':False}
                    self.last_event = event
                    with (DATA/'events.jsonl').open('a') as f:
                        f.write(json.dumps(event)+'\n')
                    print(json.dumps(event), flush=True)
        cap.release()
        with self.lock:
            self.mask = None
            self.frame = None
            self.gate.reset()
            self.status = {'floor':'UNKNOWN', 'score':0, 'error':'카메라 연결 실패 또는 영상 종료'}

PAGE = '''<!doctype html><meta charset="utf-8"><title>층수 인식 테스트</title>
<style>body{background:#101923;color:#eef4fa;font:17px system-ui;max-width:1050px;margin:30px auto;padding:16px}button,input,select{font:inherit;padding:10px;margin:5px;border-radius:7px}canvas{width:100%;background:#000;cursor:crosshair}section{background:#1b2938;padding:18px;margin:16px 0;border-radius:12px}#status{font-size:26px}a{color:#76cfff} .hint{color:#b5c6d8}#mask{width:240px;image-rendering:pixelated}</style>
<h1>엘리베이터 층수 인식</h1><p>영상에서 <b>층수 글자만 들어가는 영역</b>을 좌상 → 우상 → 우하 → 좌하 순서로 클릭하세요.<br>화살표·날짜·작은 글씨는 제외하고, 두 자리 층수도 들어갈 여백을 남기세요.</p>
<canvas id="view" width="1280" height="720"></canvas>
<button onclick="points=[];api('/roi',{points:null})">영역 다시 지정</button><span id="selection"></span>
<section><div id="status">연결 중</div><p id="detail"></p><img id="mask"><p class="hint">흰색 글자 / 검은색 배경 기준 · 카메라 위치가 바뀌면 영역을 다시 지정하세요.</p></section>
<section>목표 층 <input id="target" value="4" size="4"><button onclick="api('/target',{label:document.querySelector('#target').value})">목표 적용 / 감지 재시작</button><p id="event">목표 층 감지 대기</p><b>이 테스트는 하차·로봇 이동 명령을 보내지 않습니다.</b></section>
<section>현재 글자 등록 <input id="label" value="4" size="4"><button onclick="enroll()">현재 카메라 글자를 등록</button><p class="hint">실제 표시창의 각 층을 보여주고 해당 이름으로 등록하면 됩니다. 목표 층 외에 헷갈리는 층도 등록하세요.</p><span id="message"></span></section>
<a href="/test" target="_blank">모니터에 띄울 테스트 표시창 열기 ↗</a>
<p class="hint">제어 연동: GET /api/state · event는 마지막 기록입니다. seq와 timestamp로 중복·오래된 기록을 걸러주세요.</p>
<script>
let points=[], busy=false; const canvas=document.querySelector('#view'),ctx=canvas.getContext('2d');
async function api(path,data){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});const x=await r.json();if(!r.ok)alert(x.error);return x}
canvas.onclick=async e=>{if(points.length===4)return;const r=canvas.getBoundingClientRect();points.push([(e.clientX-r.left)/r.width,(e.clientY-r.top)/r.height]);document.querySelector('#selection').textContent=points.length+'/4';if(points.length===4)await api('/roi',{points})};
async function enroll(){const r=await api('/enroll',{label:document.querySelector('#label').value});document.querySelector('#message').textContent=r.ok?'등록 완료':''}
async function tick(){if(busy)return;busy=true;try{const s=await (await fetch('/api/state')).json();document.querySelector('#status').textContent=s.error||'현재 층: '+s.floor;document.querySelector('#detail').textContent='일치 점수 '+s.score+' · 목표 '+s.target+' · '+(s.roi?'영역 지정됨':'영역 지정 필요');document.querySelector('#event').textContent=s.event?'목표 층 감지 기록: '+s.event.floor+' / '+new Date(s.event.timestamp*1000).toLocaleTimeString():'목표 층 감지 대기'; const im=new Image();im.src='/frame.jpg?t='+Date.now();await im.decode();ctx.drawImage(im,0,0,1280,720); const p=points.length?points:(s.roi||[]);ctx.strokeStyle='#35efb2';ctx.lineWidth=3;ctx.beginPath();p.forEach(([x,y],i)=>{if(i)ctx.lineTo(x*1280,y*720);else ctx.moveTo(x*1280,y*720)});if(p.length===4)ctx.closePath();ctx.stroke();document.querySelector('#mask').src='/mask.png?t='+Date.now()}catch(e){document.querySelector('#detail').textContent='영상 수신 대기 / 연결을 확인하세요'}finally{busy=false}}
setInterval(tick,250);tick();
</script>'''

TEST_PAGE = '''<!doctype html><meta charset="utf-8"><title>테스트 층수 표시창</title>
<style>body{background:#111;color:white;text-align:center;font:20px system-ui;margin:0}header{padding:20px}button,select{font:inherit;padding:10px}img{height:60vh;max-width:95vw;object-fit:contain}p{color:#aaa}</style>
<header>층수 <select id="floor" onchange="show()"></select> <button onclick="document.documentElement.requestFullscreen()">전체 화면</button></header><img id="display"><p>이 숫자를 젯슨 카메라에 보여주세요. 표시창 영역에는 숫자와 검은 배경만 포함하세요.</p>
<script>const f=document.querySelector('#floor');['B2','B1',...Array.from({length:20},(_,i)=>String(i+1))].forEach(v=>f.add(new Option(v,v)));f.value='4';function show(){document.querySelector('#display').src='/pattern.png?floor='+encodeURIComponent(f.value)}show();</script>'''

def handler(reader):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def send(self, body, kind='application/json', code=200):
            if isinstance(body, (dict,list)):
                body = json.dumps(body, ensure_ascii=False).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header('Content-Type', kind+'; charset=utf-8' if kind.startswith('text/') else kind)
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        def do_GET(self):
            url = urlparse(self.path)
            with reader.lock:
                if url.path == '/':
                    return self.send(PAGE, 'text/html')
                if url.path == '/test':
                    return self.send(TEST_PAGE, 'text/html')
                if url.path == '/api/state':
                    status = dict(reader.status)
                    if time.time()-status.get('updated',0)>3 and not status.get('error'):
                        status.update(floor='UNKNOWN', score=0, error='카메라 영상 갱신이 멈췄습니다')
                    return self.send({**status, 'target':reader.target, 'roi':reader.roi,
                                      'event':reader.last_event, 'labels':sorted(reader.templates)})
                if url.path == '/pattern.png':
                    label = parse_qs(url.query).get('floor',['4'])[0].upper()
                    if label not in reader.templates:
                        return self.send({'error':'잘못된 층수'}, code=400)
                    img = pattern(label)
                elif url.path == '/frame.jpg':
                    img = reader.frame
                elif url.path == '/mask.png':
                    img = reader.mask
                else:
                    return self.send({'error':'not found'},code=404)
                if img is None:
                    img = np.zeros((96,160,3),np.uint8)
                ext = '.jpg' if url.path.endswith('.jpg') else '.png'
                _, buf = cv2.imencode(ext, img)
                return self.send(buf.tobytes(), 'image/jpeg' if ext=='.jpg' else 'image/png')
        def do_POST(self):
            try:
                # Reject browser requests originating from another website.
                origin = self.headers.get('Origin')
                if origin and urlparse(origin).netloc != self.headers.get('Host'):
                    return self.send({'error':'origin rejected'},code=403)
                size = int(self.headers.get('Content-Length','0'))
                if not 0 < size <= 8192:
                    raise ValueError('잘못된 요청 크기')
                obj = json.loads(self.rfile.read(size))
                with reader.lock:
                    if self.path == '/roi':
                        p = obj.get('points')
                        if p is not None:
                            a = np.asarray(p,dtype=np.float32)
                            if a.shape != (4,2) or not np.isfinite(a).all() or (a<0).any() or (a>1).any() or not cv2.isContourConvex(a) or abs(cv2.contourArea(a)) < .0005:
                                raise ValueError('네 모서리를 순서대로 넓게 지정하세요')
                        reader.roi = p
                        (DATA/'config.json').write_text(json.dumps({'roi':p}))
                    elif self.path in ('/target','/enroll'):
                        label = str(obj.get('label','')).strip().upper()
                        if not label or len(label)>3 or any(c not in 'B0123456789' for c in label):
                            raise ValueError('층수는 B1, 1, 2처럼 입력하세요')
                        if self.path == '/target':
                            if label not in reader.templates:
                                raise ValueError('해당 층수를 먼저 등록하세요')
                            reader.target = label
                        else:
                            if reader.mask is None or time.time()-reader.status.get('updated',0)>2:
                                raise ValueError('표시창 영역과 카메라 영상을 먼저 확인하세요')
                            cv2.imwrite(str(DATA/('template_'+label+'.png')),reader.mask)
                            reader.templates[label] = [reader.mask.copy()]
                    else:
                        return self.send({'error':'not found'},code=404)
                    reader.gate.reset()
                    reader.last_event = None
                self.send({'ok':True})
            except (ValueError, TypeError, cv2.error) as e:
                self.send({'error':str(e)},code=400)
    return Handler

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source',default='jetson')
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8765)
    args = parser.parse_args()
    reader = Reader(args.source)
    threading.Thread(target=reader.run,daemon=True).start()
    print('Floor reader: http://%s:%s' % (args.host,args.port),flush=True)
    ThreadingHTTPServer((args.host,args.port),handler(reader)).serve_forever()
