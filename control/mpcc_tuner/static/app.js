(() => {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const terminal = new Set(['completed', 'failed', 'cancelled']);
  const statusLabels = {building:'solver 준비 중',running:'주행 계산 중',finalizing:'결과 저장 중',completed:'완주',failed:'실행 실패',cancelled:'사용자 중단'};
  const state = {
    catalog:null, track:null, image:null, imageURL:null, job:null, frames:[], rows:[],
    selected:0, follow:true, playing:false, playRequest:null, pollTimer:null,
    polling:false, pollErrors:0, submitting:false, loading:false, trackToken:0,
    viewport:{width:1,height:1,cx:0,cy:0,scale:1}, defaultsApplied:false,
  };
  const canvas = $('map-canvas');
  const ctx = canvas.getContext('2d');
  const controls = new Map();
  const isActive = () => !!state.job && !terminal.has(state.job.status);
  const number = (v, digits=2) => Number.isFinite(Number(v)) && v !== null && v !== undefined ? Number(v).toFixed(digits) : '—';
  const compact = (v) => {
    const n = Number(v);
    if (!Number.isFinite(n)) return '—';
    if (n === 0) return '0';
    return Math.abs(n) < 0.0001 || Math.abs(n) >= 100000 ? n.toExponential(2) : String(Number(n.toPrecision(5)));
  };
  function errorMessage(message, reconnect=false) {
    $('error-text').textContent = message;
    $('error-banner').hidden = false;
    $('reconnect-button').hidden = !reconnect;
  }
  function clearError() { $('error-banner').hidden = true; }
  async function api(path, options={}) {
    const response = await fetch(path, {...options, headers:{'Content-Type':'application/json',...options.headers}});
    let data;
    try { data = await response.json(); } catch (_) { throw new Error('서버 응답을 읽지 못했습니다. 로컬 서버가 실행 중인지 확인하세요.'); }
    if (!response.ok) throw new Error(data.error || `요청 실패 (${response.status})`);
    return data;
  }
  function setBadge(status, text) {
    $('status-badge').dataset.status = status;
    $('status-badge').textContent = text || statusLabels[status] || '실행 준비';
    $('live-dot').classList.toggle('active', ['building','running','finalizing'].includes(status));
  }
  function updateButtons() {
    const busy = isActive() || state.submitting;
    $('run-button').disabled = busy || !state.track || state.loading;
    $('run-button').innerHTML = busy ? '<span>◌</span> 시뮬레이션 진행 중' : '<span>▶</span> 시뮬레이션 실행';
    $('track-select').disabled = busy || !state.catalog || state.loading;
    $('cancel-button').hidden = !isActive();
    $('cancel-button').disabled = state.job?.cancelRequested || false;
    $('draft-notice').textContent = isActive() ? '지금 바꾸는 값은 다음 실행에 적용됩니다.' : '입력값은 다음 실행에 적용됩니다.';
    $('play-button').disabled = state.frames.length < 2;
    $('timeline').disabled = state.frames.length < 2;
    $('live-button').hidden = !isActive();
    document.querySelectorAll('[data-history-id]').forEach(button => { button.disabled = busy || state.loading; });
  }
  function fromSlider(field, value) {
    const t = value / 1000;
    if (field.scale !== 'log') return field.min + t * (field.max - field.min);
    if (field.min === 0 && value === 0) return 0;
    const low = field.min > 0 ? field.min : Math.min(field.default > 0 ? field.default / 1000 : 1e-8, field.max / 1e6);
    return 10 ** (Math.log10(low) + t * (Math.log10(field.max) - Math.log10(low)));
  }
  function toSlider(field, value) {
    if (field.scale !== 'log') return (value - field.min) / (field.max - field.min) * 1000;
    if (value <= 0) return 0;
    const low = field.min > 0 ? field.min : Math.min(field.default > 0 ? field.default / 1000 : 1e-8, field.max / 1e6);
    return (Math.log10(value)-Math.log10(low))/(Math.log10(field.max)-Math.log10(low))*1000;
  }
  function updateTerminalCost() {
    const qc = Number(controls.get('contour')?.input.value);
    const multiplier = Number(controls.get('terminal_contour_multiplier')?.input.value);
    $('terminal-cost').textContent = `q꜀,N = ${compact(qc * multiplier)}`;
  }
  function createFields() {
    for (const field of state.catalog.fields) {
      const block = document.createElement('div'); block.className = 'weight-field';
      const topline = document.createElement('div'); topline.className = 'weight-topline';
      const symbol = document.createElement('span'); symbol.className='weight-symbol'; symbol.textContent=field.symbol;
      const label = document.createElement('label'); label.className='weight-label'; label.htmlFor=`weight-${field.key}`; label.textContent=field.label; label.title=field.description;
      const input = document.createElement('input');
      Object.assign(input, {id:`weight-${field.key}`,name:field.key,type:'number',min:field.min,max:field.max,step:'any',value:field.default,required:true,className:'weight-number'});
      input.setAttribute('aria-label', `${field.label} 숫자 입력`);
      const slider = document.createElement('input');
      Object.assign(slider,{type:'range',min:0,max:1000,step:1,value:toSlider(field,field.default)});
      slider.setAttribute('aria-label',`${field.label} 슬라이더`);
      const help = document.createElement('p'); help.className='weight-help'; help.textContent=field.description;
      topline.append(symbol,label,input); block.append(topline,slider,help);
      (field.group === 'advanced' ? $('advanced-fields') : $('main-fields')).append(block);
      controls.set(field.key,{field,input,slider});
      slider.addEventListener('input', () => {
        input.value=Number(fromSlider(field,Number(slider.value)).toPrecision(4));
        input.setCustomValidity(''); updateTerminalCost();
      });
      input.addEventListener('input', () => {
        const v=Number(input.value);
        if (input.value !== '' && Number.isFinite(v)) slider.value=toSlider(field,v);
        updateTerminalCost();
      });
    }
  }
  function applySettings(settings) {
    if (!settings) return;
    for (const [key,control] of controls) {
      const value=settings.weights?.[key];
      if (value !== undefined && value !== null) {
        control.input.value=value; control.slider.value=toSlider(control.field,Number(value));
      }
    }
    if (settings.speed_max !== undefined) { $('speed-input').value=settings.speed_max; $('speed-range').value=settings.speed_max; }
    updateSpeed(); updateTerminalCost();
  }
  function updateSpeed() {
    const speed=Number($('speed-input').value);
    $('speed-kmh').textContent=`${number(speed*3.6,1)} km/h · ${number(speed/0.44704,1)} mph`;
  }
  function readSettings() {
    const weights={};
    for (const [key,{input,field}] of controls) {
      const value=Number(input.value);
      if (input.value === '' || !Number.isFinite(value) || value < field.min || value > field.max) throw new Error(`${field.label}: ${field.min} ~ ${field.max} 범위의 숫자를 입력하세요.`);
      weights[key]=value;
    }
    const speed=Number($('speed-input').value);
    const limits=state.catalog.speed_limits;
    if (!Number.isFinite(speed) || speed < limits.min || speed > limits.max) throw new Error(`속도 상한은 ${limits.min} ~ ${limits.max} m/s 범위로 입력하세요.`);
    return {track:$('track-select').value,speed_max:speed,weights};
  }
  function resetReplay() {
    stopPlayback(); state.frames=[]; state.rows=[]; state.selected=0; state.follow=true;
    $('timeline').max=0; $('timeline').value=0;
    $('download-link').hidden=true; updatePlayback();
  }
  async function loadTrack(id) {
    const token=++state.trackToken;
    state.loading=true; $('map-loading').hidden=false; updateButtons();
    try {
      const track=await api(`/api/track/${encodeURIComponent(id)}`);
      if (token !== state.trackToken) return;
      if (state.imageURL !== track.map.url) {
        const image=new Image();
        await new Promise((resolve,reject) => { image.onload=resolve; image.onerror=()=>reject(new Error('지도 이미지를 불러오지 못했습니다.')); image.src=track.map.url; });
        if (token !== state.trackToken) return;
        state.image=image; state.imageURL=track.map.url;
      }
      state.track=track;
      $('track-description').textContent=`건지광장 · ${track.label} · 중심선 ${number(track.length_m,1)} m`;
      $('map-loading').hidden=true; fitMap();
      if (!state.job) showBaseline();
    } finally { if (token === state.trackToken) { state.loading=false; updateButtons(); } }
  }
  function showBaseline() {
    setBadge('ready','실행 준비'); $('map-mode').textContent='기준 주행'; $('map-progress').textContent='';
    $('run-description').textContent=`${state.catalog.baseline_label || '기본 가중치의 저장된 주행 결과'}를 표시합니다. 가중치를 바꾸고 실행하면 같은 코스에서 비교할 수 있습니다.`;
    $('run-settings').replaceChildren();
    addSettingChips(state.track.baseline?.weights,4,'기준');
    renderMetrics(state.track.baseline?.summary || null,true);
    updatePlayback(); draw();
  }
  function addSettingChips(weights,speed,prefix='실행') {
    $('run-settings').replaceChildren();
    if (!weights) return;
    const items=[`${prefix} q꜀ ${compact(weights.contour)}`,`qₗ ${compact(weights.lag)}`,`진행 보상 ${compact(weights.progress)}`,`조향 변화 ${compact(weights.increment_steering)}`,`속도 상한 ${compact(speed)} m/s`];
    items.forEach(text => { const chip=document.createElement('span'); chip.className='setting-chip'; chip.textContent=text; $('run-settings').append(chip); });
  }
  function setMetric(id,value,unit,digits=2) { $(id).replaceChildren(document.createTextNode(number(value,digits))); const small=document.createElement('small'); small.textContent=unit; $(id).append(small); }
  function renderMetrics(summary, baseline=false) {
    const rows=state.rows;
    const live=!summary && rows.length;
    const offsets=live ? rows.map(r=>Math.abs(r[11])).filter(Number.isFinite) : [];
    const times=live ? rows.map(r=>r[14]*1000).filter(Number.isFinite) : [];
    const violations=live ? rows.map(r=>r[12]).filter(Number.isFinite) : [];
    const progress=state.frames.at(-1)?.progress_m;
    const complete=summary?.completed_lap === true;
    $('time-label').textContent=complete ? (baseline?'기준 완주 시간':'완주 시간') : '시뮬레이션 시간';
    const lastTime=state.frames.at(-1)?.time_s;
    setMetric('metric-time',summary?.simulation_time_s ?? lastTime,'s');
    $('metric-time-detail').textContent=complete ? '차량의 실제 진행 기준 완주' : Number.isFinite(progress) ? `${number(progress,1)} / ${number(state.track?.length_m,1)} m 진행` : '실행 후 표시';
    const maxError=summary?.max_abs_lateral_offset_m ?? (offsets.length?Math.max(...offsets):null);
    const rms=summary?.rms_lateral_offset_m ?? (offsets.length?Math.sqrt(offsets.reduce((sum,v)=>sum+v*v,0)/offsets.length):null);
    setMetric('metric-error',maxError == null?null:maxError*100,'cm',1);
    $('metric-rms').textContent=`RMS ${number(rms == null?null:rms*100,2)} cm`;
    const violation=summary?.max_corridor_violation_m ?? (violations.length?Math.max(...violations):null);
    setMetric('metric-boundary',violation == null?null:violation*100,'cm',2);
    $('metric-boundary').parentElement.classList.toggle('alert',violation>0.002);
    const maxTime=summary?.controller_cycle_time_ms?.max ?? (times.length?Math.max(...times):null);
    setMetric('metric-timing',maxTime,'ms',2);
    $('metric-timing').parentElement.classList.toggle('alert',maxTime>20);
    const misses=summary?.controller_deadline_misses ?? (times.length?times.filter(t=>t>20).length:null);
    $('metric-deadline').textContent=`20 ms 초과 ${misses == null?'—':misses}회${live?' · 집계 중':''}`;
  }
  function renderJob() {
    const job=state.job;
    if (!job) return;
    let label=statusLabels[job.status];
    if (job.status === 'completed' && job.summary?.completed_lap !== true) label='실행 종료 · 완주 미확인';
    setBadge(job.status,label);
    $('map-mode').textContent=isActive()?'시뮬레이션 진행 중':'시뮬레이션 결과';
    const frame=state.frames.at(-1);
    $('map-progress').textContent=frame ? `${number(frame.progress_m,1)} / ${number(state.track.length_m,1)} m` : '';
    const descriptions={building:'현재 입력값을 저장하고 acados solver를 준비하고 있습니다.',running:'차량 동역학을 계산하고 있습니다. 파란 선은 차량 궤적, 주황 선은 각 제어 시점의 예측 경로입니다.',finalizing:'주행 계산이 끝났습니다. 로그와 결과를 저장하고 있습니다.',completed:'저장된 시뮬레이션 결과입니다. 시간 막대로 이동하거나 재생해서 코너별 주행과 예측을 확인하세요.',cancelled:'실행을 중단했습니다. 중단 전까지의 궤적과 설정은 보존됩니다.',failed:'시뮬레이션이 정상 완주 전에 종료됐습니다. 남아 있는 궤적과 오류를 확인하세요.'};
    $('run-description').textContent=descriptions[job.status] || job.status;
    if (job.error) {
      const summary=job.summary;
      if (summary?.failure?.message?.startsWith('Physical constraint violation')) {
        const corridor=Number(summary.max_corridor_violation_m || 0);
        $('run-description').textContent+=corridor>0
          ? ` 차량 궤적의 경계 초과가 검출되어 중단했습니다 (최대 ${number(corridor*100,2)} cm). 출발 계산 실패와는 다른 원인입니다.`
          : ' 차량 상태 또는 입력 한계를 초과하여 중단했습니다.';
      } else if (summary?.nonzero_solver_status_count>0) {
        $('run-description').textContent+=' 최적화 계산이 정상 해를 반환하지 않아 중단했습니다. 실패 명령은 차량에 적용하지 않았습니다.';
      }
      $('run-description').textContent+=` 상세: ${job.error}`;
    }
    if (job.run_path && terminal.has(job.status)) $('run-description').textContent+=` 저장 위치: ${job.run_path}`;
    if (job.solver_regularization != null && job.solver_regularization !== state.catalog.solver_profile?.levenberg_marquardt) {
      $('run-description').textContent+=' 이 기록은 이전 계산 설정입니다. 현재 기준 결과와 가중치만 다른 비교가 아닙니다.';
    }
    addSettingChips(job.weights,job.speed_max);
    $('download-link').hidden=!terminal.has(job.status);
    $('download-link').href=`/api/jobs/${encodeURIComponent(job.id)}/download`;
    renderMetrics(job.summary); updateButtons(); updatePlayback(); draw();
  }
  function mergeFrames(frames) {
    let last=state.frames.at(-1)?.index ?? -1;
    for (const frame of frames || []) {
      if (frame.index <= last) continue;
      for (const row of frame.samples || []) state.rows.push(row);
      state.frames.push({...frame,rowEnd:state.rows.length});
      last=frame.index;
    }
    if (state.follow) state.selected=Math.max(0,state.frames.length-1);
  }
  function stopPoll() { if (state.pollTimer) clearTimeout(state.pollTimer); state.pollTimer=null; }
  async function pollJob() {
    stopPoll();
    if (!state.job || state.polling || terminal.has(state.job.status)) return;
    const id=state.job.id;
    state.polling=true;
    try {
      const after=state.frames.at(-1)?.index ?? -1;
      const response=await api(`/api/jobs/${encodeURIComponent(id)}?after=${after}`);
      if (state.job?.id !== id) return;
      state.job={...state.job,...response}; mergeFrames(response.frames);
      state.pollErrors=0; renderJob();
      if (terminal.has(response.status)) await loadHistory().catch(error=>errorMessage(`결과는 저장됐지만 기록 목록을 갱신하지 못했습니다. ${error.message}`));
      else state.pollTimer=setTimeout(pollJob,650);
    } catch(error) {
      state.pollErrors++;
      if (state.pollErrors < 4) state.pollTimer=setTimeout(pollJob,1600);
      else errorMessage(`${error.message} 실행 상태를 다시 연결해 확인할 수 있습니다.`,true);
    } finally { state.polling=false; }
  }
  async function startJob(event) {
    event.preventDefault();
    if (isActive() || state.submitting || state.loading) return;
    let settings;
    try { settings=readSettings(); } catch(error) { errorMessage(error.message); return; }
    state.submitting=true; updateButtons(); clearError();
    try {
      const response=await api('/api/jobs',{method:'POST',body:JSON.stringify(settings)});
      stopPoll(); resetReplay();
      state.job={id:response.id,status:'building',...settings,summary:null};
      renderJob(); state.pollErrors=0;
      await pollJob();
    } catch(error) { errorMessage(error.message); }
    finally { state.submitting=false; updateButtons(); }
  }
  async function cancelJob() {
    if (!isActive()) return;
    $('cancel-button').disabled=true;
    try {
      await api(`/api/jobs/${encodeURIComponent(state.job.id)}/cancel`,{method:'POST',body:'{}'});
      state.job.cancelRequested=true;
      $('run-description').textContent='중단을 요청했습니다. 현재 단계가 끝나면 계산을 멈추고 부분 결과를 보존합니다.';
    } catch(error) { errorMessage(error.message); $('cancel-button').disabled=false; }
  }
  async function loadJob(id) {
    if (isActive() || state.submitting || state.loading) return;
    clearError(); stopPoll(); resetReplay(); state.loading=true; updateButtons();
    try {
      const job=await api(`/api/jobs/${encodeURIComponent(id)}?after=-1`);
      state.job=job;
      $('track-select').value=job.track;
      if (state.track?.id !== job.track) await loadTrack(job.track);
      mergeFrames(job.frames); applySettings(job); renderJob();
      if (!terminal.has(job.status)) { state.pollErrors=0; await pollJob(); }
      await loadHistory().catch(error=>errorMessage(`기록 목록을 갱신하지 못했습니다. ${error.message}`));
    } catch(error) {
      state.job=null;
      if (state.track) { $('track-select').value=state.track.id; showBaseline(); }
      errorMessage(error.message);
    }
    finally { state.loading=false; updateButtons(); }
  }
  function formatDate(input) {
    if (!input) return '';
    const date=new Date(input);
    return Number.isNaN(date.getTime()) ? String(input).replace('T',' ').slice(0,19) : date.toLocaleString('ko-KR',{month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:false});
  }
  async function loadHistory() {
    const history=await api('/api/history');
    $('history-list').replaceChildren();
    if (!history.jobs.length) {
      const empty=document.createElement('p'); empty.className='empty-history'; empty.textContent='아직 실행한 실험이 없습니다. 첫 번째 가중치를 시험해 보세요.'; $('history-list').append(empty); return;
    }
    for (const job of history.jobs) {
      const row=document.createElement('div'); row.className=`history-row${state.job?.id===job.id?' selected':''}`;
      const date=document.createElement('time'); date.textContent=formatDate(job.created_at); date.title=job.id;
      const label=document.createElement('span'); label.textContent=state.catalog.tracks.find(t=>t.id===job.track)?.label || job.track;
      const weights=document.createElement('span'); weights.className='history-weights'; weights.textContent=`q꜀ ${compact(job.weights?.contour)} · ${compact(job.speed_max)} m/s`;
      const status=document.createElement('span'); status.className=`history-status ${job.status}`; status.textContent=statusLabels[job.status] || job.status;
      const button=document.createElement('button'); button.type='button'; button.textContent='불러오기'; button.dataset.historyId=job.id; button.addEventListener('click',()=>loadJob(job.id));
      row.append(date,label,weights,status,button); $('history-list').append(row);
    }
    updateButtons();
  }
  function currentFrame() { return state.frames[state.selected] || null; }
  function updatePlayback() {
    const frame=currentFrame();
    $('timeline').max=Math.max(0,state.frames.length-1); $('timeline').value=state.selected;
    $('playback-time').textContent=`${number(frame?.time_s ?? 0)} / ${number(state.frames.at(-1)?.time_s ?? 0)} s`;
    $('play-button').textContent=state.playing?'Ⅱ':'▶';
    $('play-button').setAttribute('aria-label',state.playing?'재생 일시정지':'시뮬레이션 재생');
    const row=frame ? state.rows[frame.rowEnd-1] : null;
    $('car-speed').replaceChildren(document.createTextNode(number(frame?.state?.[3],2)));
    const unit=document.createElement('small'); unit.textContent='m/s'; $('car-speed').append(unit);
    $('car-steering').textContent=`조향 ${number(row ? row[9]*180/Math.PI:null,1)}°`;
    $('live-button').textContent=state.follow?'● 최신 위치':'최신 위치';
  }
  function stopPlayback() {
    state.playing=false;
    if (state.playRequest) cancelAnimationFrame(state.playRequest);
    state.playRequest=null; $('play-button').textContent='▶';
  }
  function startPlayback() {
    if (state.frames.length < 2) return;
    if (state.playing) { stopPlayback(); updatePlayback(); return; }
    state.follow=false;
    if (state.selected >= state.frames.length-1) state.selected=0;
    state.playing=true;
    let last=performance.now();
    let simulationTime=state.frames[state.selected].time_s;
    function tick(now) {
      if (!state.playing) return;
      simulationTime+=Math.min((now-last)/1000,0.25)*Number($('playback-rate').value); last=now;
      while (state.selected < state.frames.length-1 && state.frames[state.selected+1].time_s <= simulationTime) state.selected++;
      updatePlayback(); draw();
      if (state.selected >= state.frames.length-1) { stopPlayback(); updatePlayback(); }
      else state.playRequest=requestAnimationFrame(tick);
    }
    updatePlayback(); state.playRequest=requestAnimationFrame(tick);
  }
  function resizeCanvas() {
    const rect=canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    const first=state.viewport.width===1;
    state.viewport.width=rect.width; state.viewport.height=rect.height;
    const dpr=window.devicePixelRatio || 1;
    canvas.width=Math.round(rect.width*dpr); canvas.height=Math.round(rect.height*dpr); ctx.setTransform(dpr,0,0,dpr,0,0);
    if (first && state.track) fitMap(); else draw();
  }
  function fitMap() {
    if (!state.track) return;
    const points=[...state.track.center,...state.track.left,...state.track.right];
    const xs=points.map(p=>p[0]), ys=points.map(p=>p[1]);
    const xmin=Math.min(...xs)-3,xmax=Math.max(...xs)+3,ymin=Math.min(...ys)-3,ymax=Math.max(...ys)+3;
    const v=state.viewport; v.cx=(xmin+xmax)/2; v.cy=(ymin+ymax)/2; v.scale=Math.min(v.width/(xmax-xmin),v.height/(ymax-ymin)); draw();
  }
  function screen(x,y) { const v=state.viewport; return [v.width/2+(x-v.cx)*v.scale,v.height/2-(y-v.cy)*v.scale]; }
  function world(x,y) { const v=state.viewport; return [v.cx+(x-v.width/2)/v.scale,v.cy-(y-v.height/2)/v.scale]; }
  function line(points,color,width=1,dash=[],rowIndices=null) {
    if (!points?.length) return;
    ctx.strokeStyle=color; ctx.lineWidth=width; ctx.setLineDash(dash); ctx.lineJoin='round'; ctx.lineCap='round'; ctx.beginPath();
    let started=false;
    for (const point of points) {
      const x=point[rowIndices?.[0] ?? 0], y=point[rowIndices?.[1] ?? 1];
      if (!Number.isFinite(x) || !Number.isFinite(y)) { started=false; continue; }
      const p=screen(x,y); if (started) ctx.lineTo(...p); else {ctx.moveTo(...p);started=true;}
    }
    ctx.stroke(); ctx.setLineDash([]);
  }
  function draw() {
    const v=state.viewport;
    ctx.clearRect(0,0,v.width,v.height); ctx.fillStyle='#f8fafb'; ctx.fillRect(0,0,v.width,v.height);
    if (!state.track) return;
    if (state.image && $('map-toggle').checked) {
      const [xmin,xmax,ymin,ymax]=state.track.map.extent;
      const top=screen(xmin,ymax); ctx.globalAlpha=0.72; ctx.drawImage(state.image,top[0],top[1],(xmax-xmin)*v.scale,(ymax-ymin)*v.scale); ctx.globalAlpha=1;
    }
    const spacing=v.scale>18?2:5;
    const [minx,maxy]=world(0,0),[maxx,miny]=world(v.width,v.height);
    ctx.strokeStyle='#819bab16';ctx.lineWidth=1;ctx.beginPath();
    for(let x=Math.ceil(minx/spacing)*spacing;x<=maxx;x+=spacing){const p=screen(x,0);ctx.moveTo(p[0],0);ctx.lineTo(p[0],v.height);}
    for(let y=Math.ceil(miny/spacing)*spacing;y<=maxy;y+=spacing){const p=screen(0,y);ctx.moveTo(0,p[1]);ctx.lineTo(v.width,p[1]);}ctx.stroke();
    line(state.track.left,'#148772',1.3); line(state.track.right,'#148772',1.3); line(state.track.center,'#6b8c9c',1,[4,5]);
    if ($('baseline-toggle').checked) line(state.track.baseline?.trajectory,'#9763b8',1.8,[6,4],[1,2]);
    const frame=currentFrame();
    if (frame) {
      const visible=state.rows.slice(0,frame.rowEnd);
      line(visible,'#ffffff',4.6,[],[1,2]); line(visible,'#1665e9',2.6,[],[1,2]);
      line(frame.prediction,'#ffffff',5); line(frame.prediction,'#ec821c',2.8);
      const s=frame.state;
      if (s?.length>=3) {
        const p=screen(s[0],s[1]); const angle=-s[2];
        ctx.save();ctx.translate(...p);ctx.rotate(angle);
        ctx.fillStyle='#1665e9';ctx.strokeStyle='#fff';ctx.lineWidth=1.8;
        ctx.beginPath();ctx.moveTo(11,0);ctx.lineTo(-7,-6);ctx.lineTo(-4,0);ctx.lineTo(-7,6);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
      }
    } else if (state.track.center.length) {
      const p=screen(...state.track.center[0]);ctx.fillStyle='#fff';ctx.strokeStyle='#4a778d';ctx.lineWidth=2;ctx.beginPath();ctx.arc(...p,4,0,Math.PI*2);ctx.fill();ctx.stroke();
    }
    // Scale bar stays in screen pixels while the map moves and zooms.
    const raw=90/v.scale; const pow=10**Math.floor(Math.log10(raw)); const distance=[1,2,5,10].map(x=>x*pow).filter(x=>x<=raw).at(-1)||pow;
    const pixels=distance*v.scale,barx=18,bary=v.height-21;
    ctx.fillStyle='#ffffffdd';ctx.fillRect(barx-6,bary-20,pixels+12,30);ctx.strokeStyle='#5e7888';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(barx,bary-4);ctx.lineTo(barx,bary);ctx.lineTo(barx+pixels,bary);ctx.lineTo(barx+pixels,bary-4);ctx.stroke();ctx.fillStyle='#5e7888';ctx.font='10px sans-serif';ctx.textAlign='center';ctx.fillText(`${compact(distance)} m`,barx+pixels/2,bary-7);ctx.textAlign='left';
  }
  let drag=null;
  canvas.addEventListener('pointerdown',event=>{if(event.button!==0)return;canvas.setPointerCapture(event.pointerId);drag={x:event.clientX,y:event.clientY,cx:state.viewport.cx,cy:state.viewport.cy};canvas.classList.add('dragging');});
  canvas.addEventListener('pointermove',event=>{if(!drag)return;state.viewport.cx=drag.cx-(event.clientX-drag.x)/state.viewport.scale;state.viewport.cy=drag.cy+(event.clientY-drag.y)/state.viewport.scale;draw();});
  const endDrag=()=>{drag=null;canvas.classList.remove('dragging');};
  canvas.addEventListener('pointerup',endDrag);canvas.addEventListener('pointercancel',endDrag);
  canvas.addEventListener('wheel',event=>{
    event.preventDefault();const rect=canvas.getBoundingClientRect();const x=event.clientX-rect.left,y=event.clientY-rect.top;
    const before=world(x,y);state.viewport.scale=Math.min(500,Math.max(1,state.viewport.scale*Math.exp(-event.deltaY*0.001)));
    const after=world(x,y);state.viewport.cx+=before[0]-after[0];state.viewport.cy+=before[1]-after[1];draw();
  },{passive:false});
  $('settings-form').addEventListener('submit',startJob);
  $('cancel-button').addEventListener('click',cancelJob);
  $('reset-button').addEventListener('click',()=>{applySettings(state.catalog?.defaults);clearError();});
  $('speed-input').addEventListener('input',()=>{$('speed-range').value=$('speed-input').value;updateSpeed();});
  $('speed-range').addEventListener('input',()=>{$('speed-input').value=$('speed-range').value;updateSpeed();});
  $('track-select').addEventListener('change',async()=>{
    if(isActive())return;stopPoll();state.job=null;resetReplay();clearError();
    try{await loadTrack($('track-select').value);}catch(error){
      if(state.track){$('track-select').value=state.track.id;showBaseline();$('map-loading').hidden=true;}
      errorMessage(error.message);
    }
  });
  $('fit-button').addEventListener('click',fitMap);
  $('baseline-toggle').addEventListener('change',draw);$('map-toggle').addEventListener('change',draw);
  $('play-button').addEventListener('click',startPlayback);
  $('timeline').addEventListener('input',()=>{stopPlayback();state.follow=false;state.selected=Number($('timeline').value);updatePlayback();draw();});
  $('live-button').addEventListener('click',()=>{stopPlayback();state.follow=true;state.selected=Math.max(0,state.frames.length-1);updatePlayback();draw();});
  $('history-refresh').addEventListener('click',()=>loadHistory().catch(error=>errorMessage(error.message)));
  $('dismiss-error').addEventListener('click',clearError);
  $('reconnect-button').addEventListener('click',()=>{clearError();state.pollErrors=0;pollJob();});
  new ResizeObserver(resizeCanvas).observe($('canvas-wrap'));
  window.addEventListener('beforeunload',()=>{stopPoll();stopPlayback();});
  async function init() {
    try {
      state.catalog=await api('/api/catalog');createFields();
      const limits=state.catalog.speed_limits;
      for(const id of ['speed-input','speed-range']) Object.assign($(id),{min:limits.min,max:limits.max,step:limits.step});
      $('speed-min-label').textContent=`${limits.min} m/s`;
      $('speed-max-label').textContent=`${limits.max} m/s ≈ ${number(limits.max/0.44704,0)} mph`;
      $('track-select').replaceChildren();
      state.catalog.tracks.forEach(track=>{const option=document.createElement('option');option.value=track.id;option.textContent=track.label;$('track-select').append(option);});
      $('track-select').value=state.catalog.defaults.track;applySettings(state.catalog.defaults);
      await loadTrack(state.catalog.defaults.track);await loadHistory();updateButtons();
    } catch(error) { setBadge('failed','연결 실패');errorMessage(error.message);$('map-loading').textContent='화면을 새로고침해서 다시 연결하세요.'; }
  }
  init();
})();
