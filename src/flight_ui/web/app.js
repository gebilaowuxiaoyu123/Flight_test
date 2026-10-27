(() => {
  const $ = (id) => document.getElementById(id);
  const connEl = $('conn');
  const armedEl = $('armed');
  const modeEl = $('mode');
  const ctrlEl = $('ctrl');
  const altEl = $('alt');
  const batEl = $('bat');
  const logEl = $('log');

  let ws = null;
  let deadman = false;
  let telemetry = {};

  function log(msg) {
    logEl.textContent = `${new Date().toLocaleTimeString()}  ${msg}`;
  }

  function clamp(v) { return Math.max(-1, Math.min(1, v)); }

  class Stick {
    constructor(el, { springX = true, springY = true, initial = { x: 0, y: 0 } } = {}) {
      this.el = el;
      this.knob = el.querySelector('.knob');
      this.springX = springX;
      this.springY = springY;
      this.x = initial.x;
      this.y = initial.y;
      this.active = false;
      this.render();
      el.addEventListener('pointerdown', (e) => this.start(e));
      el.addEventListener('pointermove', (e) => { if (this.active) this.move(e); });
      el.addEventListener('pointerup', () => { if (this.active) this.end(); });
      el.addEventListener('pointercancel', () => { if (this.active) this.end(); });
    }

    center() {
      const r = this.el.getBoundingClientRect();
      return { x: r.left + r.width / 2, y: r.top + r.height / 2, r: r.width / 2 };
    }

    start(e) {
      e.preventDefault();
      // Capture this pointer so two-finger operation never mixes the sticks.
      if (this.el.setPointerCapture) {
        try { this.el.setPointerCapture(e.pointerId); } catch (_) { /* ignore */ }
      }
      this.active = true;
      this.move(e);
    }

    move(e) {
      const c = this.center();
      const max = c.r * 0.78;
      let dx = e.clientX - c.x;
      let dy = -(e.clientY - c.y);
      const len = Math.hypot(dx, dy);
      if (len > max) { dx = dx * max / len; dy = dy * max / len; }
      this.x = clamp(dx / max);
      this.y = clamp(dy / max);
      if (Math.abs(this.x) < 0.04) this.x = 0;
      if (Math.abs(this.y) < 0.04) this.y = 0;
      this.render();
    }

    end() {
      this.active = false;
      if (this.springX) this.x = 0;
      if (this.springY) this.y = 0;
      this.render();
    }

    set(x, y) {
      this.x = x;
      this.y = y;
      this.render();
    }

    render() {
      const c = this.center();
      const max = c.r * 0.78;
      this.knob.style.transform = `translate(${this.x * max}px, ${-this.y * max}px)`;
    }
  }

  const left = new Stick($('left'), { springX: true, springY: false, initial: { x: 0, y: -1 } });
  const right = new Stick($('right'), { springX: true, springY: true });

  function sendSticks() {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify({
      type: 'sticks',
      roll: right.x,
      pitch: right.y,
      yaw: left.x,
      throttle: left.y,
      deadman,
    }));
  }

  function send(obj) {
    if (!ws || ws.readyState !== WebSocket.OPEN) { log('WebSocket 未连接'); return; }
    ws.send(JSON.stringify(obj));
  }

  function setDeadman(value) {
    deadman = value;
    $('deadman').classList.toggle('active', value);
    if (!value) {
      left.set(0, -1);
      right.set(0, 0);
      sendSticks();
      log('Deadman 释放：油门回最低，输入失效');
    }
  }

  const deadmanBtn = $('deadman');
  deadmanBtn.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    if (deadmanBtn.setPointerCapture) {
      try { deadmanBtn.setPointerCapture(e.pointerId); } catch (_) { /* ignore */ }
    }
    setDeadman(true);
  });
  deadmanBtn.addEventListener('pointerup', (e) => { e.preventDefault(); setDeadman(false); });
  deadmanBtn.addEventListener('pointercancel', () => setDeadman(false));
  window.addEventListener('keydown', (e) => { if (e.code === 'Space') { e.preventDefault(); setDeadman(true); } });
  window.addEventListener('keyup', (e) => { if (e.code === 'Space') setDeadman(false); });

  $('arm').addEventListener('click', () => send({ type: 'arm' }));
  $('disarm').addEventListener('click', () => send({ type: 'disarm' }));
  $('altctl').addEventListener('click', () => send({ type: 'mode', mode: 'ALTCTL' }));
  $('posctl').addEventListener('click', () => send({ type: 'mode', mode: 'POSCTL' }));
  $('takeoff').addEventListener('click', () => send({ type: 'takeoff', altitude: 2.5 }));
  $('land').addEventListener('click', () => send({ type: 'land' }));
  $('kill').addEventListener('click', () => {
    if (confirm('确认 KILL？将立即上锁并请求飞行终止。')) send({ type: 'kill' });
  });

  function setCtrlLabel(kind) {
    if (kind === 'you') {
      ctrlEl.textContent = '本机操控';
      ctrlEl.className = 'pill ok';
    } else if (kind === 'other') {
      ctrlEl.textContent = '外部操控';
      ctrlEl.className = 'pill warn';
    } else {
      ctrlEl.textContent = '空闲';
      ctrlEl.className = 'pill';
    }
  }

  function updateTelemetry(t) {
    telemetry = { ...telemetry, ...t };
    armedEl.textContent = t.armed ? 'ARMED' : 'DISARMED';
    armedEl.className = 'pill ' + (t.armed ? 'ok' : 'bad');
    modeEl.textContent = t.mode ?? '--';
    altEl.textContent = (t.alt ?? 0).toFixed(2);
    batEl.textContent = t.battery >= 0 ? t.battery.toFixed(0) : '--';

    // 杆量回显：别人在飞的时候，把摇杆视觉上跟到真实杆量。
    // 自己握着控制权时绝不回写，否则手指拖动会被下一帧遥测拽回去。
    setCtrlLabel(t.controller);
    if (t.controller === 'other' && t.sticks) {
      left.set(t.sticks.yaw ?? 0, t.sticks.throttle ?? -1);
      right.set(t.sticks.roll ?? 0, t.sticks.pitch ?? 0);
    }
  }

  function connect() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    ws = new WebSocket(`${proto}://${location.host}/ws`);
    connEl.textContent = '连接中…';
    connEl.className = 'pill';

    ws.onopen = () => {
      connEl.textContent = '已连接';
      connEl.className = 'pill ok';
      log('WebSocket 已连接');
    };
    ws.onclose = () => {
      connEl.textContent = '连接断开';
      connEl.className = 'pill bad';
      deadman = false;
      $('deadman').classList.remove('active');
      log('WebSocket 断开，1 秒后重连');
      setTimeout(connect, 1000);
    };
    ws.onerror = () => log('WebSocket 错误');
    ws.onmessage = (event) => {
      let data;
      try { data = JSON.parse(event.data); } catch { return; }
      if (data.type === 'telemetry') updateTelemetry(data);
      if (data.type === 'ack') log(`${data.action}: ${data.ok ? 'OK' : 'REJECTED'} ${data.message || ''}`);
    };
  }

  window.addEventListener('resize', () => { left.render(); right.render(); });
  setInterval(sendSticks, 20);
  connect();
})();
