(function () {
  'use strict';

  const STATE_LABELS = {
    idle: '空闲', goto_table: '前往桌前', scanning: '扫描箱子',
    hand_up: '抬臂准备', align: '相机对位', grab: '抓取中',
    backward: '后退', to_room: '前往房间', placing: '放置中', returning: '返回桌前',
  };
  const COLOR_CN = { red: '红色', green: '绿色', yellow: '黄色' };
  const COLOR_CODE = { 1: 'red', 2: 'green', 3: 'yellow' };
  const PIPELINE = [
    'goto_table', 'scanning', 'hand_up', 'align', 'grab',
    'backward', 'to_room', 'placing', 'returning',
  ];

  let ros = null, cmdPub = null, cmdVelPub = null;
  let connected = false, currentState = 'idle';
  let activeTaskId = null, wasBusy = false, taskCounter = 1;
  let tasks = [];
  let stopTimer = null;
  let lastFrameTs = 0, fps = 0;
  const detections = { red: null, green: null, yellow: null };

  const $ = (id) => document.getElementById(id);

  function log(msg, type) {
    const box = $('log-box');
    const e = document.createElement('div');
    e.className = 'log-entry ' + (type || 'info');
    e.textContent = '[' + new Date().toLocaleTimeString('zh-CN', { hour12: false }) + '] ' + msg;
    box.appendChild(e);
    box.scrollTop = box.scrollHeight;
  }

  function setConn(status) {
    $('conn-dot').className = 'conn-dot ' + status;
    $('conn-label').textContent = { connected: 'ROS 已连接', connecting: '连接中…', error: '未连接' }[status] || '';
  }

  function stateColor(s) {
    if (s === 'idle') return '#22c55e';
    if (['align', 'grab', 'scanning'].includes(s)) return '#f59e0b';
    return '#3b82f6';
  }

  function updateStatusUI(state) {
    currentState = state || 'idle';
    const label = STATE_LABELS[currentState] || currentState;
    $('status-badge').textContent = label;
    const c = stateColor(currentState);
    $('status-badge').style.color = c;
    $('state-lamp').style.background = c;

    const idx = PIPELINE.indexOf(currentState);
    document.querySelectorAll('.pipe-step').forEach((el) => {
      const step = el.dataset.step;
      const si = PIPELINE.indexOf(step);
      el.classList.remove('current', 'done');
      if (step === currentState) el.classList.add('current');
      else if (si >= 0 && idx >= 0 && si < idx) el.classList.add('done');
    });
  }

  function syncTaskFromFsm(raw) {
    if (raw !== 'idle') {
      wasBusy = true;
      if (activeTaskId) {
        const t = tasks.find((x) => x.id === activeTaskId && x.status === 'Queued');
        if (t) { t.status = 'Running'; renderTasks(); }
      }
      return;
    }
    if (wasBusy && activeTaskId) {
      const t = tasks.find((x) => x.id === activeTaskId);
      if (t) t.status = 'Success';
      activeTaskId = null;
      wasBusy = false;
      renderTasks();
      updateStats();
    }
  }

  function renderTasks() {
    const tbody = $('task-tbody');
    tbody.innerHTML = '';
    tasks.slice().reverse().forEach((t) => {
      const tr = document.createElement('tr');
      tr.innerHTML = '<td>' + t.id + '</td><td>' + (COLOR_CN[t.color] || t.color) +
        '</td><td>房间 ' + t.room + '</td><td class="st-' + t.status + '">' + t.status + '</td>';
      tbody.appendChild(tr);
    });
  }

  function updateStats() {
    const total = tasks.length;
    const ok = tasks.filter((t) => t.status === 'Success').length;
    $('stat-total').textContent = '总任务: ' + total;
    $('stat-success').textContent = '成功率: ' + (total ? (ok / total * 100).toFixed(1) : '0') + '%';
  }

  function updateDetections() {
    const now = Date.now();
    ['red', 'green', 'yellow'].forEach((c) => {
      const card = $('det-' + c);
      const d = detections[c];
      const val = card.querySelector('.val');
      if (d && now - d.ts < 1500) {
        card.classList.add('visible');
        val.textContent = d.depth.toFixed(2) + 'm · 面积' + Math.round(d.area);
      } else {
        card.classList.remove('visible');
        val.textContent = '未检测';
        detections[c] = null;
      }
    });
  }

  function connect() {
    const url = $('ws-url').value.trim();
    if (!url) return log('请输入 WebSocket 地址', 'warn');
    if (ros) try { ros.close(); } catch (_) { /* ignore */ }

    setConn('connecting');
    log('连接 ' + url + ' …');
    ros = new ROSLIB.Ros({ url });

    ros.on('connection', () => {
      connected = true;
      setConn('connected');
      log('ROS 连接成功');
      setupTopics();
      $('btn-send').disabled = false;
    });
    ros.on('error', () => { connected = false; setConn('error'); $('btn-send').disabled = true; });
    ros.on('close', () => {
      connected = false; setConn('error'); $('btn-send').disabled = true;
      cmdPub = cmdVelPub = null;
      log('连接断开', 'warn');
    });
  }

  function compressedImageToDataUrl(msg) {
    if (!msg || !msg.data) return null;
    const fmt = (msg.format || 'jpeg').split(';')[0].replace('jpeg', 'jpeg');
    const mime = fmt.indexOf('/') >= 0 ? fmt : 'image/jpeg';
    if (typeof msg.data === 'string') {
      return 'data:' + mime + ';base64,' + msg.data;
    }
    const bytes = msg.data instanceof Uint8Array ? msg.data : new Uint8Array(msg.data);
    if (!bytes.length) return null;
    let binary = '';
    const chunk = 0x8000;
    for (let i = 0; i < bytes.length; i += chunk) {
      binary += String.fromCharCode.apply(null, bytes.subarray(i, i + chunk));
    }
    return 'data:' + mime + ';base64,' + btoa(binary);
  }

  function setupTopics() {
    cmdPub = new ROSLIB.Topic({ ros, name: '/walle/command', messageType: 'std_msgs/String' });
    cmdVelPub = new ROSLIB.Topic({ ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist' });

    new ROSLIB.Topic({ ros, name: '/task/status', messageType: 'std_msgs/String' })
      .subscribe((msg) => {
        updateStatusUI(msg.data);
        syncTaskFromFsm(msg.data);
        log('状态 → ' + (STATE_LABELS[msg.data] || msg.data));
      });

    new ROSLIB.Topic({ ros, name: '/odom', messageType: 'nav_msgs/Odometry' })
      .subscribe((msg) => {
        const p = msg.pose.pose.position;
        const q = msg.pose.pose.orientation;
        const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        $('pose-text').textContent =
          'x=' + p.x.toFixed(2) + ', y=' + p.y.toFixed(2) + ', yaw=' + yaw.toFixed(2);
      });

    new ROSLIB.Topic({ ros, name: '/vision/box_camera', messageType: 'geometry_msgs/Pose' })
      .subscribe((msg) => {
        const c = COLOR_CODE[msg.orientation.w];
        if (!c) return;
        detections[c] = { depth: msg.position.z, area: msg.orientation.z, ts: Date.now() };
        updateDetections();
      });

    new ROSLIB.Topic({ ros, name: '/vision/debug_image/compressed', messageType: 'sensor_msgs/CompressedImage' })
      .subscribe((msg) => {
        const dataUrl = compressedImageToDataUrl(msg);
        if (!dataUrl) return;
        const now = Date.now();
        if (lastFrameTs > 0) {
          const dt = (now - lastFrameTs) / 1000;
          if (dt > 0) fps = 0.8 * fps + 0.2 * (1 / dt);
        }
        lastFrameTs = now;
        $('fps-label').textContent = 'FPS: ' + fps.toFixed(1);
        const img = $('vision-preview');
        img.src = dataUrl;
        img.style.display = 'block';
        $('preview-placeholder').style.display = 'none';
      });
  }

  function sendCommand(color, room) {
    if (!connected || !cmdPub) return log('未连接 ROS', 'warn');
    const payload = color + ' ' + room;
    cmdPub.publish(new ROSLIB.Message({ data: payload }));

    const id = 'T' + String(taskCounter++).padStart(4, '0');
    activeTaskId = id;
    wasBusy = false;
    tasks.push({ id, color, room, status: 'Queued' });
    renderTasks();
    updateStats();
    log('已发送: ' + payload + ' (' + id + ')', 'cmd');
  }

  function emergencyStop() {
    if (!cmdVelPub) return log('未连接', 'warn');
    const z = new ROSLIB.Message({
      linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 },
    });
    if (stopTimer) clearInterval(stopTimer);
    let n = 0;
    stopTimer = setInterval(() => {
      cmdVelPub.publish(z);
      if (++n >= 20) { clearInterval(stopTimer); stopTimer = null; }
    }, 100);
    log('急停已发送', 'warn');
  }

  function initUI() {
    const host = window.location.hostname || 'localhost';
    $('ws-url').value = 'ws://' + host + ':9090';

    $('btn-connect').onclick = connect;
    $('btn-send').onclick = () => sendCommand($('sel-color').value, $('sel-room').value);
    $('btn-stop').onclick = emergencyStop;
    $('btn-clear').onclick = () => {
      tasks = []; activeTaskId = null; wasBusy = false;
      renderTasks(); updateStats();
    };
    document.querySelectorAll('.quick-cmd').forEach((b) => {
      b.onclick = () => {
        const [c, r] = b.dataset.cmd.split(' ');
        sendCommand(c, r);
      };
    });

    PIPELINE.forEach((s) => {
      const el = document.createElement('span');
      el.className = 'pipe-step';
      el.dataset.step = s;
      el.textContent = STATE_LABELS[s] || s;
      $('pipeline').appendChild(el);
    });

    updateStatusUI('idle');
    setInterval(updateDetections, 400);
    log('控制台就绪，请连接 rosbridge');
    log('画面需 /vision/debug_image/compressed（请用 web_ui.launch 或手动 republish）', 'warn');
  }

  document.readyState === 'loading'
    ? document.addEventListener('DOMContentLoaded', initUI)
    : initUI();
})();
