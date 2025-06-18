// static/control.js

// 1. Connect to MQTT over WebSockets
const client = mqtt.connect(`ws://${location.hostname}:9001`);

client.on('connect', () => {
  console.log('MQTT connected');
  document.querySelectorAll('button, .arrow').forEach(el => el.disabled = false);
});
client.on('error', err => console.error('MQTT error:', err));

// 2. Helper: publish safely
function publish(topic, payload) {
  if (!client.connected) {
    return console.warn('MQTT not connected yet');
  }
  client.publish(topic, payload);
}

// 3. Arrow‐pad commands map
const cmdMap = {
  up:    ['robot/serial', 'FORWARD'],
  down:  ['robot/serial', 'BACKWARD'],
  left:  ['robot/serial', 'LEFT'],
  right: ['robot/serial', 'RIGHT'],
  stop:  ['robot/serial', 'STOP']
};

// 4. Wire up pointer events (mouse/touch) for each arrow
Object.keys(cmdMap).forEach(key => {
  const el = document.getElementById(key);
  let iid;

  el.onpointerdown = () => {
    el.classList.add('active');
    publish(...cmdMap[key]);
    iid = setInterval(() => publish(...cmdMap[key]), 300);
  };

  el.onpointerup = el.onpointerleave = () => {
    el.classList.remove('active');
    clearInterval(iid);
  };
});

// 5. Wire up click‐to‐publish for all buttons with data‐mqtt
document.querySelectorAll('button[data-mqtt]').forEach(btn => {
  btn.onclick = () => {
    publish(btn.dataset.mqtt, btn.dataset.payload);
  };
});

// 6. PID form submissions
document.getElementById('innerForm').onsubmit = e => {
  e.preventDefault();
  const f = e.target;
  const msg = `inner:${f.p.value},${f.d.value},${f.i.value},${f.sp.value}`;
  publish('robot/pid', msg);
};

document.getElementById('outerForm').onsubmit = e => {
  e.preventDefault();
  const f = e.target;
  const msg = `outer:${f.p.value},${f.d.value},${f.i.value},${f.sp.value},${f.rot.value}`;
  publish('robot/pid', msg);
};

// 7. Key‐location form
document.getElementById('keyForm').onsubmit = e => {
  e.preventDefault();
  const loc = e.target.loc.value;
  publish('autonomous', `KEY:${loc}`);
};

// 8. Keyboard control
let lastKey = null;
let keyIid = null;
const keyMap = {
  ArrowUp:    'up',
  ArrowDown:  'down',
  ArrowLeft:  'left',
  ArrowRight: 'right',
  ' ':        'stop'
};

document.addEventListener('keydown', e => {
  const action = keyMap[e.key];
  if (!action || e.key === lastKey) return;

  lastKey = e.key;
  const el = document.getElementById(action);
  el.classList.add('active');
  publish(...cmdMap[action]);

  // if not STOP, start repeating
  if (action !== 'stop') {
    keyIid = setInterval(() => publish(...cmdMap[action]), 300);
  }
});

document.addEventListener('keyup', e => {
  if (!lastKey) return;
  const action = keyMap[lastKey];
  if (action) {
    document.getElementById(action).classList.remove('active');
    clearInterval(keyIid);
    lastKey = null;
  }
});
