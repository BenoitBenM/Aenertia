// mqtt-control.js

// ----------------- MQTT Setup -----------------
const brokerUrl = 'ws://172.20.10.9:9001';  // Update this if your broker is on a different IP/port
const opts = { keepalive: 30, reconnectPeriod: 1000 };
const client = mqtt.connect(brokerUrl, opts);

// ---- MQTT Event Handlers ----
client.on('connect', () => {
  console.log('[MQTT] Connected to broker:', brokerUrl);
  document.getElementById('mqtt-status').innerText = 'MQTT: Connected';
  client.subscribe('robot/battery');
  client.subscribe('robot/vb');
  client.subscribe('robot/eu');
  client.subscribe('robot/keys');
});

client.on('message', (topic, message) => {
  const msg = message.toString();
  if (topic === 'robot/vb') {
    document.getElementById('vb').innerText = `VB: ${msg}`;
  }
  else if (topic === 'robot/eu') {
    document.getElementById('eu').innerText = `EU: ${msg}`;
  }
  else if (topic === 'robot/battery') {
    document.getElementById('battery').innerText = `Battery: ${msg}%`;
  }
  else if (topic === 'robot/keys') {
    let keys = [];
    try {
      keys = JSON.parse(msg);
    } catch (e) {
      console.error('[MQTT] Failed to parse robot/keys payload:', msg);
      return;
    }
    const keyList = document.getElementById('key-list');
    keyList.innerHTML = '<ul>' +
      keys.map(k => `<li><button class="key-button" onclick="assignKeyLocation('${k}')">${k}</button></li>`).join('') +
      '</ul>';
  }
});

client.on('error', err => {
  console.error('[MQTT] Error:', err);
  document.getElementById('mqtt-status').innerText = 'MQTT: Error';
  document.getElementById('battery').innerText = 'Battery: --%';
});

client.on('close', () => {
  console.warn('[MQTT] Connection closed');
  document.getElementById('mqtt-status').innerText = 'MQTT: Disconnected';
  document.getElementById('battery').innerText = 'Battery: --%';
});

// ---- Helper to publish ----
function pub(topic, payload) {
  if (client.connected) {
    client.publish(topic, payload);
    console.log(`[MQTT] Published to ${topic}:`, payload);
  } else {
    console.warn(`[MQTT] Cannot publish, not connected. Topic: ${topic}, Payload: ${payload}`);
  }
}

// ---- Assign Key Location (from dynamic key list) ----
function assignKeyLocation(loc) {
  pub('robot/auto/key/assign', loc);
}

// ----------------- UI Clock -----------------
setInterval(() => {
  const now = new Date();
  document.getElementById('time').innerText = now.toLocaleTimeString([], {
    hour: '2-digit',
    minute: '2-digit'
  });
}, 10000);

// ----------------- Arrow/PAD Controls -----------------
const commandMap = {
  up: 'up',
  down: 'down',
  left: 'left',
  right: 'right',
  stop: 'stop',
  'up-left': 'up-left',
  'up-right': 'up-right',
  'down-left': 'down-left',
  'down-right': 'down-right'
};
const arrowTopic = 'robot/manual/command';

const activeKeys = new Set();
let movementInterval = null;

// Mouse-based arrow controls
['up', 'down', 'left', 'right'].forEach(dir => {
  const btn = document.getElementById(dir);
  if (!btn) return;
  btn.onmousedown = () => {
    activeKeys.add(dir);
    updateMovement();
  };
  btn.onmouseup = btn.onmouseleave = () => {
    activeKeys.delete(dir);
    updateMovement();
  };
});

// Keyboard-based arrow controls (with diagonal support)
const keyMap = {
  ArrowUp: 'up',
  ArrowDown: 'down',
  ArrowLeft: 'left',
  ArrowRight: 'right'
};

document.addEventListener('keydown', e => {
  const dir = keyMap[e.key];
  if (!dir) return;
  if (!activeKeys.has(dir)) {
    activeKeys.add(dir);
    updateMovement();
  }
});

document.addEventListener('keyup', e => {
  const dir = keyMap[e.key];
  if (!dir) return;
  activeKeys.delete(dir);
  updateMovement();
});

function updateMovement() {
  if (movementInterval) clearInterval(movementInterval);

  if (activeKeys.size === 0) {
    pub(arrowTopic, commandMap.stop);
    document.querySelectorAll('.arrow.active').forEach(el => el.classList.remove('active'));
    return;
  }

  document.querySelectorAll('.arrow.active').forEach(el => el.classList.remove('active'));
  activeKeys.forEach(d => {
    const b = document.getElementById(d);
    if (b) b.classList.add('active');
  });

  sendMovement();
  movementInterval = setInterval(sendMovement, 200);
}

function sendMovement() {
  const hasUp = activeKeys.has('up');
  const hasDown = activeKeys.has('down');
  const hasLeft = activeKeys.has('left');
  const hasRight = activeKeys.has('right');

  let cmd;
  if (hasUp && hasLeft) cmd = commandMap['up-left'];
  else if (hasUp && hasRight) cmd = commandMap['up-right'];
  else if (hasDown && hasLeft) cmd = commandMap['down-left'];
  else if (hasDown && hasRight) cmd = commandMap['down-right'];
  else if (hasUp) cmd = commandMap.up;
  else if (hasDown) cmd = commandMap.down;
  else if (hasLeft) cmd = commandMap.left;
  else if (hasRight) cmd = commandMap.right;
  else cmd = commandMap.stop;

  pub(arrowTopic, cmd);
}

// ----------------- Other Button Mappings -----------------
const buttonActions = [
  { id: 'btn-manual', topic: 'robot/mode', payload: 'manual' },
  { id: 'btn-autonomous', topic: 'robot/mode', payload: 'autonomous' },
  { id: 'btn-test', topic: 'robot/mode', payload: 'test' },
  { id: 'btn-flash-led', topic: 'robot/led', payload: 'flash' },
  { id: 'btn-enable-cv', topic: 'robot/cv', payload: 'enable' },
  { id: 'btn-disable-cv', topic: 'robot/cv', payload: 'disable' },
  { id: 'btn-follow', topic: 'robot/auto', payload: 'follow' },
  { id: 'btn-return-key', topic: 'robot/auto', payload: 'return' },
  { id: 'btn-show-keys', topic: 'robot/auto', payload: 'show_keys' }
];
buttonActions.forEach(({ id, topic, payload }) => {
  const btn = document.getElementById(id);
  if (!btn) return;
  btn.onclick = () => pub(topic, payload);
});

// ----------------- Assign Key Location via Input -----------------
const assignBtn = document.getElementById('btn-assign-key');
if (assignBtn) assignBtn.onclick = () => {
  const locInput = document.getElementById('key-loc');
  if (!locInput) return;
  const loc = locInput.value.trim();
  if (loc) pub('robot/auto/key/assign', loc);
};

// ----------------- PID Form Handling -----------------
const formInner = document.getElementById('form-inner');
if (formInner) formInner.onsubmit = e => {
  e.preventDefault();
  const d = new FormData(e.target);
  const payload = JSON.stringify({
    pg: +d.get('pg'),
    dg: +d.get('dg'),
    ig: +d.get('ig'),
    sp: +d.get('sp')
  });
  pub('robot/pid/inner', payload);
};

const formOuter = document.getElementById('form-outer');
if (formOuter) formOuter.onsubmit = e => {
  e.preventDefault();
  const d = new FormData(e.target);
  const payload = JSON.stringify({
    pg: +d.get('pg'),
    dg: +d.get('dg'),
    ig: +d.get('ig'),
    sp: +d.get('sp'),
    rot: +d.get('rot')
  });
  pub('robot/pid/outer', payload);
};

// ----------------- VOICE RECOGNITION -----------------
const recognition = new window.webkitSpeechRecognition();
recognition.lang = 'en-US';
recognition.continuous = false;

// Debug logs for recognition lifecycle
recognition.onstart = () => console.log('[SpeechRecognition] Started listening');
recognition.onend = () => console.log('[SpeechRecognition] Stopped listening');
recognition.onerror = (err) => console.error('[SpeechRecognition] Error:', err);

const voiceBtn = document.getElementById('start-voice');
if (voiceBtn) {
  voiceBtn.onmousedown = () => {
    console.log('[DEBUG] Voice button pressed → start recognition');
    recognition.start();
  };
  voiceBtn.onmouseup = () => {
    console.log('[DEBUG] Voice button released → stop recognition');
    recognition.stop();
  };
} else {
  console.warn('[DEBUG] Could not find button with id="start-voice"');
}

recognition.onresult = event => {
  const transcript = event.results[0][0].transcript;
  console.log('[DEBUG] Voice result received:', transcript);
  sendToChatGPT(transcript);
};

// ----------------- sendToChatGPT -----------------
function sendToChatGPT(commandText) {
  console.log('[DEBUG] Sending to Flask server:', commandText);

  // Make sure Flask (voice_server.py) is running on port 5001
  const url = 'http://127.0.0.1:5001/interpret';
  console.log('[DEBUG] About to call fetch → URL =', url);

  fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ command: commandText })
  })
    .then(res => res.json())
    .then(data => {
      if (!data || typeof data.result !== 'string') {
        console.warn('[DEBUG] Unexpected response from Flask:', data);
        alert('Error: Invalid response from interpretation server.');
        return;
      }

      const cmd = data.result.trim().toLowerCase();
      console.log('[DEBUG] GPT Interpreted:', cmd);

      // Only publish if MQTT is connected
      if (!client.connected) {
        console.warn('[DEBUG] MQTT not connected. Cannot publish command:', cmd);
        alert('MQTT is not connected. Please check your broker connection.');
        return;
      }

      if (cmd === 'follow' || cmd === 'return' || cmd === 'stop') {
        console.log('[DEBUG] Publishing to robot/auto →', cmd);
        client.publish('robot/auto', cmd);
      } else if (cmd === 'manual' || cmd === 'autonomous') {
        console.log('[DEBUG] Publishing to robot/mode →', cmd);
        client.publish('robot/mode', cmd);
      } else {
        console.warn('[DEBUG] GPT returned unrecognized command:', cmd);
        alert("Sorry, I couldn't understand that command.");
      }
    })
    .catch(err => {
      console.error('[DEBUG] Error contacting GPT API or Flask:', err);
      alert('Error: Could not contact interpretation server.');
    });
}
