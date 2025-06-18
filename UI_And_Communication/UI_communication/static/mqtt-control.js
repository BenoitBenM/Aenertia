// mqtt-control.js

console.log('[MQTT-CTRL] mqtt-control.js loaded');

window.addEventListener('DOMContentLoaded', () => {
  // === MQTT Setup ===
  const brokerUrl = 'ws://172.20.10.9:9001';
  const client = mqtt.connect(brokerUrl, {
    keepalive: 30,
    reconnectPeriod: 1000
  });

  // === In-Browser State ===
  const savedNames  = [];   // up to 4 names
  const savedCoords = {};   // name → {x,y,theta}

  // === MQTT Event Handlers ===
  client.on('connect', () => {
    console.log('[MQTT] Connected to broker:', brokerUrl);
    document.getElementById('mqtt-status').innerText = 'MQTT: Connected';

    // Subscribe to telemetry and key-location topics
    [
      'robot/battery',
      'robot/vb',
      'robot/eu',
      'robot/keys',
      'robot/auto/key/locations'
    ].forEach(topic => {
      client.subscribe(topic, err => {
        if (err) console.error('[MQTT] Subscribe error', topic, err);
        else    console.log('[MQTT] Subscribed to', topic);
      });
    });
  });

  client.on('message', (topic, buf) => {
    const msg = buf.toString();
    console.log('[MQTT] ←', topic, msg);

    // VB telemetry
    if (topic === 'robot/vb') {
      document.getElementById('vb').innerText = `VB: ${msg}`;
      return;
    }

    // EU telemetry
    if (topic === 'robot/eu') {
      document.getElementById('eu').innerText = `EU: ${msg}`;
      return;
    }

    // Battery percentage
    if (topic === 'robot/battery') {
      document.getElementById('battery').innerText = `Battery: ${msg}%`;
      return;
    }

    // Dynamic key-assignment buttons
    if (topic === 'robot/keys') {
      let keys;
      try { keys = JSON.parse(msg); }
      catch {
        console.error('[MQTT] Invalid JSON on robot/keys:', msg);
        return;
      }
      document.getElementById('key-list').innerHTML =
        '<ul>' +
        keys.map(k =>
          `<li><button class="key-button" onclick="assignKeyLocation('${k}')">${k}</button></li>`
        ).join('') +
        '</ul>';
      console.log('[UI] Rendered dynamic keys:', keys);
      return;
    }

    // Full map of saved key-locations
    if (topic === 'robot/auto/key/locations') {
      let locs;
      try { locs = JSON.parse(msg); }
      catch {
        console.error('[MQTT] Invalid JSON on key locations:', msg);
        return;
      }
      console.log('[MQTT] Received key locations:', locs);

      // Merge into savedCoords
      Object.entries(locs).forEach(([name, coord]) => {
        savedCoords[name] = coord;
      });

      // Update each of the four slots
      for (let i = 1; i <= 4; i++) {
        const name = savedNames[i - 1];
        const el   = document.getElementById(`loc-name-${i}`);
        if (!name) {
          el.innerText = '—';
        } else if (savedCoords[name]) {
          const c = savedCoords[name];
          el.innerText = `${name}: (x=${c.x.toFixed(2)}, y=${c.y.toFixed(2)}, θ=${c.theta.toFixed(2)} rad)`;
        } else {
          el.innerText = `${name}: (x=…, y=…, θ=…)`;
        }
      }
      return;
    }
  });

  client.on('error', err => {
    console.error('[MQTT] Error:', err);
    document.getElementById('mqtt-status').innerText = 'MQTT: Error';
  });

  client.on('close', () => {
    console.warn('[MQTT] Connection closed');
    document.getElementById('mqtt-status').innerText = 'MQTT: Disconnected';
  });

  // === Helper to publish ===
  function pub(topic, payload) {
    if (client.connected) {
      client.publish(topic, payload);
      console.log('[MQTT] →', topic, payload);
    } else {
      console.warn('[MQTT] Cannot publish, disconnected:', topic, payload);
    }
  }

  // === Assign Key Location ===
  window.assignKeyLocation = name => {
    console.log('[UI] assignKeyLocation()', name);
    pub('robot/auto/key/assign', name);

    // If new and there's room, reserve next slot
    if (!savedNames.includes(name) && savedNames.length < 4) {
      savedNames.push(name);
      const idx = savedNames.length;
      console.log(`[UI] Slot #${idx} assigned to "${name}"`);
      document.getElementById(`loc-name-${idx}`)
              .innerText = `${name}: (x=…, y=…, θ=…)`;
    }
  };

  document.getElementById('btn-assign-key').onclick = () => {
    console.log('[UI] btn-assign-key clicked');
    const v = document.getElementById('key-loc').value.trim();
    if (v) assignKeyLocation(v);
  };

  // === Four Location Buttons ===
  for (let i = 1; i <= 4; i++) {
    document.getElementById(`loc-btn-${i}`).onclick = () => {
      console.log(`[UI] loc-btn-${i} clicked`);
      const name = savedNames[i - 1];
      if (!name) {
        return alert(`No location ${i} assigned`);
      }
      const coord = savedCoords[name];
      if (!coord) {
        return alert(`Coordinates for "${name}" not yet known`);
      }
      pub('robot/goto_keyloc', JSON.stringify(coord));
      console.log('[MQTT] goto_keyloc →', coord);
    };
  }

  // === UI Clock ===
  setInterval(() => {
    const now = new Date();
    document.getElementById('time').innerText =
      now.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  }, 10000);

  // === Arrow/PAD Controls ===
  console.log('[UI] Initializing arrow/PAD controls...');
  const commandMap = {
    up: 'up', down: 'down', left: 'left', right: 'right',
    stop: 'stop',
    'up-left':'up-left', 'up-right':'up-right',
    'down-left':'down-left','down-right':'down-right'
  };
  const arrowTopic = 'robot/manual/command';
  const activeKeys = new Set();
  let movementInterval = null;

  ['up','down','left','right'].forEach(dir => {
    const btn = document.getElementById(dir);
    if (!btn) return;
    btn.onmousedown = () => { activeKeys.add(dir);  updateMovement(); };
    btn.onmouseup   = btn.onmouseleave = () => { activeKeys.delete(dir); updateMovement(); };
  });

  const keyMap = {
    ArrowUp: 'up',    ArrowDown: 'down',
    ArrowLeft: 'left',ArrowRight: 'right'
  };
  document.addEventListener('keydown', e => {
    const d = keyMap[e.key];
    if (d && !activeKeys.has(d)) { activeKeys.add(d); updateMovement(); }
  });
  document.addEventListener('keyup', e => {
    const d = keyMap[e.key];
    if (d) { activeKeys.delete(d); updateMovement(); }
  });

  function updateMovement() {
    if (movementInterval) clearInterval(movementInterval);

    if (activeKeys.size === 0) {
      pub(arrowTopic, commandMap.stop);
      document.querySelectorAll('.arrow.active').forEach(el => el.classList.remove('active'));
      return;
    }

    document.querySelectorAll('.arrow.active').forEach(el => el.classList.remove('active'));
    activeKeys.forEach(d => document.getElementById(d).classList.add('active'));

    sendMovement();
    movementInterval = setInterval(sendMovement, 200);
  }

  function sendMovement() {
    const has = d => activeKeys.has(d);
    let cmd = commandMap.stop;
    if (has('up') && has('left'))   cmd = commandMap['up-left'];
    else if (has('up') && has('right'))  cmd = commandMap['up-right'];
    else if (has('down') && has('left')) cmd = commandMap['down-left'];
    else if (has('down') && has('right'))cmd = commandMap['down-right'];
    else if (has('up')) cmd = commandMap.up;
    else if (has('down')) cmd = commandMap.down;
    else if (has('left')) cmd = commandMap.left;
    else if (has('right'))cmd = commandMap.right;

    pub(arrowTopic, cmd);
  }

  // === Other Button Mappings ===
  console.log('[UI] Binding mode & test buttons...');
  [
    { id:'btn-manual',      topic:'robot/mode', payload:'manual' },
    { id:'btn-autonomous',  topic:'robot/mode', payload:'autonomous' },
    { id:'btn-test',        topic:'robot/mode', payload:'test' },
    { id:'btn-flash-led',   topic:'robot/led',  payload:'flash' },
    { id:'btn-enable-cv',   topic:'robot/cv',   payload:'enable' },
    { id:'btn-disable-cv',  topic:'robot/cv',   payload:'disable' },
    { id:'btn-follow',      topic:'robot/auto', payload:'follow' }
  ].forEach(({id,topic,payload}) => {
    const b = document.getElementById(id);
    if (!b) return;
    b.onclick = () => pub(topic, payload);
  });

  // === PID Form Handling ===
  console.log('[UI] Binding PID forms...');
  const formInner = document.getElementById('form-inner');
  if (formInner) formInner.onsubmit = e => {
    e.preventDefault();
    const d = new FormData(e.target);
    pub('robot/pid/inner', JSON.stringify({
      pg:+d.get('pg'), dg:+d.get('dg'),
      ig:+d.get('ig'), sp:+d.get('sp')
    }));
  };

  const formOuter = document.getElementById('form-outer');
  if (formOuter) formOuter.onsubmit = e => {
    e.preventDefault();
    const d = new FormData(e.target);
    pub('robot/pid/outer', JSON.stringify({
      pg:+d.get('pg'), dg:+d.get('dg'),
      ig:+d.get('ig'), sp:+d.get('sp'),
      rot:+d.get('rot')
    }));
  };

  // === Voice Recognition ===
  console.log('[UI] Initializing voice recognition...');
  const recognition = new window.webkitSpeechRecognition();
  recognition.lang = 'en-US';
  recognition.continuous = false;

  recognition.onstart = () => console.log('[Voice] Started listening');
  recognition.onend   = () => console.log('[Voice] Stopped listening');
  recognition.onerror = err => console.error('[Voice] Error:', err);

  const startBtns = [
    document.getElementById('start-voice'),
    document.getElementById('start-voice-assign')
  ].filter(Boolean);

  startBtns.forEach(btn => {
    btn.onmousedown = () => {
      console.log('[Voice] button DOWN → start recognition');
      recognition.start();
    };
    btn.onmouseup = () => {
      console.log('[Voice] button UP → stop recognition');
      recognition.stop();
    };
  });

  recognition.onresult = event => {
    const transcript = event.results[0][0].transcript;
    console.log('[Voice] Transcript:', transcript);
    sendToChatGPT(transcript);
  };

  function sendToChatGPT(commandText) {
    console.log('[Voice] Sending to server:', commandText);
    fetch('http://127.0.0.1:5001/interpret', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({command:commandText})
    })
      .then(r => r.json())
      .then(data => {
        const cmd = (data.result||'').trim().toLowerCase();
        console.log('[Voice] Interpreted command:', cmd);
        if (!client.connected) {
          return alert('MQTT not connected');
        }
        if (['follow','return','stop'].includes(cmd)) {
          pub('robot/auto', cmd);
        } else if (['manual','autonomous'].includes(cmd)) {
          pub('robot/mode', cmd);
        } else {
          alert('Unrecognized voice command');
        }
      })
      .catch(err => {
        console.error('[Voice] sendToChatGPT error:', err);
        alert('Voice interpretation error');
      });
  }
});  // end DOMContentLoaded
