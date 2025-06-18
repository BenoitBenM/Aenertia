// mqtt-control.js

console.log('[MQTT-CTRL] mqtt-control.js loaded');

window.addEventListener('DOMContentLoaded', () => {
  // === MQTT Setup ===
  const brokerUrl = 'ws://172.20.10.6:9001';
  const client    = mqtt.connect(brokerUrl, { keepalive:30, reconnectPeriod:1000 });

  // === In-Browser State ===
  const savedNames  = [];    // up to 4
  const savedCoords = {};    // name → {x,y,theta}

  // === Helper to publish ===
  function pub(topic, payload) {
    if (client.connected) {
      client.publish(topic, payload);
      console.log('[MQTT] →', topic, payload);
    } else {
      console.warn('[MQTT] Cannot publish, disconnected:', topic);
    }
  }

  // === On Connect ===
  client.on('connect', () => {
    console.log('[MQTT] Connected to', brokerUrl);
    document.getElementById('mqtt-status').innerText = 'MQTT: Connected';

    // subscribe to needed topics
    ['robot/battery','robot/vb','robot/eu','robot/keys','robot/auto/key/locations']
      .forEach(t => client.subscribe(t, err => {
        if (err) console.error('[MQTT] Sub error', t, err);
        else    console.log('[MQTT] Subscribed to', t);
      }));
  });

  // === On Message ===
  client.on('message', (topic, buf) => {
    const msg = buf.toString();
    console.log('[MQTT] ←', topic, msg);

    if (topic === 'robot/vb') {
      document.getElementById('vb').innerText = `VB: ${msg}`;
      return;
    }
    if (topic === 'robot/eu') {
      document.getElementById('eu').innerText = `EU: ${msg}`;
      return;
    }
    if (topic === 'robot/battery') {
      document.getElementById('battery').innerText = `Battery: ${msg}%`;
      return;
    }
    if (topic === 'robot/keys') {
      let keys;
      try { keys = JSON.parse(msg); }
      catch { return console.error('[MQTT] bad keys JSON'); }
      document.getElementById('key-list').innerHTML =
        '<ul>'+keys.map(k=>
          `<li><button class="key-button" onclick="assignKeyLocation('${k}')">${k}</button></li>`
        ).join('')+'</ul>';
      return;
    }
    if (topic === 'robot/auto/key/locations') {
      let locs;
      try { locs = JSON.parse(msg); }
      catch { return console.error('[MQTT] bad locs JSON'); }
      console.log('[MQTT] got locations:', locs);
      Object.entries(locs).forEach(([name, coord])=>{
        savedCoords[name] = coord;
      });

      // Push to backend to save in DynamoDB
      Object.entries(locs).forEach(([name, coord]) => {
        if (coord && coord.x !== undefined && coord.y !== undefined) {
          fetch("http://localhost:8000/store_key_location", {
            method: "POST",
            headers: {
              "Content-Type": "application/json"
            },
            body: JSON.stringify({
              name: name,
              x: coord.x,
              y: coord.y,
              theta: coord.theta || 0.0
            })
          }).then(res => res.json())
            .then(data => console.log("[DynamoDB] Saved:", data))
            .catch(err => console.error("[DynamoDB] Save failed:", err));
        }
      });


      // update 4 UI slots
      for (let i=1; i<=4; i++) {
        const name = savedNames[i-1];
        const el   = document.getElementById(`loc-name-${i}`);
        if (!name) {
          el.innerText = '—';
        } else if (savedCoords[name]) {
          const c = savedCoords[name];
          el.innerText = `${name}: (x=${c.x.toFixed(2)}, y=${c.y.toFixed(2)}, θ=${c.theta.toFixed(2)} rad)`;
        } else {
          el.innerText = `${name}: (x=…, y=…, θ=…)`;
        }
      }
    }
  });

  client.on('error', e => {
    console.error('[MQTT] Error:', e);
    document.getElementById('mqtt-status').innerText = 'MQTT: Error';
  });
  client.on('close', () => {
    console.warn('[MQTT] Disconnected');
    document.getElementById('mqtt-status').innerText = 'MQTT: Disconnected';
  });

  window.assignKeyLocation = name => {
  console.log('[UI] assignKeyLocation()', name);
  // always publish—bridge will look up or re-compute coords
  pub('robot/auto/key/assign', name);
    // 2) if brand-new, stick it into the next empty slot
  let idx = savedNames.indexOf(name);
  if (idx === -1 && savedNames.length < 4) {
    savedNames.push(name);
    idx = savedNames.length - 1;
  }

  // 3) only reset **that one** slot to placeholder
  if (idx !== -1) {
    const el = document.getElementById(`loc-name-${idx+1}`);
    el.innerText = `${name}: (x=…, y=…, θ=…)`;
  }
};
  document.getElementById('btn-assign-key').onclick = () => {
    const n = document.getElementById('key-loc').value.trim();
    if (n) assignKeyLocation(n);
  };

  // === Reset Locations ===
  document.getElementById('btn-reset-locs').onclick = () => {
    console.log('[UI] Reset Locations');
    savedNames.length = 0;
    Object.keys(savedCoords).forEach(k=>delete savedCoords[k]);
    for (let i=1; i<=4; i++) {
      document.getElementById(`loc-name-${i}`).innerText = '—';
    }
  };

  // === Four Return Buttons ===
  for (let i=1; i<=4; i++) {
    document.getElementById(`loc-btn-${i}`).onclick = () => {
      console.log(`[UI] loc-btn-${i} clicked`);
      const name = savedNames[i-1];
      if (!name)  return alert(`No location #${i} assigned`);
      const coord = savedCoords[name];
      if (!coord) return alert(`Coordinates for "${name}" unknown`);
      pub('robot/goto_keyloc', JSON.stringify(coord));
      console.log('[MQTT] goto_keyloc →', coord);
    };
  }

  // === UI Clock ===
  setInterval(()=>{
    const now = new Date();
    document.getElementById('time').innerText =
      now.toLocaleTimeString([], {hour:'2-digit', minute:'2-digit'});
  }, 10000);

  // === Arrow / PAD Controls ===
  const cmdMap = { up:'up',down:'down',left:'left',right:'right',stop:'stop',
                   'up-left':'up-left','up-right':'up-right',
                   'down-left':'down-left','down-right':'down-right' };
  const arrowTopic = 'robot/manual/command';
  const activeKeys = new Set();
  let mvInterval = null;

  ['up','down','left','right'].forEach(dir=>{
    const b = document.getElementById(dir);
    if (!b) return;
    b.onmousedown = ()=>{activeKeys.add(dir); updateMv();};
    b.onmouseup   = b.onmouseleave = ()=>{activeKeys.delete(dir); updateMv();};
  });
  const keyMap = { ArrowUp:'up',ArrowDown:'down',ArrowLeft:'left',ArrowRight:'right' };
  document.addEventListener('keydown', e=>{
    const d = keyMap[e.key];
    if (d && !activeKeys.has(d)) { activeKeys.add(d); updateMv(); }
  });
  document.addEventListener('keyup', e=>{
    const d = keyMap[e.key];
    if (d) { activeKeys.delete(d); updateMv(); }
  });

  function updateMv() {
    if (mvInterval) clearInterval(mvInterval);
    if (activeKeys.size===0) {
      pub(arrowTopic, cmdMap.stop);
      document.querySelectorAll('.arrow.active')
              .forEach(el=>el.classList.remove('active'));
      return;
    }
    document.querySelectorAll('.arrow.active')
            .forEach(el=>el.classList.remove('active'));
    activeKeys.forEach(d=>document.getElementById(d).classList.add('active'));
    sendMv();
    mvInterval = setInterval(sendMv, 200);
  }

  function sendMv() {
    const has = d=>activeKeys.has(d);
    let cmd = cmdMap.stop;
    if      (has('up')&&has('left'))   cmd = cmdMap['up-left'];
    else if (has('up')&&has('right'))  cmd = cmdMap['up-right'];
    else if (has('down')&&has('left')) cmd = cmdMap['down-left'];
    else if (has('down')&&has('right'))cmd = cmdMap['down-right'];
    else if (has('up'))    cmd=cmdMap.up;
    else if (has('down'))  cmd=cmdMap.down;
    else if (has('left'))  cmd=cmdMap.left;
    else if (has('right')) cmd=cmdMap.right;
    pub(arrowTopic, cmd);
  }

  // === Other Buttons ===
  [
    {id:'btn-manual',    topic:'robot/mode', payload:'manual'},
    {id:'btn-autonomous',topic:'robot/mode', payload:'autonomous'},
    {id:'btn-test',      topic:'robot/mode', payload:'test'},
    {id:'btn-flash-led', topic:'robot/led',  payload:'flash'},
    {id:'btn-enable-cv', topic:'robot/cv',   payload:'enable'},
    {id:'btn-disable-cv',topic:'robot/cv',   payload:'disable'},
    {id:'btn-follow',    topic:'robot/auto', payload:'follow'}
  ].forEach(({id,topic,payload})=>{
    const b = document.getElementById(id);
    if (!b) return;
    b.onclick = ()=> pub(topic, payload);
  });

  // === PID Forms ===
  ['form-inner','form-outer'].forEach(formId=>{
    const f = document.getElementById(formId);
    if (!f) return;
    f.onsubmit = e=>{
      e.preventDefault();
      const d = new FormData(f);
      const obj = {};
      ['pg','dg','ig','sp','rot'].forEach(k=>{
        if (d.has(k)) obj[k] = +d.get(k);
      });
      const topic = formId==='form-inner' ? 'robot/pid/inner' : 'robot/pid/outer';
      pub(topic, JSON.stringify(obj));
    };
  });

  // === Voice Control ===
  console.log('[Voice] initializing');
  const recog = new window.webkitSpeechRecognition();
  recog.lang = 'en-US'; recog.continuous = false;
  recog.onstart = ()=>console.log('[Voice] started');
  recog.onend   = ()=>console.log('[Voice] ended');
  recog.onerror = err=>console.error('[Voice] error',err);

  const vb = document.getElementById('start-voice');
  if (vb) {
    vb.onmousedown = ()=>recog.start();
    vb.onmouseup   = ()=>recog.stop();
  }
  recog.onresult = e=>{
    const txt = e.results[0][0].transcript;
    console.log('[Voice] transcript',txt);
    fetch('http://127.0.0.1:5001/interpret',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({command:txt})
    })
    .then(r=>r.json())
    .then(d=>{
      const c = (d.result||'').trim().toLowerCase();
      console.log('[Voice] interpreted',c);
      if (!client.connected) return alert('MQTT not connected');
      if (['follow','return','stop'].includes(c)) pub('robot/auto',c);
      else if (['manual','autonomous'].includes(c)) pub('robot/mode',c);
      else alert('Unrecognized');
    })
    .catch(err=>{
      console.error('[Voice] send failed',err);
      alert('Voice error');
    });
  };

});
