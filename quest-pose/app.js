import {startQuest, touchLayout} from './xr.js';
import {axisTurn, drawScene, identity, multiply, wristAngles} from './scene.js';

const $ = id => document.getElementById(id);
const pad = $('pad');
const cameraCanvas = $('camera-frame');
const frameContext = cameraCanvas.getContext('2d');
const sides = ['left', 'right'];
const mouseHome = [0.25, 1.13, -0.4];
const freshMouse = () => ({p: [...mouseHome], q: [...identity], trigger: 0, stickY:0, a:false, b:false,
  padGrip: false, keyGrip: false, buttonGrip: false});
let mouse = freshMouse();
const modeHelp = {
  move: 'Right-drag to clutch and move the arm. Scroll while holding the right button to control depth. Left-drag to reposition the released handle.',
  rotate: 'Right-drag left or right for yaw and up or down for pitch. Scroll with the right button held to roll the wrist about the tool axis.',
  gripper: 'Right-drag toward the right to squeeze the index trigger and close the jaws. Drag back left to ease the trigger and open them. Release to hold the opening.'
};
let mouseMode = 'move';
let mode = 'observer', sessionId = '', seq = 0, side = 'right', frameName = 'mouse-local';
let xrSession = null, xrMode = '', switching = false, inFlight = false;
let state = {}, latestApplied = 0, requestNumber = 0, lastSend = -Infinity, lastRoundTrip = null;
let visibleLoss = 0, frameReset = 0, signalCounter = 0;
let events = [], nextEvent = 1, edges = new Map(), eventOverflow = false;
let gripHistory = {};
let camera = {id: '', label: '', index: null, arrival: 0, sourceAge: 0};
let cameraProblem = '';
let displayedId = '', displayPending = false, displayGeneration = 0;
let freshDisplayAt = 0;
let drag = null;
const fields = [
  ['initial','Initial hand'], ['pose','Current hand'], ['delta','From initial'],
  ['anchor','Clutch anchor'], ['clutch_delta','From clutch'], ['tool','Tool target']
];

function message(value = '') {
  $('notice').textContent = value;
  $('notice').hidden = !value;
}
function stickMode() { return state.gripper_mode === 'stick'; }
function jawReadout(value) {
  if (!stickMode()) return Math.round(value*100)+'%';
  if (Math.abs(value) <= .15) return 'Centered';
  return (value < 0 ? 'Open ' : 'Close ') + Math.round(Math.abs(value)*100)+'%';
}
function setModeControls() {
  $('mouse-start').hidden = mode !== 'observer';
  $('xr-start').hidden = mode !== 'observer';
  $('stop').hidden = mode === 'observer';
  $('mouse-start').disabled = switching;
  $('xr-start').disabled = !xrMode || switching;
  $('clutch').disabled = mode !== 'mouse';
  $('trigger').disabled = mode !== 'mouse';
  $('trigger').min = stickMode() ? '-100' : '0';
  $('jaw-control-label').textContent = stickMode() ? 'Thumbstick Y' : 'Index trigger';
  $('face-a').disabled = $('face-b').disabled = mode !== 'mouse';
  for (const name of Object.keys(modeHelp)) {
    $('mode-'+name).disabled = mode === 'quest';
    $('mode-'+name).setAttribute('aria-pressed',String(mouseMode === name));
  }
  if (mode === 'quest') {
    $('mode-help').textContent = 'Hold the side grip to move or rotate the tool. ' + (stickMode()
      ? 'Tilt the thumbstick up to open or down to close. Center it to keep the opening while moving.'
      : 'Squeeze the trigger to close or ease off to open. Release the side grip first to keep the opening.');
  } else if (stickMode() && mouseMode === 'gripper') {
    $('mode-help').textContent = 'Right-drag up from the press point to open or down to close. Move back to the press point to center the stick. With the pad focused and grip held, Up / Down arrows also work in any mode. Release the arrow to hold the opening.';
  } else $('mode-help').textContent = modeHelp[mouseMode];
}
function beginInput(name, frame) {
  mode = name;
  frameName = frame;
  sessionId = [...crypto.getRandomValues(new Uint32Array(4))].map(v => v.toString(16).padStart(8,'0')).join('');
  seq = 0; lastSend = -Infinity; inFlight = false; visibleLoss = 0; frameReset = 0;
  freshDisplayAt = performance.now();
  events = []; nextEvent = 1; edges.clear(); gripHistory = {}; eventOverflow = false; displayedId = '';
  message();
  setModeControls();
}

async function request(path, body) {
  const num = ++requestNumber;
  const options = body === undefined ? {} : {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)
  };
  const abort = new AbortController();
  const timeout = setTimeout(() => abort.abort(), 1800);
  try {
    const response = await fetch(path, {...options, cache: 'no-store', signal: abort.signal});
    const data = await response.json().catch(() => ({}));
    if (!response.ok) {
      const error = new Error(data.error || data.message || ('HTTP ' + response.status));
      error.status = response.status;
      throw error;
    }
    if (num >= latestApplied) {
      latestApplied = num;
      const oldMode = state.gripper_mode;
      if (state.recording?.selected_camera !== data.recording?.selected_camera ||
          (data.recording?.active && !state.recording?.active)) {
        displayedId = '';
        displayGeneration++;
      }
      state = data;
      if (oldMode !== data.gripper_mode) setModeControls();
      render();
    }
    return data;
  } finally { clearTimeout(timeout); }
}
async function control(action, extra = {}) {
  try {
    const result = await request('/control', {action, ...(sessionId && {session: sessionId}), ...extra});
    if (action === 'capture') {
      message(result.captured?.length ? 'Initial pose captured for ' + result.captured.join(' and ') + '. Saved initial_poses.json.'
        : 'Release the grip with the controller tracked, then capture again.');
    } else message();
    return result;
  } catch (e) {
    message(action + ' · ' + e.message);
    return null;
  }
}

function grip() { return mouse.padGrip || mouse.keyGrip || mouse.buttonGrip; }
function releaseMouseGrip() {
  mouse.padGrip = mouse.buttonGrip = mouse.keyGrip = false;
  mouse.a = mouse.b = false;
  if (stickMode()) updateJawInput(0);
  drag = null;
}
function resetEdges() { edges = new Map(); }
function enqueue(action) {
  if (events.length >= 24) {
    // Stop accepting input on sustained event backpressure. Latch hold on the next delivered sample.
    if (!eventOverflow) {
      eventOverflow = true;
      visibleLoss = ++signalCounter;
      message('Input event buffer is full. Resume locally after reconnecting.');
    }
    return;
  }
  events.push({id: nextEvent++, action});
}
function readEdges(hands) {
  for (const side of sides) {
    const hand = hands[side];
    const actions = side === 'right' ? ['record','hold'] : ['view','mark'];
    if (!hand?.pose || !touchLayout(hand.profile,side).length) {
      for (const action of actions) edges.delete(action);
      continue;
    }
    for (let i=0; i<actions.length; i++) {
      const action = actions[i], down = Boolean(hand.buttons[4+i]?.pressed);
      const previous = edges.get(action);
      if (down && previous?.source === hand.source && previous.down === false) {
        const left = hands.left;
        const returnChord = action === 'record' && left?.pose &&
          touchLayout(left.profile,'left').length && left.buttons[3]?.pressed;
        // Use the explicit finish action even if this browser's recording state
        // is stale. A finish chord received while idle must never start a trial.
        enqueue(returnChord ? 'finish_return' : action);
      }
      edges.set(action, {source:hand.source, down});
    }
  }
}

// Retain a release observed between network sends. A quick re-grip must never
// continue from a clutch anchor captured before the operator moved their hand.
function trackGripEpochs(hands) {
  for (const side of sides) {
    const hand = hands[side];
    const lane = gripHistory[side] ||= {epoch:0, engaged:false, source:null};
    if (hand?.source !== lane.source || !hand?.pose) {
      if (lane.engaged) lane.epoch++;
      lane.engaged = false;
      lane.source = hand?.source;
    }
    if (!hand) continue;
    const value = hand.buttons[1]?.value || 0;
    if (hand.pose && value >= .65) lane.engaged = true;
    if (lane.engaged && value <= .35) { lane.engaged = false; lane.epoch++; }
    hand.grip_epoch = lane.epoch;
  }
}
async function transmit(input, force = false) {
  if (!sessionId) return;
  trackGripEpochs(input.hands);
  if (!visibleLoss && input.visible) readEdges(input.hands);
  if (inFlight || (!force && performance.now()-lastSend < 28)) return;
  inFlight = true;
  const owner = sessionId, reset = frameReset, inactive = visibleLoss, started = performance.now();
  lastSend = started;
  const sample = {...input, session: owner, seq: ++seq, reset: Boolean(reset),
    visible: input.visible && !inactive, events: [...events]};
  if (mode === 'quest' && state.recording?.camera_mode === 'opencv') {
    sample.view_valid = started - freshDisplayAt <= 1200;
  }
  try {
    const result = await request('/sample', sample);
    if (owner !== sessionId) return;
    lastRoundTrip = performance.now()-started;
    if (result.accepted !== false) {
      if (reset === frameReset) frameReset = 0;
      if (inactive === visibleLoss) visibleLoss = 0;
      events = events.filter(event => event.id > result.ack);
      if (!events.length) eventOverflow = false;
    }
  } catch (e) {
    if (owner !== sessionId) return;
    if (e.status === 409 || e.status === 403 || e.status === 400) {
      if (xrSession) await xrSession.end().catch(() => {});
      detach();
      message('Input stopped · ' + e.message);
    } else {
      resetEdges();
      message('Sample delivery interrupted · ' + e.message);
    }
  } finally { if (owner === sessionId) inFlight = false; }
}
function mouseInput() {
  const button = value => ({pressed: value > .5, value, touched: value > 0});
  return {
    frame: 'mouse-local', time_ms: performance.now(), visible: !document.hidden,
    head: {p: [0,1.55,0], q: [...identity]},
    hands: {left: null, right: {
      pose: {p: [...mouse.p], q: [...mouse.q]}, handedness: 'right', profile: 'mouse', profiles: ['mouse'],
      source: 'mouse', mapping: 'xr-standard', target_ray_mode: 'tracked-pointer',
      buttons: [button(mouse.trigger), button(grip() ? 1 : 0), button(0), button(0),
        button(Number(mouse.a)), button(Number(mouse.b)), button(0)],
      axes: [0,0,0,mouse.stickY]
    }}
  };
}
function inactive() {
  if (!sessionId) return;
  visibleLoss = ++signalCounter;
  releaseMouseGrip();
  resetEdges();
  transmit({frame:frameName, time_ms:performance.now(), visible:false, head:null, hands:{left:null,right:null}}, true);
}
function detach() {
  const old = sessionId;
  sessionId = ''; mode = 'observer'; xrSession = null;
  releaseMouseGrip();
  setModeControls();
  render();
  return old;
}
async function disconnect() {
  if (!sessionId) return;
  const old = detach();
  await control('release', {session: old});
}
$('mouse-start').addEventListener('click', () => {
  mouse = freshMouse();
  $('trigger').value = '0';
  $('trigger-value').textContent = jawReadout(0);
  selectSide('right');
  beginInput('mouse','mouse-local');
  transmit(mouseInput(), true);
  pad.focus();
});
$('stop').addEventListener('click', async () => {
  if (xrSession) await xrSession.end().catch(e => message(e.message));
  else await disconnect();
});
$('xr-start').addEventListener('click', async () => {
  switching = true;
  setModeControls();
  try {
    xrSession = await startQuest(xrMode, {
      onStart: frame => beginInput('quest',frame),
      onSample: sample => transmit(sample),
      onReset: () => { frameReset = ++signalCounter; resetEdges(); },
      onInactive: inactive, onEnd: disconnect, onDisplay: acknowledgeDisplay, preview
    });
  } catch (e) { message('Quest session · ' + e.message); }
  switching = false;
  setModeControls();
});
if (navigator.xr && window.isSecureContext) {
  (async () => {
    for (const candidate of ['immersive-ar','immersive-vr']) {
      if (await navigator.xr.isSessionSupported(candidate).catch(() => false)) { xrMode = candidate; break; }
    }
    setModeControls();
  })();
}
window.addEventListener('blur', () => { if (mode === 'mouse') inactive(); });
document.addEventListener('visibilitychange', () => { if (document.hidden && mode === 'mouse') inactive(); });
window.addEventListener('pagehide', () => {
  if (sessionId) navigator.sendBeacon('/control', new Blob([JSON.stringify({action:'release',session:sessionId})], {type:'application/json'}));
});

function updateJawInput(value) {
  if (stickMode()) mouse.stickY = Math.max(-1,Math.min(1,value));
  else mouse.trigger = Math.max(0,Math.min(1,value));
  const val = stickMode() ? mouse.stickY : mouse.trigger;
  $('trigger').value = Math.round(val*100);
  $('trigger-value').textContent = jawReadout(val);
}
$('trigger').addEventListener('input', e => updateJawInput(Number(e.target.value)/100));
for (const event of ['change','blur','pointercancel']) {
  $('trigger').addEventListener(event, () => { if (stickMode()) updateJawInput(0); });
}
for (const name of Object.keys(modeHelp)) {
  $('mode-'+name).addEventListener('click', () => {
    mouseMode = name;
    setModeControls();
  });
}
for (const key of ['a','b']) {
  const face = $('face-'+key);
  function press(down) {
    if (mode !== 'mouse' || mouse[key] === down) return;
    mouse[key] = down;
    transmit(mouseInput(),true);
  }
  face.addEventListener('pointerdown', e => {
    if (e.button !== 0) return;
    face.setPointerCapture(e.pointerId);
    press(true);
  });
  for (const event of ['pointerup','pointercancel','lostpointercapture']) {
    face.addEventListener(event, () => press(false));
  }
  face.addEventListener('click', e => {
    // Keyboard and assistive activation have no physical pointer press.
    if (!e.detail) { press(true); press(false); }
  });
}
window.addEventListener('keydown', e => {
  if (mode !== 'mouse' || ![pad,$('trigger'),$('clutch')].includes(document.activeElement)) return;
  if (e.code === 'Space') {
    e.preventDefault();
    if (!mouse.keyGrip) { mouse.keyGrip = true; transmit(mouseInput(),true); }
  }
  if (!stickMode() && (e.code === 'BracketLeft' || e.code === 'BracketRight')) {
    e.preventDefault(); updateJawInput(mouse.trigger + (e.code === 'BracketRight' ? .05 : -.05));
  }
  if (stickMode() && (e.code === 'ArrowUp' || e.code === 'ArrowDown')) {
    e.preventDefault(); updateJawInput(e.code === 'ArrowUp' ? -1 : 1);
  }
});
window.addEventListener('keyup', e => {
  if (mode === 'mouse' && stickMode() && (e.code === 'ArrowUp' || e.code === 'ArrowDown')) {
    e.preventDefault(); updateJawInput(0);
  }
  if (e.code === 'Space' && mouse.keyGrip) {
    mouse.keyGrip = false;
    if (mode === 'mouse') transmit(mouseInput(),true);
  }
});
$('clutch').addEventListener('pointerdown', e => {
  if (mode !== 'mouse') return;
  e.currentTarget.setPointerCapture(e.pointerId);
  mouse.buttonGrip = true;
  transmit(mouseInput(),true);
});
for (const event of ['pointerup','pointercancel','lostpointercapture']) {
  $('clutch').addEventListener(event, () => {
    if (!mouse.buttonGrip) return;
    mouse.buttonGrip = false;
    if (mode === 'mouse') transmit(mouseInput(),true);
  });
}
pad.addEventListener('contextmenu', e => e.preventDefault());
pad.addEventListener('pointerdown', e => {
  if (mode !== 'mouse' || ![0,2].includes(e.button)) return;
  e.preventDefault(); pad.focus(); pad.setPointerCapture(e.pointerId);
  mouse.padGrip = Boolean(e.buttons & 2);
  drag = {id:e.pointerId, x:e.clientX, y:e.clientY, startY:e.clientY};
  transmit(mouseInput(), true);
});
pad.addEventListener('pointermove', e => {
  if (mode !== 'mouse' || !drag || e.pointerId !== drag.id) return;
  const dx = e.clientX-drag.x, dy = e.clientY-drag.y;
  drag.x = e.clientX; drag.y = e.clientY;
  mouse.padGrip = Boolean(e.buttons & 2);
  if (e.shiftKey || mouseMode === 'rotate') {
    mouse.q = multiply(axisTurn([0,1,0],dx*.009), multiply(mouse.q, axisTurn([1,0,0],dy*.009)));
  } else if (mouseMode === 'gripper') {
    updateJawInput(stickMode() ? (e.clientY-drag.startY)/65 : mouse.trigger + dx/160);
  } else {
    mouse.p[0] += dx*.001;
    mouse.p[1] -= dy*.001;
  }
});
for (const event of ['pointerup','pointercancel','lostpointercapture']) {
  pad.addEventListener(event, () => {
    if (stickMode() && mouseMode === 'gripper') updateJawInput(0);
    mouse.padGrip = false; drag = null;
    if (mode === 'mouse') transmit(mouseInput(), true);
  });
}
pad.addEventListener('wheel', e => {
  if (mode !== 'mouse') return;
  e.preventDefault();
  // Normalize line/page scroll devices to the equivalent pixel distance.
  const dy = e.deltaY * (e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? pad.clientHeight : 1);
  if (e.altKey || mouseMode === 'rotate') mouse.q = multiply(mouse.q,axisTurn([0,0,1],dy*.002));
  else if (mouseMode === 'move') mouse.p[2] += dy*.0004;
}, {passive:false});
for (const action of ['capture','record','finish_return','mark','view','hold','resume']) {
  $(action).addEventListener('click', () => control(action));
}
function selectSide(value) {
  side = value;
  for (const s of sides) $(s+'-tab').setAttribute('aria-pressed', String(s === side));
  render();
}
for (const s of sides) $(s+'-tab').addEventListener('click', () => selectSide(s));

function number(v, digits=3) { return Number.isFinite(v) ? (Math.abs(v) < .5*10**-digits ? 0 : v).toFixed(digits) : '—'; }
function clock(seconds) {
  const tenths = Math.floor(Math.max(0,seconds || 0)*10);
  return String(Math.floor(tenths/600)).padStart(2,'0') + ':' + String(Math.floor(tenths/10)%60).padStart(2,'0') + '.' + tenths%10;
}
const rowCells = {};
for (const [key,label] of fields) {
  const row = $('pose-table').insertRow();
  row.insertCell().textContent = label;
  rowCells[key] = Array.from({length:4}, () => row.insertCell());
}
let lastScene = '';
function updateScene(hand) {
  const robots=sides.map(s => state.hands?.[s]?.robot);
  const key=JSON.stringify([pad.clientWidth,pad.clientHeight,devicePixelRatio,side,
    hand?.pose?.q,hand?.delta?.p,hand?.clutched,hand?.buttons?.[0]?.value,
    hand?.buttons?.[4]?.pressed,hand?.buttons?.[5]?.pressed,
    ...robots.map(r=>[r?.joint_angles_rad,r?.jaw_opening_m])]);
  if(key===lastScene) return;
  drawScene(pad,state.hands,side);
  lastScene=key;
}
function render() {
  const h = state.hands?.[side], rec = state.recording || {};
  const source = h?.profile || '';
  const inputLabel = state.session ? (state.frame === 'mouse-local' || state.hands?.right?.profile === 'mouse' ? 'Virtual right Quest controller' : 'Quest controllers') : 'Waiting for input';
  $('source-label').textContent = inputLabel + (state.session && mode === 'observer' ? ' · observing' : '');
  $('clutch-state').textContent = state.held ? 'Software hold' : h?.clutched ? 'Grip engaged' : h?.status || 'Grip released';
  $('clutch').setAttribute('aria-pressed',String(grip() && mode === 'mouse'));
  $('handle-label').textContent = (side === 'right' ? 'Right' : 'Left') + ' Quest handle';
  const robot = h?.robot;
  $('jaw-opening').textContent = robot ? number(robot.jaw_opening_m * 1000,1) + ' mm' : '—';
  $('wrist-rotation').textContent = robot ? wristAngles(robot.tcp.q).map((angle,i) =>
    ['Pitch','Yaw','Roll'][i] + ' ' + number(angle,0) + '°').join(' · ') : '—';
  $('robot-joints').textContent = robot ? robot.joint_angles_rad.map((v,i) =>
    'J' + (i+1) + ' ' + number(v*180/Math.PI,1) + '°').join('  ·  ') : 'Waiting for robot joints';
  $('pickup-help').hidden = !h?.clutched || (stickMode() ? h.stick_live : h.trigger_live);
  $('pickup-help').textContent = stickMode() ? 'Center the thumbstick to take over the jaw opening.'
    : 'Grasp held. Bring the trigger to ' + number(h?.closure*100,0) + '% to take over the jaw opening.';
  const rejected=h?.status === 'outside simulated reach';
  $('reach-help').hidden = !rejected;
  $('reach-help').textContent = rejected ? 'Arm held at last reachable pose (' + (robot?.status || h.status)
    + '). Release the grip, reposition the handle, then clutch again.' : '';
  if (mode !== 'mouse') {
    const val = stickMode() ? h?.axes?.[3] || 0 : h?.buttons?.[0]?.value || 0;
    $('trigger').value = Math.round(val*100);
    $('trigger-value').textContent = jawReadout(val);
  }
  $('inspect-label').textContent = (side === 'right' ? 'Right' : 'Left') + ' hand · +X right, +Y up, −Z forward';
  $('hold-state').textContent = state.held ? (state.reason || 'Latched') : state.session ? 'Monitoring input' : 'Awaiting input';
  $('resume').hidden = !state.held;
  $('capture').disabled = !state.session || !Object.values(state.hands || {}).some(v => v?.pose);
  $('timer').textContent = clock(rec.elapsed_s);
  const returning = state.trial_return?.status === 'returning';
  $('record').textContent = rec.active ? 'Save & stay' : 'Start recording';
  $('record').disabled = returning;
  $('finish_return').hidden = !rec.active;
  $('mark').disabled = !rec.active;
  const trial = trialStatus();
  $('rec-state').textContent = rec.active ? 'Recording' : trial.title;
  $('trial-status').hidden = !trial.detail;
  $('trial-status').textContent = trial.detail;
  $('rec-state').classList.toggle('active',Boolean(rec.active));
  $('rec-error').hidden = !rec.error;
  $('rec-error').textContent = rec.error ? 'Recording error · ' + rec.error : '';
  const episode = rec.episode || rec.last_episode;
  $('episode-file').hidden = !episode;
  $('episode-file').textContent = episode ? String(episode) : '';
  $('timing').textContent = state.session ? number(state.rate_hz,1) + ' Hz · arrival age ' + number(1000*(state.age_s ?? NaN),0) + ' ms' : 'No samples';
  for (const [key] of fields) {
    const pose = h?.[key], cells = rowCells[key];
    for (let i=0;i<3;i++) cells[i].textContent = number(pose?.p?.[i]);
    cells[3].textContent = pose?.q?.map(v => number(v)).join('  ') || '—';
  }
  const layout = touchLayout(source,side);
  const labels = (h?.buttons || []).map((b,i) => (layout[i] || 'Button '+i) + ' ' + number(b.value,2) + (b.pressed ? ' ↓' : ''));
  $('inputs').textContent = (h?.pose ? 'Tracked' : 'Untracked') + (source ? ' · '+source : '') + ' · Jaw closure ' + number((h?.closure ?? 0)*100,0) + '%'
    + (labels.length ? '  |  '+labels.join('  ·  ') : '');
  if ($('raw').parentElement.open) $('raw').textContent = JSON.stringify({
    session:state.session, sequence:state.seq, frame:state.frame, head:state.head,
    hands:Object.fromEntries(sides.map(s => [s,state.hands?.[s] || null])),
    browser_sample_ms:state.time_ms, receiver_arrival_s:state.arrival_s, source_lag_s:state.source_lag_s,
    round_trip_ms:mode === 'observer' ? null : lastRoundTrip
  },null,2);
  if (mode !== 'quest') updateScene(h);
  updateCameraAge();
}
function trialStatus() {
  const rec = state.recording || {}, result = state.trial_return || {};
  const failure = {start_failed:'Trial did not start',save_failed:'Trial save failed',
                   incomplete:'Incomplete trial'}[result.status];
  if (failure) return {title:failure, detail:result.reason || rec.error || ''};
  if (rec.active) return {title:'REC ' + clock(rec.elapsed_s), detail:''};
  const title = {incomplete:'Incomplete trial', interrupted:'Interrupted trial', saved:'Trial saved'}[rec.last_episode_status]
      || (rec.episode && result.status !== 'idle' ? 'Trial saved' : 'Idle');
  const outcomes = {
    stayed:'Arms stayed at the ending pose.',
    returning:'Returning to the trial-start pose.',
    completed:'Returned to the trial-start pose.',
    skipped:'Return skipped.', cancelled:'Return cancelled.'
  };
  let detail = outcomes[result.status] || '';
  if (['skipped','cancelled'].includes(result.status) && result.reason) detail += ' ' + result.reason;
  return {title, detail};
}
function preview() {
  const rec = state.recording || {};
  const desired = rec.selected_camera;
  const matching = !desired || camera.id === desired;
  const age = matching && camera.index !== null ? camera.sourceAge + performance.now()-camera.arrival : Infinity;
  const src = rec.cameras?.find(c => c.id === desired);
  const stale = !Number.isFinite(age) || age > 1200 || rec.camera_stalled || Boolean(cameraProblem) ||
    Boolean(src?.status && src.status !== 'live');
  const liveStatus = ({unavailable:'Disconnected', stalled:'Stalled', waiting:'Waiting for frame'})[src?.status] || cameraProblem ||
      (!Number.isFinite(age) ? 'Waiting for frame' : stale ? 'Stalled' : 'Live');
  const trial = trialStatus();
  return {image:cameraCanvas, hands:rec.camera_mode === 'opencv' ? null : state.hands,
    info:{camera:matching && camera.label ? camera.label : rec.camera_label || 'Waiting for camera',
      index:matching ? camera.index : null, stale, liveStatus,
      ageBucket:Number.isFinite(age) ? Math.floor(age/100)*100 : null,
      status: (state.held ? 'HOLD · ' : '') + trial.title,
      detail:rec.error || trial.detail}};
}
function updateCameraAge() {
  const {info} = preview();
  $('camera-name').textContent = info.camera;
  $('camera-age').textContent = info.liveStatus + (info.ageBucket === null ? '' : ' · '+number(camera.sourceAge+performance.now()-camera.arrival,0)+' ms');
  cameraCanvas.style.visibility = info.stale ? 'hidden' : 'visible';
}
async function acknowledgeDisplay() {
  if (!sessionId || !camera.id || preview().info.stale) return;
  // A repeated repaint of the same frozen image must not renew its lifetime.
  freshDisplayAt = camera.arrival - camera.sourceAge;
  if (displayedId === camera.id || displayPending) return;
  displayPending = true;
  const {id,index} = camera, owner = sessionId, generation = displayGeneration;
  try {
    const resp = await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({action:'displayed',camera_id:id,index,session:owner}),signal:AbortSignal.timeout(1500)});
    if (resp.ok && owner === sessionId && generation === displayGeneration &&
        id === state.recording?.selected_camera) displayedId = id;
  } catch { /* Retry on the next displayed frame. */ }
  finally { displayPending = false; }
}
async function fetchFrame() {
  const requestedAt = performance.now();
  try {
    const r = await fetch('/camera',{cache:'no-store',signal:AbortSignal.timeout(1800)});
    if (!r.ok) throw new Error(String(r.status));
    cameraProblem = '';
    const id = r.headers.get('X-Camera-Id') || '', indexHeader = r.headers.get('X-Frame-Index');
    const index = indexHeader === null ? null : Number(indexHeader);
    if (id !== camera.id || index !== camera.index || camera.index === null) {
      const image = await createImageBitmap(await r.blob());
      if (cameraCanvas.width !== image.width || cameraCanvas.height !== image.height) {
        cameraCanvas.width=image.width; cameraCanvas.height=image.height;
      }
      frameContext.drawImage(image,0,0); image.close();
      camera = {id, label:r.headers.get('X-Camera-Label') || id || 'Camera', index,
        arrival:requestedAt,sourceAge:Number(r.headers.get('X-Frame-Age-Ms') || 0)};
      if (mode === 'mouse' && !document.hidden) requestAnimationFrame(acknowledgeDisplay);
    } else { await r.body?.cancel(); }
  } catch {
    cameraProblem = 'Feed unavailable';
  }
  updateCameraAge();
  setTimeout(fetchFrame,125);
}
async function poll() {
  if (mode === 'observer') {
    try { await request('/state'); }
    catch (e) { message('Receiver unavailable · '+e.message); }
  } else if (mode === 'mouse' && !document.hidden) {
    transmit(mouseInput());
  }
  setTimeout(poll, mode === 'observer' ? 160 : 32);
}
$('raw').parentElement.addEventListener('toggle',render);
window.addEventListener('resize',render);
setModeControls(); render(); poll(); fetchFrame();
