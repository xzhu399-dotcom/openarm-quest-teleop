import {robotVertices} from './xr_robot.js';

const touchProfiles = new Set(['oculus-touch-v3', 'oculus-touch-v2', 'oculus-touch']);
const sourceIds = new WeakMap();
let nextSourceId = 1;

// The Oculus Touch registry assigns the same component indices on both sides.
// See immersive-web/webxr-input-profiles, registry/profiles/oculus/oculus-touch-v3.json.
export function touchLayout(profile, side) {
  if (profile !== 'mouse' && !touchProfiles.has(profile)) return [];
  return ['Trigger', 'Grip', '', 'Stick', ...(side === 'right' ? ['A', 'B'] : ['X', 'Y']), 'Thumbrest', 'Menu'];
}

function trackedPose(pose) {
  if (!pose || pose.emulatedPosition) return null;
  const {position: p, orientation: q} = pose.transform;
  return {p: [p.x, p.y, p.z], q: [q.x, q.y, q.z, q.w]};
}

function readHands(frame, space) {
  const hands = {left: null, right: null};
  for (const source of frame.session.inputSources) {
    const side = source.handedness;
    if (!['left', 'right'].includes(side) || !source.gamepad || !source.gripSpace) continue;
    const {gamepad} = source;
    if (!sourceIds.has(source)) sourceIds.set(source, 'xr-' + nextSourceId++);
    const profiles = Array.from(source.profiles);
    const known = gamepad.mapping === 'xr-standard' && profiles.find(p => touchProfiles.has(p));
    hands[side] = {
      pose: trackedPose(frame.getPose(source.gripSpace, space)),
      source: sourceIds.get(source),
      profile: known || (gamepad.mapping !== 'xr-standard' ? 'unmapped' : profiles[0]) || 'unknown',
      profiles, handedness: side, target_ray_mode: source.targetRayMode, mapping: gamepad.mapping,
      buttons: gamepad.buttons.map(b => ({pressed: b.pressed, value: b.value, touched: b.touched})),
      axes: Array.from(gamepad.axes)
    };
  }
  return hands;
}

function createPanel(gl) {
  const program = gl.createProgram();
  const shaders = [
    [gl.VERTEX_SHADER, `
      attribute vec2 corner;
      attribute vec3 position, color;
      varying vec2 uv;
      varying vec3 tint;
      uniform mat4 projection, view, origin;
      uniform vec3 robotOffset;
      uniform float aspect, panelScale;
      uniform vec2 panelCenter;
      uniform bool solid;
      void main() {
        uv = (corner + 1.0) * .5;
        tint = color;
        vec2 panel = panelCenter + corner * vec2(panelScale, panelScale / aspect);
        vec4 world = solid ? vec4(position + robotOffset, 1.) : origin * vec4(panel, -1.45, 1.);
        gl_Position = projection * view * world;
      }`],
    [gl.FRAGMENT_SHADER, `
      precision mediump float;
      uniform sampler2D picture;
      uniform bool solid;
      varying vec2 uv;
      varying vec3 tint;
      void main() { gl_FragColor = solid ? vec4(tint, 1.) : texture2D(picture, uv); }`]
  ];
  for (const [type, source] of shaders) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(shader));
    gl.attachShader(program, shader);
    gl.deleteShader(shader);
  }
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));
  const buffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1,-1, 1,-1, -1,1, 1,1]), gl.STATIC_DRAW);
  const tex = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, tex);
  for (const name of [gl.TEXTURE_MIN_FILTER, gl.TEXTURE_MAG_FILTER]) {
    gl.texParameteri(gl.TEXTURE_2D, name, gl.LINEAR);
  }
  for (const name of [gl.TEXTURE_WRAP_S, gl.TEXTURE_WRAP_T]) {
    gl.texParameteri(gl.TEXTURE_2D, name, gl.CLAMP_TO_EDGE);
  }
  const corner = gl.getAttribLocation(program, 'corner');
  const position = gl.getAttribLocation(program, 'position'), color = gl.getAttribLocation(program, 'color');
  const mesh = gl.createBuffer();
  const uniforms = Object.fromEntries(['projection', 'view', 'origin', 'aspect', 'picture',
    'robotOffset','solid','panelScale','panelCenter'].map(
    key => [key, gl.getUniformLocation(program, key)]
  ));
  const screen = document.createElement('canvas');
  const ctx = screen.getContext('2d');
  let lastImageKey = '', origin = null, lastHands = null, vertexCount = 0;

  return {
    reset() { origin = null; },
    dispose() { gl.deleteTexture(tex); gl.deleteBuffer(buffer); gl.deleteBuffer(mesh); gl.deleteProgram(program); },
    draw(layer, viewer, image, info, hands) {
      gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
      gl.clearColor(.035, .085, .078, 1);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      if (!viewer) return false;
      if (!origin) origin = new Float32Array(viewer.transform.matrix);
      if (lastHands !== hands) {
        const vertices = robotVertices(hands);
        gl.bindBuffer(gl.ARRAY_BUFFER, mesh);
        gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.DYNAMIC_DRAW);
        vertexCount = vertices.length / 6;
        lastHands = hands;
      }
      gl.useProgram(program);
      gl.activeTexture(gl.TEXTURE0);
      gl.bindTexture(gl.TEXTURE_2D, tex);
      const imageKey = JSON.stringify([info.camera, info.index, info.status, info.detail,
                                       info.liveStatus, info.stale, info.ageBucket]);
      if (imageKey !== lastImageKey) {
        screen.width = Math.min(1280,Math.max(480, image.width));
        const pictureHeight = Math.round(screen.width * image.height / image.width);
        const scale = screen.width / 550, line = Math.round(27*scale);
        const font = Math.round(19*scale)+'px sans-serif';
        ctx.font = font;
        const rows = [info.camera+' · '+(info.liveStatus || (info.stale ? 'Stalled' : 'Live'))
                      +(info.ageBucket == null ? '' : ' · '+info.ageBucket+' ms'), info.status];
        let message = '';
        for (const word of (info.detail || '').split(/\s+/)) {
          const extended = message ? message+' '+word : word;
          if (message && ctx.measureText(extended).width > screen.width-22*scale) {
            rows.push(message); message = word;
          } else message = extended;
        }
        if (message) rows.push(message);
        screen.height = pictureHeight + line*(rows.length+.30);
        if (info.stale) {
          ctx.fillStyle = '#17201e';
          ctx.fillRect(0,0,screen.width,pictureHeight);
          ctx.fillStyle = '#fff'; ctx.font = font;
          ctx.textAlign = 'center';
          ctx.fillText(info.liveStatus || 'Waiting for camera',screen.width/2,pictureHeight/2);
          ctx.textAlign = 'left';
        } else ctx.drawImage(image, 0, 0, screen.width, pictureHeight);
        ctx.fillStyle = info.stale ? '#773b2f' : '#123b32';
        ctx.fillRect(0, pictureHeight, screen.width, screen.height-pictureHeight);
        ctx.fillStyle = '#fff'; ctx.font = font;
        rows.forEach((row,i) => ctx.fillText(row,10*scale,pictureHeight + line*(i+.88),screen.width-20*scale));
        gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, true);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, screen);
        lastImageKey = imageKey;
      }
      gl.uniform1i(uniforms.picture, 0);
      gl.uniform1f(uniforms.aspect, screen.width / screen.height);
      gl.uniformMatrix4fv(uniforms.origin, false, origin);
      // Keep the rig axes in the shared stationary frame. Place the robot in front
      // of the operator once, then let them look around it or lean in.
      gl.uniform3f(uniforms.robotOffset, origin[12] - origin[8]*1.1,
                   origin[13] - 1.31, origin[14] - origin[10]*1.1 + .22);
      gl.uniform1f(uniforms.panelScale, vertexCount ? .36 : .64);
      gl.uniform2f(uniforms.panelCenter, vertexCount ? .43 : 0, vertexCount ? .29 : 0);
      gl.enable(gl.DEPTH_TEST);
      for (const eye of viewer.views) {
        const vp = layer.getViewport(eye);
        gl.viewport(vp.x, vp.y, vp.width, vp.height);
        gl.uniformMatrix4fv(uniforms.view, false, eye.transform.inverse.matrix);
        gl.uniformMatrix4fv(uniforms.projection, false, eye.projectionMatrix);
        gl.disableVertexAttribArray(position);
        gl.disableVertexAttribArray(color);
        gl.uniform1i(uniforms.solid, 0);
        gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
        gl.enableVertexAttribArray(corner);
        gl.vertexAttribPointer(corner, 2, gl.FLOAT, false, 0, 0);
        gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
        if (vertexCount) {
          gl.disableVertexAttribArray(corner);
          gl.bindBuffer(gl.ARRAY_BUFFER, mesh);
          gl.enableVertexAttribArray(position);
          gl.enableVertexAttribArray(color);
          gl.vertexAttribPointer(position, 3, gl.FLOAT, false, 24, 0);
          gl.vertexAttribPointer(color, 3, gl.FLOAT, false, 24, 12);
          gl.uniform1i(uniforms.solid, 1);
          gl.drawArrays(gl.TRIANGLES, 0, vertexCount);
        }
      }
      return viewer.views.length > 0;
    }
  };
}

export async function startQuest(mode, {onStart, onSample, onReset, onInactive, onEnd, onDisplay, preview}) {
  // requestSession must be the first asynchronous operation in the click handler.
  const session = await navigator.xr.requestSession(mode, {optionalFeatures: ['local-floor']});
  let panel, started = false;
  try {
    const gl = document.getElementById('xr-canvas').getContext('webgl', {xrCompatible: true});
    if (!gl) throw new Error('WebGL is unavailable');
    await gl.makeXRCompatible();
    const layer = new XRWebGLLayer(session, gl);
    session.updateRenderState({baseLayer: layer});
    let frameName = 'local-floor', space;
    try { space = await session.requestReferenceSpace(frameName); }
    catch { frameName = 'local'; space = await session.requestReferenceSpace(frameName); }
    panel = createPanel(gl);
    space.addEventListener('reset', () => { panel.reset(); onReset(); });
    session.addEventListener('visibilitychange', () => {
      if (session.visibilityState !== 'visible') onInactive();
    });
    session.addEventListener('end', () => { panel.dispose(); onEnd(); }, {once: true});
    onStart(frameName);
    started = true;
    function frameLoop(time, frame) {
      session.requestAnimationFrame(frameLoop);
      if (session.visibilityState !== 'visible') return;
      const viewer = frame.getViewerPose(space);
      const hands = readHands(frame, space);
      onSample({time_ms: time, head: trackedPose(viewer), hands, frame: frameName, visible: true});
      const {image, info, hands:simulatedHands} = preview();
      if (panel.draw(layer, viewer, image, info, simulatedHands)) onDisplay();
    }
    session.requestAnimationFrame(frameLoop);
    return session;
  } catch (error) {
    await session.end().catch(() => {});
    if (!started && panel) panel.dispose();
    throw error;
  }
}
