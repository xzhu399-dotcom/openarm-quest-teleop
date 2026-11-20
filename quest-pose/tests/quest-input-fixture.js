// Injected only by browser_smoke.py into a fresh desktop browser page.
// The normal Start Quest handler, renderer, network and receiver run unchanged.
(() => {
  const identity = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
  function eyeTransform(x) {
    const matrix = [...identity], inverse = [...identity];
    matrix[12] = x; matrix[13] = 1.5;
    inverse[12] = -x; inverse[13] = -1.5;
    return {position: {x, y: 1.5, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1},
      matrix, inverse: {matrix: inverse}};
  }
  const head = eyeTransform(0);
  const near = .05, far = 20, focal = 1 / Math.tan(40 * Math.PI/180);
  const projection = [focal/(128/96),0,0,0, 0,focal,0,0,
    0,0,-(far+near)/(far-near),-1, 0,0,-2*far*near/(far-near),0];
  const views = ['left','right'].map(eye => ({
    eye, projectionMatrix: projection, transform: eyeTransform(eye === 'left' ? -.032 : .032)
  }));
  const hand = {position: {x: .25, y: 1.13, z: -.4}, orientation: {x: 0, y: 0, z: 0, w: 1}};
  const gamepad = {mapping: 'xr-standard', axes: [0,0,0,0], buttons:
    Array.from({length: 7}, () => ({value: 0, pressed: false, touched: false}))};
  const controller = {profiles: ['oculus-touch-v3'], handedness: 'right', targetRayMode: 'tracked-pointer',
    gripSpace: {}, gamepad};
  const leftPad = {...gamepad, axes:[...gamepad.axes], buttons:gamepad.buttons.map(b => ({...b}))};
  const left = {...controller, handedness:'left', gripSpace:{}, gamepad:leftPad};
  const leftHand = {...hand, position:{...hand.position,x:-.25}};
  const space = new EventTarget();
  const session = Object.assign(new EventTarget(), {
    inputSources: [controller,left], visibilityState: 'visible',
    async requestReferenceSpace(name) {
      if (name !== 'local-floor') throw new Error('unexpected reference space ' + name);
      return space;
    },
    updateRenderState(state) { this.renderState = state; },
    requestAnimationFrame(callback) {
      this.next = setTimeout(() => callback(performance.now(), frame), 25);
      return this.next;
    },
    async end() {
      clearTimeout(this.next);
      this.dispatchEvent(new Event('end'));
    }
  });
  const frame = {
    session,
    getViewerPose(ref) {
      if (ref !== space) throw new Error('viewer pose in wrong frame');
      return {transform: head, views};
    },
    getPose(grip, ref) {
      if (![controller.gripSpace,left.gripSpace].includes(grip) || ref !== space)
        throw new Error('controller pose in wrong frame');
      return {transform: grip === left.gripSpace ? leftHand : hand};
    }
  };
  Object.defineProperty(navigator, 'xr', {value: {
    async isSessionSupported(mode) { return mode === 'immersive-vr'; },
    async requestSession() {
      const gl = document.getElementById('xr-canvas').getContext('webgl');
      if (!gl) throw new Error('browser needs WebGL');
      gl.makeXRCompatible = async () => {};
      const draw = gl.drawArrays.bind(gl);
      gl.drawArrays = (mode, first, count) => {
        draw(mode, first, count);
        if (mode === gl.TRIANGLE_STRIP && window.questFixture.samplePanel) {
          const [x] = gl.getParameter(gl.VIEWPORT);
          const pixel = new Uint8Array(4);
          const [px,py] = window.questFixture.panelPoint || [64,48];
          gl.readPixels(x+px,py,1,1,gl.RGBA,gl.UNSIGNED_BYTE,pixel);
          window.questFixture.panelPixels[x === 0 ? 'left' : 'right'] = Array.from(pixel);
          if (x !== 0) window.questFixture.samplePanel = false;
        }
        if (mode === gl.TRIANGLES && gl.getError() === gl.NO_ERROR) {
          const [x] = gl.getParameter(gl.VIEWPORT);
          const eye = x === 0 ? 'left' : 'right';
          window.questFixture.meshVertices[eye] = count;
          if (window.questFixture.robotPixels[eye] == null) {
            // The camera panel is above the horizon. Read the lower-center
            // pixels once, immediately after drawing the mesh for this eye.
            const pixels = new Uint8Array(96*36*4);
            gl.readPixels(x+16, 8, 96,36, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
            let lit = 0;
            for (let i=0; i<pixels.length; i+=4) {
              // Clear color is approximately [9,22,20].
              if (pixels[i] > 30 && pixels[i+1] > 32) lit++;
            }
            window.questFixture.robotPixels[eye] = lit;
          }
        }
      };
      return session;
    }
  }});
  Object.defineProperty(window, 'XRWebGLLayer', {value: class {
    constructor() { this.framebuffer = null; }
    getViewport(view) { return {x: view.eye === 'left' ? 0 : 128, y: 0, width: 128, height: 96}; }
  }});
  window.questFixture = {
    meshVertices: {left: 0, right: 0},
    samplePanel: false,
    panelPixels: {},
    robotPixels: {left: null, right: null},
    hudLines: [],
    button(index, value, side='right') {
      (side === 'left' ? leftPad : gamepad).buttons[index] = {value, pressed: value > .5, touched: value > 0};
    },
    axis(index, value, side='right') { (side === 'left' ? leftPad : gamepad).axes[index] = value; },
    hand, head
  };
  const paintText = CanvasRenderingContext2D.prototype.fillText;
  CanvasRenderingContext2D.prototype.fillText = function(text, ...args) {
    if (!this.canvas.isConnected) {
      const lines = window.questFixture.hudLines;
      if (lines.length >= 50) lines.shift();
      lines.push(text);
    }
    return paintText.call(this, text, ...args);
  };
})();
