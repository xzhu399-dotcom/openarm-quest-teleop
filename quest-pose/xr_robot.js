/** Low-poly render geometry in the server's stationary simulation frame, in metres. */
const minus = (a, b) => a.map((v, i) => v - b[i]);
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const unit = v => { const length = Math.hypot(...v); return v.map(x => x / length); };
const body = [.37,.47,.50], steel = [.69,.77,.74], dark = [.20,.25,.29];

function cylinder(out, start, end, radius, color) {
  const v = minus(end, start);
  if (Math.hypot(...v) < .0001) return;
  const axis = unit(v);
  const u = unit(cross(axis, Math.abs(axis[1]) > .85 ? [1,0,0] : [0,1,0]));
  const w = cross(axis,u);
  let last;
  for (let i=0; i<=10; i++) {
    const angle = 2*Math.PI*i/10;
    const normal = u.map((n,k) => Math.cos(angle)*n + Math.sin(angle)*w[k]);
    const ring = [start,end].map(p => p.map((n,k) => n + radius*normal[k]));
    if (last) {
      const light = .72 + .25*(.3*normal[0] + .85*normal[1] + .42*normal[2]);
      const rgb = color.map(c => c*light);
      for (const p of [last[0], last[1], ring[1], last[0], ring[1], ring[0],
                       start, last[0], ring[0], end, ring[1], last[1]]) out.push(...p,...rgb);
    }
    last = ring;
  }
}

function hub(out, center, size) {
  const p = (x,y,z) => [center[0]+x*size, center[1]+y*size, center[2]+z*size];
  const rim = [p(1,0,0),p(0,0,1),p(-1,0,0),p(0,0,-1)];
  for (let i=0; i<4; i++) {
    const j = (i+1)%4;
    for (const [tip,rgb] of [[p(0,1,0),steel],[p(0,-1,0),body]]) {
      for (const point of [tip,rim[i],rim[j]]) out.push(...point,...rgb);
    }
  }
}

export function robotVertices(hands) {
  const out = [];
  const arms = Object.entries(hands || {}).filter(([,h]) => h?.robot);
  if (!arms.length) return new Float32Array();
  const center = arms.map(([,h])=>h.robot.base_m).reduce((a,b)=>a.map((x,i)=>x+b[i]),[0,0,0])
                     .map(v=>v/arms.length);
  const foot = [center[0],center[1]-.61,center[2]];
  cylinder(out,foot,[foot[0],foot[1]+.045,foot[2]],.14,body);
  cylinder(out,foot,center,.065,dark);
  for (const [side,h] of arms) {
    const r = h.robot, links = [r.base_m,...r.joint_positions_m,r.palm_m];
    const accent = side === 'left' ? [.35,.63,.74] : [.91,.68,.33];
    cylinder(out,center,r.base_m,.032,body);
    for (let i=0; i<links.length-1; i++) {
      const radius = i<4 ? .035 : .021;
      cylinder(out,links[i],links[i+1],radius, i%2 ? body : accent);
    }
    for (let i=0; i<r.joint_positions_m.length; i++)
      hub(out,r.joint_positions_m[i],i<4 ? .047 : .031);
    cylinder(out,r.finger_roots_m[0],r.finger_roots_m[1],.013,accent);
    for (let i=0; i<2; i++) {
      cylinder(out,r.palm_m,r.finger_roots_m[i],.016,dark);
      cylinder(out,r.finger_roots_m[i],r.fingers_m[i],.006,steel);
    }
    // A side-specific mark makes wrist roll visible even with the jaws together.
    const q = r.tcp.q, [x,y,z,w] = q;
    const up = [2*(x*y-z*w), 1-2*(x*x+z*z), 2*(y*z+x*w)];
    cylinder(out,r.palm_m,r.palm_m.map((v,i)=>v+.032*up[i]),.008,accent);
  }
  return new Float32Array(out);
}
