export const identity = [0, 0, 0, 1];

export function multiply(a,b) {
  const [x,y,z,w] = a, [i,j,k,r] = b;
  const q = [w*i+x*r+y*k-z*j, w*j-x*k+y*r+z*i, w*k+x*j-y*i+z*r, w*r-x*i-y*j-z*k];
  const n = Math.hypot(...q);
  return q.map(v => v/n);
}

export function axisTurn(axis,radians) {
  const s = Math.sin(radians/2);
  return [...axis.map(v => v*s),Math.cos(radians/2)];
}

// Yaw about tracking Y, pitch about X and roll about local Z (Y-X-Z Euler display).
// The simulator and mouse continue to use quaternions through the pitch singularity.
export function wristAngles([x,y,z,w]) {
  const sinPitch = Math.max(-1,Math.min(1,2*(w*x-y*z)));
  const singular = Math.abs(sinPitch) > 0.999999;
  const yaw = singular ? Math.atan2(2*(w*y-x*z),1-2*(y*y+z*z))
                       : Math.atan2(2*(x*z+w*y),1-2*(x*x+y*y));
  const roll = singular ? 0 : Math.atan2(2*(x*y+w*z),1-2*(x*x+z*z));
  return [Math.asin(sinPitch),yaw,roll].map(v => v*180/Math.PI);
}

const add = (a,b) => a.map((v,i) => v+b[i]);
const sub = (a,b) => a.map((v,i) => v-b[i]);
const scale = (v,k) => v.map(n => n*k);
const dot = (a,b) => a.reduce((n,v,i) => n+v*b[i],0);
const cross = ([x,y,z],[a,b,c]) => [y*c-z*b,z*a-x*c,x*b-y*a];
const unit = v => scale(v,1/Math.hypot(...v));
const lerp = (a,b,t) => add(a,scale(sub(b,a),t));

function rotate(q,v) {
  const xyz = q.slice(0,3), t = scale(cross(xyz,v),2);
  return add(v,add(scale(t,q[3]),cross(xyz,t)));
}

const right = [.8,0,-.6];
const down = [.6*.42,-Math.sqrt(1-.42*.42),.8*.42];
const near = cross(down,right);
const flat = p => [dot(p,right),dot(p,down),dot(p,near)];
const steel = [178,192,186], graphite = [73,89,88], rubber = [48,60,59];
const mint = [146,214,188], lime = [213,232,135];

function color(rgb,light=1) {
  return 'rgb('+rgb.map(v => Math.round(Math.min(255,v*light))).join(',')+')';
}

// Orthographic camera. All three views use the same right/up/forward convention.
class View {
  constructor(ctx,center,origin,zoom) {
    this.ctx = ctx; this.center = center; this.origin = flat(origin); this.zoom = zoom;
    this.drawables = [];
  }
  project(p) {
    const v = flat(p);
    return [this.center[0]+(v[0]-this.origin[0])*this.zoom,
            this.center[1]+(v[1]-this.origin[1])*this.zoom, v[2]];
  }
  line(a,b,ink,width=1) {
    const c=this.ctx, p=this.project(a), q=this.project(b);
    c.beginPath(); c.moveTo(p[0],p[1]); c.lineTo(q[0],q[1]);
    c.strokeStyle=ink; c.lineWidth=width; c.stroke();
  }
  rod(a,b,r1,r2,rgb) {
    const s=this.project(a), t=this.project(b), dx=t[0]-s[0], dy=t[1]-s[1];
    const length=Math.hypot(dx,dy);
    if(length<.1 && Math.abs(s[2]-t[2])<.0002) return;
    const normalX=length ? -dy/length : 1, normalY=length ? dx/length : 0;
    const rs=r1*this.zoom, rt=r2*this.zoom;
    const facing=Math.abs(dot(unit(sub(b,a)),near));
    const z=(s[2]+t[2])*.5;
    this.drawables.push({z,draw:() => {
      const c=this.ctx;
      // Shade a projected cylinder silhouette directly. Rendering one shape per link
      // keeps telemetry responsive on low-power software-backed canvas devices.
      const grad=c.createLinearGradient(s[0]-normalX*rs,s[1]-normalY*rs,
                                         s[0]+normalX*rs,s[1]+normalY*rs);
      const lit=normalX*.4+normalY*.6 > 0;
      grad.addColorStop(0,color(rgb,lit ? .59 : .81));
      grad.addColorStop(lit ? .67 : .33,color(rgb,1.04));
      grad.addColorStop(1,color(rgb,lit ? .81 : .59));
      const angle=Math.atan2(normalY,normalX);
      c.beginPath();
      c.ellipse(s[0],s[1],rs,Math.max(.05,rs*facing),angle,0,Math.PI);
      c.ellipse(t[0],t[1],rt,Math.max(.05,rt*facing),angle,Math.PI,2*Math.PI);
      c.closePath(); c.fillStyle=grad; c.fill();
      if(facing>.12) {
        const front=s[2]>t[2] ? s:t, radius=s[2]>t[2]?rs:rt;
        c.fillStyle=color(rgb,.86);
        c.beginPath(); c.ellipse(front[0],front[1],radius,radius*facing,angle,0,2*Math.PI); c.fill();
      }
    }});
  }
  ball(center,r,rgb) {
    const [x,y,z]=this.project(center), radius=Math.max(1,r*this.zoom);
    this.drawables.push({z,draw:() => {
      const c=this.ctx, grad=c.createRadialGradient(x-radius*.38,y-radius*.48,radius*.07,x,y,radius);
      grad.addColorStop(0,color(rgb,1.15)); grad.addColorStop(.57,color(rgb,.86)); grad.addColorStop(1,color(rgb,.43));
      c.fillStyle=grad; c.beginPath(); c.arc(x,y,radius,0,2*Math.PI); c.fill();
    }});
  }
  flush() {
    this.drawables.sort((a,b)=>a.z-b.z);
    for(const shape of this.drawables) shape.draw();
    this.drawables.length=0;
  }
}

function axes(view,p,q,size) {
  for (const [vector,ink] of [[[1,0,0],'#ef9b87'],[[0,1,0],'#c2e496'],[[0,0,1],'#85b8e3']]) {
    view.line(p,add(p,rotate(q,scale(vector,size))),ink,1.7);
  }
}

function gripper(view,robot,active) {
  const {tcp,palm_m:palm,finger_roots_m:roots,fingers_m:tips} = robot;
  const local = v => add(tcp.p,rotate(tcp.q,v));
  const tint=active ? lime : steel;
  view.rod(local([0,0,.118]),palm,.023,.026,graphite);
  view.rod(local([-.055,0,.088]),local([.055,0,.088]),.015,.015,tint);
  for (let i=0;i<2;i++) {
    const sign=i===0 ? -1:1;
    // Fingertips mark the inner jaw surfaces, so keep a visible pad even at contact.
    const outside=rotate(tcp.q,[sign*.004,0,0]);
    const end=add(tips[i],outside), root=roots[i], knuckle=lerp(root,end,.70);
    view.rod(root,knuckle,.009,.007,tint);
    view.rod(knuckle,end,.007,.006,rubber);
    view.ball(root,.010,graphite);
  }
}

function arm(view,robot,active) {
  const points=[robot.base_m,...robot.joint_positions_m,robot.palm_m];
  const shell=active ? steel : [111,137,134], trim=active ? mint : [92,127,121];
  for (let i=0;i<points.length-1;i++) {
    const a=points[i], b=points[i+1], length=Math.hypot(...sub(b,a));
    if(length<.006) continue;
    const radius=i<=2 ? .025 : .020;
    view.rod(a,b,radius,radius*.84,graphite);
    if(length>.065) {
      view.rod(lerp(a,b,.13),lerp(a,b,.70),radius*1.19,radius*1.02,shell);
      view.rod(lerp(a,b,.70),lerp(a,b,.78),radius*1.02,radius*.97,trim);
    }
  }
  for(let i=0;i<robot.joint_positions_m.length;i++) {
    const p=robot.joint_positions_m[i];
    if(i>0 && Math.hypot(...sub(p,robot.joint_positions_m[i-1]))<.003) continue;
    view.ball(p,i<4 ? .032 : .024,i%2 ? graphite : trim);
  }
  gripper(view,robot,active);
}

function robotStage(ctx,hands,side,w,h) {
  const robots = Object.values(hands || {}).map(hand=>hand?.robot).filter(Boolean);
  if (!robots.length) return;
  // Include the fixed work volume when framing so small wrist motions keep a stable view.
  const bounds=[[-.45,.39,.07],[.45,.39,.07],[-.43,1.25,-.06],[.43,1.25,-.06],
    [-.42,.96,-.59],[.42,.96,-.59]];
  for (const r of robots) bounds.push(...r.joint_positions_m,...r.fingers_m);
  const projected=bounds.map(flat);
  const xs=projected.map(p=>p[0]), ys=projected.map(p=>p[1]);
  const x0=Math.min(...xs), x1=Math.max(...xs), y0=Math.min(...ys), y1=Math.max(...ys);
  const z=Math.min((w-28)/(x1-x0),(h-60)/(y1-y0));
  const view=new View(ctx,[w/2-z*(x0+x1)/2,(h+12)/2-z*(y0+y1)/2],[0,0,0],z);
  const mastTop=robots.reduce((p,r)=>add(p,scale(r.base_m,1/robots.length)),[0,0,0]);
  const floor=mastTop[1]-.70, foot=[mastTop[0],floor,mastTop[2]];
  for (let i=-12;i<=12;i++) {
    const d=i*.1, ink=i%5 ? '#2c4644' : '#3c5550';
    view.line([d,floor,-1.2],[d,floor,1.2],ink);
    view.line([-1.2,floor,d],[1.2,floor,d],ink);
  }
  view.rod(foot,add(foot,[0,.025,0]),.155,.143,graphite);
  view.rod(add(foot,[0,.025,0]),add(foot,[0,.062,0]),.105,.076,steel);
  view.rod(add(foot,[0,.062,0]),mastTop,.049,.042,graphite);
  view.rod(add(foot,[0,.12,0]),lerp(foot,mastTop,.74),.053,.052,steel);
  for(const s of ['left','right']) {
    const robot=hands?.[s]?.robot;
    if(!robot) continue;
    view.rod(mastTop,robot.base_m,.041,.040,graphite);
    arm(view,robot,s===side);
  }
  view.flush();
}

function handleDetail(ctx,hand,side,x,w,h) {
  const q=hand?.pose?.q || identity;
  const p=hand?.delta?.p || [0,0,0];
  const zoom=Math.min(w*.68/.16,(h-42)/.24);
  // Scale translation separately from the close-up to keep the entire grip visible.
  const view=new View(ctx,[x+w*.50,h*.59],[0,.012,0],zoom), offset=scale(p,.24);
  const point=v=>add(offset,rotate(q,v));
  const rod=(a,b,r1,r2,rgb)=>view.rod(point(a),point(b),r1,r2,rgb);
  const handed=side==='left' ? -1 : 1;
  const shell=hand?.pose ? [211,223,213] : [153,172,162];
  rod([0,-.081,.027],[0,-.051,.021],.012,.020,shell);
  rod([0,-.051,.021],[0,-.005,0],.020,.024,shell);
  rod([0,-.005,0],[0,.011,-.009],.023,.029,shell);
  rod([0,.011,-.009],[0,.017,-.01],.027,.026,graphite);
  // The circular guard is attached to the body and rotates with the physical grip.
  let last=null;
  for(let i=0;i<=24;i++) {
    const angle=2*Math.PI*i/24;
    const v=[.052*Math.cos(angle),.035+.049*Math.sin(angle),-.020+.028*Math.sin(angle)];
    if(last) rod(last,v,.0058,.0058,shell);
    last=v;
  }
  const trigger=hand?.buttons?.[0]?.value || 0;
  rod([0,-.004,-.030],[0,-.024,-.035+.012*trigger],.013,.008,trigger>.05 ? lime : graphite);
  rod([handed*.018,-.045,.019],[handed*.022,-.021,.006],.007,.008,hand?.clutched ? lime : mint);
  rod([-handed*.010,.017,-.017],[-handed*.010,.024,-.017],.005,.008,rubber);
  for(const [id,v] of [[4,[handed*.011,.021,-.002]],[5,[handed*.015,.021,-.016]]]) {
    view.ball(point(v),.0048,hand?.buttons?.[id]?.pressed ? lime : steel);
  }
  view.flush();
  if(hand?.pose) axes(view,point([0,-.074,.025]),q,.030);
}

export function drawScene(canvas,hands,side) {
  const ctx=canvas.getContext('2d');
  const w=canvas.clientWidth, h=canvas.clientHeight;
  if(!w || !h) return;
  const dpr=Math.min(window.devicePixelRatio || 1,2);
  if(canvas.width!==Math.round(w*dpr) || canvas.height!==Math.round(h*dpr)) {
    canvas.width=Math.round(w*dpr); canvas.height=Math.round(h*dpr);
  }
  ctx.setTransform(dpr,0,0,dpr,0,0); ctx.lineJoin='round'; ctx.lineCap='round';
  const split=Math.round(w*.68), column=w-split, top=Math.round(h*.58);
  const glow=ctx.createRadialGradient(split*.42,h*.28,0,split*.48,h*.5,split*.8);
  glow.addColorStop(0,'#253f3b'); glow.addColorStop(1,'#112524');
  ctx.fillStyle=glow; ctx.fillRect(0,0,w,h);
  ctx.save(); ctx.beginPath(); ctx.rect(0,0,split,h); ctx.clip();
  robotStage(ctx,hands,side,split,h);
  ctx.restore();
  ctx.fillStyle='#1b3431'; ctx.fillRect(split,0,column,top);
  ctx.fillStyle='#182e2c'; ctx.fillRect(split,top,column,h-top);
  ctx.fillStyle='#405b52'; ctx.fillRect(split,0,1,h); ctx.fillRect(split,top,column,1);
  const hand=hands?.[side];
  ctx.save(); ctx.beginPath(); ctx.rect(split,0,column,top); ctx.clip();
  handleDetail(ctx,hand,side,split,column,top); ctx.restore();
  const robot=hand?.robot;
  if(robot) {
    ctx.save(); ctx.beginPath(); ctx.rect(split,top,column,h-top); ctx.clip();
    const center=add(robot.tcp.p,rotate(robot.tcp.q,[0,0,.052]));
    const zoom=Math.min((column-22)/.20,(h-top-40)/.16);
    const detail=new View(ctx,[split+column*.50,top+(h-top)*.53],center,Math.max(1,zoom));
    gripper(detail,robot,true); detail.flush();
    axes(detail,robot.tcp.p,robot.tcp.q,.027);
    ctx.restore();
  }
}
