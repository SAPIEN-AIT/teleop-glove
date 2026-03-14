// ── State ────────────────────────────────────────────────────────
// angles[0..3] = index, middle, ring, pinky → [MCP, PIP, DIP] in degrees
// angles[4]    = thumb                      → [CMC, MCP] in degrees
let angles = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0]];
let rotX = -15, rotY = -20, zoom = 1;
let pollTimer = null;
let dragging = false, lastMX = 0, lastMY = 0;

const canvas = document.getElementById('hand');
const ctx    = canvas.getContext('2d');
const CW = canvas.width, CH = canvas.height;

// ── Math ─────────────────────────────────────────────────────────
const R = d => d * Math.PI / 180;

function rotateX(p, a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [p[0], p[1]*c - p[2]*s, p[1]*s + p[2]*c];
}
function rotateY(p, a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [p[0]*c + p[2]*s, p[1], -p[0]*s + p[2]*c];
}
function project(p) {
  const fov = 600 * zoom;
  const z   = p[2] + 400;
  return [CW/2 + p[0]*fov/z, CH/2 + p[1]*fov/z, p[2]];
}
function transform(p) {
  return project(rotateY(rotateX(p, R(rotX)), R(rotY)));
}

// ── Geometry ─────────────────────────────────────────────────────
const PALM_PTS  = [[-70,60,0],[70,60,0],[75,-40,8],[-75,-40,8]];
const KNUCKLE   = [[-52,-38,8],[-18,-44,9],[18,-42,8],[52,-36,6]];
const SEG       = [[56,38,28],[60,42,30],[54,38,27],[44,30,22]];
const THUMB_BASE = [-74, 20, 2];
const THUMB_SEG  = [46, 36];

// ── Draw helpers ─────────────────────────────────────────────────
function drawBone(p1, p2, width, color, jointColor) {
  const a = transform(p1), b = transform(p2);
  ctx.beginPath();
  ctx.moveTo(a[0], a[1]);
  ctx.lineTo(b[0], b[1]);
  ctx.strokeStyle = color;
  ctx.lineWidth   = width;
  ctx.lineCap     = 'round';
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(a[0], a[1], width * 0.55, 0, Math.PI*2);
  ctx.fillStyle = jointColor;
  ctx.fill();
}

function drawFinger(base, segs, joints, boneCol, jointCol, tipCol) {
  let p = [...base], ax = -Math.PI/2;
  const pts = [p];
  for (let i = 0; i < segs.length; i++) {
    ax += R(joints[i] || 0);
    p = [p[0], p[1] + Math.cos(ax)*segs[i], p[2] - Math.sin(ax)*segs[i]];
    pts.push([...p]);
  }
  for (let i = 0; i < pts.length-1; i++) {
    drawBone(pts[i], pts[i+1], Math.max(8 - i*1.5, 4), boneCol, jointCol);
  }
  const tp = transform(pts[pts.length-1]);
  ctx.beginPath(); ctx.arc(tp[0], tp[1], 5, 0, Math.PI*2);
  ctx.fillStyle = tipCol; ctx.fill();
}

function drawThumb(base, segs, joints, col, jointCol, tipCol) {
  const cmc = R(joints[0]||0), mcp = R(joints[1]||0);
  const p1 = [
    base[0] - Math.cos(cmc)*segs[0],
    base[1] + Math.sin(cmc)*segs[0]*0.6,
    base[2] - Math.sin(cmc)*segs[0]*0.4,
  ];
  const p2 = [
    p1[0] - Math.sin(mcp)*segs[1]*0.5,
    p1[1] + Math.cos(mcp)*segs[1]*0.3,
    p1[2] - Math.cos(mcp)*segs[1],
  ];
  drawBone(base, p1, 9, col, jointCol);
  drawBone(p1,   p2, 6.5, col, jointCol);
  const tp = transform(p2);
  ctx.beginPath(); ctx.arc(tp[0], tp[1], 5, 0, Math.PI*2);
  ctx.fillStyle = tipCol; ctx.fill();
}

// ── Main draw ────────────────────────────────────────────────────
function drawHand() {
  ctx.clearRect(0, 0, CW, CH);

  // palm
  const pts = PALM_PTS.map(transform);
  ctx.beginPath();
  ctx.moveTo(pts[0][0], pts[0][1]);
  pts.forEach(p => ctx.lineTo(p[0], p[1]));
  ctx.closePath();
  ctx.fillStyle   = '#ddd8cc';
  ctx.strokeStyle = '#c8c2b4';
  ctx.lineWidth   = 1;
  ctx.fill(); ctx.stroke();

  // thumb
  drawThumb(THUMB_BASE, THUMB_SEG, angles[4], '#7c3aed', '#e2e8f0', '#534AB7');

  // four fingers
  for (let f = 0; f < 4; f++) {
    drawFinger(KNUCKLE[f], SEG[f], angles[f], '#378ADD', '#e2e8f0', '#185FA5');
  }
}

// ── API polling ──────────────────────────────────────────────────
/*
  Expected JSON:
  {
    "fingers": [[mcp,pip,dip], [mcp,pip,dip], [mcp,pip,dip], [mcp,pip,dip]],
    "thumb":   [cmc, mcp]
  }
*/
function applyData(data) {
  const f = data.fingers || data.angles;
  if (Array.isArray(f)) {
    for (let i = 0; i < Math.min(f.length, 4); i++)
      for (let j = 0; j < Math.min(f[i].length, 3); j++)
        angles[i][j] = Math.max(0, Math.min(90, +f[i][j]));
  }
  const t = data.thumb;
  if (Array.isArray(t)) {
    angles[4][0] = Math.max(0, Math.min(70, +t[0]));
    angles[4][1] = Math.max(0, Math.min(70, +t[1]));
  }
  drawHand();
}

async function fetchAngles() {
  try {
    const r = await fetch('/api/v1/finger/all', { signal: AbortSignal.timeout(2000) });
    applyData(await r.json());
  } catch(e) { console.warn('fetch failed', e); }
}

export function startPolling() {
  stopPolling();
  fetchAngles();
  pollTimer = setInterval(fetchAngles, 50);  // 20 Hz
}
export function stopPolling() {
  if (pollTimer) { clearInterval(pollTimer); pollTimer = null; }
}

// ── Mouse / touch drag to rotate ─────────────────────────────────
canvas.addEventListener('mousedown', e => { dragging=true; lastMX=e.clientX; lastMY=e.clientY; });
window.addEventListener('mouseup',   ()  => dragging=false);
window.addEventListener('mousemove', e  => {
  if (!dragging) return;
  rotY += (e.clientX - lastMX) * 0.5;
  rotX  = Math.max(-90, Math.min(90, rotX + (e.clientY - lastMY) * 0.5));
  lastMX = e.clientX; lastMY = e.clientY;
  drawHand();
});
canvas.addEventListener('wheel', e => {
  e.preventDefault();
  zoom = Math.max(0.4, Math.min(2.5, zoom - e.deltaY * 0.001));
  drawHand();
}, { passive: false });

drawHand();