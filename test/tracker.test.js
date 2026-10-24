'use strict';
const T = require('../lib/tracker.js');
let seed = 7; const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
const w = 640, h = 360, fps = 30, nFrames = 75;

function render(t, balls, shirt) {
  const px = new Uint8ClampedArray(w * h * 4);
  for (let i = 0; i < w * h; i++) {
    const n = (rnd() - 0.5) * 8;
    px[i*4] = 50 + n; px[i*4+1] = 90 + n; px[i*4+2] = 170 + n; px[i*4+3] = 255;
  }
  const rect = (x0, y0, x1, y1, c) => { for (let y = Math.max(0, y0|0); y < Math.min(h, y1|0); y++) for (let x = Math.max(0, x0|0); x < Math.min(w, x1|0); x++) { const i = (y*w+x)*4; px[i]=c[0]; px[i+1]=c[1]; px[i+2]=c[2]; } };
  rect(0, 200, w, 203, [235, 235, 235]);            // 靜止白線
  rect(520, 20, 560, 40, [220, 220, 60]);          // 靜止黃色招牌（不該被偵測）
  rect(shirt.x(t), 150, shirt.x(t) + 34, 215, [215, 215, 50]); // 移動中的黃色衣服（太大）
  rect(300 + 40 * Math.sin(t * 2), 120, 316 + 40 * Math.sin(t * 2), 160, [20, 20, 30]); // 深色球員
  balls.forEach(b => {
    const p = b.pos(t); if (!p) return;
    const r = b.r;
    for (let y = -r - 1; y <= r + 1; y++) for (let x = -r - 1; x <= r + 1; x++) if (x*x + y*y <= r*r + 1) {
      const xx = Math.round(p[0] + x), yy = Math.round(p[1] + y); if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
      const i = (yy*w+xx)*4; px[i] = 205; px[i+1] = 228; px[i+2] = 55;
    }
  });
  return px;
}
const A = { r: 2.5, pos: t => (t >= 0 && t < 2.2) ? [50 + 380 * t, 250 - 260 * t + 0.5 * 560 * t * t] : null };
const B = { r: 2, pos: t => (t >= 0.5 && t < 2.4) ? [600 - 300 * (t - 0.5), 100 + 90 * (t - 0.5) + 0.5 * 300 * (t - 0.5) ** 2] : null };
const shirt = { x: t => 40 + 150 * t };

const bufs = [T.makeFrame(w, h), T.makeFrame(w, h), T.makeFrame(w, h)];
const buf = T.newBuf(w, h), tr = new T.Tracker();
const ring = []; let slot = 0, totalCands = 0, t0 = Date.now();
for (let f = 0; f < nFrames; f++) {
  const t = f / fps;
  T.features(render(t, [A, B], shirt), w, h, bufs[slot % 3]);
  ring.push({ t, f: bufs[slot % 3] }); slot++;
  if (ring.length === 3) {
    const c = T.detect(ring[0].f, ring[1].f, ring[2].f, w, h, T.DEFAULT_DETECT, null, buf);
    totalCands += c.length; tr.update(ring[1].t, c); ring.shift();
  }
}
tr.flush();
const res = tr.results();
console.log('每格平均候選數', (totalCands / (nFrames - 2)).toFixed(1), '| 處理時間(ms/格，含合成畫面)', ((Date.now() - t0) / nFrames).toFixed(1));
console.log('軌跡數', res.length);
res.forEach(r => {
  let maxErr = 0; const truth = [A, B];
  r.pts.forEach(p => { let best = 1e9; truth.forEach(b => { const q = b.pos(p.t); if (q) best = Math.min(best, Math.hypot(q[0] - p.x, q[1] - p.y)); }); maxErr = Math.max(maxErr, best); });
  console.log(' 軌跡', r.id, '點數', r.pts.length, '時間', r.pts[0].t.toFixed(2), '→', r.pts[r.pts.length-1].t.toFixed(2), '最大位置誤差(px)', maxErr.toFixed(2));
});
