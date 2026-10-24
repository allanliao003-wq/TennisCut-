'use strict';
/*
 * 擬真轉播畫面測試：小球（半徑約 2 px）、動態模糊、感測器雜訊、鏡頭輕微晃動、
 * 觀眾席閃動、球員身體與揮動的手臂。目標是球的軌跡要被追到，球員與觀眾不能被追到。
 */
const T = require('../lib/tracker.js');
let seed = 12345;
const rnd = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff; };

const W = 960, H = 540, FPS = 30, N = 100;

/* 球的路徑：發球 → 落地 → 回擊（方向反轉） */
function ballAt(t) {
  if (t < 1.4) {                                  // 由遠端往近端，中途落地
    const u = t / 1.4;
    return [700 - 380 * u, 180 + 300 * u + 90 * Math.sin(Math.PI * u * 2)];
  }
  if (t < 1.55) return null;                      // 擊球瞬間被球拍擋住（偵測會斷）
  const u = (t - 1.55) / 1.4;
  if (u > 1) return null;
  return [320 + 420 * u, 480 - 330 * u + 150 * u * u];
}

function render(t) {
  const px = new Uint8ClampedArray(W * H * 4);
  const shakeX = Math.round(Math.sin(t * 5.1) * 1.5), shakeY = Math.round(Math.cos(t * 4.3) * 1.2);
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const i = (y * W + x) * 4;
      let r, g, b;
      if (y < 170) {                              // 觀眾席：雜亂且會閃動
        const c = ((x * 7 + y * 13) % 64) + (rnd() < 0.08 ? 60 * rnd() : 0);
        r = 60 + c * 0.7; g = 55 + c * 0.6; b = 70 + c * 0.8;
      } else {                                     // 球場
        r = 42; g = 86; b = 170;
      }
      const n = (rnd() - 0.5) * 10;
      px[i] = r + n; px[i + 1] = g + n; px[i + 2] = b + n; px[i + 3] = 255;
    }
  }
  const rect = (x0, y0, x1, y1, c) => {
    for (let y = Math.max(0, (y0 + shakeY) | 0); y < Math.min(H, (y1 + shakeY) | 0); y++)
      for (let x = Math.max(0, (x0 + shakeX) | 0); x < Math.min(W, (x1 + shakeX) | 0); x++) {
        const i = (y * W + x) * 4; px[i] = c[0]; px[i + 1] = c[1]; px[i + 2] = c[2];
      }
  };
  rect(150, 300, 810, 304, [235, 238, 240]);      // 球場線（跟著鏡頭晃）
  rect(120, 480, 840, 485, [235, 238, 240]);
  rect(60, 60, 900, 100, [30, 60, 150]);           // 廣告看板
  rect(700, 66, 760, 94, [220, 225, 70]);          // 看板上的黃色字（靜止，不該被偵測）

  /* 球員：身體 + 快速揮動的手臂與球拍（周圍一整片都在動） */
  const pxp = 330 + 60 * Math.sin(t * 1.6), pyp = 430;
  rect(pxp - 16, pyp - 70, pxp + 16, pyp + 10, [40, 44, 52]);
  const ang = t * 9;
  for (let k = 0; k < 14; k++) {
    const ax = pxp + 14 + k * 2.4 * Math.cos(ang), ay = pyp - 56 + k * 2.4 * Math.sin(ang);
    rect(ax - 3, ay - 3, ax + 3, ay + 3, [205, 210, 200]);
  }
  const rx = pxp + 14 + 36 * Math.cos(ang), ry = pyp - 56 + 36 * Math.sin(ang);
  rect(rx - 9, ry - 9, rx + 9, ry + 9, [210, 215, 60]);   // 黃色拍框，會動、不孤立

  /* 球：沿著這一格的移動方向畫出動態模糊 */
  const p0 = ballAt(t - 1 / FPS), p1 = ballAt(t);
  if (p1) {
    const steps = 7;
    for (let s = 0; s < steps; s++) {
      const f = p0 ? s / (steps - 1) : 1;
      const bx = (p0 ? p0[0] + (p1[0] - p0[0]) * f : p1[0]) + shakeX;
      const by = (p0 ? p0[1] + (p1[1] - p0[1]) * f : p1[1]) + shakeY;
      for (let y = -3; y <= 3; y++) for (let x = -3; x <= 3; x++) {
        const d = Math.hypot(x, y); if (d > 2.6) continue;
        const xx = Math.round(bx + x), yy = Math.round(by + y);
        if (xx < 0 || yy < 0 || xx >= W || yy >= H) continue;
        const i = (yy * W + xx) * 4, a = (1 - d / 3) * (p0 ? 0.42 : 0.9);   // 模糊後變淡
        px[i] = px[i] * (1 - a) + 210 * a;
        px[i + 1] = px[i + 1] * (1 - a) + 232 * a;
        px[i + 2] = px[i + 2] * (1 - a) + 90 * a;
      }
    }
  }
  return px;
}

const bufs = [T.makeFrame(W, H), T.makeFrame(W, H), T.makeFrame(W, H)];
const buf = T.newBuf(W, H), ring = [], frames = [];
let slot = 0, totalCands = 0, t0 = Date.now();
for (let f = 0; f < N; f++) {
  const t = f / FPS;
  T.features(render(t), W, H, bufs[slot % 3]);
  ring.push({ t, f: bufs[slot % 3] }); slot++;
  if (ring.length === 3) {
    const c = T.detect(ring[0].f, ring[1].f, ring[2].f, W, H, T.DEFAULT_DETECT, null, buf);
    totalCands += c.length; frames.push({ t: ring[1].t, c }); ring.shift();
  }
}
console.log('擬真畫面：每格平均候選數', (totalCands / frames.length).toFixed(1),
  '| 偵測耗時(ms/格，含繪製)', ((Date.now() - t0) / N).toFixed(1));

/* 偵測階段：球的位置附近應該有候選點 */
let hit = 0, tot = 0;
frames.forEach(fr => {
  const p = ballAt(fr.t); if (!p) return;
  tot++;
  if (fr.c.some(c => Math.hypot(c.x - p[0], c.y - p[1]) < 14)) hit++;
});
if (process.env.DBG2) {
  frames.filter((_, i) => i % 15 === 0).forEach(fr => {
    const b = ballAt(fr.t), pxp = 330 + 60 * Math.sin(fr.t * 1.6), ang = fr.t * 9;
    const rx = pxp + 14 + 36 * Math.cos(ang), ry = 374 + 36 * Math.sin(ang);
    const near = (p, c) => p && Math.hypot(c.x - p[0], c.y - p[1]) < 16;
    const ball = fr.c.filter(c => near(b, c)), rack = fr.c.filter(c => near([rx, ry], c));
    const crowd = fr.c.filter(c => c.y < 175);
    const show = a => a.map(c => `s${c.s|0}/a${c.area}/iso${c.iso}`).join(' ') || '-';
    console.log('t', fr.t.toFixed(2), '| 球', show(ball), '| 拍', show(rack), '| 觀眾', crowd.length, show(crowd.slice(0,2)));
  });
}
console.log('偵測到球的格數比例', (hit / tot * 100).toFixed(0) + '%', `(${hit}/${tot})`);
if (hit / tot < 0.6) { console.error('偵測階段漏球太多'); process.exit(1); }

/* 追蹤階段 */
const r = T.pathsFrom(frames, {});
console.log('追蹤層級', r.levelName, '| 路徑數', r.paths.length,
  '| 各層嘗試', r.tried.map(x => `${x.name}:原始${x.raw}→路徑${x.paths}`).join(' '));
if (!r.paths.length) { console.error('沒有追到任何路徑'); process.exit(1); }

let onBall = 0, offBall = 0, covered = 0;
r.paths.forEach(p => p.pts.forEach(q => {
  const b = ballAt(q.t);
  if (b && Math.hypot(b[0] - q.x, b[1] - q.y) < 20) { onBall++; covered++; } else offBall++;
}));
console.log('路徑點落在球上的比例', (onBall / (onBall + offBall) * 100).toFixed(0) + '%',
  '| 覆蓋球出現的格數', (covered / tot * 100).toFixed(0) + '%');
if (process.env.DBG) r.paths.forEach(p => { console.log('--- path', p.id); p.pts.filter((_,i)=>i%8===0).forEach(q => { const b=ballAt(q.t); const pxp=330+60*Math.sin(q.t*1.6), ang=q.t*9; const rx=pxp+14+36*Math.cos(ang), ry=430-56+36*Math.sin(ang); console.log('  t',q.t.toFixed(2),'pt',q.x.toFixed(0),q.y.toFixed(0),'ball',b?b.map(v=>v.toFixed(0)).join(','):'-','racket',rx.toFixed(0),ry.toFixed(0)); }); });
r.paths.forEach(p => console.log(' 路徑', p.id, '片段數', p.parts, '點數', p.pts.length,
  '時間', p.pts[0].t.toFixed(2), '→', p.pts[p.pts.length - 1].t.toFixed(2)));
if (onBall / (onBall + offBall) < 0.8) { console.error('追到太多不是球的東西'); process.exit(1); }
if (covered / tot < 0.5) { console.error('球的軌跡覆蓋率太低'); process.exit(1); }

/* 串接測試：擊球反向 + 漏偵測的三段片段應接成一條路徑 */
(function () {
  const mk = (id, t0, n, x0, y0, vx, vy) => ({ id, pts: Array.from({ length: n }, (_, i) => ({ t: t0 + i / 30, x: x0 + vx * i / 30, y: y0 + vy * i / 30, a: 8 })) });
  const A = mk(1, 0, 20, 100, 300, 600, -200);
  const B = mk(2, 0.8, 20, 540, 207, 500, 300);
  const C = mk(3, 1.6, 20, 880, 400, -700, 100);
  const paths = T.stitch([A, B, C]);
  console.log('串接路徑數', paths.length, '最長路徑片段數', Math.max(...paths.map(p => p.parts)));
  if (!paths.some(p => p.parts >= 3)) { console.error('串接失敗'); process.exit(1); }
})();

/* 抖動軌跡（像球員肢體）應被擋掉，平滑的球軌跡應保留 */
(function () {
  const tr = new T.Tracker();
  for (let f = 0; f < 60; f++) {
    const t = f / 30;
    tr.update(t, [
      { x: 200 + 500 * t, y: 600 - 400 * t * (1 - t) * 4, area: 8, s: 120, iso: 2 },
      { x: 900 + 40 * (rnd() - 0.5) * 2, y: 300 + 40 * (rnd() - 0.5) * 2, area: 10, s: 110, iso: 3 }
    ]);
  }
  tr.flush();
  const res = tr.results();
  console.log('抖動測試：保留軌跡數', res.length, '（預期 1，位於球的軌道上）');
  if (res.length !== 1 || res[0].pts[0].x > 400) { console.error('抖動軌跡未被擋掉'); process.exit(1); }
})();

console.log('tracker 測試全部通過');
