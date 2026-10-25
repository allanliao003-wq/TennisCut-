'use strict';
const G = require('../lib/geometry.js');
let seed = 12345; const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
const gauss = () => { let u = 0, v = 0; while (!u) u = rnd(); v = rnd(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const nrm = a => { const n = Math.hypot(...a); return a.map(x => x / n); };

function makeCam(C, target, f, W, H) {
  const fw = nrm(target.map((x, i) => x - C[i])), right = nrm(cross(fw, [0, 0, 1])), down = cross(fw, right);
  const R = [right, down, fw];
  const t = R.map(r => -(r[0]*C[0] + r[1]*C[1] + r[2]*C[2]));
  return { R, t, C, f, cx: W/2, cy: H/2, imgW: W, imgH: H };
}
function proj(cam, P) {
  const c = cam.R.map((r, i) => r[0]*P[0] + r[1]*P[1] + r[2]*P[2] + cam.t[i]);
  return [cam.cx + cam.f * c[0] / c[2], cam.cy + cam.f * c[1] / c[2]];
}
const W = 1920, H = 1080, Wc = 10.97, Lc = 23.77;
const cams = {
  'behind-baseline 2.2m': makeCam([5.485, -6, 2.2], [5.485, 12, 0.5], 1500, W, H),
  'fence-high 3.5m': makeCam([5.485, -8, 3.5], [5.485, 12, 0.5], 1300, W, H),
  'side-offset 1.6m': makeCam([-5, -4, 1.6], [5.485, 12, 0.5], 1500, W, H)
};

function flight(p0, landing, vh, fps, bounce) {
  const dx = landing[0] - p0[0], dy = landing[1] - p0[1], dist = Math.hypot(dx, dy), T = dist / vh;
  const vz = (-p0[2] + 0.5 * 9.81 * T * T) / T, v = [dx / T, dy / T, vz];
  const pts = []; for (let t = 0; t <= T + 1e-9; t += 1 / fps) pts.push({ t, P: [p0[0]+v[0]*t, p0[1]+v[1]*t, p0[2]+v[2]*t-0.5*9.81*t*t] });
  return { v, speed: Math.hypot(...v), T, pts };
}

const shots = [
  { name: '發球 162km/h（沿光軸）', p0: [7, -0.5, 2.8], land: [4, 18.5, 0], vh: 45 },
  { name: '底線抽球 100km/h（朝相機）', p0: [3, 22.5, 1.0], land: [6, 2, 0], vh: 27.5 },
  { name: '斜線抽球 90km/h', p0: [1, 1, 1.0], land: [9, 20, 0], vh: 24.5 },
  { name: '小球 60km/h', p0: [5, 14, 1.0], land: [5.5, 20, 0], vh: 16.5 }
];

function run(camName, sh, fps, noise, corners_noise, useAnchor) {
  const cam = cams[camName];
  const dst = [[0,0],[Wc,0],[Wc,Lc],[0,Lc]].map(p => { const q = proj(cam, [p[0], p[1], 0]); return [q[0] + gauss()*corners_noise, q[1] + gauss()*corners_noise]; });
  const cal = G.calibrate(dst, 'doubles', W, H);
  const fl = flight(sh.p0, sh.land, sh.vh, fps);
  const obs = fl.pts.map(o => { const q = proj(cam, o.P); return { t: o.t, u: q[0] + gauss()*noise, v: q[1] + gauss()*noise }; });
  const anchors = [];
  if (useAnchor) { const L = obs[obs.length-1], g = G.pixelToGround(cal, L.u, L.v); anchors.push({ t: L.t, X: g[0], Y: g[1], Z: 0, wt: 2 }); }
  const fit = G.fitFlight(cal, obs, anchors);
  return { cal, fit, truth: fl.speed, n: obs.length, cam };
}

// 1) 標定：焦距與相機位置
for (const name of Object.keys(cams)) {
  const cam = cams[name];
  const r = run(name, shots[0], 30, 0, 0, false);
  console.log(name, '| f 真值', cam.f, '估計', r.cal.f.toFixed(0), '| 相機位置誤差(m)', Math.hypot(r.cal.C[0]-cam.C[0], r.cal.C[1]-cam.C[1], r.cal.C[2]-cam.C[2]).toFixed(3), '| 無雜訊球速誤差', ((r.fit.speed0 - r.truth)*3.6).toFixed(2), 'km/h');
}
// 2) 蒙地卡羅：角點誤差 1.5px、球偵測誤差 0.8px、30fps
console.log('\n=== 角點誤差 1.5px，球偵測誤差 0.8px，30fps，200 次 ===');
for (const name of Object.keys(cams)) {
  for (const sh of shots) {
    for (const anchor of [false, true]) {
      const errs = [], sigs = [];
      for (let k = 0; k < 200; k++) {
        const r = run(name, sh, 30, 0.8, 1.5, anchor);
        if (!r.fit) continue;
        errs.push((r.fit.speed0 - r.truth) / r.truth * 100); sigs.push(r.fit.sigmaSpeed / r.truth * 100);
      }
      errs.sort((a, b) => a - b);
      const med = errs[errs.length >> 1], p90 = Math.abs(errs[Math.floor(errs.length * 0.9)]) , p10 = Math.abs(errs[Math.floor(errs.length * 0.1)]);
      const medAbs = errs.map(Math.abs).sort((a, b) => a - b)[errs.length >> 1];
      console.log(name.padEnd(22), sh.name.padEnd(26), anchor ? '含彈跳' : '無錨點', '| 中位誤差%', med.toFixed(1).padStart(6), '| 中位絕對%', medAbs.toFixed(1).padStart(5), '| p10/p90 %', (errs[Math.floor(errs.length*0.1)]).toFixed(0), '/', (errs[Math.floor(errs.length*0.9)]).toFixed(0), '| 回報σ%', (sigs.sort((a,b)=>a-b)[sigs.length>>1]).toFixed(1), '| 球數', 'n=' + run(name, sh, 30, 0, 0, false).n);
    }
  }
}

/* 3) 球場格線疊圖：標定後畫出來的線，是不是真的落在球場線上？
      這是使用者在預覽畫面上看到的東西，誤差以像素計。 */
console.log('\n=== 球場格線疊圖誤差（像素）===');
for (const name of Object.keys(cams)) {
  const cam = cams[name];
  for (const cn of [0, 1.5, 4]) {
    const dst = [[0,0],[Wc,0],[Wc,Lc],[0,Lc]].map(p => { const q = proj(cam, [p[0], p[1], 0]); return [q[0] + gauss()*cn, q[1] + gauss()*cn]; });
    const cal = G.calibrate(dst, 'doubles', W, H);
    const L = G.courtLines('doubles');
    let maxCorner = 0, maxLine = 0, maxNet = 0;
    [[0,0,0],[Wc,0,0],[Wc,Lc,0],[0,Lc,0]].forEach((p, i) => {
      const q = G.project(cal, p); maxCorner = Math.max(maxCorner, Math.hypot(q[0]-dst[i][0], q[1]-dst[i][1]));
    });
    L.lines.forEach(seg => seg.forEach(p => {
      const a = G.project(cal, p), b = proj(cam, p);
      maxLine = Math.max(maxLine, Math.hypot(a[0]-b[0], a[1]-b[1]));
    }));
    L.net.forEach(p => { const a = G.project(cal, p), b = proj(cam, p); maxNet = Math.max(maxNet, Math.hypot(a[0]-b[0], a[1]-b[1])); });
    console.log(name.padEnd(22), '角點雜訊', String(cn).padStart(3), 'px | 四角誤差', maxCorner.toFixed(2),
      '| 所有球場線最大誤差', maxLine.toFixed(2), '| 網子頂端(離地1.07m)', maxNet.toFixed(2), '| 焦距來源', cal.fFrom);
    if (cn === 0 && maxLine > 0.5) { console.error('無雜訊時球場線就對不上'); process.exit(1); }
  }
}

/* 4) 物理過濾：真實飛行應保留，球員腳步／身體晃動應剔除 */
console.log('\n=== 物理過濾（拋物線檢查）===');
(function () {
  const cam = cams['behind-baseline 2.2m'];
  const dst = [[0,0],[Wc,0],[Wc,Lc],[0,Lc]].map(p => proj(cam, [p[0], p[1], 0]));
  const cal = G.calibrate(dst, 'doubles', W, H);
  const mk = (id, pts) => ({ id, pts });
  const tracks = [];
  // 真球：底線抽球
  const shot = flight([2.5, 1.0, 0.9], [8.0, 20.0, 0], 100 / 3.6, 30, false);
  tracks.push(mk(1, shot.pts.map(p => { const q = proj(cam, p.P); return { t: p.t, u: q[0] + gauss()*0.8, v: q[1] + gauss()*0.8 }; })));
  // 腳步：貼地、來回、慢
  tracks.push(mk(2, Array.from({length: 30}, (_, i) => {
    const t = i / 30, P = [5.4 + 0.9 * Math.sin(t * 7), -0.4 + 0.25 * Math.sin(t * 11), 0.03 + 0.04 * Math.abs(Math.sin(t * 13))];
    const q = proj(cam, P); return { t, u: q[0] + gauss()*1.2, v: q[1] + gauss()*1.2 };
  })));
  // 身體／手臂：約在 1.2 公尺高抖動
  tracks.push(mk(3, Array.from({length: 30}, (_, i) => {
    const t = i / 30, P = [4.0 + 0.5 * Math.sin(t * 9), 1.2 + 0.4 * Math.cos(t * 8), 1.2 + 0.3 * Math.sin(t * 15)];
    const q = proj(cam, P); return { t, u: q[0] + gauss()*1.5, v: q[1] + gauss()*1.5 };
  })));
  const fl = G.physicalFlights(cal, tracks, { imgW: W, imgH: H });
  const srcs = [...new Set(fl.map(f => f.src))];
  console.log('保留的飛行段來自軌跡：', srcs.join(',') || '(無)', '（預期只有 1）', '| 段數', fl.length,
    '| 速度', fl.map(f => (f.fit.speed0*3.6).toFixed(0) + 'km/h').join(','));
  if (!srcs.includes(1)) { console.error('真球被誤刪'); process.exit(1); }
  if (srcs.includes(2) || srcs.includes(3)) { console.error('球員軌跡沒被剔除'); process.exit(1); }
})();
