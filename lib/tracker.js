/*
 * 網球偵測與多球追蹤（固定機位或緩慢移動的轉播鏡頭）
 *
 * 偵測：用前後三格的最小差分找出「只有這一格有」的移動像素（亮度為主、黃綠色為輔），
 *       連成小區塊後，用大小、形狀與「周圍有沒有其他移動像素」篩出網球候選點。
 *       球員的手、球拍、衣服雖然也在動，但周圍一整片都在動，會被孤立性檢查擋掉。
 * 追蹤：alpha-beta 濾波＋距離門檻，可同時追多顆球；擊球、彈跳或漏偵測造成的斷點
 *       再用 stitch() 依位置與時間接回同一條路徑。
 *
 * 設計重點：偵測時用較低的門檻把候選點全部留下來，靈敏度在「離線」階段才套用，
 *          所以使用者調整靈敏度不需要重新解碼影片。
 */
(function (root) {
  'use strict';

  var DEFAULT_DETECT = {
    thr: 26,        // 抽取候選點的低門檻（離線階段才用 scoreMin 篩）
    minArea: 4, maxArea: 170, maxDim: 60, maxElong: 12,
    isoR: 14,       // 孤立性檢查半徑
    maxCand: 24, maxPix: 40000
  };
  var DEFAULT_TRACK = {
    scoreMin: 42, isoMax: 60,
    initGate: 90, gateBase: 24, gateK: 1.6,
    maxMissConf: 8, maxGapT: 0.4, maxMissTent: 2, confirmHits: 3,
    minPts: 4, minDisp: 24, minDur: 0.08,
    smoothTol: 3.2, smoothFrac: 0.55, vAvgMin: 140,
    vMin: 90, vMax: 5000, maxTent: 80, alpha: 0.6
  };
  var DEFAULT_STITCH = { maxGap: 0.5, base: 45, k: 1.2, minPts: 10, minDisp: 110, overlap: 0.35 };

  function makeFrame(w, h) { return { luma: new Uint8Array(w * h), yel: new Uint8Array(w * h) }; }

  /* RGBA → 亮度與黃綠色程度。黃綠色只當加權，不當硬門檻：
     轉播畫面裡的球常常因為過曝或動態模糊而接近白色。 */
  function features(rgba, w, h, out) {
    var n = w * h, luma = out.luma, yel = out.yel, i, p;
    for (i = 0, p = 0; i < n; i++, p += 4) {
      var r = rgba[p], g = rgba[p + 1], b = rgba[p + 2];
      luma[i] = (r * 77 + g * 150 + b * 29) >> 8;
      var mn = r < g ? r : g, y = mn - b;
      yel[i] = y < 0 ? 0 : (y > 255 ? 255 : y);
    }
    return out;
  }

  function newBuf(w, h, cap) {
    return { mask: new Uint8Array(w * h), sc: new Uint8Array(w * h), lab: new Int32Array(w * h), list: new Int32Array(cap || DEFAULT_DETECT.maxPix) };
  }

  /* 偵測 cur 這一格的網球候選點。回傳 {x,y,area,s,iso,yel}，s 是這個區塊的最強移動強度。 */
  function detect(prev, cur, next, w, h, opt, roi, buf) {
    var o = {}, kk;
    for (kk in DEFAULT_DETECT) o[kk] = (opt && opt[kk] !== undefined) ? opt[kk] : DEFAULT_DETECT[kk];
    var thr = o.thr, mask = buf.mask, sc = buf.sc, lab = buf.lab, list = buf.list, cap = list.length;
    var n = w * h, cnt = 0, overflow = false, i;
    var cl = cur.luma, pl = prev.luma, nl = next.luma, cy = cur.yel, py = prev.yel, ny = next.yel;
    for (i = 0; i < n; i++) {
      var c = cl[i] - pl[i]; if (c < 0) c = -c;
      var d = cl[i] - nl[i]; if (d < 0) d = -d;
      var ml = c < d ? c : d;
      var s = ml;
      if (s + 64 >= thr) {                       // 黃綠色只在亮度差分已有一定強度時加分
        var a = cy[i] - py[i]; if (a < 0) a = -a;
        var b = cy[i] - ny[i]; if (b < 0) b = -b;
        s = ml + 0.6 * (a < b ? a : b);
      }
      if (s >= thr) {
        if (cnt >= cap) { overflow = true; break; }
        mask[i] = 1; sc[i] = s > 255 ? 255 : s; list[cnt++] = i;
      }
    }
    var cands = [], k;
    if (!overflow) {
      var comps = [], lid = 0;
      for (k = 0; k < cnt; k++) {
        var s0 = list[k];
        if (mask[s0] !== 1) continue;
        lid++;
        var stack = [s0], area = 0, sumS = 0, sx = 0, sy = 0, sumYel = 0, peak = 0, x0 = 1e9, x1 = -1, y0 = 1e9, y1 = -1;
        mask[s0] = 2;
        while (stack.length) {
          var q = stack.pop(), qx = q % w, qy = (q / w) | 0, w8 = sc[q];
          lab[q] = lid; area++; sumS += w8; sx += qx * w8; sy += qy * w8; sumYel += cy[q];
          if (w8 > peak) peak = w8;
          if (qx < x0) x0 = qx; if (qx > x1) x1 = qx; if (qy < y0) y0 = qy; if (qy > y1) y1 = qy;
          for (var dy = -1; dy <= 1; dy++) {
            var ny2 = qy + dy; if (ny2 < 0 || ny2 >= h) continue;
            for (var dx = -1; dx <= 1; dx++) {
              if (!dx && !dy) continue;
              var nx2 = qx + dx; if (nx2 < 0 || nx2 >= w) continue;
              var ni = ny2 * w + nx2;
              if (mask[ni] === 1) { mask[ni] = 2; stack.push(ni); }
            }
          }
        }
        var bw = x1 - x0 + 1, bh = y1 - y0 + 1, mx = bw > bh ? bw : bh, mnn = bw < bh ? bw : bh;
        if (area < o.minArea || area > o.maxArea || mx > o.maxDim || mx > mnn * o.maxElong) continue;
        var cx = sx / sumS, cyy = sy / sumS;
        if (roi && !roi(cx, cyy)) continue;
        comps.push({ x: cx, y: cyy, area: area, s: peak, yel: sumYel / area, lid: lid, bw: bw, bh: bh });
      }
      /* 孤立性：球周圍是空的，球員的手或拍子周圍還有一大片在動 */
      var R = o.isoR;
      comps.forEach(function (cp) {
        var ax = Math.round(cp.x), ay = Math.round(cp.y), yq, xq, other = 0;
        for (yq = Math.max(0, ay - R); yq <= Math.min(h - 1, ay + R); yq++) {
          var base = yq * w;
          for (xq = Math.max(0, ax - R); xq <= Math.min(w - 1, ax + R); xq++) {
            var ii = base + xq;
            if (mask[ii] && lab[ii] !== cp.lid) other++;      // 只算「不屬於自己」的移動像素
          }
        }
        cp.iso = other;
        /* 像球的程度：小、圓、孤立、偏黃綠。用來決定保留哪些候選點， */
        /* 不能用移動強度排序，否則球員和球拍會把球擠掉。 */
        var mx = cp.bw > cp.bh ? cp.bw : cp.bh, mn2 = cp.bw < cp.bh ? cp.bw : cp.bh;
        var fill = cp.area / Math.max(1, cp.bw * cp.bh);
        var sizePen = cp.area < 6 ? (6 - cp.area) * 12 : (cp.area > 120 ? (cp.area - 120) * 0.5 : 0);
        cp.ball = 100 - sizePen - Math.min(45, other * 0.6) + Math.min(25, cp.yel * 0.18)
          + 15 * fill - Math.min(20, (mx / Math.max(1, mn2)) * 2) + Math.min(15, cp.s * 0.06);
        cands.push(cp);
      });
    }
    for (k = 0; k < cnt; k++) { mask[list[k]] = 0; lab[list[k]] = 0; }
    cands.sort(function (a, b) { return b.ball - a.ball; });
    if (cands.length > o.maxCand) cands.length = o.maxCand;
    return cands;
  }

  /* 球的飛行很平滑：任一小段（6 點）在影像上近似二次曲線。
     球員肢體、衣服皺褶、雜訊則會抖動，殘差明顯較大。 */
  function fitRms(pts, i0, n, deg) {
    var t0 = pts[i0].t, m = deg + 1, r, c, k;
    function fit(key) {
      var ata = [], atb = [];
      for (r = 0; r < m; r++) { ata.push(new Array(m).fill(0)); atb.push(0); }
      for (k = 0; k < n; k++) {
        var tt = pts[i0 + k].t - t0, row = [], y = pts[i0 + k][key];
        for (r = 0; r < m; r++) row.push(Math.pow(tt, r));
        for (r = 0; r < m; r++) { atb[r] += row[r] * y; for (c = 0; c < m; c++) ata[r][c] += row[r] * row[c]; }
      }
      for (r = 0; r < m; r++) {
        var piv = r; for (c = r + 1; c < m; c++) if (Math.abs(ata[c][r]) > Math.abs(ata[piv][r])) piv = c;
        var tmp = ata[r]; ata[r] = ata[piv]; ata[piv] = tmp; var tb = atb[r]; atb[r] = atb[piv]; atb[piv] = tb;
        if (Math.abs(ata[r][r]) < 1e-12) return null;
        for (c = r + 1; c < m; c++) { var f = ata[c][r] / ata[r][r]; for (var q = r; q < m; q++) ata[c][q] -= f * ata[r][q]; atb[c] -= f * atb[r]; }
      }
      var x = new Array(m).fill(0);
      for (r = m - 1; r >= 0; r--) { var sum = atb[r]; for (c = r + 1; c < m; c++) sum -= ata[r][c] * x[c]; x[r] = sum / ata[r][r]; }
      var ss = 0;
      for (k = 0; k < n; k++) { var tt2 = pts[i0 + k].t - t0, pred = 0; for (r = 0; r < m; r++) pred += x[r] * Math.pow(tt2, r); var e = pts[i0 + k][key] - pred; ss += e * e; }
      return Math.sqrt(ss / n);
    }
    var ru = fit('x'), rv = fit('y');
    return ru === null || rv === null ? 1e9 : Math.max(ru, rv);
  }
  function smoothFrac(pts, tol) {
    var n = pts.length, W = 6;
    if (n < W) return fitRms(pts, 0, n, n >= 4 ? 2 : 1) <= tol ? 1 : 0;
    var good = 0, tot = 0, i;
    for (i = 0; i + W <= n; i++) { tot++; if (fitRms(pts, i, W, 2) <= tol) good++; }
    return good / tot;
  }

  function Tracker(opt) {
    this.o = {};
    var k, d = DEFAULT_TRACK;
    for (k in d) this.o[k] = (opt && opt[k] !== undefined) ? opt[k] : d[k];
    this.tracks = []; this.done = []; this.nextId = 1; this.seen = 0; this.used = 0;
  }
  Tracker.prototype.reset = function () { this.tracks = []; };

  Tracker.prototype._accept = function (tr) {
    var o = this.o, p = tr.pts, n = p.length;
    if (tr.state !== 'conf' || n < o.minPts) return false;
    var disp = Math.hypot(p[n - 1].x - p[0].x, p[n - 1].y - p[0].y), dur = p[n - 1].t - p[0].t, path = 0, i;
    if (!(disp >= o.minDisp && dur >= o.minDur)) return false;
    for (i = 1; i < n; i++) path += Math.hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
    if (path / dur < o.vAvgMin) return false;
    tr.smooth = smoothFrac(p, o.smoothTol);
    return tr.smooth >= o.smoothFrac;
  };

  Tracker.prototype._retire = function (tr) { if (this._accept(tr)) this.done.push(tr); };

  Tracker.prototype.update = function (t, cands0) {
    var o = this.o, j, i, self = this;
    var cands = [];
    for (j = 0; j < cands0.length; j++) {
      self.seen++;
      var cc = cands0[j];
      if (cc.s !== undefined && cc.s < o.scoreMin) continue;
      if (cc.iso !== undefined && cc.iso > o.isoMax) continue;
      cands.push(cc); self.used++;
    }
    var used = [];
    for (j = 0; j < cands.length; j++) used.push(false);
    var order = this.tracks.slice().sort(function (a, b) { return (b.state === 'conf') - (a.state === 'conf'); });
    var keep = [];
    for (i = 0; i < order.length; i++) {
      var tr = order[i], dt = t - tr.lastT;
      if (dt <= 0) { keep.push(tr); continue; }
      var px, py, gate;
      if (tr.hits >= 2) {
        px = tr.x + tr.vx * dt; py = tr.y + tr.vy * dt;
        gate = o.gateBase + o.gateK * Math.hypot(tr.vx, tr.vy) * dt;
      } else { px = tr.x; py = tr.y; gate = o.initGate; }
      var best = -1, bd = gate;
      for (j = 0; j < cands.length; j++) {
        if (used[j]) continue;
        var dd = Math.hypot(cands[j].x - px, cands[j].y - py);
        if (dd < bd) { bd = dd; best = j; }
      }
      if (best >= 0) {
        var c = cands[best], mvx = (c.x - tr.x) / dt, mvy = (c.y - tr.y) / dt;
        if (tr.hits === 1) {
          var sp = Math.hypot(mvx, mvy);
          if (sp > o.vMax) { tr.miss++; if (tr.miss > o.maxMissTent) continue; keep.push(tr); continue; }
          tr.vx = mvx; tr.vy = mvy;
        } else {
          tr.vx = tr.vx * (1 - o.alpha) + mvx * o.alpha; tr.vy = tr.vy * (1 - o.alpha) + mvy * o.alpha;
        }
        used[best] = true;
        tr.x = c.x; tr.y = c.y; tr.lastT = t; tr.hits++; tr.miss = 0;
        tr.pts.push({ t: t, x: c.x, y: c.y, a: c.area });
        if (tr.state === 'tent' && tr.hits >= o.confirmHits) {
          var p = tr.pts, nn = p.length, path = 0, k2;
          for (k2 = 1; k2 < nn; k2++) path += Math.hypot(p[k2].x - p[k2 - 1].x, p[k2].y - p[k2 - 1].y);
          var net = Math.hypot(p[nn - 1].x - p[0].x, p[nn - 1].y - p[0].y), spd = path / Math.max(1e-6, p[nn - 1].t - p[0].t);
          if (path > 0 && net / path > 0.55 && spd >= o.vMin && spd <= o.vMax) tr.state = 'conf';
          else { continue; }
        }
        keep.push(tr);
      } else {
        tr.miss++;
        var limit = tr.state === 'conf' ? o.maxMissConf : o.maxMissTent;
        if (tr.miss > limit && (tr.state !== 'conf' || t - tr.pts[tr.pts.length - 1].t > o.maxGapT)) { this._retire(tr); continue; }
        if (tr.hits >= 2) { tr.x = px; tr.y = py; }
        tr.lastT = t;
        keep.push(tr);
      }
    }
    var tent = 0;
    keep.forEach(function (x) { if (x.state === 'tent') tent++; });
    for (j = 0; j < cands.length; j++) {
      if (used[j] || tent >= o.maxTent) continue;
      keep.push({ id: this.nextId++, state: 'tent', hits: 1, miss: 0, x: cands[j].x, y: cands[j].y, vx: 0, vy: 0, lastT: t, pts: [{ t: t, x: cands[j].x, y: cands[j].y, a: cands[j].area }] });
      tent++;
    }
    this.tracks = keep;
  };

  Tracker.prototype.flush = function () {
    var self = this;
    this.tracks.forEach(function (tr) { self._retire(tr); });
    this.tracks = [];
  };

  Tracker.prototype.results = function () {
    var arr = this.done.slice().sort(function (a, b) { return b.pts.length - a.pts.length; }), out = [];
    arr.forEach(function (tr) {
      var dup = false;
      for (var i = 0; i < out.length && !dup; i++) {
        var o = out[i], same = 0, tot = 0;
        tr.pts.forEach(function (p) {
          tot++;
          for (var j = 0; j < o.pts.length; j++) {
            if (Math.abs(o.pts[j].t - p.t) < 1e-3 && Math.hypot(o.pts[j].x - p.x, o.pts[j].y - p.y) < 10) { same++; break; }
          }
        });
        if (tot && same / tot > 0.5) dup = true;
      }
      if (!dup) out.push(tr);
    });
    out.sort(function (a, b) { return a.pts[0].t - b.pts[0].t; });
    return out;
  };

  /* 把被擊球、彈跳或漏偵測切斷的片段接成整個回合的連續路徑 */
  function endVel(pts, atEnd) {
    var n = pts.length, k = Math.min(4, n - 1), a, b;
    if (k < 1) return [0, 0];
    if (atEnd) { a = pts[n - 1 - k]; b = pts[n - 1]; } else { a = pts[0]; b = pts[k]; }
    var dt = Math.max(1e-3, b.t - a.t);
    return [(b.x - a.x) / dt, (b.y - a.y) / dt];
  }
  function stitch(tracks, opt) {
    var o = {}, k;
    for (k in DEFAULT_STITCH) o[k] = (opt && opt[k] !== undefined) ? opt[k] : DEFAULT_STITCH[k];
    var n = tracks.length, pairs = [], i, j;
    for (i = 0; i < n; i++) {
      var A = tracks[i], la = A.pts[A.pts.length - 1], va = endVel(A.pts, true);
      for (j = 0; j < n; j++) {
        if (i === j) continue;
        var B = tracks[j], fb = B.pts[0], gap = fb.t - la.t;
        if (gap < -0.02 || gap > o.maxGap) continue;
        var vb = endVel(B.pts, false), vm = Math.max(Math.hypot(va[0], va[1]), Math.hypot(vb[0], vb[1]));
        var reach = o.base + o.k * vm * Math.max(0, gap);
        var d = Math.hypot(fb.x - la.x, fb.y - la.y);
        if (d > reach) continue;
        pairs.push({ i: i, j: j, cost: d / reach + 0.3 * Math.max(0, gap) });
      }
    }
    pairs.sort(function (a, b) { return a.cost - b.cost; });
    var next = [], prev = [];
    pairs.forEach(function (p) {
      if (next[p.i] !== undefined || prev[p.j] !== undefined) return;
      var r = p.i; while (prev[r] !== undefined) r = prev[r];
      if (r === p.j) return;                       // 避免成環
      next[p.i] = p.j; prev[p.j] = p.i;
    });
    var out = [];
    for (i = 0; i < n; i++) {
      if (prev[i] !== undefined) continue;
      var pts = [], c = i, parts = 0;
      while (c !== undefined) { parts++; tracks[c].pts.forEach(function (q) { if (!pts.length || q.t > pts[pts.length - 1].t) pts.push(q); }); c = next[c]; }
      var disp = Math.hypot(pts[pts.length - 1].x - pts[0].x, pts[pts.length - 1].y - pts[0].y);
      if (pts.length >= o.minPts && disp >= o.minDisp) out.push({ id: tracks[i].id, parts: parts, pts: pts });
    }
    /* 同一時間只留最像球的那條（轉播畫面通常只有一顆球在飛） */
    out.sort(function (a, b) { return b.pts.length - a.pts.length; });
    var kept = [];
    out.forEach(function (p) {
      var t0 = p.pts[0].t, t1 = p.pts[p.pts.length - 1].t, clash = false;
      kept.forEach(function (q) {
        var a = q.pts[0].t, b = q.pts[q.pts.length - 1].t, ov = Math.min(t1, b) - Math.max(t0, a);
        if (ov > o.overlap * (t1 - t0)) clash = true;
      });
      if (!clash) kept.push(p);
    });
    kept.sort(function (a, b) { return a.pts[0].t - b.pts[0].t; });
    return kept;
  }
  Tracker.prototype.paths = function (opt) { return stitch(this.results(), opt); };

  /*
   * 離線階段：吃「解碼時存下來的候選點」，跑追蹤＋串接。
   * frames: [{ t, c: [{x,y,area,s,iso}] }]
   * 找不到路徑時會自動放寬條件重試，回傳 { paths, level, levelName, tried }。
   */
  function LEVELS(base) {
    var mk = function (over) { var o = {}, k; for (k in base) o[k] = base[k]; for (k in over) o[k] = over[k]; return o; };
    return [
      { name: '標準', track: mk({}), stitch: {} },
      { name: '放寬', track: mk({ scoreMin: base.scoreMin * 0.7, isoMax: base.isoMax * 2, smoothFrac: 0.4, smoothTol: base.smoothTol * 1.4, confirmHits: 3, minPts: 4, vAvgMin: 100 }), stitch: { minPts: 8, minDisp: 80, maxGap: 0.7 } },
      { name: '最寬鬆', track: mk({ scoreMin: base.scoreMin * 0.45, isoMax: 1e9, smoothFrac: 0.25, smoothTol: base.smoothTol * 2, confirmHits: 3, minPts: 3, minDisp: 15, vAvgMin: 60, vMin: 50 }), stitch: { minPts: 6, minDisp: 50, maxGap: 0.9, overlap: 0.6 } }
    ];
  }
  function pathsFrom(frames, opt, maxLevel) {
    var base = {}, k;
    for (k in DEFAULT_TRACK) base[k] = (opt && opt[k] !== undefined) ? opt[k] : DEFAULT_TRACK[k];
    var levels = LEVELS(base), tried = [], i;
    for (i = 0; i < levels.length && i <= (maxLevel === undefined ? 2 : maxLevel); i++) {
      var tr = new Tracker(levels[i].track), j;
      for (j = 0; j < frames.length; j++) tr.update(frames[j].t, frames[j].c);
      tr.flush();
      var raw = tr.results(), paths = stitch(raw, levels[i].stitch);
      tried.push({ name: levels[i].name, raw: raw.length, paths: paths.length, used: tr.used, seen: tr.seen });
      if (paths.length) return { paths: paths, level: i, levelName: levels[i].name, tried: tried };
    }
    return { paths: [], level: -1, levelName: null, tried: tried };
  }

  var api = {
    DEFAULT_DETECT: DEFAULT_DETECT, DEFAULT_TRACK: DEFAULT_TRACK, DEFAULT_STITCH: DEFAULT_STITCH,
    makeFrame: makeFrame, features: features, newBuf: newBuf, detect: detect,
    Tracker: Tracker, stitch: stitch, smoothFrac: smoothFrac, pathsFrom: pathsFrom
  };
  root.TCTracker = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
