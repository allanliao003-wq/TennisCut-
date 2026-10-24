/*
 * 球場幾何與球速估算
 *
 * 球場座標（公尺）：X 沿底線方向（左到右），Y 沿球場長度（近端底線 = 0），Z 向上。
 * 使用者在畫面上點四個角（近端左、近端右、遠端右、遠端左），由已知的球場尺寸求出
 * 平面單應矩陣，再假設像素為正方形、主點在畫面中央，推算焦距與相機位置。
 * 球在空中時不在地面上，所以球速不是直接用地面比例尺換算，而是把球的 2D 軌跡
 * 擬合成受重力的拋物線（已知 g = 9.81 m/s²），重力提供了公尺的尺度。
 */
(function (root) {
  'use strict';

  var GRAV = 9.81;

  function solve(A, b) {
    var n = b.length, M = [], i, j, k;
    for (i = 0; i < n; i++) { M.push(A[i].slice()); M[i].push(b[i]); }
    for (i = 0; i < n; i++) {
      var p = i;
      for (k = i + 1; k < n; k++) if (Math.abs(M[k][i]) > Math.abs(M[p][i])) p = k;
      if (Math.abs(M[p][i]) < 1e-18) return null;
      var tmp = M[i]; M[i] = M[p]; M[p] = tmp;
      for (k = i + 1; k < n; k++) {
        var f = M[k][i] / M[i][i];
        if (f === 0) continue;
        for (j = i; j <= n; j++) M[k][j] -= f * M[i][j];
      }
    }
    var x = new Array(n);
    for (i = n - 1; i >= 0; i--) {
      var s = M[i][n];
      for (j = i + 1; j < n; j++) s -= M[i][j] * x[j];
      x[i] = s / M[i][i];
    }
    return x;
  }

  function homographyFromPoints(src, dst) {
    var A = [], b = [], i;
    for (i = 0; i < 4; i++) {
      var X = src[i][0], Y = src[i][1], u = dst[i][0], v = dst[i][1];
      A.push([X, Y, 1, 0, 0, 0, -u * X, -u * Y]); b.push(u);
      A.push([0, 0, 0, X, Y, 1, -v * X, -v * Y]); b.push(v);
    }
    var h = solve(A, b);
    if (!h) return null;
    return [[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], 1]];
  }

  function applyH(H, x, y) {
    var w = H[2][0] * x + H[2][1] * y + H[2][2];
    return [(H[0][0] * x + H[0][1] * y + H[0][2]) / w, (H[1][0] * x + H[1][1] * y + H[1][2]) / w];
  }

  function invert3(M) {
    var a = M[0][0], b = M[0][1], c = M[0][2], d = M[1][0], e = M[1][1], f = M[1][2], g = M[2][0], h = M[2][1], i = M[2][2];
    var A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
    var det = a * A + b * B + c * C;
    if (Math.abs(det) < 1e-18) return null;
    return [
      [A / det, -(b * i - c * h) / det, (b * f - c * e) / det],
      [B / det, (a * i - c * g) / det, -(a * f - c * d) / det],
      [C / det, -(a * h - b * g) / det, (a * e - b * d) / det]
    ];
  }

  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
  function norm(a) { return Math.sqrt(dot(a, a)); }
  function scale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
  function unit(a) { return scale(a, 1 / norm(a)); }

  function courtDims(type) {
    return type === 'singles' ? { w: 8.23, l: 23.77 } : { w: 10.97, l: 23.77 };
  }

  /* corners: [[u,v] x4]，順序為近端左、近端右、遠端右、遠端左（影片原始像素） */
  function calibrate(corners, type, imgW, imgH) {
    var d = courtDims(type);
    var world = [[0, 0], [d.w, 0], [d.w, d.l], [0, d.l]];
    var H = homographyFromPoints(world, corners);
    if (!H) return null;
    var Hinv = invert3(H);
    if (!Hinv) return null;
    var cx = imgW / 2, cy = imgH / 2;
    var Hc = [
      [H[0][0] - cx * H[2][0], H[0][1] - cx * H[2][1], H[0][2] - cx * H[2][2]],
      [H[1][0] - cy * H[2][0], H[1][1] - cy * H[2][1], H[1][2] - cy * H[2][2]],
      [H[2][0], H[2][1], H[2][2]]
    ];
    var h1 = [Hc[0][0], Hc[1][0], Hc[2][0]], h2 = [Hc[0][1], Hc[1][1], Hc[2][1]], h3 = [Hc[0][2], Hc[1][2], Hc[2][2]];
    var fa = null, fb = null, f, assumed = false, v;
    var den1 = h1[2] * h2[2];
    if (Math.abs(den1) > 1e-12) { v = -(h1[0] * h2[0] + h1[1] * h2[1]) / den1; if (v > 0) fa = Math.sqrt(v); }
    var den2 = h2[2] * h2[2] - h1[2] * h1[2];
    if (Math.abs(den2) > 1e-12) { v = ((h1[0] * h1[0] + h1[1] * h1[1]) - (h2[0] * h2[0] + h2[1] * h2[1])) / den2; if (v > 0) fb = Math.sqrt(v); }
    var lo = 0.45 * imgW, hi = 3 * imgW;
    var okA = fa && fa >= lo && fa <= hi, okB = fb && fb >= lo && fb <= hi;
    if (okA && okB && fa / fb > 0.7 && fa / fb < 1.43) f = Math.sqrt(fa * fb);
    else if (okA) f = fa;
    else if (okB) f = fb;
    else { f = 0.85 * imgW; assumed = true; }

    var a1 = [h1[0] / f, h1[1] / f, h1[2]], a2 = [h2[0] / f, h2[1] / f, h2[2]], a3 = [h3[0] / f, h3[1] / f, h3[2]];
    var lam = 2 / (norm(a1) + norm(a2));
    var r1 = scale(a1, lam), r2 = scale(a2, lam), t = scale(a3, lam);
    if (t[2] < 0) { r1 = scale(r1, -1); r2 = scale(r2, -1); t = scale(t, -1); }
    var r1n = unit(r1);
    var r2o = [r2[0] - dot(r2, r1n) * r1n[0], r2[1] - dot(r2, r1n) * r1n[1], r2[2] - dot(r2, r1n) * r1n[2]];
    var r2n = unit(r2o);
    var r3 = cross(r1n, r2n);
    var R = [[r1n[0], r2n[0], r3[0]], [r1n[1], r2n[1], r3[1]], [r1n[2], r2n[2], r3[2]]];
    var C = [0, 0, 0], i, j;
    for (j = 0; j < 3; j++) { for (i = 0; i < 3; i++) C[j] -= R[i][j] * t[i]; }
    var warn = null;
    if (C[2] < 0.3) warn = '推算出的相機高度不合理，請確認四個角的點選順序（近端左、近端右、遠端右、遠端左）。';
    return { type: type, dims: d, f: f, fAssumed: assumed, cx: cx, cy: cy, imgW: imgW, imgH: imgH, R: R, t: t, C: C, H: H, Hinv: Hinv, warn: warn };
  }

  function camPoint(cal, P) {
    var R = cal.R, t = cal.t;
    return [R[0][0] * P[0] + R[0][1] * P[1] + R[0][2] * P[2] + t[0],
            R[1][0] * P[0] + R[1][1] * P[1] + R[1][2] * P[2] + t[1],
            R[2][0] * P[0] + R[2][1] * P[1] + R[2][2] * P[2] + t[2]];
  }
  function project(cal, P) {
    var c = camPoint(cal, P);
    if (c[2] <= 1e-6) return null;
    return [cal.cx + cal.f * c[0] / c[2], cal.cy + cal.f * c[1] / c[2]];
  }
  function rayFor(cal, u, v) {
    var dc = [(u - cal.cx) / cal.f, (v - cal.cy) / cal.f, 1], R = cal.R;
    var dw = [R[0][0] * dc[0] + R[1][0] * dc[1] + R[2][0] * dc[2],
              R[0][1] * dc[0] + R[1][1] * dc[1] + R[2][1] * dc[2],
              R[0][2] * dc[0] + R[1][2] * dc[1] + R[2][2] * dc[2]];
    return unit(dw);
  }
  function pixelToGround(cal, u, v) { return applyH(cal.Hinv, u, v); }

  /* 球場線段（世界座標）。回傳 { lines: [[P1,P2]...], net: [P1,P2] } */
  function courtLines(type) {
    var d = courtDims(type), w = d.w, l = d.l, s = type === 'singles' ? 0 : 1.37;
    var sl = 5.485, fl = l - 5.485, mid = l / 2, cxm = w / 2;
    var lines = [
      [[0, 0, 0], [w, 0, 0]], [[w, 0, 0], [w, l, 0]], [[w, l, 0], [0, l, 0]], [[0, l, 0], [0, 0, 0]],
      [[s, sl, 0], [w - s, sl, 0]], [[s, fl, 0], [w - s, fl, 0]], [[cxm, sl, 0], [cxm, fl, 0]],
      [[cxm, 0, 0], [cxm, 0.2, 0]], [[cxm, l, 0], [cxm, l - 0.2, 0]]
    ];
    if (s > 0) { lines.push([[s, 0, 0], [s, l, 0]]); lines.push([[w - s, 0, 0], [w - s, l, 0]]); }
    return { lines: lines, net: [[-0.914, mid, 1.07], [w + 0.914, mid, 1.07]], netBase: [[-0.914, mid, 0], [w + 0.914, mid, 0]], dims: d };
  }

  /* 球可能出現的空間（用來限制偵測範圍）投影後的凸包，座標為影片原始像素 */
  function roiPolygon(cal, margin) {
    var d = cal.dims, m = margin || 4, pts = [], xs = [-m, d.w + m], ys = [-m * 2, d.l + m * 2], zs = [0, 9], a, b, c;
    for (a = 0; a < 2; a++) for (b = 0; b < 2; b++) for (c = 0; c < 2; c++) {
      var q = project(cal, [xs[a], ys[b], zs[c]]);
      if (q) pts.push(q);
    }
    // 球場本身四個角與地面也加入，避免相機很高或很低時凸包缺角
    [[0, 0], [d.w, 0], [d.w, d.l], [0, d.l]].forEach(function (p) { var q = project(cal, [p[0], p[1], 0]); if (q) pts.push(q); });
    if (pts.length < 3) return null;
    return convexHull(pts);
  }
  function convexHull(P) {
    P = P.slice().sort(function (a, b) { return a[0] - b[0] || a[1] - b[1]; });
    function cr(o, a, b) { return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]); }
    var lo = [], up = [], i;
    for (i = 0; i < P.length; i++) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], P[i]) <= 0) lo.pop(); lo.push(P[i]); }
    for (i = P.length - 1; i >= 0; i--) { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], P[i]) <= 0) up.pop(); up.push(P[i]); }
    lo.pop(); up.pop();
    return lo.concat(up);
  }
  function pointInPoly(poly, x, y) {
    var inside = false, i, j;
    for (i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      var xi = poly[i][0], yi = poly[i][1], xj = poly[j][0], yj = poly[j][1];
      if (((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) inside = !inside;
    }
    return inside;
  }

  function skew(d) { return [[0, -d[2], d[1]], [d[2], 0, -d[0]], [-d[1], d[0], 0]]; }

  /*
   * 把一段飛行軌跡擬合成受重力的拋物線。
   * obs: [{t,u,v}]，anchors: [{t, X, Y, Z, wt}]（已知在地面的點，例如彈跳點）
   * 每個觀測給出「球必須落在這條射線上」的線性條件，未知數是起點 p0 與初速 v0 共 6 個，
   * 所以是線性最小平方；以深度倒數加權近似像素誤差，迭代 3 次。
   */
  function fitFlight(cal, obs, anchors, sigmaPx) {
    var n = obs.length, t0 = obs[0].t, C = cal.C, i, k, a, b, it;
    var rays = obs.map(function (o) { return rayFor(cal, o.u, o.v); });
    var ctr = [cal.dims.w / 2, cal.dims.l / 2, 1];
    var d0 = norm([C[0] - ctr[0], C[1] - ctr[1], C[2] - ctr[2]]);
    var depth = obs.map(function () { return d0; });
    var aDepth = (anchors || []).map(function () { return d0; });
    var theta = null, N = null;
    var use = obs.map(function () { return true; });

    function build() {
      N = []; for (a = 0; a < 6; a++) { N.push([0, 0, 0, 0, 0, 0]); }
      var r = [0, 0, 0, 0, 0, 0];
      function acc(row, bb) {
        for (var p = 0; p < 6; p++) { if (row[p] === 0) continue; for (var q = 0; q < 6; q++) N[p][q] += row[p] * row[q]; r[p] += row[p] * bb; }
      }
      for (i = 0; i < n; i++) {
        if (!use[i]) continue;
        var tau = obs[i].t - t0, A = skew(rays[i]), w = 1 / depth[i];
        var cc = [C[0], C[1], C[2] + 0.5 * GRAV * tau * tau];
        for (k = 0; k < 3; k++) {
          var row = [w * A[k][0], w * A[k][1], w * A[k][2], w * tau * A[k][0], w * tau * A[k][1], w * tau * A[k][2]];
          acc(row, w * (A[k][0] * cc[0] + A[k][1] * cc[1] + A[k][2] * cc[2]));
        }
      }
      (anchors || []).forEach(function (an, ai) {
        var tau = an.t - t0, w = (an.wt || 2) / aDepth[ai];
        var P = [an.X, an.Y, an.Z], g = [0, 0, -GRAV];
        for (var ax = 0; ax < 3; ax++) {
          var row = [0, 0, 0, 0, 0, 0]; row[ax] = w; row[3 + ax] = w * tau;
          acc(row, w * (P[ax] - 0.5 * g[ax] * tau * tau));
        }
      });
      for (a = 0; a < 6; a++) N[a][a] += 1e-12;
      return r;
    }
    function pos(th, tau) { return [th[0] + th[3] * tau, th[1] + th[4] * tau, th[2] + th[5] * tau - 0.5 * GRAV * tau * tau]; }
    function residuals() {
      var res = [];
      for (i = 0; i < n; i++) {
        var q = project(cal, pos(theta, obs[i].t - t0));
        res.push(q ? Math.hypot(q[0] - obs[i].u, q[1] - obs[i].v) : 1e9);
      }
      return res;
    }

    for (it = 0; it < 4; it++) {
      var rhs = build();
      theta = solve(N, rhs);
      if (!theta) return null;
      for (i = 0; i < n; i++) { var P = pos(theta, obs[i].t - t0); depth[i] = Math.max(2, norm([P[0] - C[0], P[1] - C[1], P[2] - C[2]])); }
      (anchors || []).forEach(function (an, ai) { aDepth[ai] = Math.max(2, norm([an.X - C[0], an.Y - C[1], an.Z - C[2]])); });
      if (it === 1) {
        // 去除離群觀測後重新擬合
        var res = residuals(), sorted = res.slice().sort(function (x, y) { return x - y; });
        var med = sorted[sorted.length >> 1], lim = Math.max(3 * med, 0.004 * cal.imgW), kept = 0;
        for (i = 0; i < n; i++) { use[i] = res[i] <= lim; if (use[i]) kept++; }
        if (kept < 5) for (i = 0; i < n; i++) use[i] = true;
      }
    }
    var res2 = residuals(), s2 = 0, c2 = 0;
    for (i = 0; i < n; i++) if (use[i]) { s2 += res2[i] * res2[i]; c2++; }
    var rms = Math.sqrt(s2 / Math.max(1, c2));

    // 速度不確定度：cov = σ² N⁻¹，σ 以弧度計（像素誤差 / 焦距）
    var sig = (sigmaPx || Math.max(1, rms)) / cal.f, cov = [];
    for (a = 0; a < 6; a++) {
      var e = [0, 0, 0, 0, 0, 0]; e[a] = 1;
      cov.push(solve(N, e) || [0, 0, 0, 0, 0, 0]);
    }
    var v0 = [theta[3], theta[4], theta[5]], sp = norm(v0), vh = sp > 0 ? scale(v0, 1 / sp) : [1, 0, 0], varv = 0;
    for (a = 0; a < 3; a++) for (b = 0; b < 3; b++) varv += vh[a] * vh[b] * cov[3 + a][3 + b];
    var sigmaV = sig * Math.sqrt(Math.max(0, varv));
    return { t0: t0, p0: [theta[0], theta[1], theta[2]], v0: v0, speed0: sp, sigmaSpeed: sigmaV, rms: rms, n: c2 };
  }

  function velocityAt(fit, t) {
    var tau = t - fit.t0;
    return [fit.v0[0], fit.v0[1], fit.v0[2] - GRAV * tau];
  }
  function speedAt(fit, t) { return norm(velocityAt(fit, t)); }
  function posAt(fit, t) {
    var tau = t - fit.t0;
    return [fit.p0[0] + fit.v0[0] * tau, fit.p0[1] + fit.v0[1] * tau, fit.p0[2] + fit.v0[2] * tau - 0.5 * GRAV * tau * tau];
  }

  /* 找出影像上的彈跳點：球先往下（v 增加）再往上，且在局部最低點 */
  function findBounces(pts, imgH) {
    var out = [], thr = 0.12 * imgH, i, n = pts.length;
    for (i = 2; i < n - 2; i++) {
      var dtIn = pts[i].t - pts[i - 2].t, dtOut = pts[i + 2].t - pts[i].t;
      if (dtIn <= 0 || dtOut <= 0) continue;
      var vin = (pts[i].v - pts[i - 2].v) / dtIn, vout = (pts[i + 2].v - pts[i].v) / dtOut;
      if (vin > thr && vout < -0.35 * thr && (vin - vout) > 2 * thr) {
        var isMax = true, j;
        for (j = Math.max(0, i - 2); j <= Math.min(n - 1, i + 2); j++) if (pts[j].v > pts[i].v + 0.5) { isMax = false; break; }
        if (isMax) out.push(i);
      }
    }
    return out;
  }

  /*
   * 把每條軌跡依彈跳點與擊球聲切成飛行段，逐段估算球速。
   * 回傳 segments：{ trackId, t0, t1, kind('hit'|'bounce'|'start'), ok, kmh, sigmaKmh, rms, fit }
   */
  function computeShots(cal, tracks, hitTimes, opt) {
    opt = opt || {};
    var imgH = opt.imgH || cal.imgH, imgW = opt.imgW || cal.imgW, out = [];
    tracks.forEach(function (tr) {
      var pts = tr.pts, n = pts.length;
      if (n < 6) return;
      var splits = {}, kind = {};
      findBounces(pts, imgH).forEach(function (i) { splits[i] = 'bounce'; });
      (hitTimes || []).forEach(function (ht) {
        var best = -1, bd = 0.12, i;
        for (i = 2; i < n - 2; i++) { var dd = Math.abs(pts[i].t - ht); if (dd < bd) { bd = dd; best = i; } }
        if (best >= 0 && !splits[best]) splits[best] = 'hit';
      });
      var idx = Object.keys(splits).map(Number).sort(function (a, b) { return a - b; }), cuts = [], last = -10;
      idx.forEach(function (i) { if (i - last >= 3) { cuts.push(i); last = i; } });
      var bounds = [0].concat(cuts, [n - 1]), s;
      for (s = 0; s < bounds.length - 1; s++) {
        var i0 = bounds[s], i1 = bounds[s + 1], seg = pts.slice(i0, i1 + 1);
        if (seg.length < 5 || seg[seg.length - 1].t - seg[0].t < 0.12) continue;
        var startKind = s === 0 ? 'start' : splits[i0], endKind = s === bounds.length - 2 ? null : splits[i1];
        var anchors = [];
        if (opt.bounceAnchors && startKind === 'bounce') { var g0 = pixelToGround(cal, seg[0].u, seg[0].v); anchors.push({ t: seg[0].t, X: g0[0], Y: g0[1], Z: 0, wt: 2 }); }
        if (opt.bounceAnchors && endKind === 'bounce') { var L = seg[seg.length - 1], g1 = pixelToGround(cal, L.u, L.v); anchors.push({ t: L.t, X: g1[0], Y: g1[1], Z: 0, wt: 2 }); }
        var fit = fitFlight(cal, seg.map(function (p) { return { t: p.t, u: p.u, v: p.v }; }), anchors);
        var rec = { trackId: tr.id, t0: seg[0].t, t1: seg[seg.length - 1].t, kind: startKind, endKind: endKind, n: seg.length, ok: false, kmh: null, sigmaKmh: null, rms: null, fit: fit };
        if (fit) {
          var kmh = fit.speed0 * 3.6, sg = fit.sigmaSpeed * 3.6;
          rec.kmh = kmh; rec.sigmaKmh = sg; rec.rms = fit.rms;
          rec.ok = kmh >= 60 && kmh <= 270 && fit.rms <= 0.004 * imgW && sg <= Math.max(12, 0.3 * kmh);
        }
        out.push(rec);
      }
    });
    out.sort(function (a, b) { return a.t0 - b.t0; });
    return out;
  }

  var api = {
    GRAV: GRAV, solve: solve, homographyFromPoints: homographyFromPoints, applyH: applyH, invert3: invert3,
    courtDims: courtDims, calibrate: calibrate, project: project, rayFor: rayFor, pixelToGround: pixelToGround,
    courtLines: courtLines, roiPolygon: roiPolygon, pointInPoly: pointInPoly, fitFlight: fitFlight,
    velocityAt: velocityAt, speedAt: speedAt, posAt: posAt, findBounces: findBounces, computeShots: computeShots
  };
  root.TCGeometry = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
