/* インタラクティブデモ: ICP / IMU ドリフト / 式と因子の対応 / 2D 再現実験の再生 / トップページのスキャン描画
   すべて素の JavaScript (ライブラリなし)。色は CSS 変数から読むのでライト/ダークの両方で読める */
(function () {
  "use strict";

  // ------------------------------------------------------------------ 共通
  function css(name) {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  }
  function el(tag, attrs, html) {
    var e = document.createElement(tag);
    if (attrs) Object.keys(attrs).forEach(function (k) { e.setAttribute(k, attrs[k]); });
    if (html != null) e.innerHTML = html;
    return e;
  }
  function fitCanvas(canvas, aspect) {
    var r = window.devicePixelRatio || 1;
    var w = canvas.clientWidth || canvas.parentElement.clientWidth || 600;
    var h = Math.round(w * aspect);
    canvas.width = Math.round(w * r);
    canvas.height = Math.round(h * r);
    canvas.style.height = h + "px";
    var ctx = canvas.getContext("2d");
    ctx.setTransform(r, 0, 0, r, 0, 0);
    return { ctx: ctx, w: w, h: h };
  }
  function view(bounds, w, h, pad) {
    // bounds = [xmin, xmax, ymin, ymax] を縦横比を保って canvas に収める (y は上向き)
    pad = pad == null ? 12 : pad;
    var sx = (w - 2 * pad) / (bounds[1] - bounds[0]);
    var sy = (h - 2 * pad) / (bounds[3] - bounds[2]);
    var s = Math.min(sx, sy);
    var cx = (bounds[0] + bounds[1]) / 2, cy = (bounds[2] + bounds[3]) / 2;
    return {
      s: s,
      x: function (x) { return w / 2 + (x - cx) * s; },
      y: function (y) { return h / 2 - (y - cy) * s; },
    };
  }
  function mulberry32(a) {
    return function () {
      a |= 0; a = (a + 0x6d2b79f5) | 0;
      var t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  function gaussFactory(rand) {
    return function () {
      var u = Math.max(rand(), 1e-12), v = rand();
      return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
    };
  }
  function wrap(a) { return Math.atan2(Math.sin(a), Math.cos(a)); }
  function onResize(target, fn) {
    if (window.ResizeObserver) new ResizeObserver(function () { fn(); }).observe(target);
    else window.addEventListener("resize", fn);
  }
  function onTheme(fn) {
    if (window.matchMedia) {
      var mq = window.matchMedia("(prefers-color-scheme: dark)");
      if (mq.addEventListener) mq.addEventListener("change", fn);
    }
    new MutationObserver(fn).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
  }
  function shell(container, title, sub) {
    container.classList.add("demo");
    container.innerHTML = "";
    var head = el("div", { class: "demo-head" }, "<strong>" + title + "</strong><span>" + sub + "</span>");
    var body = el("div", { class: "demo-body" });
    container.appendChild(head);
    container.appendChild(body);
    return body;
  }
  function seg(options, current, onChange) {
    var box = el("div", { class: "seg", role: "group" });
    options.forEach(function (o) {
      var b = el("button", { type: "button", "aria-pressed": o[0] === current ? "true" : "false" }, o[1]);
      b.addEventListener("click", function () {
        box.querySelectorAll("button").forEach(function (x) { x.setAttribute("aria-pressed", "false"); });
        b.setAttribute("aria-pressed", "true");
        onChange(o[0]);
      });
      box.appendChild(b);
    });
    return box;
  }

  // ------------------------------------------------------------------ 1. ICP
  function demoICP(root) {
    var body = shell(root, "ICP を 1 ステップずつ", "点対点と点対面で、収束の仕方を比べてみよう");
    var canvas = el("canvas", { "aria-label": "ICP の位置合わせの様子" });
    body.appendChild(canvas);
    var controls = el("div", { class: "controls" });
    var method = "p2p";
    controls.appendChild(seg([["p2p", "点対点 ICP"], ["p2l", "点対面 ICP"]], method, function (m) { method = m; reset(); }));
    var stepBtn = el("button", { type: "button", class: "btn primary" }, "1 ステップ");
    var runBtn = el("button", { type: "button", class: "btn" }, "収束まで");
    var resetBtn = el("button", { type: "button", class: "btn" }, "リセット");
    controls.appendChild(stepBtn); controls.appendChild(runBtn); controls.appendChild(resetBtn);
    var lab = el("label", { class: "slider-row" }, "<span>初期のずれ</span>");
    var slider = el("input", { type: "range", min: "0.3", max: "2.2", step: "0.1", value: "1", id: "icp-offset" });
    lab.appendChild(slider);
    lab.appendChild(el("span", null, ""));
    controls.appendChild(lab);
    body.appendChild(controls);
    var readout = el("div", { class: "readout" });
    body.appendChild(readout);
    body.appendChild(el("div", { class: "legend" },
      '<span><i class="dot" style="--c:var(--muted)"></i>ターゲット (地図側)</span>' +
      '<span><i class="dot" style="--c:var(--lidar)"></i>ソース (動かす側)</span>' +
      '<span><i style="--c:var(--imu)"></i>対応 (最近傍)</span>'));

    // 形状: 部屋の角 + 箱 + 斜めの壁
    var segs = [[-4, -2, 4, -2], [4, -2, 4, 3], [-4, -2, -4, 1], [-4, 1, -2, 3], [0, 0.5, 1.2, 0.5], [1.2, 0.5, 1.2, 1.5], [1.2, 1.5, 0, 1.5], [0, 1.5, 0, 0.5]];
    var rand = mulberry32(7), gauss = gaussFactory(rand);
    function sample(phase, noise) {
      var pts = [], nrm = [];
      segs.forEach(function (s) {
        var dx = s[2] - s[0], dy = s[3] - s[1], L = Math.hypot(dx, dy);
        for (var d = phase; d < L; d += 0.3) {
          pts.push([s[0] + dx * d / L + noise * gauss(), s[1] + dy * d / L + noise * gauss()]);
          nrm.push([-dy / L, dx / L]);
        }
      });
      return { pts: pts, nrm: nrm };
    }
    var tgt = sample(0.0, 0.01);
    var srcWorld = sample(0.15, 0.02).pts; // 違う位置で取ったサンプル
    var TRUE = { th: 0.3, x: 0.6, y: -0.4 };
    // ソース点はセンサ座標 = TRUE^{-1} * 世界座標
    var c0 = Math.cos(TRUE.th), s0 = Math.sin(TRUE.th);
    var src = srcWorld.map(function (p) {
      var dx = p[0] - TRUE.x, dy = p[1] - TRUE.y;
      return [c0 * dx + s0 * dy, -s0 * dx + c0 * dy];
    });
    var est, iter, corr, timer = null;

    function reset() {
      var k = parseFloat(slider.value);
      est = { th: TRUE.th - 0.35 * k, x: TRUE.x - 0.9 * k, y: TRUE.y + 0.6 * k };
      iter = 0; corr = [];
      stop();
      findCorr();
      draw();
    }
    function transformed() {
      var c = Math.cos(est.th), s = Math.sin(est.th);
      return src.map(function (p) { return [c * p[0] - s * p[1] + est.x, s * p[0] + c * p[1] + est.y]; });
    }
    function findCorr() {
      var P = transformed();
      corr = [];
      P.forEach(function (p, i) {
        var best = -1, bd = 1e9;
        tgt.pts.forEach(function (q, j) {
          var d = (p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1]);
          if (d < bd) { bd = d; best = j; }
        });
        if (bd < 2.0 * 2.0) corr.push([i, best]);
      });
      return P;
    }
    function step() {
      var P = findCorr();
      if (corr.length < 3) return;
      if (method === "p2p") {
        var mp = [0, 0], mq = [0, 0];
        corr.forEach(function (c) { mp[0] += P[c[0]][0]; mp[1] += P[c[0]][1]; mq[0] += tgt.pts[c[1]][0]; mq[1] += tgt.pts[c[1]][1]; });
        mp[0] /= corr.length; mp[1] /= corr.length; mq[0] /= corr.length; mq[1] /= corr.length;
        var sxx = 0, sxy = 0;
        corr.forEach(function (c) {
          var a = [P[c[0]][0] - mp[0], P[c[0]][1] - mp[1]], b = [tgt.pts[c[1]][0] - mq[0], tgt.pts[c[1]][1] - mq[1]];
          sxx += a[0] * b[0] + a[1] * b[1];
          sxy += a[0] * b[1] - a[1] * b[0];
        });
        var dth = Math.atan2(sxy, sxx), c = Math.cos(dth), s = Math.sin(dth);
        var tx = mq[0] - (c * mp[0] - s * mp[1]), ty = mq[1] - (s * mp[0] + c * mp[1]);
        est = { th: wrap(est.th + dth), x: c * est.x - s * est.y + tx, y: s * est.x + c * est.y + ty };
      } else {
        var H = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], g = [0, 0, 0];
        var cs = Math.cos(est.th), sn = Math.sin(est.th);
        corr.forEach(function (cc) {
          var p = src[cc[0]], q = tgt.pts[cc[1]], n = tgt.nrm[cc[1]];
          var pw = P[cc[0]];
          var e = n[0] * (pw[0] - q[0]) + n[1] * (pw[1] - q[1]);
          var dpx = -sn * p[0] - cs * p[1], dpy = cs * p[0] - sn * p[1];
          var J = [n[0], n[1], n[0] * dpx + n[1] * dpy];
          for (var a = 0; a < 3; a++) { g[a] += J[a] * e; for (var b = 0; b < 3; b++) H[a][b] += J[a] * J[b]; }
        });
        for (var d = 0; d < 3; d++) H[d][d] += 1e-6;
        var dx = solve3(H, g.map(function (v) { return -v; }));
        est = { th: wrap(est.th + dx[2]), x: est.x + dx[0], y: est.y + dx[1] };
      }
      iter++;
      findCorr();
      draw();
    }
    function solve3(A, b) {
      var M = A.map(function (r, i) { return r.concat([b[i]]); });
      for (var i = 0; i < 3; i++) {
        var p = i;
        for (var r = i + 1; r < 3; r++) if (Math.abs(M[r][i]) > Math.abs(M[p][i])) p = r;
        var t = M[i]; M[i] = M[p]; M[p] = t;
        for (var r2 = i + 1; r2 < 3; r2++) {
          var f = M[r2][i] / M[i][i];
          for (var c = i; c < 4; c++) M[r2][c] -= f * M[i][c];
        }
      }
      var x = [0, 0, 0];
      for (var k = 2; k >= 0; k--) {
        var s = M[k][3];
        for (var j = k + 1; j < 3; j++) s -= M[k][j] * x[j];
        x[k] = s / M[k][k];
      }
      return x;
    }
    function stop() { if (timer) { clearInterval(timer); timer = null; runBtn.textContent = "収束まで"; } }
    function draw() {
      var f = fitCanvas(canvas, 0.62), ctx = f.ctx;
      var V = view([-5.2, 5.2, -3.6, 3.8], f.w, f.h, 10);
      ctx.clearRect(0, 0, f.w, f.h);
      var P = transformed();
      ctx.strokeStyle = css("--imu"); ctx.lineWidth = 1; ctx.globalAlpha = 0.75;
      ctx.beginPath();
      corr.forEach(function (c) {
        ctx.moveTo(V.x(P[c[0]][0]), V.y(P[c[0]][1]));
        ctx.lineTo(V.x(tgt.pts[c[1]][0]), V.y(tgt.pts[c[1]][1]));
      });
      ctx.stroke(); ctx.globalAlpha = 1;
      var r = Math.max(2.2, f.w / 260);
      ctx.fillStyle = css("--muted");
      tgt.pts.forEach(function (q) { ctx.beginPath(); ctx.arc(V.x(q[0]), V.y(q[1]), r, 0, 7); ctx.fill(); });
      ctx.fillStyle = css("--lidar");
      P.forEach(function (p) { ctx.beginPath(); ctx.arc(V.x(p[0]), V.y(p[1]), r, 0, 7); ctx.fill(); });
      var et = Math.hypot(est.x - TRUE.x, est.y - TRUE.y) * 100, er = Math.abs(wrap(est.th - TRUE.th)) * 180 / Math.PI;
      var md = 0;
      corr.forEach(function (c) { md += Math.hypot(P[c[0]][0] - tgt.pts[c[1]][0], P[c[0]][1] - tgt.pts[c[1]][1]); });
      md = corr.length ? md / corr.length : 0;
      readout.innerHTML = "<span>反復 <b>" + iter + "</b></span><span>並進誤差 <b>" + et.toFixed(1) + " cm</b></span><span>回転誤差 <b>" + er.toFixed(2) +
        "°</b></span><span>対応点 <b>" + corr.length + "</b></span><span>平均対応距離 <b>" + md.toFixed(3) + " m</b></span>";
    }
    stepBtn.addEventListener("click", function () { stop(); step(); });
    runBtn.addEventListener("click", function () {
      if (timer) { stop(); return; }
      runBtn.textContent = "停止";
      var n = 0;
      timer = setInterval(function () { step(); if (++n >= 40) stop(); }, 120);
    });
    resetBtn.addEventListener("click", reset);
    slider.addEventListener("input", reset);
    onResize(canvas, draw);
    onTheme(draw);
    reset();
  }

  // ------------------------------------------------------------------ 2. IMU ドリフト
  function demoIMU(root) {
    var body = shell(root, "IMU だけで積分すると", "バイアスを変えて、円運動の軌跡がどうずれるか見てみよう (2D)");
    var canvas = el("canvas", { "aria-label": "IMU 積分の軌跡" });
    body.appendChild(canvas);
    var controls = el("div", { class: "controls" });
    function slider(label, id, min, max, step, val, fmt) {
      var lab = el("label", { class: "slider-row" }, "<span>" + label + "</span>");
      var s = el("input", { type: "range", id: id, min: min, max: max, step: step, value: val });
      var out = el("b", { style: "font-family:var(--font-mono);font-weight:500;font-size:13px" });
      lab.appendChild(s); lab.appendChild(out);
      controls.appendChild(lab);
      function upd() { out.textContent = fmt(parseFloat(s.value)); }
      upd();
      s.addEventListener("input", function () { upd(); draw(); });
      return s;
    }
    var sBa = slider("加速度バイアス", "imu-ba", "0", "0.2", "0.01", "0.05", function (v) { return v.toFixed(2) + " m/s²"; });
    var sBg = slider("ジャイロバイアス", "imu-bg", "0", "1", "0.05", "0.2", function (v) { return v.toFixed(2) + " °/s"; });
    var sT = slider("積分時間", "imu-t", "2", "30", "1", "15", function (v) { return v.toFixed(0) + " 秒"; });
    var noiseLab = el("label", null, "");
    var noise = el("input", { type: "checkbox", id: "imu-noise", checked: "checked" });
    noiseLab.appendChild(noise); noiseLab.appendChild(document.createTextNode("ノイズあり"));
    noise.addEventListener("change", function () { draw(); });
    controls.appendChild(noiseLab);
    body.appendChild(controls);
    var readout = el("div", { class: "readout" });
    body.appendChild(readout);
    body.appendChild(el("div", { class: "legend" },
      '<span><i class="dash" style="--c:var(--ink)"></i>真の軌跡 (半径 5 m の円)</span>' +
      '<span><i style="--c:var(--imu)"></i>IMU だけで積分した軌跡</span>'));
    body.appendChild(el("p", { class: "eq-note", style: "font-size:13px;color:var(--muted);min-height:0" },
      "2D なので、3D で起きる「傾きによる重力の漏れ込み」は再現していません。ジャイロバイアスは向きのずれとして効きます。"));

    function draw() {
      var R = 5, v = 1.2, w = v / R, dt = 0.01;
      var T = parseFloat(sT.value), ba = parseFloat(sBa.value), bg = parseFloat(sBg.value) * Math.PI / 180;
      var g = gaussFactory(mulberry32(11));
      var useNoise = noise.checked;
      var n = Math.round(T / dt);
      var truth = [], est = [];
      var th = Math.PI / 2, px = R, py = 0, vx = 0, vy = v;
      for (var k = 0; k <= n; k++) {
        var t = k * dt;
        truth.push([R * Math.cos(w * t), R * Math.sin(w * t)]);
        est.push([px, py]);
        // 計測 (ボディ座標: x 前, y 左)。円運動の向心加速度は左向き v^2/R
        var gm = w + bg + (useNoise ? 0.003 * g() : 0);
        var ax = 0 + ba + (useNoise ? 0.03 * g() : 0), ay = v * v / R + (useNoise ? 0.03 * g() : 0);
        var c = Math.cos(th), s = Math.sin(th);
        var awx = c * ax - s * ay, awy = s * ax + c * ay;
        px += vx * dt + 0.5 * awx * dt * dt; py += vy * dt + 0.5 * awy * dt * dt;
        vx += awx * dt; vy += awy * dt;
        th += gm * dt;
      }
      var xs = truth.concat(est).map(function (p) { return p[0]; }), ys = truth.concat(est).map(function (p) { return p[1]; });
      var b = [Math.min.apply(null, xs) - 1, Math.max.apply(null, xs) + 1, Math.min.apply(null, ys) - 1, Math.max.apply(null, ys) + 1];
      var f = fitCanvas(canvas, 0.62), ctx = f.ctx;
      var V = view(b, f.w, f.h, 12);
      ctx.clearRect(0, 0, f.w, f.h);
      ctx.setLineDash([6, 5]); ctx.strokeStyle = css("--ink"); ctx.lineWidth = 1.5; ctx.globalAlpha = 0.6;
      ctx.beginPath(); truth.forEach(function (p, i) { (i ? ctx.lineTo : ctx.moveTo).call(ctx, V.x(p[0]), V.y(p[1])); }); ctx.stroke();
      ctx.setLineDash([]); ctx.globalAlpha = 1; ctx.strokeStyle = css("--imu"); ctx.lineWidth = 2.2;
      ctx.beginPath(); est.forEach(function (p, i) { (i ? ctx.lineTo : ctx.moveTo).call(ctx, V.x(p[0]), V.y(p[1])); }); ctx.stroke();
      function dot(p, col, r) { ctx.fillStyle = col; ctx.beginPath(); ctx.arc(V.x(p[0]), V.y(p[1]), r, 0, 7); ctx.fill(); }
      dot(truth[0], css("--ink"), 4);
      dot(truth[n], css("--ink"), 4);
      dot(est[n], css("--imu"), 4.5);
      // スケールバー (5 m)
      ctx.strokeStyle = css("--muted"); ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(14, f.h - 14); ctx.lineTo(14 + 5 * V.s, f.h - 14); ctx.stroke();
      ctx.fillStyle = css("--muted"); ctx.font = "12px " + css("--font-mono");
      ctx.fillText("5 m", 18, f.h - 20);
      var err = Math.hypot(est[n][0] - truth[n][0], est[n][1] - truth[n][1]);
      readout.innerHTML = "<span>" + T.toFixed(0) + " 秒後の位置誤差 <b>" + err.toFixed(2) + " m</b></span><span>加速度バイアスだけの目安 ½·b·t² = <b>" + (0.5 * ba * T * T).toFixed(2) + " m</b></span>";
    }
    onResize(canvas, draw);
    onTheme(draw);
    draw();
  }

  // ------------------------------------------------------------------ 3. 式と因子の対応
  function demoEquation(root) {
    var body = shell(root, "式の項と因子の対応", "項をタップすると、ファクターグラフの対応する因子が強調されます");
    var terms = [
      ["all", "すべて", "var(--ink)", "論文の目的関数は 4 種類の因子の和です。各項をタップして、グラフのどこに当たるか確かめましょう。"],
      ["scan", "第 1 項  Σ Σ f<sup>P</sup>(x<sub>i</sub>, x<sub>j</sub>)", "var(--lidar)", "スキャン間マッチング因子 (青)。各状態を 1 つ前・2 つ前のフレームと VGICP で比べます。2 つの状態をつなぐ二項因子です。"],
      ["map", "第 2 項  Σ f<sup>P</sup>(x<sub>i</sub>, M)", "var(--map)", "地図マッチング因子 (緑)。各状態のフレームを地図点群と比べます。地図は固定なので、状態 1 つだけにつながる単項因子です。地図との対応が取れない場所では自然に効かなくなります。"],
      ["imu", "第 3 項  Σ f<sup>I</sup>(x<sub>i-1</sub>, x<sub>i</sub>)", "var(--imu)", "IMU 事前積分因子 (オレンジ)。連続する状態の動きが IMU の事前積分 ΔR, Δv, Δp と整合するかを測ります。バイアスもここで推定されます。"],
      ["marg", "第 4 項  C(X)", "var(--muted)", "周辺化因子 (灰色)。ウィンドウから出た古い状態 (点線の丸) が持っていた情報を、残った状態への事前分布としてまとめたものです。"],
    ];
    var bar = el("div", { class: "eq-terms", role: "group" });
    var note = el("p", { class: "eq-note" });
    var graph = el("div", { class: "eq-graph" });
    body.appendChild(bar); body.appendChild(graph); body.appendChild(note);

    var X = [70, 190, 310, 430, 550, 670], Y = 150;
    var s = '<svg viewBox="0 0 760 290" role="img" aria-label="論文のファクターグラフ">';
    s += '<rect x="130" y="24" width="600" height="218" rx="14" fill="var(--accent-soft)" stroke="var(--lidar)" stroke-dasharray="6 5"/>';
    s += '<text x="146" y="46" font-size="14" fill="var(--lidar)" font-weight="700">最適化ウィンドウ</text>';
    var g = { scan: "", map: "", imu: "", marg: "" };
    for (var i = 1; i < 5; i++) {
      g.imu += '<line x1="' + (X[i] + 22) + '" y1="' + Y + '" x2="' + (X[i + 1] - 22) + '" y2="' + Y + '" stroke="var(--imu)" stroke-width="2"/>';
      g.imu += '<rect x="' + ((X[i] + X[i + 1]) / 2 - 7) + '" y="' + (Y - 7) + '" width="14" height="14" fill="var(--imu)"/>';
    }
    for (var j = 2; j <= 5; j++) {
      var a = X[j - 1], b2 = X[j];
      g.scan += '<path d="M' + (a + 12) + "," + (Y - 20) + " Q" + (a + b2) / 2 + "," + (Y - 70) + " " + (b2 - 12) + "," + (Y - 20) + '" fill="none" stroke="var(--lidar)" stroke-width="2"/>';
      g.scan += '<rect x="' + ((a + b2) / 2 - 7) + '" y="' + (Y - 52) + '" width="14" height="14" fill="var(--lidar)"/>';
    }
    for (var k = 3; k <= 5; k++) {
      var a2 = X[k - 2], b3 = X[k];
      g.scan += '<path d="M' + a2 + "," + (Y - 24) + " Q" + (a2 + b3) / 2 + "," + (Y - 150) + " " + b3 + "," + (Y - 24) + '" fill="none" stroke="var(--lidar)" stroke-width="2" opacity="0.8"/>';
      g.scan += '<rect x="' + ((a2 + b3) / 2 - 7) + '" y="' + (Y - 94) + '" width="14" height="14" fill="var(--lidar)"/>';
    }
    for (var m = 1; m <= 5; m++) {
      g.map += '<line x1="' + X[m] + '" y1="' + (Y + 22) + '" x2="' + X[m] + '" y2="' + (Y + 60) + '" stroke="var(--map)" stroke-width="2"/>';
      g.map += '<rect x="' + (X[m] - 7) + '" y="' + (Y + 54) + '" width="14" height="14" fill="var(--map)"/>';
      g.map += '<line x1="' + X[m] + '" y1="' + (Y + 68) + '" x2="' + X[m] + '" y2="' + (Y + 106) + '" stroke="var(--map)" stroke-width="1.5" stroke-dasharray="3 3"/>';
    }
    g.map += '<rect x="130" y="256" width="600" height="28" rx="6" fill="none" stroke="var(--map)"/>';
    g.map += '<text x="430" y="275" font-size="13" text-anchor="middle" fill="var(--ink)">地図点群 M (固定)</text>';
    g.marg += '<path d="M' + (X[1] + 14) + "," + (Y + 18) + " Q" + (X[1] + X[2]) / 2 + "," + (Y + 52) + " " + (X[2] - 14) + "," + (Y + 18) + '" fill="none" stroke="var(--muted)" stroke-width="2"/>';
    g.marg += '<rect x="' + ((X[1] + X[2]) / 2 - 7) + '" y="' + (Y + 28) + '" width="14" height="14" fill="var(--muted)"/>';
    g.marg += '<path d="M' + (X[0] + 22) + "," + (Y + 8) + " Q" + (X[0] + 70) + "," + (Y + 40) + " " + ((X[1] + X[2]) / 2 - 10) + "," + (Y + 35) + '" fill="none" stroke="var(--muted)" stroke-dasharray="4 4"/>';
    g.marg += '<circle cx="' + X[0] + '" cy="' + Y + '" r="22" fill="var(--surface)" stroke="var(--muted)" stroke-dasharray="4 3"/>';
    g.marg += '<text x="' + X[0] + '" y="' + (Y + 5) + '" font-size="14" text-anchor="middle" fill="var(--muted)" font-style="italic">x₀</text>';
    Object.keys(g).forEach(function (key) { s += '<g data-term="' + key + '">' + g[key] + "</g>"; });
    var names = ["x₁", "x₂", "x₃", "x₄", "x₅ (最新)"];
    for (var q = 1; q <= 5; q++) {
      s += '<circle cx="' + X[q] + '" cy="' + Y + '" r="22" fill="var(--surface)" stroke="var(--ink)" stroke-width="' + (q === 5 ? 2.6 : 1.6) + '"/>';
      s += '<text x="' + X[q] + '" y="' + (Y + 5) + '" font-size="14" text-anchor="middle" fill="var(--ink)" font-style="italic">' + names[q - 1].split(" ")[0] + "</text>";
    }
    s += "</svg>";
    graph.innerHTML = s;

    function select(key) {
      bar.querySelectorAll("button").forEach(function (b) { b.setAttribute("aria-pressed", b.dataset.key === key ? "true" : "false"); });
      graph.querySelectorAll("[data-term]").forEach(function (gg) {
        gg.classList.toggle("dim", key !== "all" && gg.getAttribute("data-term") !== key);
      });
      var t = terms.filter(function (x) { return x[0] === key; })[0];
      note.textContent = t[3];
    }
    terms.forEach(function (t) {
      var b = el("button", { type: "button", "data-key": t[0], style: "--c:" + t[2] }, t[1]);
      b.addEventListener("click", function () { select(t[0]); });
      bar.appendChild(b);
    });
    select("all");
  }

  // ------------------------------------------------------------------ 4. 2D 再現実験の再生
  var METHODS = [
    ["map_only", "A. 地図マッチングのみ", "--imu"],
    ["lio_only", "B. LiDAR-IMU のみ", "--map"],
    ["proposed", "C. 提案手法", "--lidar"],
  ];
  function demoReplay(root) {
    var D = window.SW_DATA;
    if (!D) { root.innerHTML = "<p>データ (data/sliding_window_result.js) が読み込めませんでした。</p>"; return; }
    var body = shell(root, "2D 再現実験の再生", "右側 (色付き) は地図点群を消した領域。提案手法は青、ウィンドウ内の状態を大きな点で表示");
    var canvas = el("canvas", { "aria-label": "推定軌跡の再生" });
    var chart = el("canvas", { "aria-label": "位置誤差の時間変化" });
    body.appendChild(canvas);
    var controls = el("div", { class: "controls" });
    var play = el("button", { type: "button", class: "btn primary" }, "再生");
    var slider = el("input", { type: "range", id: "replay-frame", min: "0", max: String(D.t.length - 1), step: "1", value: "0", "aria-label": "フレーム" });
    controls.appendChild(play);
    controls.appendChild(slider);
    var speed = 2;
    controls.appendChild(seg([[1, "×1"], [2, "×2"], [4, "×4"]], speed, function (v) { speed = v; }));
    body.appendChild(controls);
    var show = { map_only: true, lio_only: true, proposed: true };
    var toggles = el("div", { class: "controls" });
    METHODS.forEach(function (m) {
      var lab = el("label", null, "");
      var cb = el("input", { type: "checkbox", id: "replay-" + m[0], checked: "checked" });
      cb.addEventListener("change", function () { show[m[0]] = cb.checked; draw(); });
      lab.appendChild(cb);
      lab.appendChild(el("span", { style: "display:inline-block;width:16px;height:3px;border-radius:2px;background:var(" + m[2] + ")" }));
      lab.appendChild(document.createTextNode(m[1]));
      toggles.appendChild(lab);
    });
    body.appendChild(toggles);
    var readout = el("div", { class: "readout" });
    body.appendChild(readout);
    body.appendChild(chart);
    body.appendChild(el("div", { class: "legend" },
      '<span><i class="dash" style="--c:var(--ink)"></i>真値</span>' +
      '<span><i class="dot" style="--c:var(--muted)"></i>地図点群</span>' +
      '<span><i class="dot" style="--c:var(--lidar)"></i>最新スキャン (提案手法の推定姿勢で表示)</span>'));

    var frame = 0, timer = null, acc = 0, last = 0;
    var err = {};
    METHODS.forEach(function (m) {
      err[m[0]] = D.methods[m[0]].map(function (p, i) { return Math.hypot(p[0] - D.truth[i][0], p[1] - D.truth[i][1]); });
    });

    function draw() {
      var f = fitCanvas(canvas, 0.64), ctx = f.ctx;
      var V = view([-21, 21, -13, 13], f.w, f.h, 8);
      ctx.clearRect(0, 0, f.w, f.h);
      ctx.fillStyle = css("--shade");
      ctx.fillRect(V.x(D.map_x_max), V.y(13), V.x(21) - V.x(D.map_x_max), V.y(-13) - V.y(13));
      ctx.strokeStyle = css("--line"); ctx.lineWidth = 1;
      ctx.beginPath();
      D.segments.forEach(function (s) { ctx.moveTo(V.x(s[0][0]), V.y(s[0][1])); ctx.lineTo(V.x(s[1][0]), V.y(s[1][1])); });
      ctx.stroke();
      ctx.fillStyle = css("--muted");
      var pr = Math.max(0.9, f.w / 700);
      D.map_points.forEach(function (p) { ctx.fillRect(V.x(p[0]) - pr / 2, V.y(p[1]) - pr / 2, pr, pr); });
      // 真値
      ctx.setLineDash([5, 4]); ctx.strokeStyle = css("--ink"); ctx.globalAlpha = 0.55; ctx.lineWidth = 1.4;
      ctx.beginPath();
      D.truth.forEach(function (p, i) { if (i <= frame) (i ? ctx.lineTo : ctx.moveTo).call(ctx, V.x(p[0]), V.y(p[1])); });
      ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha = 1;
      // 各手法
      METHODS.forEach(function (m) {
        if (!show[m[0]]) return;
        var P = D.methods[m[0]];
        ctx.strokeStyle = css(m[2]); ctx.lineWidth = 2;
        ctx.beginPath();
        for (var i = 0; i <= frame; i++) (i ? ctx.lineTo : ctx.moveTo).call(ctx, V.x(P[i][0]), V.y(P[i][1]));
        ctx.stroke();
      });
      // ウィンドウ内の状態 (提案手法)
      var ws = D.window_start[frame] || 0;
      if (show.proposed) {
        ctx.fillStyle = css("--lidar");
        for (var k = ws; k <= frame; k++) {
          var p = D.methods.proposed[k];
          ctx.beginPath(); ctx.arc(V.x(p[0]), V.y(p[1]), Math.max(2.5, f.w / 230), 0, 7); ctx.fill();
        }
        // 最新スキャン
        var pose = D.methods.proposed[frame], c = Math.cos(pose[2]), s = Math.sin(pose[2]);
        ctx.globalAlpha = 0.85;
        D.scans[frame].forEach(function (q) {
          var x = c * q[0] - s * q[1] + pose[0], y = s * q[0] + c * q[1] + pose[1];
          ctx.fillRect(V.x(x) - 1.5, V.y(y) - 1.5, 3, 3);
        });
        ctx.globalAlpha = 1;
      }
      // ロボットの向き
      METHODS.forEach(function (m) {
        if (!show[m[0]]) return;
        var p = D.methods[m[0]][frame];
        if (Math.abs(p[0]) > 40 || Math.abs(p[1]) > 40) return;
        var x = V.x(p[0]), y = V.y(p[1]), a = -p[2], L = Math.max(8, f.w / 70);
        ctx.fillStyle = css(m[2]);
        ctx.beginPath();
        ctx.moveTo(x + L * Math.cos(a), y + L * Math.sin(a));
        ctx.lineTo(x + 0.6 * L * Math.cos(a + 2.4), y + 0.6 * L * Math.sin(a + 2.4));
        ctx.lineTo(x + 0.6 * L * Math.cos(a - 2.4), y + 0.6 * L * Math.sin(a - 2.4));
        ctx.closePath(); ctx.fill();
      });
      drawChart();
      var inMap = D.truth[frame][0] < D.map_x_max;
      var html = "<span>時刻 <b>" + D.t[frame].toFixed(1) + " s</b></span><span>" + (inMap ? "地図 <b>あり</b>" : "地図 <b>なし</b>") +
        "</span><span>ウィンドウ内の状態 <b>" + (frame - ws + 1) + "</b> 個</span>";
      METHODS.forEach(function (m) {
        var e = err[m[0]][frame];
        html += "<span>" + m[1].split(" ")[0] + " 誤差 <b>" + (e < 1 ? (e * 100).toFixed(1) + " cm" : e.toFixed(1) + " m") + "</b></span>";
      });
      readout.innerHTML = html;
    }
    function drawChart() {
      var f = fitCanvas(chart, 0.3), ctx = f.ctx;
      var padL = 40, padB = 18, padT = 8, W = f.w - padL - 8, H = f.h - padB - padT;
      ctx.clearRect(0, 0, f.w, f.h);
      var T = D.t[D.t.length - 1];
      var lo = Math.log10(0.003), hi = Math.log10(50);
      function X(t) { return padL + (t / T) * W; }
      function Y(e) { var v = Math.log10(Math.max(e, 0.003)); return padT + (1 - (v - lo) / (hi - lo)) * H; }
      // 地図なし区間
      ctx.fillStyle = css("--shade");
      var startNo = -1;
      for (var i = 0; i <= D.t.length; i++) {
        var no = i < D.t.length && D.truth[i][0] >= D.map_x_max;
        if (no && startNo < 0) startNo = i;
        if (!no && startNo >= 0) { ctx.fillRect(X(D.t[startNo]), padT, X(D.t[i - 1]) - X(D.t[startNo]), H); startNo = -1; }
      }
      ctx.strokeStyle = css("--line"); ctx.fillStyle = css("--muted"); ctx.font = "11px " + css("--font-mono"); ctx.lineWidth = 1;
      [0.01, 0.1, 1, 10].forEach(function (v) {
        ctx.beginPath(); ctx.moveTo(padL, Y(v)); ctx.lineTo(padL + W, Y(v)); ctx.stroke();
        ctx.fillText(v >= 1 ? v + " m" : v * 100 + " cm", 2, Y(v) + 4);
      });
      METHODS.forEach(function (m) {
        if (!show[m[0]]) return;
        ctx.strokeStyle = css(m[2]); ctx.lineWidth = 1.6;
        ctx.beginPath();
        err[m[0]].forEach(function (e, k) { (k ? ctx.lineTo : ctx.moveTo).call(ctx, X(D.t[k]), Y(e)); });
        ctx.stroke();
      });
      ctx.strokeStyle = css("--ink"); ctx.lineWidth = 1.2;
      ctx.beginPath(); ctx.moveTo(X(D.t[frame]), padT); ctx.lineTo(X(D.t[frame]), padT + H); ctx.stroke();
      ctx.fillStyle = css("--muted");
      ctx.fillText("位置誤差 (対数) / 時間 →", padL + 4, f.h - 4);
    }
    function tick(ts) {
      if (!timer) return;
      if (!last) last = ts;
      acc += (ts - last) / 1000 * 5 * speed; // データは 5 Hz
      last = ts;
      if (acc >= 1) {
        frame = Math.min(D.t.length - 1, frame + Math.floor(acc));
        acc -= Math.floor(acc);
        slider.value = String(frame);
        draw();
        if (frame >= D.t.length - 1) { stop(); return; }
      }
      requestAnimationFrame(tick);
    }
    function stop() { timer = null; play.textContent = "再生"; }
    play.addEventListener("click", function () {
      if (timer) { stop(); return; }
      if (frame >= D.t.length - 1) frame = 0;
      timer = 1; last = 0; acc = 0; play.textContent = "一時停止";
      requestAnimationFrame(tick);
    });
    slider.addEventListener("input", function () { frame = parseInt(slider.value, 10); draw(); });
    onResize(canvas, draw);
    onTheme(draw);
    frame = Math.min(D.t.length - 1, 95);
    slider.value = String(frame);
    draw();
  }

  // ------------------------------------------------------------------ 5. トップページのスキャン描画
  function heroScan(canvas) {
    var D = window.SW_DATA;
    if (!D) return;
    var frame = 60, timer = null, visible = true;
    var reduce = window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;
    function draw() {
      var f = fitCanvas(canvas, 0.62), ctx = f.ctx;
      var V = view([-21, 21, -13, 13], f.w, f.h, 6);
      ctx.clearRect(0, 0, f.w, f.h);
      ctx.fillStyle = css("--shade");
      ctx.fillRect(V.x(D.map_x_max), V.y(13), V.x(21) - V.x(D.map_x_max), V.y(-13) - V.y(13));
      ctx.fillStyle = css("--muted");
      D.map_points.forEach(function (p) { ctx.fillRect(V.x(p[0]) - 0.6, V.y(p[1]) - 0.6, 1.2, 1.2); });
      ctx.strokeStyle = css("--line"); ctx.lineWidth = 1;
      ctx.beginPath();
      D.segments.forEach(function (s) { if (s[0][0] >= D.map_x_max || s[1][0] >= D.map_x_max) { ctx.moveTo(V.x(s[0][0]), V.y(s[0][1])); ctx.lineTo(V.x(s[1][0]), V.y(s[1][1])); } });
      ctx.stroke();
      var P = D.methods.proposed;
      ctx.strokeStyle = css("--lidar"); ctx.lineWidth = 2;
      ctx.beginPath();
      for (var i = Math.max(0, frame - 60); i <= frame; i++) (i > Math.max(0, frame - 60) ? ctx.lineTo : ctx.moveTo).call(ctx, V.x(P[i][0]), V.y(P[i][1]));
      ctx.stroke();
      var pose = D.truth[frame], c = Math.cos(pose[2]), s = Math.sin(pose[2]);
      var ox = V.x(pose[0]), oy = V.y(pose[1]);
      ctx.strokeStyle = css("--lidar"); ctx.globalAlpha = 0.16; ctx.lineWidth = 1;
      ctx.beginPath();
      D.scans[frame].forEach(function (q) {
        var x = c * q[0] - s * q[1] + pose[0], y = s * q[0] + c * q[1] + pose[1];
        ctx.moveTo(ox, oy); ctx.lineTo(V.x(x), V.y(y));
      });
      ctx.stroke(); ctx.globalAlpha = 1;
      ctx.fillStyle = css("--lidar");
      D.scans[frame].forEach(function (q) {
        var x = c * q[0] - s * q[1] + pose[0], y = s * q[0] + c * q[1] + pose[1];
        ctx.beginPath(); ctx.arc(V.x(x), V.y(y), 1.8, 0, 7); ctx.fill();
      });
      ctx.fillStyle = css("--ink");
      ctx.beginPath(); ctx.arc(ox, oy, 4, 0, 7); ctx.fill();
    }
    function start() {
      if (reduce || timer) return;
      timer = setInterval(function () { if (!visible) return; frame = (frame + 1) % D.t.length; draw(); }, 140);
    }
    if (window.IntersectionObserver) {
      new IntersectionObserver(function (en) { visible = en[0].isIntersecting; }).observe(canvas);
    }
    onResize(canvas, draw);
    onTheme(draw);
    draw();
    start();
  }

  // ------------------------------------------------------------------ 6. 共分散行列と楕円
  function demoCov(root) {
    var body = shell(root, "共分散行列と楕円", "スライダーでばらつきを変えてみよう。図をタップすると誤差ベクトル e を置ける。壁に沿う向きと垂直な向きに同じ長さだけ置いて、マハラノビス距離を比べてみよう");
    var canvas = el("canvas", { "aria-label": "2 次元の正規分布の楕円" });
    body.appendChild(canvas);
    var controls = el("div", { class: "controls" });
    function slider(label, id, min, max, step, val, fmt) {
      var lab = el("label", { class: "slider-row" }, "<span>" + label + "</span>");
      var s = el("input", { type: "range", id: id, min: min, max: max, step: step, value: val });
      var out = el("b", { style: "font-family:var(--font-mono);font-weight:500;font-size:13px" });
      lab.appendChild(s); lab.appendChild(out);
      controls.appendChild(lab);
      function upd() { out.textContent = fmt(parseFloat(s.value)); }
      upd();
      s.addEventListener("input", function () { upd(); draw(); });
      return s;
    }
    var sA = slider("沿う方向の σ", "cov-along", "0.05", "0.6", "0.01", "0.40", function (v) { return v.toFixed(2) + " m"; });
    var sC = slider("垂直方向の σ", "cov-across", "0.02", "0.6", "0.01", "0.05", function (v) { return v.toFixed(2) + " m"; });
    var sT = slider("壁の傾き", "cov-angle", "-90", "90", "1", "30", function (v) { return v.toFixed(0) + "°"; });
    body.appendChild(controls);
    var mats = el("div", { class: "readout", style: "align-items:flex-start" });
    body.appendChild(mats);
    var readout = el("div", { class: "readout" });
    body.appendChild(readout);
    body.appendChild(el("div", { class: "legend" },
      '<span><i class="dot" style="--c:var(--muted)"></i>この分布から取ったサンプル</span>' +
      '<span><i style="--c:var(--lidar)"></i>マハラノビス距離 1, 2, 3 の楕円</span>' +
      '<span><i style="--c:var(--imu)"></i>誤差ベクトル e (タップで移動)</span>'));

    var g = gaussFactory(mulberry32(5));
    var base = [];
    for (var i = 0; i < 400; i++) base.push([g(), g()]);
    var e = [0.26, 0.15]; // 壁 (30°) に沿って 0.3 m
    var B = [-1.2, 1.2, -0.9, 0.9];

    function params() {
      var a = parseFloat(sA.value), c = parseFloat(sC.value), th = parseFloat(sT.value) * Math.PI / 180;
      var co = Math.cos(th), si = Math.sin(th);
      // Σ = R diag(a², c²) R^T
      var sxx = co * co * a * a + si * si * c * c;
      var syy = si * si * a * a + co * co * c * c;
      var sxy = co * si * (a * a - c * c);
      var det = sxx * syy - sxy * sxy;
      return { a: a, c: c, co: co, si: si, S: [sxx, sxy, syy], I: [syy / det, -sxy / det, sxx / det] };
    }
    function fmt(v) { var t = v.toFixed(4); return (v >= 0 ? "&nbsp;" : "") + t; }
    function matHTML(name, m) {
      return '<span style="display:inline-grid;grid-template-columns:auto auto;gap:0 10px;align-items:center">' +
        '<span>' + name + ' =</span><span style="font-family:var(--font-mono);color:var(--ink);line-height:1.5;border-left:1.5px solid var(--muted);border-right:1.5px solid var(--muted);padding:0 6px">' +
        fmt(m[0]) + "&nbsp;&nbsp;" + fmt(m[1]) + "<br>" + fmt(m[1]) + "&nbsp;&nbsp;" + fmt(m[2]) + "</span></span>";
    }
    function draw() {
      var P = params();
      var f = fitCanvas(canvas, 0.75), ctx = f.ctx;
      var V = view(B, f.w, f.h, 6);
      ctx.clearRect(0, 0, f.w, f.h);
      // 軸
      ctx.strokeStyle = css("--line"); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(V.x(B[0]), V.y(0)); ctx.lineTo(V.x(B[1]), V.y(0));
      ctx.moveTo(V.x(0), V.y(B[2])); ctx.lineTo(V.x(0), V.y(B[3])); ctx.stroke();
      ctx.fillStyle = css("--muted"); ctx.font = "12px " + css("--font-mono");
      ctx.fillText("x", V.x(B[1]) - 12, V.y(0) - 6); ctx.fillText("y", V.x(0) + 6, V.y(B[3]) + 14);
      // 壁
      ctx.strokeStyle = css("--ink"); ctx.globalAlpha = 0.15; ctx.lineWidth = 6;
      ctx.beginPath(); ctx.moveTo(V.x(-2 * P.co), V.y(-2 * P.si)); ctx.lineTo(V.x(2 * P.co), V.y(2 * P.si)); ctx.stroke();
      ctx.globalAlpha = 1;
      // サンプル: (u, v) = (a·g1, c·g2) を回転
      ctx.fillStyle = css("--muted");
      base.forEach(function (q) {
        var u = P.a * q[0], v = P.c * q[1];
        var x = P.co * u - P.si * v, y = P.si * u + P.co * v;
        ctx.fillRect(V.x(x) - 1.2, V.y(y) - 1.2, 2.4, 2.4);
      });
      // 楕円
      ctx.strokeStyle = css("--lidar");
      [1, 2, 3].forEach(function (k) {
        ctx.lineWidth = k === 1 ? 2 : 1.1;
        ctx.beginPath();
        for (var j = 0; j <= 72; j++) {
          var t = j / 72 * 2 * Math.PI, u = k * P.a * Math.cos(t), v = k * P.c * Math.sin(t);
          var x = P.co * u - P.si * v, y = P.si * u + P.co * v;
          (j ? ctx.lineTo : ctx.moveTo).call(ctx, V.x(x), V.y(y));
        }
        ctx.stroke();
      });
      // 誤差ベクトル
      ctx.strokeStyle = css("--imu"); ctx.fillStyle = css("--imu"); ctx.lineWidth = 2.2;
      ctx.beginPath(); ctx.moveTo(V.x(0), V.y(0)); ctx.lineTo(V.x(e[0]), V.y(e[1])); ctx.stroke();
      ctx.beginPath(); ctx.arc(V.x(e[0]), V.y(e[1]), 6, 0, 7); ctx.fill();
      ctx.fillText("e", V.x(e[0]) + 9, V.y(e[1]) - 8);

      var I = P.I;
      var d2 = I[0] * e[0] * e[0] + 2 * I[1] * e[0] * e[1] + I[2] * e[1] * e[1];
      var rho = P.S[1] / Math.sqrt(P.S[0] * P.S[2]);
      mats.innerHTML = matHTML("Σ", P.S) + matHTML("Σ⁻¹", I) + "<span>相関係数 ρ = <b>" + rho.toFixed(2) + "</b></span>";
      var dens = Math.exp(-0.5 * d2);
      readout.innerHTML = "<span>e = (<b>" + e[0].toFixed(2) + "</b>, <b>" + e[1].toFixed(2) + "</b>)</span>" +
        "<span>長さ |e| = <b>" + Math.hypot(e[0], e[1]).toFixed(3) + " m</b></span>" +
        "<span>マハラノビス距離 √(eᵀΣ⁻¹e) = <b>" + Math.sqrt(d2).toFixed(2) + "</b> (σ 何個分)</span>" +
        "<span>確率密度は頂上の <b>" + (dens > 1e-3 ? dens.toFixed(3) : dens.toExponential(1)) + "</b> 倍</span>";
    }
    function pick(ev) {
      var r = canvas.getBoundingClientRect();
      var V = view(B, r.width, r.height, 6);
      var px = ev.clientX - r.left, py = ev.clientY - r.top;
      e = [(px - V.x(0)) / V.s, -(py - V.y(0)) / V.s];
      draw();
    }
    canvas.addEventListener("pointerdown", function (ev) { pick(ev); canvas.setPointerCapture && canvas.setPointerCapture(ev.pointerId); });
    canvas.addEventListener("pointermove", function (ev) { if (ev.buttons) pick(ev); });
    canvas.style.cursor = "crosshair";
    onResize(canvas, draw);
    onTheme(draw);
    draw();
  }

  // ------------------------------------------------------------------ 7. NDT / VGICP / GICP の地図の作り方
  function demoVoxel(root) {
    var body = root.getAttribute("data-mode") === "f3"
      ? shell(root, "F3-SLAM: 点ごとに選ばれるボクセル", "大きさが半分ずつの 3 層のボクセルから、点ごとに「平面で、なるべく大きい」ものが選ばれる。ボクセルの大きさを変えて、壁・角・柱で選ばれ方がどう変わるか見よう")
      : shell(root, "NDT・VGICP・GICP の地図の作り方", "同じ点群から作る分布を切り替えて比べよう。ボクセルの大きさや位置を変えると、NDT の穴や角のぼやけ方が変わる");
    var canvas = el("canvas", { "aria-label": "ボクセルと正規分布の比較" });
    body.appendChild(canvas);
    var mode = root.getAttribute("data-mode") || "ndt";
    var controls = el("div", { class: "controls" });
    controls.appendChild(seg([["ndt", "NDT"], ["vgicp", "VGICP"], ["gicp", "GICP (点ごと)"], ["f3", "F3-SLAM"]], mode, function (m) { mode = m; draw(); }));
    body.appendChild(controls);
    var sliders = el("div", { class: "controls" });
    // 同じページに 2 つ置いても id が重ならないようにする
    demoVoxel.count = (demoVoxel.count || 0) + 1;
    var idSuffix = demoVoxel.count > 1 ? "-" + demoVoxel.count : "";
    function slider(label, id, min, max, step, val, fmt) {
      var lab = el("label", { class: "slider-row" }, "<span>" + label + "</span>");
      var sl = el("input", { type: "range", id: id + idSuffix, min: min, max: max, step: step, value: val });
      var out = el("b", { style: "font-family:var(--font-mono);font-weight:500;font-size:13px" });
      lab.appendChild(sl); lab.appendChild(out);
      sliders.appendChild(lab);
      function upd() { out.textContent = fmt(parseFloat(sl.value)); }
      upd();
      sl.addEventListener("input", function () { upd(); draw(); });
      return sl;
    }
    var sV = slider("ボクセルの大きさ", "vox-size", "0.4", "3.0", "0.1", "1.0", function (v) { return v.toFixed(1) + " m"; });
    var sO = slider("格子をずらす", "vox-shift", "0", "1", "0.05", "0", function (v) { return (v * 100).toFixed(0) + " %"; });
    var sM = slider("最少点数 (NDT・F3)", "vox-min", "1", "8", "1", "3", function (v) { return v.toFixed(0) + " 点"; });
    body.appendChild(sliders);
    var readout = el("div", { class: "readout" });
    body.appendChild(readout);
    var note = el("p", { class: "eq-note", style: "font-size:13px;color:var(--muted);min-height:0" });
    body.appendChild(note);

    // 部屋の角・壁・柱を、センサ (0, 0) から見た LiDAR のように点で表す (遠いほどまばら)
    var segs = [[-5, -2.4, 3.5, -2.4], [-5, -2.4, -5, 3.0], [-5, 3.0, -2.2, 4.2], [1.2, 0.6, 2.2, 0.6], [2.2, 0.6, 2.2, 1.6], [2.2, 1.6, 1.2, 1.6], [3.5, -2.4, 4.6, -1.2]];
    // 円柱 (平面でない形の例)。センサから見える側だけ点になる
    for (var ci = 0; ci < 10; ci++) {
      var a0 = -0.15 * Math.PI + ci * 0.09 * Math.PI, a1 = a0 + 0.09 * Math.PI, cx = -2.2, cy = 1.9, cr = 0.4;
      segs.push([cx + cr * Math.cos(a0), cy + cr * Math.sin(a0), cx + cr * Math.cos(a1), cy + cr * Math.sin(a1)]);
    }
    var g = gaussFactory(mulberry32(17));
    var pts = [];
    segs.forEach(function (sg) {
      var L = Math.hypot(sg[2] - sg[0], sg[3] - sg[1]), t = 0;
      while (t <= L) {
        var x = sg[0] + (sg[2] - sg[0]) * t / L, y = sg[1] + (sg[3] - sg[1]) * t / L;
        pts.push([x + 0.012 * g(), y + 0.012 * g()]);
        t += 0.05 * Math.hypot(x, y); // 角度間隔一定のビーム → 距離に比例して点の間隔が広がる
      }
    });
    function eig2(a, b, c) {
      var tr = a + c, d = Math.sqrt(Math.max((a - c) * (a - c) / 4 + b * b, 0));
      var l1 = tr / 2 + d, l2 = tr / 2 - d;
      var ang = Math.abs(b) < 1e-12 ? (a >= c ? 0 : Math.PI / 2) : Math.atan2(l1 - a, b);
      return [l1, Math.max(l2, 0), ang];
    }
    // 点ごとの共分散 (近傍 8 点、GICP 流に固有値を (1, 0.001) にそろえる)
    var pcov = pts.map(function (p) {
      var nb = pts.map(function (q) { return [(q[0] - p[0]) * (q[0] - p[0]) + (q[1] - p[1]) * (q[1] - p[1]), q]; })
        .sort(function (u, v) { return u[0] - v[0]; }).slice(0, 8).map(function (u) { return u[1]; });
      var mx = 0, my = 0;
      nb.forEach(function (q) { mx += q[0]; my += q[1]; }); mx /= nb.length; my /= nb.length;
      var a = 0, b = 0, c = 0;
      nb.forEach(function (q) { a += (q[0] - mx) * (q[0] - mx); b += (q[0] - mx) * (q[1] - my); c += (q[1] - my) * (q[1] - my); });
      var e = eig2(a, b, c), co = Math.cos(e[2]), si = Math.sin(e[2]), l1 = 1, l2 = 0.001;
      return [co * co * l1 + si * si * l2, co * si * (l1 - l2), si * si * l1 + co * co * l2];
    });
    var B = [-5.6, 5.2, -3.0, 4.8];
    function ell(ctx, V, mx, my, a, b, c, len, fill, stroke) {
      var e = eig2(a, b, c), r1 = len, r2 = len * Math.sqrt(e[1] / Math.max(e[0], 1e-12));
      r2 = Math.max(r2, len * 0.06);
      ctx.beginPath();
      for (var k = 0; k <= 40; k++) {
        var t = k / 40 * 2 * Math.PI, u = r1 * Math.cos(t), v = r2 * Math.sin(t);
        var x = mx + Math.cos(e[2]) * u - Math.sin(e[2]) * v, y = my + Math.sin(e[2]) * u + Math.cos(e[2]) * v;
        (k ? ctx.lineTo : ctx.moveTo).call(ctx, V.x(x), V.y(y));
      }
      ctx.closePath();
      ctx.globalAlpha = 0.25; ctx.fillStyle = fill; ctx.fill();
      ctx.globalAlpha = 1; ctx.strokeStyle = stroke; ctx.lineWidth = 1.3; ctx.stroke();
    }
    // F3-SLAM: 3 層のボクセル (大きさ s, s/2, s/4) を作り、点ごとに「平面で、なるべく大きい」ボクセルを選ぶ
    function drawF3(ctx, V, s0, sh, minPts) {
      var sizes = [s0, s0 / 2, s0 / 4], THETA = 0.01, cols = [css("--lidar"), css("--map"), css("--imu")];
      var layers = sizes.map(function (s) {
        var cells = {};
        pts.forEach(function (p) {
          var k = Math.floor((p[0] - sh) / s) + "," + Math.floor((p[1] - sh) / s);
          var c = cells[k] || (cells[k] = { n: 0, sx: 0, sy: 0, sxx: 0, sxy: 0, syy: 0 });
          c.n++; c.sx += p[0]; c.sy += p[1]; c.sxx += p[0] * p[0]; c.sxy += p[0] * p[1]; c.syy += p[1] * p[1];
        });
        Object.keys(cells).forEach(function (k) {
          var c = cells[k], mx = c.sx / c.n, my = c.sy / c.n;
          c.mx = mx; c.my = my;
          c.a = c.sxx / c.n - mx * mx; c.b = c.sxy / c.n - mx * my; c.c = c.syy / c.n - my * my;
          var e = eig2(c.a, c.b, c.c);
          c.plane = c.n >= minPts && e[0] > 0 && e[1] / e[0] < THETA;
          c.e = e;
        });
        return { s: s, cells: cells };
      });
      var chosen = {}, count = [0, 0, 0, 0, 0];
      pts.forEach(function (p) {
        var pick = null;
        for (var li = 0; li < layers.length; li++) {
          var L = layers[li], k = Math.floor((p[0] - sh) / L.s) + "," + Math.floor((p[1] - sh) / L.s), c = L.cells[k];
          if (!c || c.n < minPts || Math.hypot(p[0] - c.mx, p[1] - c.my) > L.s) continue;
          pick = [li, k, c];
          if (c.plane) break;
        }
        if (pick) chosen[pick[0] + "|" + pick[1]] = pick;
      });
      Object.keys(chosen).forEach(function (key) {
        var li = chosen[key][0], k = chosen[key][1], c = chosen[key][2], s = layers[li].s, ij = k.split(",").map(Number);
        count[li]++; count[c.plane ? 3 : 4]++;
        if (c.plane) {
          var ang = c.e[2], h = 0.45 * s;
          ctx.strokeStyle = css("--ink"); ctx.lineWidth = 2.4;
          ctx.beginPath(); ctx.moveTo(V.x(c.mx - h * Math.cos(ang)), V.y(c.my - h * Math.sin(ang))); ctx.lineTo(V.x(c.mx + h * Math.cos(ang)), V.y(c.my + h * Math.sin(ang))); ctx.stroke();
        } else {
          ell(ctx, V, c.mx, c.my, c.a, c.b, c.c, 0.45 * s, "#e87ba4", "#c2507c");
        }
        ctx.strokeStyle = cols[li]; ctx.lineWidth = 2;
        ctx.strokeRect(V.x(ij[0] * s + sh), V.y((ij[1] + 1) * s + sh), s * V.s, s * V.s);
      });
      return count;
    }
    function draw() {
      var vs = parseFloat(sV.value), sh = parseFloat(sO.value) * vs, minPts = parseInt(sM.value, 10);
      var f = fitCanvas(canvas, 0.72), ctx = f.ctx, V = view(B, f.w, f.h, 6);
      ctx.clearRect(0, 0, f.w, f.h);
      var cells = {};
      pts.forEach(function (p, i) {
        var k = Math.floor((p[0] - sh) / vs) + "," + Math.floor((p[1] - sh) / vs);
        (cells[k] = cells[k] || []).push(i);
      });
      var keys = Object.keys(cells), nOk = 0, nHole = 0;
      if (mode !== "gicp" && mode !== "f3") {
        ctx.strokeStyle = css("--line"); ctx.lineWidth = 1;
        ctx.beginPath();
        for (var x = Math.floor((B[0] - sh) / vs) * vs + sh; x <= B[1]; x += vs) { ctx.moveTo(V.x(x), V.y(B[2])); ctx.lineTo(V.x(x), V.y(B[3])); }
        for (var y = Math.floor((B[2] - sh) / vs) * vs + sh; y <= B[3]; y += vs) { ctx.moveTo(V.x(B[0]), V.y(y)); ctx.lineTo(V.x(B[1]), V.y(y)); }
        ctx.stroke();
      }
      if (mode !== "f3") keys.forEach(function (k) {
        var ids = cells[k], ij = k.split(",").map(Number);
        var mx = 0, my = 0;
        ids.forEach(function (i) { mx += pts[i][0]; my += pts[i][1]; }); mx /= ids.length; my /= ids.length;
        if (mode === "ndt") {
          if (ids.length < minPts) {
            nHole++;
            ctx.globalAlpha = 0.16; ctx.fillStyle = css("--imu");
            ctx.fillRect(V.x(ij[0] * vs + sh), V.y((ij[1] + 1) * vs + sh), vs * V.s, vs * V.s);
            ctx.globalAlpha = 1;
            return;
          }
          var a = 0, b = 0, c = 0;
          ids.forEach(function (i) { var dx = pts[i][0] - mx, dy = pts[i][1] - my; a += dx * dx; b += dx * dy; c += dy * dy; });
          a /= ids.length; b /= ids.length; c /= ids.length;
          var e = eig2(a, b, c), lmin = 0.01 * e[0];
          if (e[1] < lmin) { var co = Math.cos(e[2]), si = Math.sin(e[2]); a = co * co * e[0] + si * si * lmin; b = co * si * (e[0] - lmin); c = si * si * e[0] + co * co * lmin; }
          nOk++;
          ell(ctx, V, mx, my, a, b, c, 0.45 * vs, css("--lidar"), css("--lidar"));
        } else if (mode === "vgicp") {
          var A = 0, Bc = 0, C = 0;
          ids.forEach(function (i) { A += pcov[i][0]; Bc += pcov[i][1]; C += pcov[i][2]; });
          nOk++;
          ell(ctx, V, mx, my, A / ids.length, Bc / ids.length, C / ids.length, 0.45 * vs, css("--map"), css("--map"));
        }
      });
      if (mode === "gicp") {
        pts.forEach(function (p, i) { ell(ctx, V, p[0], p[1], pcov[i][0], pcov[i][1], pcov[i][2], 0.16, css("--opt"), css("--opt")); });
      }
      var f3count = null;
      if (mode === "f3") f3count = drawF3(ctx, V, vs, sh, minPts);
      ctx.fillStyle = css("--muted");
      pts.forEach(function (p) { ctx.beginPath(); ctx.arc(V.x(p[0]), V.y(p[1]), 2, 0, 7); ctx.fill(); });
      ctx.fillStyle = css("--ink");
      ctx.beginPath(); ctx.moveTo(V.x(0), V.y(0.22)); ctx.lineTo(V.x(-0.16), V.y(-0.12)); ctx.lineTo(V.x(0.16), V.y(-0.12)); ctx.fill();
      var html = "<span>点 <b>" + pts.length + "</b></span>";
      if (mode === "f3") {
        var lv = [vs, vs / 2, vs / 4];
        html += "<span>選ばれたボクセル: " + lv.map(function (s, i) { return s.toFixed(2) + " m <b>" + f3count[i] + "</b>"; }).join(" / ") + "</span><span>平面 <b>" + f3count[3] + "</b> 非平面 <b>" + f3count[4] + "</b></span>";
        note.textContent = "F3-SLAM は大きさが半分ずつの 3 層のボクセルを持ち、点ごとに大きいボクセルから調べて「平面 (2D では線) で、なるべく大きい」ものを選ぶ。太い直線 = 平面として点対面で合わせる。桃色の楕円 = 非平面として点対分布 (NDT 型) で合わせる。枠の色は選ばれた層 (青 > 緑 > 橙 の順に小さい)。ボクセルの大きさのスライダーは一番上の層の大きさ。";
      } else if (mode === "gicp") {
        html += "<span>分布 <b>" + pts.length + "</b> 個 (点ごと)</span>";
        note.textContent = "GICP は点ごとに、近くの 8 点から分布を作る。ボクセルで区切らないので、角でも壁ごとに向きの正しい分布になる。その代わり、対応を探すたびに最近傍点の探索 (kd-tree) が必要。";
      } else if (mode === "ndt") {
        html += "<span>点の入ったボクセル <b>" + keys.length + "</b></span><span>分布ができた <b>" + nOk + "</b></span><span>穴 (点が足りない) <b>" + nHole + "</b></span>";
        note.textContent = "NDT はボクセル内の点の位置から共分散を計算する。点が少ないボクセル (オレンジ) は分布を作れず、地図に穴が開く。センサから遠く点がまばらな所ほど穴が増える。角を含むボクセルは丸っこい分布になる。";
      } else {
        html += "<span>点の入ったボクセル <b>" + keys.length + "</b></span><span>分布ができた <b>" + nOk + "</b> (全部)</span>";
        note.textContent = "VGICP はボクセル内の「点ごとの分布 (GICP と同じもの)」を平均する。点が 1 つでも分布ができるので穴が開かない。角を含むボクセルが丸っこくなるのは NDT と同じ。";
      }
      readout.innerHTML = html;
    }
    onResize(canvas, draw);
    onTheme(draw);
    draw();
  }

  // ------------------------------------------------------------------ 起動
  var map = { icp: demoICP, imu: demoIMU, equation: demoEquation, replay: demoReplay, cov: demoCov, voxel: demoVoxel };
  document.querySelectorAll("[data-demo]").forEach(function (node) {
    var f = map[node.getAttribute("data-demo")];
    if (f) {
      try { f(node); } catch (e) { node.textContent = "デモの初期化に失敗しました: " + e.message; }
    }
  });
  var hero = document.getElementById("hero-scan");
  if (hero) heroScan(hero);
})();
