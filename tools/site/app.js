/* サイト共通の小さなスクリプト: 目次ドロワー、読了チェック、表と図の整形 */
(function () {
  "use strict";

  var KEY = "lidar-imu-study:done";

  function loadDone() {
    try {
      return JSON.parse(localStorage.getItem(KEY) || "{}") || {};
    } catch (e) {
      return {};
    }
  }
  function saveDone(d) {
    try {
      localStorage.setItem(KEY, JSON.stringify(d));
    } catch (e) {
      /* ストレージが使えない環境では保存しない */
    }
  }

  // ---- 目次ドロワー (スマホ) ----
  var btn = document.querySelector(".menu-btn");
  var scrim = null;
  function closeNav() {
    document.body.classList.remove("nav-open");
    if (btn) btn.setAttribute("aria-expanded", "false");
    if (scrim) { scrim.remove(); scrim = null; }
  }
  if (btn) {
    btn.addEventListener("click", function () {
      if (document.body.classList.contains("nav-open")) { closeNav(); return; }
      document.body.classList.add("nav-open");
      btn.setAttribute("aria-expanded", "true");
      scrim = document.createElement("div");
      scrim.className = "scrim";
      scrim.addEventListener("click", closeNav);
      document.body.appendChild(scrim);
    });
    document.addEventListener("keydown", function (e) { if (e.key === "Escape") closeNav(); });
  }

  // ---- 読了チェック ----
  var done = loadDone();
  function paintDone() {
    document.querySelectorAll("[data-done-slug]").forEach(function (el) {
      el.classList.toggle("on", !!done[el.getAttribute("data-done-slug")]);
    });
    document.querySelectorAll("[data-done-badge]").forEach(function (el) {
      el.hidden = !done[el.getAttribute("data-done-badge")];
    });
  }
  var toggle = document.getElementById("read-toggle");
  if (toggle) {
    var slug = toggle.getAttribute("data-slug");
    toggle.checked = !!done[slug];
    toggle.addEventListener("change", function () {
      done[slug] = toggle.checked;
      saveDone(done);
      paintDone();
    });
  }
  paintDone();

  // ---- 表をスクロール可能な枠で包む / 図を枠で包む ----
  document.querySelectorAll(".article table").forEach(function (t) {
    if (t.parentElement.classList.contains("table-wrap")) return;
    var w = document.createElement("div");
    w.className = "table-wrap";
    t.parentNode.insertBefore(w, t);
    w.appendChild(t);
  });
  document.querySelectorAll(".article p > img:only-child").forEach(function (img) {
    var p = img.parentElement;
    var fig = document.createElement("figure");
    fig.className = "fig";
    fig.appendChild(img.cloneNode(true));
    if (img.alt) {
      var cap = document.createElement("figcaption");
      cap.textContent = img.alt;
      fig.appendChild(cap);
    }
    p.replaceWith(fig);
  });
  // ---- 数式 (KaTeX)。katex のスクリプトは defer なので DOMContentLoaded 後に描画する ----
  function renderMath() {
    if (!window.renderMathInElement) return;
    window.renderMathInElement(document.querySelector("main") || document.body, {
      delimiters: [
        { left: "\\[", right: "\\]", display: true },
        { left: "\\(", right: "\\)", display: false },
      ],
      throwOnError: false,
      strict: "ignore",
    });
  }
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", renderMath);
  else renderMath();

  // 本文中のタスクリストのチェックボックスは操作できるようにする (保存はしない)
  document.querySelectorAll(".article input[type=checkbox][disabled]").forEach(function (c) {
    c.disabled = false;
  });
})();
