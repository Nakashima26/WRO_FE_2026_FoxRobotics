"use strict";
// Fox dashboard. Sin librerías: corre offline desde la Pi.
// Marco del tapete: x este, y norte, mm, origen al centro. Rumbo h en grados, horario desde el norte:
// dirección = (sin h, cos h). Montajes de sensores: (derecha, adelante, ángulo relativo al rumbo).

const $ = (s) => document.querySelector(s);
const $$ = (s) => [...document.querySelectorAll(s)];
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const fmt = (v, d = 0) => (v === null || v === undefined || Number.isNaN(v)) ? "—" : Number(v).toFixed(d);

let S = { tele: {}, sys: {} };     // estado que se dibuja (en vivo o el del replay)
let SLIVE = S;                     // último estado en vivo
const REP = { on: false };         // replay de una corrida (pestaña Corridas)
let FIELD = null, fieldKey = "";
const trail = [];
const hist = {};                   // sparklines
const HIST_N = 150;

function toast(msg, bad = false) {
  const d = document.createElement("div");
  d.textContent = msg; if (bad) d.className = "bad";
  $("#toast").appendChild(d); setTimeout(() => d.remove(), 5000);
}
async function api(path, body) {
  const opt = body === undefined ? {} : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) };
  const r = await fetch(path, opt);
  const ct = r.headers.get("content-type") || "";
  const out = ct.includes("json") ? await r.json() : await r.text();
  if (!r.ok || (out && out.error)) throw new Error((out && out.error) || r.statusText);
  return out;
}

// ── Pestañas ────────────────────────────────────────────────────────────────
$$("nav .tab").forEach((b) => b.onclick = () => {
  $$("nav .tab").forEach((x) => x.classList.toggle("on", x === b));
  $$(".tab-page").forEach((p) => p.classList.toggle("on", p.id === "tab-" + b.dataset.tab));
  if (b.dataset.tab === "sys") { loadLogs(); loadRuns(); loadCalib(); }
  if (b.dataset.tab === "color") loadCalib();
  if (b.dataset.tab === "runs") { loadRepList(); drawState(); if (REP.on) { REP.evShown = -2; repEvents(false); } }
});

// ── Servicio ────────────────────────────────────────────────────────────────
$$("[data-svc]").forEach((b) => b.onclick = async () => {
  const a = b.dataset.svc;
  if (a !== "start" && S.tele.armed && !confirm("El carro está ARMADO (corriendo). ¿Seguro?")) return;
  try { toast((await api("/api/service", { action: a })).msg || a); } catch (e) { toast(e.message, true); }
});

// ── Stream ──────────────────────────────────────────────────────────────────
function connect() {
  const es = new EventSource("/api/stream");
  es.onmessage = (ev) => {
    try {
      SLIVE = JSON.parse(ev.data);
      if (REP.on) return;            // en replay lo en vivo se guarda pero no se dibuja
      S = SLIVE; onState();
    } catch (e) { console.error(e); }
  };
  es.onerror = () => { setPill("#pill-link", "desconectado", "bad"); es.close(); setTimeout(connect, 2000); };
}
// ?still=1: sin stream en vivo (capturas con Chrome headless de un momento de una corrida).
const STILL = new URLSearchParams(location.search).has("still");
if (STILL) api("/api/state").then((d) => { SLIVE = d; if (!REP.on) { S = d; onState(); } }).catch(() => {});
else { connect(); $("#hud").src = "/api/hud.mjpg?fps=6"; }

function setPill(sel, txt, cls) { const e = $(sel); e.textContent = txt; e.className = "pill " + (cls || ""); }

function push(name, v) {
  if (v === null || v === undefined || Number.isNaN(Number(v))) return;
  (hist[name] = hist[name] || []).push(Number(v));
  if (hist[name].length > HIST_N) hist[name].shift();
}

// Lecturas ToF vigentes: armado = ACK, desarmado = PRE (antes del botón).
function tofNow() {
  const t = S.tele || {}, a = t.ack || {}, p = t.pre || {};
  if (t.pre) return { L: p.L, R: p.R, B: p.B, F: p.F, raw: p.rl, h: p.h, src: "PRE" };
  if (t.ack) return { L: a.tL, R: a.tR, B: a.tB, F: a.tF, src: "ACK" };
  return null;
}

// ── Firmware del ESP ────────────────────────────────────────────────────────
// La open (RondaAbierta) manda fw=open en PRE y ACK; obstáculos manda fw=obs en PRE.
// Sin fw pero con PRE/ACK = obstáculos con firmware anterior a este campo.
function fwNow() {
  const t = S.tele || {};
  const f = (t.pre && t.pre.fw) || (t.ack && t.ack.fw);
  if (f) return String(f);
  return t.pre || t.ack ? "obs" : null;
}
function listOf(v) {
  if (Array.isArray(v)) return v.map(Number);
  if (typeof v === "string") return v.split("/").map(Number);
  if (typeof v === "number") return [v];
  return null;
}

function onState() { feedState(); drawState(); }
// Acumula lo que depende de la historia (sparklines, rastro, pose de la open).
function feedState() {
  const t = S.tele || {}, a = t.ack || {};
  const tof = tofNow();
  if (tof) { push("tL", tof.L); push("tR", tof.R); push("tF", tof.F); push("tB", tof.B); }
  push("yaw", a.yaw); push("vel", a.vel); push("srv", a.srv); push("pw", a.pw); push("fps", t.fps);
  push("ue", a.ue);
  if (t.loc && t.loc.pose) {
    if (t.loc.sd) push("sdx", Math.hypot(t.loc.sd[0], t.loc.sd[1]));
    const p = t.loc.pose, last = trail[trail.length - 1];
    if (!last || Math.hypot(p[0] - last[0], p[1] - last[1]) > 15) { trail.push(p); if (trail.length > 3000) trail.shift(); }
  }
  if (fwNow() === "open") openUpdate();
  if (!REP.on) for (const l of t.esp_rx || []) espRx(l);
}
function drawState() {
  const t = S.tele || {}, sy = S.sys || {};
  const age = t._age;
  if (REP.on) setPill("#pill-link", "REPLAY " + REP.name, "warn");
  else setPill("#pill-link", age === null || age === undefined ? "runtime sin datos" : age < 1.5 ? "en línea" : `sin datos ${fmt(age, 0)} s`,
          age !== null && age < 1.5 ? "ok" : "bad");
  setPill("#pill-svc", "servicio " + (sy.service || "?"), sy.service === "active" ? "ok" : "bad");
  const fw = fwNow(), bld = (t.pre && t.pre.bld) || "";
  setPill("#pill-fw", fw === "open" ? `ESP: Open ${bld}` :
          fw === "obs" ? `ESP: Obstáculos ${bld} · sorpresa ${t.surprise ? "ON" : "OFF"}` : "ESP: sin datos",
          fw === "open" ? "ok" : fw === "obs" ? (t.surprise ? "warn" : "ok") : "bad");
  setPill("#pill-arm", t.armed ? "ARMADO" : "esperando botón", t.armed ? "warn" : "");
  const a = t.ack || {};
  setPill("#pill-est", "est " + (a.est || "—"));
  setPill("#pill-tc", `esquina ${fmt(a.tc ?? t.tc)}/${fmt(a.tpr ?? t.tpr)}`);
  setPill("#pill-fps", `${fmt(t.fps, 0)} fps`, t.fps >= 30 ? "ok" : t.fps ? "warn" : "");
  setPill("#pill-temp", `${fmt(sy.temp_c, 0)} °C`, sy.temp_c == null ? "" : sy.temp_c > 75 ? "bad" : sy.temp_c > 65 ? "warn" : "ok");
  const rad = sy.radios || {};
  const on = rad.wlan || rad.bluetooth;
  setPill("#pill-radio", sy.radios ? (on ? "radios ON" : "radios OFF") : "radios ?", sy.radios ? (on ? "warn" : "ok") : "");

  ensureField();
  if ($("#tab-live").classList.contains("on")) { drawMap(); drawRadar(); drawSensors(); drawChecklist(); drawLoc(); }
  if ($("#tab-tof").classList.contains("on")) drawTofLive();
  if ($("#tab-sys").classList.contains("on")) drawSys();
  if ($("#tab-esp").classList.contains("on")) drawEsp();
  if ($("#tab-runs").classList.contains("on")) { drawMap($("#rep-map")); repDrawChart(); }
}

// ── Geometría ───────────────────────────────────────────────────────────────
async function ensureField() {
  const m = (S.tele && S.tele.map) || {}, l = (S.tele && S.tele.loc) || {};
  const w0 = l.park_w ?? m.park_w ?? "";
  const key = `${m.parking || ""}|${Math.round(w0 || 0)}`;
  if (key === fieldKey) return;
  fieldKey = key;
  try { FIELD = await api(`/api/field?parking=${m.parking || ""}&w0=${w0 || ""}`); } catch (e) { fieldKey = ""; }
}
function mountWorld(pose, mount) {
  const [x, y, h] = pose, [right, fwd, ang] = mount, r = h * Math.PI / 180;
  const ox = x + fwd * Math.sin(r) + right * Math.cos(r), oy = y + fwd * Math.cos(r) - right * Math.sin(r);
  const a = (h + ang) * Math.PI / 180;
  return { ox, oy, dx: Math.sin(a), dy: Math.cos(a) };
}
function segments() {
  if (!FIELD) return [];
  const o = FIELD.outer, i = FIELD.inner, segs = [];
  const sq = (h) => [[-h, -h], [h, -h], [h, h], [-h, h]];
  const poly = (pts) => pts.forEach((p, k) => segs.push([p, pts[(k + 1) % pts.length]]));
  poly(sq(o)); poly(sq(i));
  const conf = (S.tele.map && S.tele.map.confirmed) || [];
  const s = FIELD.sign / 2;
  for (const [sec, seat] of conf) {
    const c = FIELD.seats[`${sec}/${seat}`]; if (!c) continue;
    poly([[c[0] - s, c[1] - s], [c[0] + s, c[1] - s], [c[0] + s, c[1] + s], [c[0] - s, c[1] + s]]);
  }
  for (const b of FIELD.barriers || []) poly(b);
  return segs;
}
function raycast(ox, oy, dx, dy, segs) {
  let best = Infinity;
  for (const [[ax, ay], [bx, by]] of segs) {
    const ex = bx - ax, ey = by - ay, den = dx * ey - dy * ex;
    if (Math.abs(den) < 1e-9) continue;
    const t = ((ax - ox) * ey - (ay - oy) * ex) / den, u = ((ax - ox) * dy - (ay - oy) * dx) / den;
    if (t > 0 && u >= 0 && u <= 1 && t < best) best = t;
  }
  return best;
}

// ── Mapa ────────────────────────────────────────────────────────────────────
function drawMap(cv = $("#map")) {
  const g = cv.getContext("2d"), W = cv.width;
  g.clearRect(0, 0, W, W);
  if (fwNow() === "open") return drawOpen(g, W);
  if (!FIELD) return;
  const span = FIELD.outer * 2 + 200, k = W / span;
  const X = (x) => (x + span / 2) * k, Y = (y) => (span / 2 - y) * k;
  // tapete
  g.fillStyle = css("--mat"); g.fillRect(X(-FIELD.outer), Y(FIELD.outer), 2 * FIELD.outer * k, 2 * FIELD.outer * k);
  g.lineWidth = 6; g.strokeStyle = "#111"; g.strokeRect(X(-FIELD.outer), Y(FIELD.outer), 2 * FIELD.outer * k, 2 * FIELD.outer * k);
  g.fillStyle = "#222"; g.fillRect(X(-FIELD.inner), Y(FIELD.inner), 2 * FIELD.inner * k, 2 * FIELD.inner * k);
  const line = (a, b, col, w) => { g.strokeStyle = col; g.lineWidth = w; g.beginPath(); g.moveTo(X(a[0]), Y(a[1])); g.lineTo(X(b[0]), Y(b[1])); g.stroke(); };
  FIELD.orange.forEach(([a, b]) => line(a, b, css("--orange"), 3));
  FIELD.blue.forEach(([a, b]) => line(a, b, css("--blue"), 3));
  // cajón
  g.fillStyle = "#c98a4b";
  for (const b of FIELD.barriers || []) { g.beginPath(); b.forEach((p, i) => i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1]))); g.closePath(); g.fill(); }
  // asientos
  g.fillStyle = "#0002";
  for (const c of Object.values(FIELD.seats)) { g.beginPath(); g.arc(X(c[0]), Y(c[1]), 3, 0, 7); g.fill(); }
  const t = S.tele || {};
  const conf = (t.map && t.map.confirmed) || [];
  const s = FIELD.sign;
  for (const [sec, seat, col] of conf) {
    const c = FIELD.seats[`${sec}/${seat}`]; if (!c) continue;
    g.fillStyle = col === "Red" ? css("--red") : col === "Green" ? css("--green") : "#999";
    g.fillRect(X(c[0] - s / 2), Y(c[1] + s / 2), s * k, s * k);
    g.strokeStyle = "#000"; g.lineWidth = 1; g.strokeRect(X(c[0] - s / 2), Y(c[1] + s / 2), s * k, s * k);
  }
  // ruta
  if ($("#opt-route").checked && t.route && t.route.length > 1) {
    g.strokeStyle = css("--route"); g.lineWidth = 2.5; g.setLineDash([]); g.beginPath();
    t.route.forEach((p, i) => i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1]))); g.stroke();
  }
  // rastro
  if ($("#opt-trail").checked && trail.length > 1) {
    g.strokeStyle = "#7d4cff99"; g.lineWidth = 2; g.beginPath();
    trail.forEach((p, i) => i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1]))); g.stroke();
  }
  const pose = t.loc ? t.loc.pose : null;
  $("#map-sub").textContent = pose ? `x ${fmt(pose[0])} y ${fmt(pose[1])} rumbo ${fmt(pose[2], 1)}°` :
    (t.map && t.map.direction ? `sentido ${t.map.direction}, cajón ${t.map.parking}` : "sin pose (el carro se ubica al salir del cajón)");
  if (!pose) return;
  // incertidumbre
  if (t.loc.sd) { g.strokeStyle = "#7d4cff"; g.lineWidth = 1; g.beginPath(); g.ellipse(X(pose[0]), Y(pose[1]), Math.max(2, 2 * t.loc.sd[0] * k), Math.max(2, 2 * t.loc.sd[1] * k), 0, 0, 7); g.stroke(); }
  // cámara (cono del BEV ~83° hasta ~700 mm)
  if ($("#opt-cam").checked) {
    const m = mountWorld(pose, [0, 120, 0]), r0 = pose[2] * Math.PI / 180, half = 41.5 * Math.PI / 180, R = 700;
    g.fillStyle = "#4c9aff22"; g.beginPath(); g.moveTo(X(m.ox), Y(m.oy));
    for (let a = -half; a <= half + 1e-6; a += half / 10) g.lineTo(X(m.ox + R * Math.sin(r0 + a)), Y(m.oy + R * Math.cos(r0 + a)));
    g.closePath(); g.fill();
  }
  // rayos ToF
  if ($("#opt-rays").checked) {
    const tof = tofNow() || {}, segs = segments(), last = t.loc.last || {};
    const map = { tL: ["tof_left", tof.L], tR: ["tof_right", tof.R], tB: ["tof_rear", tof.B], tF: ["tof_front", tof.F] };
    for (const [key, [mname, z]] of Object.entries(map)) {
      const mount = FIELD.mounts[mname]; if (!mount) continue;
      const m = mountWorld(pose, mount), exp = raycast(m.ox, m.oy, m.dx, m.dy, segs);
      if (z > 0) {
        const used = last[key] ? " (usado)" : "";
        g.strokeStyle = css("--ray"); g.lineWidth = 2; g.beginPath(); g.moveTo(X(m.ox), Y(m.oy));
        g.lineTo(X(m.ox + z * m.dx), Y(m.oy + z * m.dy)); g.stroke();
        g.fillStyle = css("--ray"); g.beginPath(); g.arc(X(m.ox + z * m.dx), Y(m.oy + z * m.dy), 3.5, 0, 7); g.fill();
        if (Number.isFinite(exp) && exp < 1500) {
          g.strokeStyle = Math.abs(z - exp) > 60 ? css("--bad") : css("--ray"); g.lineWidth = 2;
          g.beginPath(); g.arc(X(m.ox + exp * m.dx), Y(m.oy + exp * m.dy), 6, 0, 7); g.stroke();
        }
      }
    }
  }
  drawCar(g, X, Y, k, pose);
}
function drawCar(g, X, Y, k, pose) {
  const car = (FIELD && FIELD.car) || { len: 176, width: 125, rear_overhang: 21.8 };
  const r = pose[2] * Math.PI / 180, f = [Math.sin(r), Math.cos(r)], rt = [Math.cos(r), -Math.sin(r)];
  const P = (fw, ri) => [pose[0] + fw * f[0] + ri * rt[0], pose[1] + fw * f[1] + ri * rt[1]];
  // El carro del gemelo: el modelo 3D visto desde arriba (pieza por pieza, de abajo hacia arriba).
  const top = FIELD && FIELD.car_top;
  if (top && top.length) {
    g.lineJoin = "round";
    for (const part of top) {
      g.fillStyle = part.rgb; g.beginPath();
      part.pts.forEach(([ri, fw], i) => { const p = P(fw, ri); i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1])); });
      g.closePath(); g.fill();
    }
    return;
  }
  const b = -car.rear_overhang, fr = car.len - car.rear_overhang, w = car.width / 2;
  const pts = [P(b, -w), P(fr, -w), P(fr, w), P(b, w)];
  g.fillStyle = "#1e66f5cc"; g.strokeStyle = "#fff"; g.lineWidth = 1.5; g.beginPath();
  pts.forEach((p, i) => i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1]))); g.closePath(); g.fill(); g.stroke();
  const n = P(fr, 0); g.fillStyle = "#fff"; g.beginPath(); g.arc(X(n[0]), Y(n[1]), 3, 0, 7); g.fill();
}

// ── Radar (vista desde el carro) ────────────────────────────────────────────
function drawRadar() {
  const cv = $("#radar"), g = cv.getContext("2d"), W = cv.width;
  g.clearRect(0, 0, W, W);
  const R = 1000, k = (W / 2 - 20) / R, cx = W / 2, cy = W / 2 + 40;
  g.strokeStyle = "#2b313b"; g.fillStyle = "#8a94a3"; g.font = "11px system-ui"; g.lineWidth = 1;
  for (const d of [200, 400, 600, 800]) { g.beginPath(); g.arc(cx, cy, d * k, 0, 7); g.stroke(); g.fillText(`${d / 10} cm`, cx + 3, cy - d * k - 2); }
  const pose = [0, 0, 0], X = (x) => cx + x * k, Y = (y) => cy - y * k;
  const tof = tofNow();
  if (FIELD && tof) {
    const raw = tof.raw || [], order = { L: 0, R: 1, B: 2, F: 3 };
    for (const [key, mname] of [["L", "tof_left"], ["R", "tof_right"], ["B", "tof_rear"], ["F", "tof_front"]]) {
      const mount = FIELD.mounts[mname]; if (!mount) continue;
      if (key === "B" && fwNow() === "open") continue;   // la open no trae trasero
      const m = mountWorld(pose, mount), z = tof[key], ok = z > 0;
      const zz = ok ? Math.min(z, R) : R * 0.25;
      g.strokeStyle = ok ? css("--ray") : css("--bad"); g.setLineDash(ok ? [] : [4, 4]); g.lineWidth = 3;
      g.beginPath(); g.moveTo(X(m.ox), Y(m.oy)); g.lineTo(X(m.ox + zz * m.dx), Y(m.oy + zz * m.dy)); g.stroke(); g.setLineDash([]);
      // cono del sensor (~27°)
      g.fillStyle = ok ? "#ffd84d18" : "#e5484d18"; g.beginPath(); g.moveTo(X(m.ox), Y(m.oy));
      const a0 = Math.atan2(m.dx, m.dy);
      for (let a = -0.235; a <= 0.236; a += 0.047) g.lineTo(X(m.ox + zz * Math.sin(a0 + a)), Y(m.oy + zz * Math.cos(a0 + a)));
      g.closePath(); g.fill();
      const lx = X(m.ox + (zz + 60) * m.dx), ly = Y(m.oy + (zz + 60) * m.dy);
      g.fillStyle = ok ? "#fff" : css("--bad"); g.font = "bold 15px system-ui"; g.textAlign = "center";
      g.fillText(ok ? `${key} ${Math.round(z)}` : `${key} —`, lx, ly);
      if (raw[order[key]] !== undefined) { g.font = "11px system-ui"; g.fillStyle = "#8a94a3"; g.fillText(`crudo ${raw[order[key]]}`, lx, ly + 14); }
      g.textAlign = "start";
    }
  } else {
    g.fillStyle = "#8a94a3"; g.font = "14px system-ui"; g.fillText("sin lecturas del ESP", 20, 30);
  }
  drawCar(g, X, Y, k, pose);
  g.fillStyle = "#8a94a3"; g.font = "11px system-ui"; g.fillText(tof ? `fuente: ${tof.src === "PRE" ? "antes del botón (PRE)" : "carrera (ACK)"}` : "", 10, W - 10);
}

// ── Sensores ────────────────────────────────────────────────────────────────
const SENSORS = [
  ["tL", "ToF izq", "mm"], ["tR", "ToF der", "mm"], ["tF", "ToF frente", "mm"], ["tB", "ToF atrás", "mm"],
  ["yaw", "Rumbo (yaw)", "°"], ["vel", "Velocidad", "mm/s"], ["pw", "PWM", ""], ["srv", "Servo", ""],
  ["fps", "Cámara/loop", "fps"], ["ue", "Errores UART", ""], ["sdx", "Incertidumbre pose", "mm"],
];
function spark(cv, arr) {
  const g = cv.getContext("2d"), W = cv.width = cv.clientWidth || 140, H = cv.height = 34;
  g.clearRect(0, 0, W, H); if (!arr || arr.length < 2) return;
  let lo = Math.min(...arr), hi = Math.max(...arr); if (hi - lo < 1e-6) { hi += 1; lo -= 1; }
  g.strokeStyle = css("--accent"); g.lineWidth = 1.5; g.beginPath();
  arr.forEach((v, i) => { const x = i / (HIST_N - 1) * W, y = H - 2 - (v - lo) / (hi - lo) * (H - 4); i ? g.lineTo(x, y) : g.moveTo(x, y); });
  g.stroke();
}
function drawSensors() {
  const box = $("#sensors");
  if (!box.children.length) box.innerHTML = SENSORS.map(([k, n]) => `<div class="sensor" id="s-${k}"><div class="n">${n}</div><div class="v">—</div><canvas></canvas></div>`).join("");
  for (const [k, , u] of SENSORS) {
    const el = $(`#s-${k}`), a = hist[k] || [], v = a[a.length - 1];
    el.querySelector(".v").innerHTML = v === undefined ? "—" : `${fmt(v, Math.abs(v) < 10 && k !== "ue" ? 1 : 0)} <small>${u}</small>`;
    el.classList.toggle("bad", (k.startsWith("t") && k.length === 2 && v !== undefined && v <= 0) || (k === "ue" && v > 0));
    spark(el.querySelector("canvas"), a);
  }
}

// ── Checklist ───────────────────────────────────────────────────────────────
function drawChecklist() {
  const t = S.tele || {}, sy = S.sys || {}, items = [];
  const add = (st, txt, d = "") => items.push(`<li class="${st}"><b>${st === "ok" ? "✓" : st === "bad" ? "✗" : "!"}</b>${txt}<span class="d">${d}</span></li>`);
  add(sy.service === "active" ? "ok" : "bad", "Servicio wro-runtime", sy.service || "?");
  add(t._age !== null && t._age < 1.5 ? "ok" : "bad", "Telemetría del runtime", t._age === null ? "nunca" : `hace ${fmt(t._age, 1)} s`);
  const esp = !!t.ack || !!t.pre;
  add(esp ? "ok" : "bad", "ESP32 respondiendo", t.pre ? "PRE (quieto)" : t.ack ? "ACK (carrera)" : "nada (¿ESP apagado o firmware viejo?)");
  const tof = tofNow();
  if (tof) {
    const open = fwNow() === "open", keys = open ? ["L", "R", "F"] : ["L", "R", "F", "B"];
    const bad = keys.filter((k) => !(tof[k] > 0));
    add(bad.length ? (bad.length > 1 ? "bad" : "warn") : "ok", `${keys.length} ToF con lectura`, bad.length ? `sin: ${bad.join(" ")}` : keys.map((k) => tof[k]).join("/"));
    if (t.pre && t.pre.i) {
      const okI = open ? "1101" : "1111";   // open: L/R/-/F (sin trasero)
      add(String(t.pre.i) === okI ? "ok" : "bad", "ToF iniciados al arrancar el ESP", `i=${t.pre.i}`);
    }
  } else add("bad", "4 ToF con lectura", "sin datos");
  add(t.fps >= 30 ? "ok" : t.fps >= 20 ? "warn" : "bad", "Cámara y loop", `${fmt(t.fps, 0)} fps`);
  add(sy.temp_c == null ? "warn" : sy.temp_c < 70 ? "ok" : sy.temp_c < 80 ? "warn" : "bad", "Temperatura de la Pi", `${fmt(sy.temp_c, 0)} °C`);
  add(sy.throttled === 0 ? "ok" : sy.throttled === null || sy.throttled === undefined ? "warn" : "bad", "Sin bajo voltaje ni throttling",
      sy.throttled ? "0x" + sy.throttled.toString(16) + " (revisa la pila)" : "");
  add(sy.disk_free_gb > 2 ? "ok" : "warn", "Espacio para grabar", `${fmt(sy.disk_free_gb, 1)} GB`);
  add(sy.calib ? "ok" : "warn", "Calibración de la sede", sy.calib ? "cargada al arrancar" : "sin archivo (valores del repo)");
  const cc = t.color;
  if (cc) add(cc.floor ? "ok" : "warn", "Corrección de color por piso", cc.floor ? `ganancias ${cc.gains.map((v) => v.toFixed(2)).join("/")}` : "todavía no mide el piso");
  const rad = sy.radios || {};
  if (!sy.radios) add("warn", "Radios para la ronda (regla 11.10)", "no se pudo leer rfkill");
  else add(rad.wlan || rad.bluetooth ? "warn" : "ok", "Radios para la ronda (regla 11.10)", rad.wlan || rad.bluetooth ? "Wi-Fi/BT encendidos: apágalos antes de la ronda" : "apagados");
  $("#checklist").innerHTML = items.join("");
}
function drawLoc() {
  if (fwNow() === "open") return drawLocOpen();
  const t = S.tele || {}, l = t.loc, a = t.ack || {}, m = t.map || {};
  const rows = [
    ["Sentido / cajón", `${m.direction || "—"} / ${m.parking || "—"}`],
    ["Arranque ubicado", l ? (l.fixed ? `sí (δ pose ${fmt(l.shift)} mm, cajón w0 ${fmt(l.park_w)})` : "todavía no") : "—"],
    ["Pose x, y, rumbo", l ? `${fmt(l.pose[0])}, ${fmt(l.pose[1])}, ${fmt(l.pose[2], 1)}°` : "—"],
    ["σ pose", l ? `${fmt(l.sd[0])} / ${fmt(l.sd[1])} mm, ${fmt(l.sd[2], 2)}°` : "—"],
    ["Lecturas usadas / tiradas", l ? `${l.used} / ${l.rej}` : "—"],
    ["Postes confirmados", (m.confirmed || []).map((c) => `${c[0]}/${c[1]} ${c[2] === "Red" ? "🔴" : "🟢"}`).join("  ") || "—"],
    ["Odómetro", `${fmt(a.od)} mm`],
    ["Tiempos (ms)", Object.entries(t.timing || {}).map(([k, v]) => `${k} ${v}`).join(" · ") || "—"],
  ];
  $("#loc-table").innerHTML = rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join("");
}

// ── Sistema ─────────────────────────────────────────────────────────────────
function drawSys() {
  const sy = S.sys || {};
  const thr = sy.throttled;
  const thrTxt = thr === null || thr === undefined ? "?" : thr === 0 ? "ok" :
    [[0, "bajo voltaje AHORA"], [1, "frecuencia limitada"], [2, "throttling"], [3, "límite térmico"], [16, "hubo bajo voltaje"], [18, "hubo throttling"]]
      .filter(([b]) => thr & (1 << b)).map(([, s]) => s).join(", ");
  const rows = [
    ["Commit", sy.git], ["Servicio", `${sy.service} desde ${sy.service_since || "?"}`], ["Temperatura", `${fmt(sy.temp_c, 1)} °C`],
    ["Alimentación", thrTxt], ["Carga", (sy.load || []).join(" ")], ["Disco libre", `${fmt(sy.disk_free_gb, 1)} GB`],
    ["IPs", (sy.ips || []).join("  ")], ["Wi-Fi", sy.wifi_ssid || "—"],
    ["Radios", sy.radios ? Object.entries(sy.radios).map(([k, v]) => `${k} ${v ? "ON" : "off"}`).join(", ") : "?"],
    ["Calibración", sy.calib || "sin archivo"],
  ];
  $("#sys-table").innerHTML = rows.map(([k, v]) => `<tr><td>${k}</td><td>${v ?? "—"}</td></tr>`).join("");
}
$("#btn-ronda").onclick = async () => {
  if (!confirm("Apaga Wi-Fi y Bluetooth. Si estás conectado por Wi-Fi vas a perder el dashboard. ¿Seguir?")) return;
  try { toast((await api("/api/radio", { on: false })).msg); } catch (e) { toast(e.message, true); }
};
$("#btn-radio-on").onclick = async () => { try { toast((await api("/api/radio", { on: true })).msg); } catch (e) { toast(e.message, true); } };
async function loadLogs() {
  try {
    const f = $("#log-f").value, q = $("#log-q").value.trim();
    const txt = await api(`/api/logs?n=300&f=${encodeURIComponent(q || f)}`);
    const el = $("#log"), atBottom = el.scrollTop + el.clientHeight >= el.scrollHeight - 30;
    el.textContent = txt; if (atBottom) el.scrollTop = el.scrollHeight;
  } catch (e) { $("#log").textContent = e.message; }
}
$("#btn-log").onclick = loadLogs; $("#log-f").onchange = loadLogs;
setInterval(() => { if ($("#tab-sys").classList.contains("on") && $("#log-auto").checked) loadLogs(); }, 3000);
async function loadRuns() {
  try {
    const r = await api("/api/runs");
    $("#runs").innerHTML = "<tr><th>archivo</th><th>MB</th><th>fecha</th></tr>" +
      r.map((f) => `<tr><td><a href="/api/runs/${f.name}">${f.name}</a></td><td>${f.mb}</td><td>${f.t}</td></tr>`).join("");
  } catch (e) { $("#runs").innerHTML = `<tr><td>${e.message}</td></tr>`; }
}
$("#btn-runs").onclick = loadRuns;
let CALIB = {};
async function loadCalib() {
  try {
    const r = await api("/api/calib"); CALIB = r.data || {};
    $("#calib-path").textContent = r.path;
    const show = { ...CALIB }; if (show._hist) show._hist = `${show._hist.length} versiones anteriores`;
    $("#calib-json").textContent = JSON.stringify(show, null, 1) || "{}";
    if (!bandsInit) initBands();
  } catch (e) { $("#calib-json").textContent = e.message; }
}
$("#btn-calib-undo").onclick = async () => {
  if (!confirm("¿Regresar a la calibración anterior?")) return;
  try { toast((await api("/api/calib/undo", {})).msg); loadCalib(); } catch (e) { toast(e.message, true); }
};

$("#btn-hud").onclick = () => {
  const img = $("#hud");
  if (img.dataset.off) { img.src = "/api/hud.mjpg?fps=6"; delete img.dataset.off; $("#btn-hud").textContent = "pausar"; }
  else { img.src = ""; img.dataset.off = 1; $("#btn-hud").textContent = "reanudar"; }
};
$("#btn-clear-trail").onclick = () => { trail.length = 0; };

// ── Calibración de ToF ──────────────────────────────────────────────────────
const REAL = [100, 200, 300, 400, 500, 600, 700, 800];
const tofPts = { L: [], R: [] };        // [real, crudoMediana, sigma]
let espTable = null;                    // {L: [...], R: [...], src}
function median(a) { const s = [...a].sort((x, y) => x - y), n = s.length; return n ? (n % 2 ? s[(n - 1) / 2] : (s[n / 2 - 1] + s[n / 2]) / 2) : NaN; }
function std(a) { if (a.length < 2) return 0; const m = a.reduce((x, y) => x + y, 0) / a.length; return Math.sqrt(a.reduce((x, y) => x + (y - m) ** 2, 0) / (a.length - 1)); }

async function preWindow(sec) {
  const since = Date.now() / 1000 - sec;
  const r = await api(`/api/pre?since=${since}`);
  return r.pre.map(([, d]) => d);
}
function drawTofLive() {
  const p = (S.tele || {}).pre;
  const box = $("#tof-live");
  if (!p) { box.innerHTML = `<div class="sensor bad" style="grid-column: span 4"><div class="n">Sin lecturas PRE</div><div class="v small">El servicio tiene que estar corriendo y el carro sin arrancar o ya TERMINADO. Si no llega nada: firmware viejo o ESP apagado.</div></div>`; return; }
  const raw = p.rl || [];
  box.innerHTML = [["L", 0], ["R", 1], ["B", 2], ["F", 3]].map(([k, i]) =>
    `<div class="sensor ${raw[i] > 0 ? "" : "bad"}"><div class="n">${k} crudo · corregido</div><div class="v">${raw[i] ?? "—"} <small>· ${p[k] ?? "—"} mm</small></div></div>`).join("");
}
$("#btn-tof-cap").onclick = async () => {
  const rL = parseFloat($("#tof-real-L").value), rR = parseFloat($("#tof-real-R").value);
  if (!(rL > 0) && !(rR > 0)) return toast("Escribe la distancia real de al menos un lado", true);
  try {
    const pre = await preWindow(2.2);
    if (pre.length < 5) return toast("Pocas lecturas (¿el carro ya arrancó?). Espera 2 s y vuelve a capturar.", true);
    for (const [side, real, i] of [["L", rL, 0], ["R", rR, 1]]) {
      if (!(real > 0)) continue;
      const vals = pre.map((d) => (d.rl || [])[i]).filter((v) => v > 0);
      if (vals.length < 4) { toast(`${side}: sin lecturas válidas`, true); continue; }
      const m = median(vals), sd = std(vals);
      tofPts[side] = tofPts[side].filter((q) => Math.abs(q[0] - real) > 5);
      tofPts[side].push([real, m, sd]); tofPts[side].sort((a, b) => a[0] - b[0]);
      if (sd > 15) toast(`${side}: lecturas inestables (σ ${sd.toFixed(0)} mm). ¿Está paralelo y quieto?`, true);
    }
    drawTofCal();
  } catch (e) { toast(e.message, true); }
};
// Tabla leído(real) a 100..800: interpolación lineal entre lo capturado; fuera, la pendiente del extremo.
function fitTable(pts) {
  if (pts.length < 2) return null;
  const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  const at = (r) => {
    let i = 1; while (i < xs.length - 1 && r > xs[i]) i++;
    const x0 = xs[i - 1], x1 = xs[i], y0 = ys[i - 1], y1 = ys[i];
    return y0 + (r - x0) * (y1 - y0) / (x1 - x0);
  };
  const out = REAL.map((r) => Math.round(at(r)));
  for (let k = 1; k < out.length; k++) if (out[k] <= out[k - 1]) out[k] = out[k - 1] + 1;
  return out;
}
function drawTofCal() {
  const tb = $("#tof-points tbody"); let n = 0;
  tb.innerHTML = ["L", "R"].flatMap((s) => tofPts[s].map((p, j) =>
    `<tr><td>${++n}</td><td>${s}</td><td>${p[0]}</td><td>${p[1].toFixed(0)}</td><td>${p[2].toFixed(1)}</td><td>${(p[1] - p[0]).toFixed(0)}</td><td><button class="small" data-del="${s}:${j}">✕</button></td></tr>`)).join("");
  $$("[data-del]").forEach((b) => b.onclick = () => { const [s, j] = b.dataset.del.split(":"); tofPts[s].splice(+j, 1); drawTofCal(); });
  const fit = { L: fitTable(tofPts.L), R: fitTable(tofPts.R) };
  const row = (name, arr) => `<tr><td>${name}</td>${REAL.map((_, i) => `<td>${arr ? arr[i] : "—"}</td>`).join("")}</tr>`;
  $("#tof-table").innerHTML = `<tr><th>real</th>${REAL.map((r) => `<th>${r}</th>`).join("")}</tr>` +
    row("L nueva", fit.L) + row("R nueva", fit.R) +
    (espTable ? row(`L ESP (${espTable.src[0]})`, espTable.L) + row(`R ESP (${espTable.src[1]})`, espTable.R) : "");
  // gráfica
  const cv = $("#tof-chart"), g = cv.getContext("2d"), W = cv.width, H = cv.height, P = 34;
  g.clearRect(0, 0, W, H);
  const X = (v) => P + (v / 900) * (W - P - 10), Y = (v) => H - P - (v / 900) * (H - P - 10);
  g.strokeStyle = "#2b313b"; g.fillStyle = "#8a94a3"; g.font = "11px system-ui"; g.lineWidth = 1;
  for (let v = 0; v <= 900; v += 100) { g.beginPath(); g.moveTo(X(v), Y(0)); g.lineTo(X(v), Y(900)); g.moveTo(X(0), Y(v)); g.lineTo(X(900), Y(v)); g.stroke(); g.fillText(v, X(v) - 8, H - P + 14); g.fillText(v, 2, Y(v) + 4); }
  g.fillText("real (mm)", W - 70, H - 4); g.fillText("crudo", 4, 12);
  g.strokeStyle = "#8a94a3"; g.setLineDash([4, 4]); g.beginPath(); g.moveTo(X(0), Y(0)); g.lineTo(X(900), Y(900)); g.stroke(); g.setLineDash([]);
  const series = [["L", "#ffd84d"], ["R", "#4c9aff"]];
  for (const [s, col] of series) {
    if (espTable) { g.strokeStyle = col + "66"; g.lineWidth = 1.5; g.beginPath(); espTable[s].forEach((v, i) => i ? g.lineTo(X(REAL[i]), Y(v)) : g.moveTo(X(REAL[i]), Y(v))); g.stroke(); }
    if (fit[s]) { g.strokeStyle = col; g.lineWidth = 2.5; g.beginPath(); fit[s].forEach((v, i) => i ? g.lineTo(X(REAL[i]), Y(v)) : g.moveTo(X(REAL[i]), Y(v))); g.stroke(); }
    g.fillStyle = col; for (const p of tofPts[s]) { g.beginPath(); g.arc(X(p[0]), Y(p[1]), 4, 0, 7); g.fill(); }
  }
  g.fillStyle = "#8a94a3"; g.fillText("línea gruesa = nueva · delgada = la del ESP · punteada = perfecta", P + 6, 14);
}
function espRx(line) {
  const el = $("#esp-rx"); el.textContent = (el.textContent + "\n" + line).split("\n").slice(-12).join("\n").trim();
  const m = line.match(/^CALQ L=([\d/]+) R=([\d/]+) src=(\w+)\/(\w+)/);
  if (m) { espTable = { L: m[1].split("/").map(Number), R: m[2].split("/").map(Number), src: [m[3], m[4]] }; drawTofCal(); }
  if (line.startsWith("CALT ok")) toast("ESP: tabla guardada");
  if (line.startsWith("CALT error")) toast("ESP rechazó la tabla: " + line, true);
}
async function sendEsp(line) {
  if (!S.tele.pre) return toast("El ESP no está mandando PRE: solo acepta antes del botón o ya TERMINADO.", true);
  try { await api("/api/esp", { line }); toast("enviado: " + line.slice(0, 40)); } catch (e) { toast(e.message, true); }
}
$("#btn-calq").onclick = () => sendEsp("CALQ");
for (const [side, i] of [["L", 0], ["R", 1]]) {
  $(`#btn-calt-${side}`).onclick = async () => {
    const tab = fitTable(tofPts[side]);
    if (!tab) return toast(`${side}: captura al menos 2 distancias (mejor 3+)`, true);
    if (!confirm(`Enviar al ESP la tabla ${side}: ${tab.join(", ")}`)) return;
    await sendEsp(`CALT,${i},${tab.join(",")}`);
    try {
      const d = (await api("/api/calib")).data || {};
      d.tof = d.tof || {}; d.tof[side] = tab; d.tof[side + "_pts"] = tofPts[side]; d.tof.t = new Date().toISOString();
      await api("/api/calib", { data: d });
    } catch (e) { toast("no se guardó el registro: " + e.message, true); }
    setTimeout(() => sendEsp("CALQ"), 600);
  };
}
$("#btn-calt-fw").onclick = async () => {
  if (!confirm("¿Regresar L y R a la tabla del firmware?")) return;
  await sendEsp("CALT,0,FW"); setTimeout(() => sendEsp("CALT,1,FW"), 400); setTimeout(() => sendEsp("CALQ"), 900);
};

// ── Calibración de color ────────────────────────────────────────────────────
const LABELS = [["Piso", "#d9d4c7"], ["Rojo", "#ff4d4f"], ["Verde", "#2ecc71"], ["Rosa", "#ff5fd2"], ["Naranja", "#ff9f1a"], ["Azul", "#3d7bff"]];
let curLabel = "Rojo", rects = [], snapImg = null, drag = null;
$("#labels").innerHTML = LABELS.map(([l, c]) => `<button class="small${l === curLabel ? " on" : ""}" data-lab="${l}"><i style="background:${c}"></i>${l}</button>`).join("");
$$("[data-lab]").forEach((b) => b.onclick = () => { curLabel = b.dataset.lab; $$("[data-lab]").forEach((x) => x.classList.toggle("on", x === b)); });
$("#btn-snap").onclick = async () => {
  try {
    await api("/api/snap", {});
    snapImg = new Image(); snapImg.onload = drawSnap; snapImg.src = "/api/raw.jpg?t=" + Date.now();
  } catch (e) { toast(e.message, true); }
};
$("#btn-clear-rects").onclick = () => { rects = []; drawSnap(); };
function snapXY(ev) { const cv = $("#snap"), r = cv.getBoundingClientRect(); return [(ev.clientX - r.left) * cv.width / r.width, (ev.clientY - r.top) * cv.height / r.height]; }
$("#snap").onpointerdown = (ev) => { if (!snapImg) return; drag = snapXY(ev); $("#snap").setPointerCapture(ev.pointerId); };
$("#snap").onpointermove = (ev) => { if (!drag) return; drawSnap(); const [x, y] = snapXY(ev), g = $("#snap").getContext("2d"); g.strokeStyle = "#fff"; g.setLineDash([4, 3]); g.strokeRect(drag[0], drag[1], x - drag[0], y - drag[1]); g.setLineDash([]); };
$("#snap").onpointerup = (ev) => {
  if (!drag) return; const [x, y] = snapXY(ev);
  const r = { label: curLabel, x: Math.min(x, drag[0]), y: Math.min(y, drag[1]), w: Math.abs(x - drag[0]), h: Math.abs(y - drag[1]) };
  drag = null; if (r.w > 4 && r.h > 4) rects.push(r); drawSnap();
};
function drawSnap() {
  const cv = $("#snap"), g = cv.getContext("2d");
  g.clearRect(0, 0, cv.width, cv.height);
  if (snapImg) g.drawImage(snapImg, 0, 0, cv.width, cv.height);
  for (const r of rects) {
    const col = (LABELS.find((l) => l[0] === r.label) || [0, "#fff"])[1];
    g.strokeStyle = col; g.lineWidth = 2; g.strokeRect(r.x, r.y, r.w, r.h);
    g.fillStyle = col; g.font = "bold 12px system-ui"; g.fillText(r.label, r.x + 3, r.y + 13);
  }
}
let lastStats = [];
$("#btn-analyze").onclick = async () => {
  if (!rects.length) return toast("Marca al menos un recuadro", true);
  try {
    lastStats = (await api("/api/color/analyze", { rects })).regions;
    $("#color-stats").innerHTML = "<tr><th>región</th><th>px</th><th>H p5/50/95</th><th>S</th><th>V</th><th>G/R</th><th>B/R</th></tr>" +
      lastStats.map((s) => `<tr><td>${s.label}</td><td>${s.n}</td><td>${s.H.join(" / ")}</td><td>${s.S.join(" / ")}</td><td>${s.V.join(" / ")}</td><td>${s.GR}</td><td>${s.BR}</td></tr>`).join("");
    suggestBands(lastStats);
  } catch (e) { toast(e.message, true); }
};

// Bandas: {Red: [[lo,hi],[lo,hi]], Green: [[lo,hi]], Blue: [lo,hi]}
const DEF = { Red: [[[0, 110, 40], [5, 255, 200]], [[168, 110, 40], [179, 255, 235]]], Green: [[[30, 35, 25], [85, 255, 255]]], Blue: [[95, 70, 60], [115, 255, 200]] };
let BANDS = JSON.parse(JSON.stringify(DEF)), bandsInit = false;
function initBands() {
  const c = CALIB.colors || {};
  if (c.Red) BANDS.Red = c.Red; if (c.Green) BANDS.Green = c.Green;
  if (CALIB.config && CALIB.config.ROUTE_FIX_BLUE_HSV) BANDS.Blue = CALIB.config.ROUTE_FIX_BLUE_HSV;
  bandsInit = true; drawBands();
}
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, Math.round(v)));
function suggestBands(stats) {
  const by = (l) => stats.filter((s) => s.label === l);
  const merge = (arr, key, q) => q === 0 ? Math.min(...arr.map((s) => s[key][0])) : Math.max(...arr.map((s) => s[key][2]));
  const red = by("Rojo"), green = by("Verde"), blue = by("Azul");
  if (red.length) {
    const hlo = merge(red, "H", 0) - 4, hhi = merge(red, "H", 2) + 4;
    const smin = clamp(Math.max(60, merge(red, "S", 0) * 0.75), 0, 255), vmin = clamp(Math.max(30, merge(red, "V", 0) * 0.6), 0, 255);
    const r = [];
    if (hhi >= 0) r.push([[0, smin, vmin], [clamp(Math.max(hhi, 3), 0, 25), 255, 255]]);
    if (hlo < 0) r.push([[clamp(180 + Math.min(hlo, -2), 150, 179), smin, vmin], [179, 255, 255]]);
    else if (hlo > 0 && hlo > 150) r.push([[clamp(hlo, 150, 179), smin, vmin], [179, 255, 255]]);
    BANDS.Red = r;
  }
  if (green.length) {
    BANDS.Green = [[[clamp(merge(green, "H", 0) - 6, 20, 90), clamp(Math.max(25, merge(green, "S", 0) * 0.7), 0, 255), clamp(Math.max(20, merge(green, "V", 0) * 0.6), 0, 255)],
                    [clamp(merge(green, "H", 2) + 6, 40, 100), 255, 255]]];
  }
  if (blue.length) {
    BANDS.Blue = [[clamp(merge(blue, "H", 0) - 4, 80, 130), clamp(merge(blue, "S", 0) * 0.75, 0, 255), clamp(merge(blue, "V", 0) * 0.7, 0, 255)],
                  [clamp(merge(blue, "H", 2) + 4, 90, 140), 255, clamp(merge(blue, "V", 2) * 1.25, 0, 255)]];
  }
  drawBands();
  toast("Propuesta lista: revisa la vista previa antes de guardar");
}
function drawBands() {
  const rows = [];
  const hdr = `<div class="band"><span></span>${["H min", "S min", "V min", "H max", "S max", "V max"].map((h) => `<span class="h">${h}</span>`).join("")}</div>`;
  const inp = (path, v) => `<input type="number" min="0" max="255" value="${v}" data-band="${path}">`;
  BANDS.Red.forEach((r, i) => rows.push(`<div class="band"><b style="color:var(--red)">Rojo ${i + 1}</b>${[0, 1].flatMap((j) => r[j].map((v, c) => inp(`Red.${i}.${j}.${c}`, v))).join("")}</div>`));
  BANDS.Green.forEach((r, i) => rows.push(`<div class="band"><b style="color:var(--green)">Verde</b>${[0, 1].flatMap((j) => r[j].map((v, c) => inp(`Green.${i}.${j}.${c}`, v))).join("")}</div>`));
  rows.push(`<div class="band"><b style="color:var(--blue)">Azul</b>${[0, 1].flatMap((j) => BANDS.Blue[j].map((v, c) => inp(`Blue.${j}.${c}`, v))).join("")}</div>`);
  $("#bands").innerHTML = hdr + rows.join("");
  $$("[data-band]").forEach((el) => el.onchange = () => {
    const p = el.dataset.band.split("."), v = clamp(+el.value, 0, 255);
    if (p[0] === "Blue") BANDS.Blue[+p[1]][+p[2]] = v; else BANDS[p[0]][+p[1]][+p[2]][+p[3]] = v;
  });
}
drawBands();
$("#btn-preview").onclick = async () => {
  if (!snapImg) return toast("Toma una foto primero", true);
  try {
    const cur = CALIB.colors || {};
    const [a, b] = await Promise.all([api("/api/color/preview", { colors: cur, rects }), api("/api/color/preview", { colors: { Red: BANDS.Red, Green: BANDS.Green }, rects })]);
    $("#prev-cur").src = a.img; $("#prev-new").src = b.img;
    const labs = Object.keys(b.stats);
    $("#prev-stats").innerHTML = "<tr><th>región</th><th>rojo actual</th><th>rojo nuevo</th><th>verde actual</th><th>verde nuevo</th></tr>" +
      labs.map((l) => `<tr><td>${l}</td><td>${a.stats[l]?.Red ?? "—"} %</td><td>${b.stats[l]?.Red ?? "—"} %</td><td>${a.stats[l]?.Green ?? "—"} %</td><td>${b.stats[l]?.Green ?? "—"} %</td></tr>`).join("");
  } catch (e) { toast(e.message, true); }
};
$("#btn-color-save").onclick = async () => {
  if (!confirm("¿Guardar estas bandas de color? Se aplican al reiniciar el servicio.")) return;
  try {
    const d = (await api("/api/calib")).data || {};
    d.colors = { Red: BANDS.Red, Green: BANDS.Green };
    if ($("#opt-save-blue").checked) { d.config = d.config || {}; d.config.ROUTE_FIX_BLUE_HSV = BANDS.Blue; }
    d.colors_t = new Date().toISOString();
    toast((await api("/api/calib", { data: d })).msg);
    loadCalib();
  } catch (e) { toast(e.message, true); }
};
drawTofCal();


// ── Ronda abierta ───────────────────────────────────────────────────────────
// La open no tiene pose: se estima en cada recta con el ToF interior (distancia a la
// isla), el frontal (distancia a la pared del fondo) y el rumbo contra el objetivo.
// Recta k = sección k+1 del mapa; CCW (dir L) gira a la izquierda, CW (dir R) a la derecha.
// La sección 1 se dibuja abajo (no se sabe la orientación real de la pista).
const OPEN_H = 1500, OPEN_CAR = { len: 176, width: 125, rear_overhang: 21.8 };
const openTrail = [];
let openLastTc = -1, openPose = null;
function openInfo() {
  const t = S.tele || {}, a = (t.ack && t.ack.fw === "open") ? t.ack : {}, p = (t.pre && t.pre.fw === "open") ? t.pre : {};
  return {
    dir: a.dir || p.dir || "?", map: listOf(a.map ?? p.map) || [0, 0, 0, 0], mc: listOf(p.mc), mapok: a.mapok ?? p.mapok,
    tc: a.tc ?? p.tc, est: a.est, yaw: a.yaw, ao: a.ao, L: a.tL, R: a.tR, F: a.tF,
    mod: a.mod, trim: a.trim, pw: a.pw, srv: a.srv, running: !!t.ack && t.ack.fw === "open",
  };
}
// Marco de la recta k: (u a lo largo, v desde la pared de afuera hacia la isla) -> mundo.
function openFrame(k, ccw) {
  const th = (ccw ? 1 : -1) * k * Math.PI / 2, c = Math.cos(th), s = Math.sin(th);
  const rot = (x, y) => [x * c - y * s, x * s + y * c];
  return {
    P: (u, v) => rot(ccw ? -OPEN_H + u : OPEN_H - u, -OPEN_H + v),
    h0: (ccw ? 90 : -90) - (ccw ? 1 : -1) * k * 90,   // rumbo de la recta (0 = +y, horario)
  };
}
function openWidth(o, k) { const w = o.map[k]; return w > 0 ? w * 10 : 0; }
// Filtros de la pose estimada (run 11:33: el frontal de lejos brincaba 1560 -> 1000 -> 1720 mm
// en la misma recta y el rastro salía en zigzag):
//  - a lo largo solo con la pared del fondo cerca (OPEN_F_MAX); más lejos el frontal no sirve;
//  - dentro de una recta el carro no va hacia atrás ni brinca más de OPEN_U_SALTO;
//  - lo lateral se suaviza; cada recta empieza de cero.
const OPEN_F_MAX = 1200, OPEN_U_ATRAS = 80, OPEN_U_SALTO = 500, OPEN_V_ALFA = 0.35, OPEN_LAT_FWD = 70;
let openSt = { tc: -1, u: null, v: null, key: "" };
function openUpdate() {
  const o = openInfo();
  if (!o.running) return;
  if (o.tc !== undefined && o.tc < openLastTc) openTrail.length = 0;   // arrancó otra corrida
  openLastTc = o.tc ?? openLastTc;
  if (o.est !== "S" || o.dir === "?" || o.tc === undefined) return;
  const key = [o.tc, o.L, o.R, o.F, o.yaw].join("|");
  if (key === openSt.key) return;   // el mismo ACK otra vez (el estado se repite hasta que llega otro)
  openSt.key = key;
  if (o.tc !== openSt.tc) openSt = { tc: o.tc, u: null, v: null, key };
  const ccw = o.dir === "L", k = ((o.tc % 4) + 4) % 4, w = openWidth(o, k) || 1000, half = OPEN_CAR.width / 2;
  if (o.F > 0 && o.F < OPEN_F_MAX) {
    const u = 2 * OPEN_H - o.F - (OPEN_CAR.len - OPEN_CAR.rear_overhang);
    if (openSt.u === null || (u > openSt.u - OPEN_U_ATRAS && u < openSt.u + OPEN_U_SALTO)) openSt.u = Math.max(u, openSt.u ?? u);
  }
  if (openSt.u === null) return;   // lejos del fondo: no se sabe dónde va a lo largo
  // Lateral: la pared de afuera sirve en toda la recta; la isla solo mientras el sensor
  // todavía la tiene al lado. Pasando su orilla el ToF interior ve la pared de enfrente
  // del siguiente pasillo (~90-100 cm) y el carro "brincaba" hacia afuera.
  const dInt = ccw ? o.L : o.R, dOut = ccw ? o.R : o.L;
  const finIsla = 2 * OPEN_H - (openWidth(o, (k + 1) % 4) || 1000), uSensor = openSt.u + OPEN_LAT_FWD;
  let v = null;
  if (dOut > 0 && dOut < w - half) v = dOut + half;
  else if (dInt > 0 && dInt < w && uSensor < finIsla - 50) v = w - dInt - half;
  if (v !== null) openSt.v = openSt.v === null ? v : openSt.v + OPEN_V_ALFA * (v - openSt.v);
  if (openSt.v === null) return;
  const f = openFrame(k, ccw), [x, y] = f.P(openSt.u, openSt.v);
  const dev = (o.yaw ?? 0) - (o.ao ?? 0);   // + = a la izquierda
  openPose = [x, y, f.h0 - dev, k];
  const last = openTrail[openTrail.length - 1];
  if (!last || Math.hypot(x - last[0], y - last[1]) > 15) { openTrail.push(openPose); if (openTrail.length > 1500) openTrail.shift(); }
}
function drawOpen(g, W) {
  const o = openInfo(), ccw = o.dir !== "R";
  const span = 2 * OPEN_H + 200, kk = W / span;
  const X = (x) => (x + span / 2) * kk, Y = (y) => (span / 2 - y) * kk;
  g.fillStyle = css("--mat"); g.fillRect(X(-OPEN_H), Y(OPEN_H), 2 * OPEN_H * kk, 2 * OPEN_H * kk);
  g.lineWidth = 6; g.strokeStyle = "#111"; g.strokeRect(X(-OPEN_H), Y(OPEN_H), 2 * OPEN_H * kk, 2 * OPEN_H * kk);
  // Isla: cada lado a la distancia medida de la pared de afuera (sin medir = 100 cm, punteado).
  const ext = {}, sideOf = [];
  for (let k = 0; k < 4; k++) {
    const f = openFrame(k, ccw), a = f.P(0, 0), b = f.P(2 * OPEN_H, 0);
    const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
    const side = Math.abs(mid[0]) > Math.abs(mid[1]) ? (mid[0] > 0 ? "E" : "W") : (mid[1] > 0 ? "N" : "S");
    const w = openWidth(o, k);
    ext[side] = OPEN_H - (w || 1000); sideOf[k] = { side, w, mid };
  }
  const x0 = -ext.W, x1 = ext.E, y0 = -ext.S, y1 = ext.N;
  g.fillStyle = "#222"; g.fillRect(X(x0), Y(y1), (x1 - x0) * kk, (y1 - y0) * kk);
  const faces = { S: [[x0, y0], [x1, y0]], N: [[x0, y1], [x1, y1]], W: [[x0, y0], [x0, y1]], E: [[x1, y0], [x1, y1]] };
  g.font = "bold 16px system-ui"; g.textAlign = "center";
  for (let k = 0; k < 4; k++) {
    const { side, w, mid } = sideOf[k], [a, b] = faces[side];
    g.strokeStyle = w ? (w === 600 ? css("--orange") : css("--accent")) : "#8a94a3"; g.lineWidth = 5;
    g.setLineDash(w ? [] : [10, 8]); g.beginPath(); g.moveTo(X(a[0]), Y(a[1])); g.lineTo(X(b[0]), Y(b[1])); g.stroke(); g.setLineDash([]);
    const sc = 1 - (w || 1000) / 2 / OPEN_H, c = [mid[0] * sc, mid[1] * sc];   // centro del pasillo
    const now = o.running && o.tc !== undefined && ((o.tc % 4) + 4) % 4 === k;
    g.fillStyle = now ? "#d9480f" : "#222";   // sobre el tapete claro
    const mc = o.mc && o.mc[k] ? ` (${Math.round(o.mc[k])})` : "";
    g.fillText(`S${k + 1}: ${w ? w / 10 + " cm" : "?"}${mc}`, X(c[0]), Y(c[1]));
  }
  g.textAlign = "start";
  // flecha de sentido en la sección 1
  const f0 = openFrame(0, ccw), pa = f0.P(300, 250), pb = f0.P(700, 250);
  g.strokeStyle = "#fff"; g.lineWidth = 3; g.beginPath(); g.moveTo(X(pa[0]), Y(pa[1])); g.lineTo(X(pb[0]), Y(pb[1])); g.stroke();
  g.beginPath(); g.arc(X(pb[0]), Y(pb[1]), 6, 0, 7); g.fillStyle = "#fff"; g.fill();
  // rastro estimado: puntos (solo rectas; el giro no se estima)
  if ($("#opt-trail").checked && openTrail.length) {
    g.fillStyle = "#7d4cffaa";
    for (const p of openTrail) { g.beginPath(); g.arc(X(p[0]), Y(p[1]), 2.5, 0, 7); g.fill(); }
  }
  $("#map-sub").textContent = `Open · ${o.dir === "L" ? "CCW (izquierda)" : o.dir === "R" ? "CW (derecha)" : "sentido ?"} · mapa ${o.map.join("/")}${o.mapok ? " listo" : ""}` +
    (o.running ? ` · recta ${(o.tc ?? 0) + 1} (S${(((o.tc ?? 0) % 4) + 4) % 4 + 1})` : "");
  if (!openPose || !o.running) return;
  const pose = openPose;
  if ($("#opt-rays").checked) {
    const r = pose[2] * Math.PI / 180, fwd = [Math.sin(r), Math.cos(r)], rt = [Math.cos(r), -Math.sin(r)];
    const ref = OPEN_CAR.len - OPEN_CAR.rear_overhang;
    const rays = [[o.L, [-OPEN_CAR.width / 2, 70], [-rt[0], -rt[1]]], [o.R, [OPEN_CAR.width / 2, 70], rt], [o.F, [0, ref], fwd]];
    for (const [z, [ri, fw], d] of rays) {
      if (!(z > 0)) continue;
      const ox = pose[0] + fw * fwd[0] + ri * rt[0], oy = pose[1] + fw * fwd[1] + ri * rt[1];
      g.strokeStyle = css("--ray"); g.lineWidth = 2; g.beginPath(); g.moveTo(X(ox), Y(oy)); g.lineTo(X(ox + z * d[0]), Y(oy + z * d[1])); g.stroke();
      g.fillStyle = css("--ray"); g.beginPath(); g.arc(X(ox + z * d[0]), Y(oy + z * d[1]), 3.5, 0, 7); g.fill();
    }
  }
  drawCar(g, X, Y, kk, pose);
}
function drawLocOpen() {
  const o = openInfo(), t = S.tele || {};
  const est = { S: "siguiendo", G: "girando", T: "terminando" }[o.est] || (o.running ? o.est : "quieto");
  const rows = [
    ["Firmware", `RondaAbierta ${(t.pre && t.pre.bld) || ""}`],
    ["Sentido", o.dir === "L" ? "CCW (izquierda)" : o.dir === "R" ? "CW (derecha)" : "todavía no gira"],
    ["Estado / giros", `${est} · ${fmt(o.tc)}/12`],
    ["Mapa (pasillo por sección)", o.map.map((w, k) => `S${k + 1} ${w || "?"}${o.mc && o.mc[k] ? ` (${Math.round(o.mc[k])})` : ""}`).join(" · ") + (o.mapok ? " ✓" : "")],
    ["Rumbo / objetivo", `${fmt(o.yaw, 1)}° / ${fmt(o.ao, 1)}°`],
    ["Modo de manejo", `${o.mod || "—"} · trim ${fmt(o.trim, 1)}°`],
    ["PWM / servo", `${fmt(o.pw)} / ${fmt(o.srv)}`],
    ["Cómo se dibuja", "pose estimada por recta: ToF interior → isla, frontal → pared del fondo. El giro no se dibuja."],
  ];
  $("#loc-table").innerHTML = rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join("");
}


// ── ESP: qué firmware trae, regla sorpresa y flasheo desde la compu ─────────
// El flasheo solo existe cuando el dashboard se abre con scripts/fox_pc.py en la compu
// (localhost:8091): ese programa atiende /local/* y le pasa todo lo demás a la Pi.
let FLASH = null, flashPoll = null, sorpFile = null;
function drawEsp() {
  const t = S.tele || {}, fw = fwNow(), pre = t.pre || {}, a = t.ack || {};
  const rows = [
    ["Firmware", fw === "open" ? "Open (RondaAbierta)" : fw === "obs" ? "Obstáculos (PurePursuitCarrera)" : "sin datos del ESP"],
    ["Compilado", pre.bld || "— (firmware sin este dato o corriendo)"],
    ["Estado", t.armed ? "corriendo" : t.pre ? "quieto" : "—"],
    ["Regla sorpresa (runtime)", fw === "open" ? "no aplica en la open" : t.surprise ? "ON" : "OFF"],
    ["Último reset del ESP", pre.rst !== undefined ? `motivo ${pre.rst}` : "—"],
  ];
  $("#esp-now").innerHTML = rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join("");
  $("#sorp-file").textContent = sorpFile === null ? "" : `carrera.json: ${sorpFile ? "ON" : "OFF"}`;
}
async function loadSorp() { try { sorpFile = (await api("/api/surprise")).on; } catch (e) { sorpFile = null; } drawEsp(); }
for (const [id, on] of [["#btn-sorp-on", true], ["#btn-sorp-off", false]]) {
  $(id).onclick = async () => {
    if (S.tele.armed && !confirm("El carro está corriendo. ¿Reiniciar el servicio igual?")) return;
    try { const d = await api("/api/surprise", { on }); sorpFile = d.on; toast(d.msg || d.error, !!d.error); } catch (e) { toast(e.message, true); }
    drawEsp();
  };
}
async function flashStatus() {
  try { FLASH = await api("/local/flash/status"); } catch (e) { FLASH = null; }
  $("#flash-off").style.display = FLASH ? "none" : "";
  $("#flash-on").style.display = FLASH ? "" : "none";
  if (!FLASH) return;
  const sel = $("#flash-port"), cur = sel.value;
  sel.innerHTML = FLASH.ports.map((p) => `<option value="${p.port}">${p.port}${p.esp ? " (ESP)" : ""}</option>`).join("") ||
    `<option value="">sin puertos</option>`;
  if (cur && FLASH.ports.some((p) => p.port === cur)) sel.value = cur;
  const g = FLASH.git || {};
  $("#flash-git").textContent = `código: ${g.branch || "?"} · ${g.commit || "?"}` +
    (g.esp_sin_commit && g.esp_sin_commit.length ? ` · sin commit: ${g.esp_sin_commit.join(", ")}` : "") +
    (FLASH.cli ? "" : " · ¡no encuentro arduino-cli!");
  showJob(FLASH.job);
}
function showJob(j) {
  if (!j) return;
  $("#flash-state").textContent = j.busy ? `${j.state}… ${fmt(j.secs, 0)} s` :
    j.ok === null ? "" : j.ok ? `✓ listo en ${fmt(j.secs, 0)} s` : `✗ ${j.state}`;
  const box = $("#flash-log"); box.textContent = (j.log || []).join("\n"); box.scrollTop = box.scrollHeight;
  $$("[data-flash]").forEach((b) => b.disabled = !!j.busy);
}
async function pollJob() {
  try {
    const j = (await api("/local/flash/job")).job; showJob(j);
    if (!j.busy) { clearInterval(flashPoll); flashPoll = null; toast(j.ok ? "ESP flasheada y servicio reiniciado" : "El flasheo falló (ve el log)", !j.ok); loadSorp(); }
  } catch (e) { clearInterval(flashPoll); flashPoll = null; }
}
$("#btn-flash-ports").onclick = flashStatus;
$$("[data-flash]").forEach((b) => b.onclick = async () => {
  const sketch = b.dataset.flash, body = { sketch, port: $("#flash-port").value, force: $("#flash-force").checked };
  if (b.dataset.sorp !== undefined) body.surprise = b.dataset.sorp === "1";
  const name = sketch === "open" ? "Open" : `Obstáculos (sorpresa ${body.surprise ? "ON" : "OFF"})`;
  if (S.tele.armed && !confirm("El carro está corriendo. ¿Flashear igual?")) return;
  if (!confirm(`¿Flashear ${name} en ${body.port || "el puerto de la ESP"}? Si el código no cambió solo sube (~10 s); si cambió compila (~1 min). Al final reinicia el servicio.`)) return;
  try {
    const d = await api("/local/flash", body); toast(d.msg || d.error, !!d.error);
    if (!d.error && !flashPoll) flashPoll = setInterval(pollJob, 1000);
  } catch (e) { toast(e.message, true); }
});
$$('[data-tab="esp"]').forEach((b) => b.addEventListener("click", () => { flashStatus(); loadSorp(); }));


// ── Corridas: replay de una corrida grabada ─────────────────────────────────
// El runtime guarda <video>_tele.jsonl con el MISMO estado que llega en vivo (run_log.py); las
// corridas de antes se reconstruyen del journal (pose aproximada, sin ruta). El replay pone ese
// estado en S y usa el mismo dibujo de "En vivo" (mapa, radar, sensores, open), más esta pestaña.
let REP_LIST = [];
async function loadRepList() {
  try {
    REP_LIST = await api("/api/runs");
    const src = { runtime: "completa", journal: "del journal (aprox.)", "?": "?" };
    $("#rep-runs").innerHTML = "<tr><th>corrida</th><th>fecha</th><th>MB</th><th>telemetría</th><th></th></tr>" +
      REP_LIST.map((f) => `<tr class="${REP.on && REP.name === f.name ? "sel" : ""}"><td>${f.name.replace(".avi", "")}</td><td>${f.t}</td><td>${f.mb}</td>` +
        `<td>${f.tele ? src[f.tele] || f.tele : '<span class="muted">se arma del journal</span>'}</td>` +
        `<td><button class="small ok" data-rep="${f.name}">ver</button> <a href="/api/runs/${f.name}" title="descargar .avi">⬇</a></td></tr>`).join("");
    $$("[data-rep]").forEach((b) => b.onclick = () => repOpen(b.dataset.rep));
  } catch (e) { $("#rep-runs").innerHTML = `<tr><td>${e.message}</td></tr>`; }
}
$("#btn-rep-list").onclick = loadRepList;

async function repOpen(name) {
  repPause();
  $("#rep-title").textContent = `cargando ${name}…`;
  let d;
  try { d = await api(`/api/runs/${encodeURIComponent(name)}/timeline`); }
  catch (e) { $("#rep-title").textContent = ""; return toast(`${name}: ${e.message}`, true); }
  const st = d.states || [];
  if (!st.length) { $("#rep-title").textContent = ""; return toast(`${name}: sin estados`, true); }
  Object.assign(REP, {
    on: true, name, d, st, ev: d.events || [], i: 0, applied: -1, playing: false, timer: null,
    t0: st[0].t, t1: st[st.length - 1].t, tcur: st[0].t, frameIdx: -1, frameBusy: false, evShown: -1,
  });
  // tiempo del video: el .avi arranca con el header (o el 1er estado) y termina con el último estado
  REP.v0 = (d.hdr && d.hdr.t) || REP.t0;
  document.body.classList.add("replay");
  $("#repbar").style.display = "";
  $("#rep-name").textContent = name.replace(".avi", "");
  $("#rep-slider").max = Math.max(1, Math.round((REP.t1 - REP.t0) * 10));
  $("#hud").dataset.live = $("#hud").dataset.live || $("#hud").src;
  $("#rep-title").textContent = `${name.replace(".avi", "")} · ${d.summary.src === "journal" ? "reconstruida del journal (pose aprox., sin ruta)" : "telemetría completa"}`;
  repSummary();
  repEvents(true);
  loadRepList();
  repGo(0);
}

function repReset() {
  trail.length = 0; openTrail.length = 0; openPose = null; openLastTc = -1;
  openSt = { tc: -1, u: null, v: null, key: "" };
  for (const k of Object.keys(hist)) delete hist[k];
}
// Pone el estado i en S; lo que depende de la historia se re-acumula desde el principio si se va para atrás.
function repGo(i) {
  if (!REP.on) return;
  i = Math.max(0, Math.min(REP.st.length - 1, i));
  if (i < REP.applied) { repReset(); REP.applied = -1; }
  const sys = (SLIVE && SLIVE.sys) || {};
  for (let j = REP.applied + 1; j <= i; j++) { S = { tele: { ...REP.st[j], _age: 0 }, sys }; feedState(); }
  REP.applied = i; REP.i = i;
  if (!REP.playing) REP.tcur = REP.st[i].t;
  drawState();
  repUI();
}
function repSeekT(t) {
  REP.tcur = Math.max(REP.t0, Math.min(REP.t1, t));
  const st = REP.st;
  let lo = 0, hi = st.length - 1;
  while (lo < hi) { const m = (lo + hi + 1) >> 1; if (st[m].t <= REP.tcur) lo = m; else hi = m - 1; }
  repGo(lo);
}
function repTick() {
  const sp = parseFloat($("#rep-speed").value) || 1, now = performance.now();
  REP.tcur += (now - REP.lastTick) / 1000 * sp; REP.lastTick = now;
  if (REP.tcur >= REP.t1) { REP.tcur = REP.t1; repSeekT(REP.t1); return repPause(); }
  let i = REP.i;
  while (i + 1 < REP.st.length && REP.st[i + 1].t <= REP.tcur) i++;
  if (i !== REP.i) repGo(i); else repUI();
}
function repPlay() {
  if (!REP.on) return;
  if (REP.i >= REP.st.length - 1) repGo(0);
  REP.playing = true; REP.lastTick = performance.now();
  if (!REP.timer) REP.timer = setInterval(repTick, 50);
  $("#rep-play").textContent = "⏸";
}
function repPause() {
  if (REP.timer) clearInterval(REP.timer);
  REP.timer = null; REP.playing = false;
  if ($("#rep-play")) $("#rep-play").textContent = "▶";
}
function repExit() {
  repPause();
  REP.on = false;
  document.body.classList.remove("replay");
  $("#repbar").style.display = "none";
  if ($("#hud").dataset.live) $("#hud").src = $("#hud").dataset.live;
  repReset();
  S = SLIVE; onState();
  history.replaceState(null, "", location.pathname);
  loadRepList();
}
$("#rep-play").onclick = () => REP.playing ? repPause() : repPlay();
$("#rep-back").onclick = () => repSeekT(REP.tcur - 5);
$("#rep-fwd").onclick = () => repSeekT(REP.tcur + 5);
$("#rep-exit").onclick = repExit;
$("#rep-slider").oninput = (e) => repSeekT(REP.t0 + e.target.value / 10);
document.addEventListener("keydown", (e) => {
  if (!REP.on || ["INPUT", "SELECT", "TEXTAREA"].includes(document.activeElement.tagName)) return;
  if (e.key === " ") { e.preventDefault(); REP.playing ? repPause() : repPlay(); }
  else if (e.key === "ArrowRight") repGo(REP.i + (e.shiftKey ? 10 : 1));
  else if (e.key === "ArrowLeft") repGo(REP.i - (e.shiftKey ? 10 : 1));
});

const repFmtT = (s) => { const a = Math.abs(s); return `${s < -0.05 ? "-" : ""}${Math.floor(a / 60)}:${(a % 60).toFixed(1).padStart(4, "0")}`; };
function repUI() {
  const rel = REP.tcur - REP.t0;
  if (!STILL) history.replaceState(null, "", `?run=${encodeURIComponent(REP.name)}&t=${rel.toFixed(1)}&tab=${($("nav .tab.on") || {}).dataset?.tab || "runs"}`);
  $("#rep-slider").value = Math.round(rel * 10);
  const a = (REP.st[REP.i] || {}).ack || {};
  $("#rep-time").textContent = `${repFmtT(rel)} / ${repFmtT(REP.t1 - REP.t0)}`;
  $("#rep-now").textContent = `est ${a.est ?? "—"} · esquina ${fmt(a.tc)}/${fmt(a.tpr)} · pwm ${fmt(a.pw)} · servo ${fmt(a.srv)}`;
  repFrame();
  repEvents(false);
}
// Cuadro del .avi que va con el tiempo actual (uno a la vez: el siguiente se pide al llegar el anterior).
function repFrame() {
  const vi = REP.d.video || {}, n = vi.frames || 0;
  if (!n) return;
  const span = Math.max(0.1, REP.t1 - REP.v0);
  const idx = Math.max(0, Math.min(n - 1, Math.round((REP.tcur - REP.v0) / span * (n - 1))));
  if (idx === REP.frameIdx || REP.frameBusy) return;
  REP.frameIdx = idx; REP.frameBusy = true;
  const url = `/api/runs/${encodeURIComponent(REP.name)}/frame?i=${idx}`;
  const img = new Image();
  img.onload = img.onerror = () => {
    REP.frameBusy = false;
    if (!REP.on) return;
    $("#rep-frame").src = img.src; $("#hud").src = img.src;
    repFrame();   // por si el tiempo ya avanzó
  };
  img.src = url;
}

function repSummary() {
  const s = REP.d.summary || {};
  const tof = (k) => s[k] ? `mín ${fmt(s[k].min / 10, 1)} cm · ${fmt(s[k].lost, 1)} % sin lectura` : "—";
  const ests = Object.entries(s.est_count || {}).map(([k, v]) => `${k}×${v}`).join(" ");
  const rows = [
    ["Datos", s.src === "journal" ? "reconstruida del journal (pose aproximada, sin ruta)" : `telemetría completa (${s.n} estados)`],
    ["Duración", s.armed_s !== undefined ? `${fmt(s.armed_s, 1)} s` : "—"],
    ["Firmware / sentido", `${s.fw === "open" ? "Open" : "Obstáculos"}${REP.d.hdr && REP.d.hdr.surprise ? " + sorpresa" : ""} · ${s.dir === "L" ? "CCW" : s.dir === "R" ? "CW" : "—"}`],
    ["Esquinas", `${fmt(s.tc_max)}/${fmt(s.tpr)}`],
    ["Estado final", `${s.est_end ?? "—"} <span class="muted">(estados: ${ests || "—"})</span>`],
    ["Distancia (odómetro)", s.dist_m !== undefined ? `${fmt(s.dist_m, 2)} m` : "—"],
    ["ToF L", tof("tL")], ["ToF R", tof("tR")], ["ToF F", tof("tF")], ["ToF B", tof("tB")],
    ["UART (errores en la corrida)", s.ue !== undefined ? `ue +${fmt(s.ue)} · uf +${fmt(s.uf)}` : "—"],
    ["Park", (s.park || []).map((l) => l.replace(/^\[PARK[^\]]*\]\s*/, "")).join("<br>") || "—"],
    ["Errores", (s.errors || []).length ? s.errors.join("<br>") : "ninguno"],
  ];
  $("#rep-sum").innerHTML = rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join("");
}

function repEvents(rebuild) {
  const box = $("#rep-ev");
  if (rebuild) {
    const q = $("#rep-ev-q").value.trim().toLowerCase(), esp = $("#rep-ev-esp").checked;
    REP.evIdx = [];
    const html = [];
    REP.ev.forEach((e, k) => {
      if (!esp && e[2] === "esp") return;
      if (q && !e[1].toLowerCase().includes(q)) return;
      const err = /AVISO|Traceback|Error/.test(e[1]);
      REP.evIdx.push(k);
      html.push(`<div data-k="${k}" class="${e[2] === "esp" ? "esp" : ""}${err ? " err" : ""}"><span class="t">${repFmtT(e[0] - REP.t0)}</span>${e[2] === "esp" ? "ESP " : ""}${e[1].replace(/</g, "&lt;")}</div>`);
    });
    box.innerHTML = html.join("") || '<span class="muted">sin eventos</span>';
    $$("#rep-ev div[data-k]").forEach((d) => d.onclick = () => { const e = REP.ev[+d.dataset.k]; repSeekT(e[0]); });
    REP.evShown = -1;
  }
  // el último evento que ya pasó
  let cur = -1;
  for (const k of REP.evIdx || []) { if (REP.ev[k][0] <= REP.tcur) cur = k; else break; }
  if (cur === REP.evShown) return;
  const old = box.querySelector("div.cur"); if (old) old.classList.remove("cur");
  REP.evShown = cur;
  const el = cur >= 0 ? box.querySelector(`div[data-k="${cur}"]`) : null;
  if (el) {
    el.classList.add("cur");
    if ($("#rep-ev-follow").checked) box.scrollTop = el.offsetTop - box.offsetTop - box.clientHeight / 2;
  }
}
$("#rep-ev-q").oninput = () => REP.on && repEvents(true);
$("#rep-ev-esp").onchange = () => REP.on && repEvents(true);

function repDrawChart() {
  const cv = $("#rep-chart"), g = cv.getContext("2d"), W = cv.width, H = cv.height;
  g.clearRect(0, 0, W, H);
  if (!REP.on) { g.fillStyle = "#8a94a3"; g.font = "20px system-ui"; g.fillText("Elige una corrida para ver sus gráficas", 20, 40); return; }
  const L = 46, R = W - 10, top1 = 10, h1 = H * 0.58, top2 = top1 + h1 + 14, h2 = H - top2 - 22;
  const X = (t) => L + (t - REP.t0) / Math.max(0.1, REP.t1 - REP.t0) * (R - L);
  g.font = "12px system-ui"; g.lineWidth = 1;
  // franjas por estado del ESP
  const estCol = { S: "#ffffff06", I: "#2fbf7118", E: "#e0a52618", G: "#4c9aff18", T: "#e5484d18" };
  let s0 = 0;
  for (let j = 1; j <= REP.st.length; j++) {
    const a = j < REP.st.length ? ((REP.st[j].ack || {}).est) : null, b = (REP.st[s0].ack || {}).est;
    if (j === REP.st.length || a !== b) {
      if (b && estCol[b]) { g.fillStyle = estCol[b]; g.fillRect(X(REP.st[s0].t), top1, X(REP.st[Math.min(j, REP.st.length - 1)].t) - X(REP.st[s0].t), H - top1 - 20); }
      s0 = j;
    }
  }
  // rejilla
  g.strokeStyle = "#2b313b"; g.fillStyle = "#8a94a3";
  for (const cm of [0, 50, 100, 150]) { const y = top1 + h1 - cm / 150 * h1; g.beginPath(); g.moveTo(L, y); g.lineTo(R, y); g.stroke(); g.fillText(`${cm}`, 6, y + 4); }
  for (const v of [0, 128, 255]) { const y = top2 + h2 - v / 255 * h2; g.beginPath(); g.moveTo(L, y); g.lineTo(R, y); g.stroke(); g.fillText(`${v}`, 6, y + 4); }
  const dur = REP.t1 - REP.t0, step = dur > 120 ? 20 : dur > 40 ? 10 : 5;
  for (let s = 0; s <= dur; s += step) { const x = X(REP.t0 + s); g.fillText(`${s}s`, x - 8, H - 5); }
  // eventos importantes
  for (const e of REP.ev) {
    const c = /\[PARK/.test(e[1]) ? "#ff4d4f" : /\[ROUTE\] (plan#|arranque)/.test(e[1]) ? "#4c9aff" : /AVISO|Error|Traceback/.test(e[1]) ? "#e5484d" : null;
    if (!c) continue;
    g.strokeStyle = c + "aa"; g.beginPath(); g.moveTo(X(e[0]), top1); g.lineTo(X(e[0]), H - 20); g.stroke();
  }
  const series = (key, col, y0, hh, max, scale = 1) => {
    g.strokeStyle = col; g.lineWidth = 1.6; g.beginPath(); let pen = false;
    for (const s of REP.st) {
      const v = (s.ack || {})[key];
      if (!(typeof v === "number") || v < 0) { pen = false; continue; }
      const y = y0 + hh - Math.min(max, v * scale) / max * hh;
      if (pen) g.lineTo(X(s.t), y); else g.moveTo(X(s.t), y);
      pen = true;
    }
    g.stroke();
  };
  series("tL", css("--accent"), top1, h1, 150, 0.1); series("tR", css("--green"), top1, h1, 150, 0.1);
  series("tF", css("--ray"), top1, h1, 150, 0.1); series("tB", css("--pink"), top1, h1, 150, 0.1);
  series("pw", css("--orange"), top2, h2, 255); series("srv", "#e6e9ee", top2, h2, 255);
  // cursor
  const x = X(REP.tcur);
  g.strokeStyle = "#fff"; g.lineWidth = 2; g.beginPath(); g.moveTo(x, top1); g.lineTo(x, H - 20); g.stroke();
}
$("#rep-chart").onclick = (ev) => {
  if (!REP.on) return;
  const cv = $("#rep-chart"), r = cv.getBoundingClientRect(), px = (ev.clientX - r.left) * cv.width / r.width;
  const L = 46, R = cv.width - 10;
  repSeekT(REP.t0 + (px - L) / (R - L) * (REP.t1 - REP.t0));
};

// Enlace directo a un momento de una corrida: ?run=orillas242.avi&t=17&tab=live (t en s desde el arranque).
(async () => {
  const q = new URLSearchParams(location.search), run = q.get("run"), tab = q.get("tab");
  if (run) {
    await repOpen(run);
    if (REP.on && q.get("t")) repSeekT(REP.t0 + parseFloat(q.get("t")));
  }
  if (tab) { const b = document.querySelector(`nav .tab[data-tab="${tab}"]`); if (b) b.click(); }
  if (run) setTimeout(() => REP.on && repGo(REP.i), 1500);   // otra vez, ya con la geometría de la pista
})();
