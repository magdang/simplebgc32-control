/*
 * test_web_page.js — regression tests for web/index.html's control logic.
 *
 * The page is a single self-contained file with no build step and no module
 * boundary, so there is nothing to import. Instead the real <script> block is
 * evaluated against a minimal DOM shim and driven with synthetic events. That
 * keeps the tests honest: they exercise the shipped code rather than a copy of
 * it, and a change to the page is a change to what is under test.
 *
 * The shim implements only what the page actually touches. It is deliberately
 * dumb — anything it cannot answer should fail loudly rather than be faked
 * into looking like it worked.
 *
 * Each case corresponds to a defect that was found and fixed.
 *
 *   node test/test_web_page.js
 */

"use strict";

const fs = require("fs");
const path = require("path");
const vm = require("vm");

const ROOT = path.dirname(__dirname);
const HTML = fs.readFileSync(path.join(ROOT, "web", "index.html"), "utf8");
const SCRIPT = HTML.match(/<script[^>]*>([\s\S]*?)<\/script>/)[1];

/* ------------------------------------------------------------- DOM shim -- */

function mkEl(id) {
  const listeners = {};
  const el = {
    id, value: "", checked: false, textContent: "", innerHTML: "",
    className: "", disabled: false, hidden: false, style: {},
    clientWidth: 300, dataset: {}, options: [],
    classList: {
      _s: new Set(),
      add(c) { this._s.add(c); },
      remove(c) { this._s.delete(c); },
      toggle(c, on) { on ? this._s.add(c) : this._s.delete(c); },
      contains(c) { return this._s.has(c); },
    },
    getBoundingClientRect: () => ({ left: 0, top: 0, width: 300, height: 300 }),
    setPointerCapture() {}, releasePointerCapture() {},
    appendChild() {}, focus() {}, remove() {},
    setAttribute() {}, removeAttribute() {},
    querySelector: () => null, querySelectorAll: () => [],
    insertRow: () => mkEl("row"), insertCell: () => mkEl("cell"),
    deleteRow() {}, get rows() { return []; },
    addEventListener(t, f) { (listeners[t] ||= []).push(f); },
    dispatch(t, ev) {
      for (const f of listeners[t] || [])
        f(Object.assign({ type: t, preventDefault() {}, target: el }, ev));
    },
    listeners,
  };
  return el;
}

const els = {};
for (const m of HTML.matchAll(/id="([^"]+)"/g)) els[m[1]] = mkEl(m[1]);

// The action buttons the page finds with querySelectorAll(".actions .btn").
const actionButtons = [...HTML.matchAll(/data-act="([^"]+)"/g)].map(m => {
  const el = els["btn-" + m[1]] = mkEl("btn-" + m[1]);
  el.dataset.act = m[1];
  el.disabled = true;
  return el;
});
els.motorBtn.dataset.act = "motors";
els.clearHomeBtn.dataset.act = "clearhome";

const winListeners = {}, docListeners = {};
const posts = [];
const intervals = [];

const sandbox = {
  console,
  document: {
    getElementById: id => els[id] || (els[id] = mkEl(id)),
    querySelectorAll: sel => (sel === ".actions .btn" ? actionButtons : []),
    addEventListener: (t, f) => { (docListeners[t] ||= []).push(f); },
    createElement: tag => mkEl("new-" + tag),
    createElementNS: (ns, tag) => mkEl("new-" + tag),
    activeElement: null,
    hidden: false,
    body: mkEl("body"),
  },
  addEventListener: (t, f) => { (winListeners[t] ||= []).push(f); },
  setInterval: (fn, ms) => { intervals.push({ fn, ms }); return intervals.length; },
  clearInterval: () => {},
  setTimeout: () => 0,
  clearTimeout: () => {},
  requestAnimationFrame: () => 0,
  performance: { now: () => Date.now() },
  navigator: { getGamepads: () => [] },
  location: { href: "http://127.0.0.1:8080/" },
  fetch(url, opts) {
    if (url === "/api/status")
      return Promise.resolve({ ok: true, json: async () => STATUS });
    if (url === "/api/live")
      return Promise.resolve({ ok: true, json: async () => ({ valid: false }) });
    posts.push({ url, body: (opts && opts.body) || "" });
    return Promise.resolve({ ok: true, status: 200, json: async () => ({ ok: true }) });
  },
};
sandbox.window = sandbox;
vm.createContext(sandbox);
vm.runInContext(SCRIPT, sandbox, { filename: "web/index.html<script>" });

/* --------------------------------------------------------------- driver -- */

const fire = (t, ev) => {
  for (const f of winListeners[t] || [])
    f(Object.assign({ type: t, preventDefault() {}, target: {} }, ev));
};
// One turn of the page's 20 Hz republisher.
const republish = () => { for (const t of intervals) if (t.ms === 50) t.fn(); };
const rates = () => posts.filter(p => p.url === "/api/control/rate");
const movingRates = () => rates().filter(p => !/pan=0\.000&tilt=0\.000/.test(p.body));
const settle = () => new Promise(r => setImmediate(r));

let STATUS = null;
function status(control = {}) {
  return {
    link: { port: "/dev/ttyUSB0", baud: 115200, port_open: true,
            board_responding: true, error: "", frames_rx: 10, timeouts: 0,
            last_frame_age_s: 0.04, ports: [], note: "" },
    board: { known: false },
    profile: { valid: false },
    telemetry: { valid: false, angles: null },
    limits: { source: "built-in default" },
    controller: { present: false },
    control: Object.assign({
      allowed: true, armed: true, moving: false, speed_deg_s: 30,
      calib_running: false, calib_pending: false, calib_skipped: false,
      calib_elapsed: 0, calib_total: 4.5,
      last_cmd: "", last_cmd_hex: "", last_cmd_age: 0,
      limits_on: false, lim_yaw_min: -170, lim_yaw_max: 170,
      lim_pitch_min: -90, lim_pitch_max: 40,
      blocked_yaw: false, blocked_pitch: false, limit_stale: false,
      custom_home: false, home_pitch: 0, home_yaw: 0,
    }, control),
    warnings: [], config_summary: "",
  };
}

let checks = 0, failures = 0;
function check(what, cond, extra = "") {
  checks++;
  if (cond) { console.log("ok   " + what); }
  else { failures++; console.log("FAIL " + what + (extra ? "\n     " + extra : "")); }
}
const section = t => console.log("\n== " + t + " ==");

/* ---------------------------------------------------------------- tests -- */

async function testStopBeatsAHeldControl() {
  /*
   * Posting /api/control/stop is only half a stop. The republisher above
   * resends whatever `intent` still holds, and keydown auto-repeats while the
   * key is physically down, so the rate went straight back within 50 ms and
   * the camera carried on as though nothing had been pressed.
   */
  section("Stop must survive a control that is still held");
  STATUS = status();
  sandbox.render(STATUS);

  fire("keydown", { key: "d", repeat: false });
  republish();
  check("holding a key publishes a rate",
        rates().some(p => /pan=1\.000/.test(p.body)));

  posts.length = 0;
  fire("keydown", { key: " ", repeat: false });     // the panic key
  await settle();
  check("the panic key posts a stop",
        posts.some(p => p.url === "/api/control/stop"));

  posts.length = 0;
  for (let i = 0; i < 8; i++) {
    fire("keydown", { key: "d", repeat: true });    // auto-repeat, key still down
    republish();
  }
  check("no moving rate is republished after Stop", movingRates().length === 0,
        JSON.stringify(movingRates().slice(0, 3)));

  posts.length = 0;
  fire("keyup", { key: "d" });
  fire("keydown", { key: "d", repeat: false });
  republish();
  check("releasing and re-pressing drives again",
        movingRates().length > 0);
  fire("keyup", { key: "d" });
}

function testStopStaysAvailableWhileDisarmed() {
  /*
   * The daemon deliberately lets a stop through without arming. Greying out
   * the panic control took that back.
   */
  section("Stop stays available while disarmed");
  const stopBtn = actionButtons.find(b => b.dataset.act === "stop");
  const homeBtn = actionButtons.find(b => b.dataset.act === "home");

  sandbox.render(status({ armed: false }));
  check("Stop is enabled when disarmed", stopBtn.disabled === false);
  check("other actions are disabled when disarmed", homeBtn.disabled === true);

  sandbox.render(status({ allowed: false, armed: false }));
  check("Stop is disabled in a read-only session", stopBtn.disabled === true);

  sandbox.render(status());
  check("everything is enabled when armed",
        stopBtn.disabled === false && homeBtn.disabled === false);
}

async function testEditedLimitsSurviveThePoll() {
  /*
   * render() rewrote the limit fields from the daemon on every 250 ms poll,
   * exempting only the control that happened to have focus. Tabbing out of one
   * field to type a second restored the first, and Apply then sent back the
   * value that had been restored — so only the last-touched field ever took
   * effect.
   */
  section("Edited limit fields must survive the poll");
  const F = id => els[id];
  STATUS = status({ lim_yaw_min: -170, lim_yaw_max: 170 });
  sandbox.render(STATUS);
  check("the poll populates the fields",
        F("limYawMin").value === -170 && F("limYawMax").value === 170,
        `${F("limYawMin").value} / ${F("limYawMax").value}`);

  F("limYawMin").value = "-90";
  F("limYawMin").dispatch("input");
  sandbox.document.activeElement = F("limYawMax");   // focus moves away
  sandbox.render(STATUS);
  check("an edited field survives once focus moves off it",
        F("limYawMin").value === "-90", "got " + F("limYawMin").value);

  F("limYawMax").value = "90";
  F("limYawMax").dispatch("input");
  F("limOn").checked = true;
  F("limOn").dispatch("change");
  sandbox.document.activeElement = null;
  sandbox.render(STATUS);
  sandbox.render(STATUS);
  check("every edit survives repeated polls",
        F("limYawMin").value === "-90" && F("limYawMax").value === "90" &&
        F("limOn").checked === true,
        `${F("limYawMin").value} / ${F("limYawMax").value} / ${F("limOn").checked}`);
  check("unapplied edits are flagged", F("limNote").hidden === false);

  posts.length = 0;
  await els.limApply.listeners.click[0]({ preventDefault() {} });
  const applied = posts.find(p => p.url === "/api/limits");
  check("Apply sends every edited field",
        !!applied && /yaw_min=-90/.test(applied.body) &&
        /yaw_max=90/.test(applied.body) && /enabled=1/.test(applied.body),
        applied ? applied.body : "nothing was posted");

  sandbox.render(status({ limits_on: true, lim_yaw_min: -90, lim_yaw_max: 90 }));
  check("the fields track the daemon again once applied",
        F("limNote").hidden === true);
}

function testDisarmZeroesIntent() {
  section("Losing the arm state must zero the intent");
  STATUS = status();
  sandbox.render(STATUS);
  fire("keydown", { key: "d", repeat: false });
  posts.length = 0;

  sandbox.render(status({ armed: false }));     // the daemon reports disarmed
  republish();
  check("no rate is published once disarmed", movingRates().length === 0,
        JSON.stringify(movingRates().slice(0, 3)));
  fire("keyup", { key: "d" });
}

(async () => {
  await testStopBeatsAHeldControl();
  testStopStaysAvailableWhileDisarmed();
  await testEditedLimitsSurviveThePoll();
  testDisarmZeroesIntent();

  console.log("\n" + "-".repeat(40));
  console.log(`${checks} checks, ${failures} failures`);
  process.exit(failures ? 1 : 0);
})();
