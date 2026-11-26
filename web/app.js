import { extent, routeView, timeView } from "./charts.js";
import { replayView } from "./replay.js";
import { setupVehicleSettings } from "./vehicle-settings.js";

const byId = id => document.getElementById(id);
const form = byId("setup");
const field = name => form.elements.namedItem(name);
const modes = {
  skidpad: { name: "9.125 m Skidpad", description: "Fixed Radius", fields: { initial: 25.2, lateral: 5.5 / 9.81, warmup: 1 } },
  straight: { name: "75 m Straight-Line Acceleration", description: "Full Throttle", fields: { initial: 0 } },
  custom: { name: "Custom Track", description: "Import a closed LINE/ARC track CSV", fields: { initial: 0, lateral: 5.5 / 9.81, warmup: 0 } },
};
const fileFields = ["vehicle", "suspension", "tire", "motor", "track", "aero"];
const vehicleSettings = setupVehicleSettings(byId("vehicle-fields"), form);
const progress = byId("progress");
const exportData = byId("export-data");
const downloadZip = byId("download-zip");
const paintRoute = routeView(byId("map"));
const paintTime = timeView(byId("chart"), byId("chart-legend"));
const paintReplay = replayView(byId("vehicle-scene"), byId("gg-diagram"));
const emptyCourse = [];
let courses = { straight: { length: 75 } };
let selectedMode = "skidpad";
const modeFields = {};
let activeJob;
let worker;
let result;
let cursor = 0;
let animation = 0;
let revision = 0;
let downloadUrls = [];

function status(message, tone = "") {
  byId("status").textContent = message;
  byId("status").dataset.tone = tone;
}

function updateBusy(busy) {
  byId("run").disabled = busy;
  byId("stop").hidden = !busy;
}

function toggleFields(selector, show) {
  for (const row of form.querySelectorAll(selector)) {
    row.hidden = !show;
    row.querySelector("input").disabled = !show;
  }
}

function courseName(mode) {
  return mode === "custom" && field("track").files[0]
    ? field("track").files[0].name : modes[mode].name;
}

function updateCourse() {
  const mode = selectedMode;
  const length = courses[mode]?.length;
  byId("course-name").textContent = courseName(mode);
  byId("course-meta").textContent = length > 0
    ? `${modes[mode].description} · ${length.toFixed(2)} m` : modes[mode].description;
  if (!result) {
    byId("timing-label").textContent = mode === "straight" ? "Elapsed Time" : "Best Lap";
    byId("distance").textContent = length > 0 ? length.toFixed(2) : "—";
    byId("map-caption").textContent = courseName(mode);
  }
}

function selectMode(mode, restore = true) {
  if (restore) {
    modeFields[selectedMode] = Object.fromEntries(Object.keys(modes[selectedMode].fields).map(key => [key, field(key).value]));
  }
  selectedMode = mode;
  for (const [key, value] of Object.entries(modeFields[mode] || modes[mode].fields)) field(key).value = value;
  toggleFields("[data-closed]", mode !== "straight");
  byId("track-import").hidden = field("track").disabled = mode !== "custom";
  updateCourse();
}

function redraw() {
  paintRoute(result?.centerline || emptyCourse, result?.series, cursor);
  paintTime(`${revision}:${byId("signal").value}`, result?.series, byId("signal").value, cursor, result?.warmupEnd);
  paintReplay(result, cursor, byId("camera").value === "overhead");
}

function seek(index) {
  if (!result) return;
  cursor = index;
  byId("scrub").value = index;
  byId("play-time").textContent = `${result.series.time_s[index].toFixed(2)} s`;
  const series = result.series;
  byId("replay-speed").textContent = `${(series.speed_mps[index] * 3.6).toFixed(1)} km/h`;
  byId("replay-ax").textContent = `${(series.ax_mps2[index] / 9.81).toFixed(2)} g`;
  byId("replay-ay").textContent = `${(series.ay_mps2[index] / 9.81).toFixed(2)} g`;
  for (const pedal of ["throttle", "brake"]) {
    byId(pedal).value = series[pedal][index];
    byId(`${pedal}-value`).textContent = `${(100 * series[pedal][index]).toFixed(0)}%`;
  }
  redraw();
}

function pause() {
  if (animation) cancelAnimationFrame(animation);
  animation = 0;
  byId("play").textContent = "Play";
}

function play() {
  if (!result) return;
  if (animation) return pause();
  const times = result.series.time_s;
  if (cursor === times.length - 1) seek(0);
  const startTime = times[cursor], started = performance.now(), rate = Number(byId("play-rate").value);
  let index = cursor;
  byId("play").textContent = "Pause";
  function tick(now) {
    const time = startTime + rate * (now - started) / 1000;
    while (index + 1 < times.length && times[index + 1] <= time) index++;
    if (cursor !== index) seek(index);
    if (index < times.length - 1) animation = requestAnimationFrame(tick);
    else pause();
  }
  animation = requestAnimationFrame(tick);
}

function tableRow(values, tag) {
  const row = document.createElement("tr");
  for (const value of values) {
    const cell = document.createElement(tag);
    if (tag === "th") cell.scope = "col";
    cell.textContent = value;
    row.append(cell);
  }
  return row;
}

function showResult(data, downloads) {
  pause();
  result = data;
  revision++;
  const straight = data.mode === "straight";
  const [slowest, fastest] = extent(data.series.speed_mps);
  const [lateralMin, lateralMax] = extent(data.series.ay_mps2);
  byId("timing-label").textContent = straight ? "Elapsed Time" : "Best Lap";
  byId("time").textContent = extent(data.times)[0].toFixed(3);
  byId("peak-speed").textContent = (3.6 * fastest).toFixed(2);
  byId("peak-lateral").textContent = (Math.max(-lateralMin, lateralMax) / 9.81).toFixed(2);
  byId("distance").textContent = data.length.toFixed(2);
  byId("map-caption").textContent = `${data.courseName} · Simulation Results`;
  byId("min-speed").textContent = (3.6 * slowest).toFixed(1);
  byId("max-speed").textContent = (3.6 * fastest).toFixed(1);
  byId("results").removeAttribute("data-empty");
  byId("map-legend").hidden = byId("replay-controls").disabled = byId("signal").disabled = false;
  byId("scrub").max = data.series.time_s.length - 1;
  byId("timing-head").replaceChildren(tableRow(straight
    ? ["Distance (m)", "Time (s)", "Finish Speed (km/h)"] : ["Timed Lap", "Lap Time (s)", "Start (s)", "Finish (s)"], "th"));
  byId("timing-body").replaceChildren(...data.timing.map(row => tableRow(straight
    ? [row.distance_m.toFixed(2), row.elapsed_time_s.toFixed(3), row.exit_speed_kmh.toFixed(2)]
    : [row.lap, row.lap_time_s.toFixed(3), row.start_time_s.toFixed(3), row.finish_time_s.toFixed(3)], "td")));
  byId("run-deviation").textContent = `Maximum centerline deviation: ${data.deviation.toFixed(3)} m`;
  downloadUrls.forEach(url => URL.revokeObjectURL(url));
  downloadUrls = [];
  for (const link of document.querySelectorAll("[data-download]")) {
    const name = link.dataset.download;
    const mime = name.endsWith(".zip") ? "application/zip" : "text/csv;charset=utf-8";
    link.href = URL.createObjectURL(new Blob([downloads[name]], { type: mime }));
    downloadUrls.push(link.href);
  }
  byId("exports").hidden = false;
  exportData.disabled = false;
  seek(0);
  if (!window.matchMedia("(prefers-reduced-motion: reduce)").matches) play();
}

function finish(message, tone) {
  activeJob = undefined;
  updateBusy(false);
  progress.value = tone === "success" ? 100 : 0;
  status(message, tone);
}

function createWorker() {
  const instance = new Worker(new URL("./worker.js", import.meta.url), { type: "module" });
  instance.onmessage = ({ data }) => {
    if (worker !== instance || !activeJob) return;
    if (data.kind === "loading") status("Downloading Python and NumPy…");
    if (data.kind === "solving") status("Loading the vehicle and solving for initial equilibrium…");
    if (data.kind === "progress") {
      progress.value = 100 * Math.min(.999, Math.max(0, data.distance / (data.length * data.total)));
      const stage = activeJob.mode === "straight" ? "Straight-Line Acceleration" : `Lap ${data.lap} of ${data.total}`;
      status(`${stage} · ${progress.value.toFixed(1)}% · ${data.time.toFixed(1)} s simulated`);
    }
    if (data.kind === "result") {
      showResult(data.result, data.downloads);
      finish("Simulation complete", "success");
    }
    if (data.kind === "error") finish(`Simulation failed · ${data.message}`, "error");
  };
  instance.onerror = () => {
    if (worker !== instance) return;
    instance.terminate();
    worker = undefined;
    finish("Could not start the solver. Open this page over HTTP or HTTPS.", "error");
  };
  return instance;
}

form.addEventListener("submit", async event => {
  event.preventDefault();
  if (activeJob) return;
  const mode = selectedMode;
  const job = { mode, courseName: courseName(mode), overrides: vehicleSettings() };
  for (const key of ["initial", "step", "limit", ...(mode === "straight"
    ? [] : ["width", "speed", "lateral", "laps", "warmup"])]) job[key] = field(key).valueAsNumber;
  activeJob = job;
  updateBusy(true);
  progress.value = 0;
  status("Preparing inputs…");
  try {
    const files = {};
    for (const key of fileFields) {
      const file = field(key).disabled ? null : field(key).files[0];
      if (file) files[key] = file;
    }
    const contents = Object.fromEntries(await Promise.all(Object.entries(files).map(async ([key, file]) => [key, await file.arrayBuffer()])));
    if (activeJob !== job) return;
    worker ??= createWorker();
    worker.postMessage({ options: job, files: contents }, Object.values(contents));
  } catch (error) {
    if (activeJob === job) finish(`Could not run the simulation · ${error.message}`, "error");
  }
});

form.addEventListener("invalid", event => {
  for (let details = event.target.closest("details"); details; details = details.parentElement.closest("details")) {
    details.open = true;
  }
}, true);
form.addEventListener("change", event => {
  if (event.target.name === "mode") selectMode(event.target.value);
  if (event.target.name === "track") updateCourse();
});
byId("stop").addEventListener("click", () => {
  worker?.terminate();
  worker = undefined;
  finish("Simulation stopped");
});
exportData.addEventListener("click", () => downloadZip.click());
byId("signal").addEventListener("change", redraw);
byId("play").addEventListener("click", play);
byId("restart").addEventListener("click", () => { pause(); seek(0); });
byId("camera").addEventListener("change", redraw);
byId("play-rate").addEventListener("change", () => {
  if (animation) { pause(); play(); }
});
byId("scrub").addEventListener("input", event => {
  pause();
  seek(Number(event.target.value));
});

const resize = new ResizeObserver(redraw);
resize.observe(byId("map").parentElement);
resize.observe(byId("chart").parentElement);
resize.observe(byId("vehicle-scene").parentElement);
resize.observe(byId("gg-diagram"));
selectMode(field("mode").value, false);
fetch(new URL("./courses.json", import.meta.url)).then(response => {
  if (!response.ok) throw new Error("Could not load course data.");
  return response.json();
}).then(data => {
  courses = data;
  updateCourse();
}).catch(error => {
  if (!activeJob && !result) status(error.message, "error");
});
