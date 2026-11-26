const runtimeUrl = "https://cdn.jsdelivr.net/pyodide/v0.27.7/full/";
const inputNames = {
  vehicle: "vehicle.json", suspension: "suspension.json", tire: "tire.tir",
  motor: "motor.csv", track: "track.csv", aero: "aero.json",
};
const outputNames = ["telemetry.csv", "telemetry_summary.csv", "run.zip"];
const work = "/tmp/vehicle14dof";
let runtime;

async function initialize() {
  const [module, archive] = await Promise.all([
    import(runtimeUrl + "pyodide.mjs"),
    fetch(new URL("./solver.zip", import.meta.url)).then(response => {
      if (!response.ok) throw new Error("Could not load the solver package.");
      return response.arrayBuffer();
    }),
  ]);
  const python = await module.loadPyodide({ indexURL: runtimeUrl, stdout: console.log, stderr: console.warn });
  await python.loadPackage("numpy");
  python.unpackArchive(archive, "zip", { extractDir: "/solver" });
  python.runPython("import sys\nsys.path.insert(0, '/solver')\nimport browser_solver");
  python.FS.mkdirTree(work + "/uploads");
  return python;
}

self.onmessage = async ({ data: { options, files } }) => {
  try {
    if (!runtime) {
      self.postMessage({ kind: "loading" });
      runtime = initialize().catch(error => {
        runtime = undefined;
        throw new Error(`Could not load the browser solver. Check your connection to jsDelivr. ${error.message}`);
      });
    }
    const python = await runtime;
    for (const [key, name] of Object.entries(inputNames)) {
      const destination = work + "/uploads/" + name;
      if (python.FS.analyzePath(destination).exists) python.FS.unlink(destination);
      if (files[key]) python.FS.writeFile(destination, new Uint8Array(files[key]));
    }
    self.postMessage({ kind: "solving" });
    python.globals.set("web_payload", JSON.stringify(options));
    python.globals.set("web_progress", (lap, total, time, distance, length) =>
      self.postMessage({ kind: "progress", lap, total, time, distance, length }));
    try {
      const result = JSON.parse(python.runPython(
        `browser_solver.run(web_payload, web_progress, "${work}")`,
      ));
      const downloads = Object.fromEntries(outputNames.map(name => [
        name, python.FS.readFile(work + "/" + name).buffer,
      ]));
      self.postMessage({ kind: "result", result, downloads }, Object.values(downloads));
    } finally {
      python.globals.delete("web_payload");
      python.globals.delete("web_progress");
      for (const name of outputNames) {
        const path = work + "/" + name;
        if (python.FS.analyzePath(path).exists) python.FS.unlink(path);
      }
    }
  } catch (error) {
    const detail = String(error.message || error).trim().split("\n").at(-1);
    const message = /\p{Script=Han}/u.test(detail)
      ? "The solver could not complete the simulation. Check the imported files and vehicle settings."
      : detail;
    console.error(error);
    self.postMessage({ kind: "error", message });
  }
};
