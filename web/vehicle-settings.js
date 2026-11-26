const body = [
  ["suspension.vehicle.m", "Total Vehicle Mass", "kg", 1],
  ["suspension.vehicle.h_cg", "CG Height", "m", 0],
  ["suspension.vehicle.lf", "CG to Front Axle", "m", .01],
  ["suspension.vehicle.lr", "CG to Rear Axle", "m", .01],
  ["suspension.vehicle.tf", "Front Track Width", "m", .01],
  ["suspension.vehicle.tr", "Rear Track Width", "m", .01],
  ["suspension.vehicle.Ixx", "Roll Inertia", "kg·m²", .01],
  ["suspension.vehicle.Iyy", "Pitch Inertia", "kg·m²", .01],
  ["vehicle.yaw_inertia_kg_m2", "Yaw Inertia", "kg·m²", .01],
];

const power = [
  ["vehicle.dc_voltage", "DC Bus Voltage", "V", 1],
  ["vehicle.gear_ratio", "Gear Reduction Ratio", "", .01],
  ["vehicle.gear_efficiency", "Drivetrain Efficiency", "", .01, 1],
  ["vehicle.aero.enabled", "Enable Aerodynamics", "", null, null, true],
  ["vehicle.aero.coefficients.CL0", "Baseline Modal CL", ""],
  ["vehicle.aero.coefficients.CD0", "Baseline Modal CD", ""],
  ["vehicle.aero.coefficients.front_balance", "Modal Front Downforce Share", "", 0, 1],
];

const brake = [
  ["vehicle.max_pedal_force_n", "Max. Pedal Force", "N", 0],
  ["vehicle.front_brake_bias", "Front Balance Bar Split", "", 0, 1],
  ["vehicle.hydraulic_brake.pedal_ratio", "Pedal Ratio", "", .01],
  ["vehicle.hydraulic_brake.pad_mu", "Pad Friction Coefficient", "", 0],
  ...["front", "rear"].flatMap((axle, index) => {
    const label = index === 0 ? "Front" : "Rear";
    return [
      [`vehicle.hydraulic_brake.${axle}_master_diameter_m`, `${label} Master Cylinder Bore`, "m", .001],
      [`vehicle.hydraulic_brake.${axle}_caliper_diameter_m`, `${label} Caliper Piston Diameter`, "m", .001],
      [`vehicle.hydraulic_brake.${axle}_disc_diameter_m`, `${label} Effective Disc Diameter`, "m", .001],
      [`vehicle.hydraulic_brake.${axle}_pistons`, `${label} Caliper Piston Count`, "", 1, null, false, 1],
    ];
  }),
];

const axles = [
  { end: "front", label: "Front Wheels", corner: "FL", index: 0 },
  { end: "rear", label: "Rear Wheels", corner: "RL", index: 2 },
];
const sharedWheels = [
  ["vehicle.tire_pressure_kpa.0", "Tire Pressure", "kPa", 1],
  ["vehicle.wheel_inertia_kg_m2.0", "Wheel Rotational Inertia", "kg·m²", .01],
  ["suspension.tires.FL.k", "Vertical Stiffness", "N/mm", .01],
  ["suspension.tires.FL.c", "Vertical Damping", "N·s/m", 0],
];

function wheel({ end, corner, index }) {
  return [
    [`vehicle.${end}_free_radius_m`, "Free Radius", "m", .01],
    [`vehicle.static_camber_deg.${index}`, "Static Camber", "°", -20, 20],
    [`vehicle.drive_scale.${index}`, "Drive Torque Scale", "", 0, 1],
    [`suspension.tires.${corner}.mu`, "Unsprung Mass", "kg", .01],
  ];
}

const channelNames = {
  heave_f: "Front Heave", heave_r: "Rear Heave", roll_f: "Front Roll", roll_r: "Rear Roll",
  FL: "Front Direct-Acting", RL: "Rear Direct-Acting",
};
const dampingDirections = { compression_coefficient: "compression", rebound_coefficient: "rebound" };
const dampingNumber = new Intl.NumberFormat("en", { maximumSignificantDigits: 5 });

function channel(name) {
  return [
    [`suspension.channels.${name}.main_rate`, "Main Spring Rate", "N/mm", 0],
    [`suspension.channels.${name}.mr_fixed`, "Fixed Motion Ratio", "", .000001],
    [`suspension.channels.${name}.compression_coefficient`, "Compression Damping", "N·s/m", 0],
    [`suspension.channels.${name}.rebound_coefficient`, "Rebound Damping", "N·s/m", 0],
    [`suspension.channels.${name}.jounce_mm`, "Bump Stop Travel", "mm", 0],
    [`suspension.channels.${name}.rebound_mm`, "Rebound Limit", "mm", 0],
    [`suspension.channels.${name}.rebound_rate`, "Rebound Stop Rate", "N/mm", 0],
  ];
}

function makeFields(specifications) {
  const grid = document.createElement("div");
  grid.className = "field-grid advanced";
  for (const [key, caption, unit, min, max, boolean, step, displayScale] of specifications) {
    const label = document.createElement("label");
    label.append(caption);
    const input = document.createElement(boolean ? "select" : "input");
    input.dataset.setting = key;
    if (displayScale) input.dataset.displayScale = displayScale;
    if (boolean) {
      input.append(new Option("Use Source Setting", ""), new Option("Enabled", "true"), new Option("Disabled", "false"));
      label.append(input);
    } else {
      input.type = "number";
      input.step = step ?? "any";
      if (min != null) input.min = min;
      if (max != null) input.max = max;
      const wrapper = document.createElement("span");
      wrapper.className = "input-unit";
      wrapper.append(input);
      if (unit) {
        const suffix = document.createElement("span");
        suffix.textContent = unit;
        wrapper.append(suffix);
      }
      label.append(wrapper);
      if (dampingDirections[key.split(".").at(-1)]) {
        const hint = document.createElement("small");
        hint.className = "damper-reference";
        hint.id = `${key}-reference`;
        input.setAttribute("aria-describedby", hint.id);
        label.append(hint);
      }
    }
    grid.append(label);
  }
  return grid;
}

function makeGroup(title, specifications) {
  const details = document.createElement("details");
  const summary = document.createElement("summary");
  summary.textContent = title;
  details.append(summary, makeFields(specifications));
  return details;
}

export function setupVehicleSettings(container, form) {
  const wheels = makeGroup("Tires and Wheels (Left–Right Symmetric)", sharedWheels);
  const wheelNote = document.createElement("p");
  wheelNote.className = "field-note";
  wheelNote.textContent = "All four wheels share the settings above. For imported data, shared settings use the front-left wheel and axle settings use the left wheel of each axle.";
  wheels.append(wheelNote);
  axles.forEach(axle => wheels.append(makeGroup(axle.label, wheel(axle))));
  const suspension = makeGroup("Suspension and Geometry", [
    ["vehicle.suspension_geometry.load_transfer_enabled", "Roll Center and Anti Load Transfer", "", null, null, true],
    ["suspension.vehicle.h_rc_f", "Front Roll Center Height", "mm", -10000, 10000, false, "any", 1000],
    ["suspension.vehicle.h_rc_r", "Rear Roll Center Height", "mm", -10000, 10000, false, "any", 1000],
    ["suspension.vehicle.dive_front", "Front Anti-Dive Under Braking", "%", -200, 200, false, "any", 100],
    ["suspension.vehicle.lift_rear", "Rear Anti-Lift Under Braking", "%", -200, 200, false, "any", 100],
    ["suspension.vehicle.lift_front", "Front Anti-Lift Under Acceleration", "%", -200, 200, false, "any", 100],
    ["suspension.vehicle.squat_rear", "Rear Anti-Squat Under Acceleration", "%", -200, 200, false, "any", 100],
  ]);
  const geometryNote = document.createElement("p");
  geometryNote.className = "field-note";
  geometryNote.textContent = "Enable load transfer to apply roll center and anti geometry. Roll center height is measured from the road. Anti values are percentages of total vehicle load transfer.";
  const camber = makeGroup("Camber Gain (per 25 mm of Compression)", axles.map(({ label, index }) => [
    `vehicle.suspension_geometry.camber_gain_deg_per_25mm.${index}`, label, "°", -30, 30,
  ]));
  const camberNote = document.createElement("p");
  camberNote.className = "field-note";
  camberNote.textContent = "Negative values tilt the top of the wheel inward under compression.";
  camber.append(camberNote);
  const tableNote = document.createElement("p");
  tableNote.className = "field-note";
  tableNote.hidden = true;
  const springNote = document.createElement("p");
  springNote.className = "field-note";
  springNote.textContent = "A spring rate adjusts the main spring or replaces its lookup table. Enter a fixed damping coefficient for either direction, or leave it blank to keep the reference curve. Settings apply to enabled channels.";
  suspension.append(geometryNote, tableNote, camber, springNote);
  for (const [name, label] of Object.entries(channelNames)) suspension.append(makeGroup(label, channel(name)));
  container.append(makeGroup("Body and Center of Gravity", body), makeGroup("Powertrain and Aerodynamics", power), wheels, makeGroup("Hydraulic Brakes", brake), suspension);
  const inputs = Array.from(container.querySelectorAll("[data-setting]"));
  const defaults = fetch(new URL("./vehicle-defaults.json", import.meta.url)).then(response => {
    if (!response.ok) throw new Error();
    return response.json();
  }).catch(() => ({}));
  let versions = 0;

  async function refreshPlaceholders() {
    const version = ++versions;
    const settings = await defaults;
    const profiles = await Promise.all(["vehicle", "suspension"].map(async key => {
      const file = form.elements.namedItem(key).files[0];
      if (!file) return [key, settings[key]];
      try {
        return [key, { ...settings[key], ...JSON.parse((await file.text()).replace(/^\uFEFF/, "")) }];
      } catch {
        return [key, settings[key]];
      }
    }));
    if (version !== versions) return;
    const reference = Object.fromEntries(profiles);
    const defaultSuspension = reference.suspension?.channels === settings.suspension?.channels;
    const tables = reference.vehicle?.suspension_geometry?.tables;
    const fixedGeometry = {
      roll_center: !!tables?.roll_center && !tables.roll_center.relative,
      anti: !!tables?.anti && !tables.anti.relative,
      camber: !!tables?.camber,
    };
    const locked = [];
    if (fixedGeometry.roll_center) locked.push("Roll Center");
    if (fixedGeometry.anti) locked.push("Anti");
    if (fixedGeometry.camber) locked.push("Camber Gain");
    tableNote.hidden = !locked.length;
    tableNote.textContent = locked.length ? `Imported lookup tables control ${locked.join(", ")}.` : "";
    for (const input of inputs) {
      const path = input.dataset.setting;
      const parts = path.split(".");
      const scale = Number(input.dataset.displayScale || 1);
      const isCamber = parts.includes("camber_gain_deg_per_25mm");
      const isMotion = parts.at(-1) === "mr_fixed";
      const isSpring = parts.at(-1) === "main_rate";
      const direction = dampingDirections[parts.at(-1)];
      const readPath = isMotion ? [...parts.slice(0, -1), "mr"] : parts;
      let value = readPath.reduce((object, key) => object?.[key], reference);
      if (path === "vehicle.front_free_radius_m" || path === "vehicle.rear_free_radius_m") {
        value ??= reference.vehicle?.free_radius_m;
      }
      if (path.startsWith("suspension.vehicle.h_rc_")) input.disabled = fixedGeometry.roll_center;
      if (parts[0] === "suspension" && parts[1] === "vehicle" && scale === 100) {
        input.disabled = fixedGeometry.anti;
      }
      if (isCamber) {
        input.disabled = fixedGeometry.camber;
        value ??= 0;
      }
      if (isMotion) value = value?.length > 1 ? "Use Source Curve" : value?.[0]?.[1];
      if (isSpring) {
        const kind = reference.suspension?.channels?.[parts[2]]?.spring_kind;
        input.min = kind === "tender" ? .000001 : 0;
        if (kind === "stages" || kind === "force") value = "Use Source Curve";
      }
      if (direction) {
        const name = parts[2];
        const channel = reference.suspension?.channels?.[name];
        const curve = channel?.[direction];
        const hint = input.closest("label").querySelector(".damper-reference");
        hint.textContent = "";
        if (Array.isArray(curve) && curve.length) {
          const critical = defaultSuspension ? settings.critical_damping?.[name] : undefined;
          const ratio = channel.damper_ratio && critical == null;
          const force = (channel.damper_kind?.[direction] ?? channel.damper_kind) === "force";
          const unit = force ? "N" : "N·s/m";
          const values = curve.slice(0, 2)
            .filter(row => Array.isArray(row) && row.length === 2 && row.every(Number.isFinite))
            .map(([speed, raw]) => {
              const amount = dampingNumber.format(raw * (critical ?? 1));
              const damping = ratio ? `ζ ${amount}` : `${amount} ${unit}`;
              return curve.length === 1 ? damping : `${damping} ${force ? "at" : "from"} ${dampingNumber.format(speed)} m/s`;
            });
          if (curve.length > 2) values.push(`+${curve.length - 2} more`);
          if (values.length) hint.textContent = `${defaultSuspension ? "LP03 default" : "Imported"}: ${values.join(" · ")}`;
          if (values.length && !force && !ratio && curve.length === 1) value = curve[0][1] * (critical ?? 1);
        }
      }
      if (input.tagName === "SELECT") {
        input.options[0].textContent = typeof value === "boolean" ? `Use Source Setting (${value ? "Enabled" : "Disabled"})` : "Use Source Setting";
      } else {
        input.placeholder = value == null ? ""
          : String(typeof value === "number" ? Number((value * scale).toPrecision(7)) : value);
      }
    }
  }

  form.addEventListener("change", event => {
    if (["vehicle", "suspension"].includes(event.target.name)) refreshPlaceholders();
  });
  refreshPlaceholders();
  return () => Object.fromEntries(inputs.filter(input => !input.disabled && input.value !== "").map(input => [
    input.dataset.setting,
    input.tagName === "SELECT" ? input.value === "true" : input.valueAsNumber / Number(input.dataset.displayScale || 1),
  ]));
}
