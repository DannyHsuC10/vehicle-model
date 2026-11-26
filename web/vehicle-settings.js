const body = [
  ["suspension.vehicle.m", "整車質量", "kg", 1],
  ["suspension.vehicle.h_cg", "重心高度", "m", 0],
  ["suspension.vehicle.lf", "重心至前軸", "m", .01],
  ["suspension.vehicle.lr", "重心至後軸", "m", .01],
  ["suspension.vehicle.tf", "前輪距", "m", .01],
  ["suspension.vehicle.tr", "後輪距", "m", .01],
  ["suspension.vehicle.Ixx", "側傾慣量", "kg·m²", .01],
  ["suspension.vehicle.Iyy", "俯仰慣量", "kg·m²", .01],
  ["vehicle.yaw_inertia_kg_m2", "偏航慣量", "kg·m²", .01],
];

const power = [
  ["vehicle.dc_voltage", "匯流排電壓", "V", 1],
  ["vehicle.gear_ratio", "減速比", "", .01],
  ["vehicle.gear_efficiency", "傳動效率", "", .01, 1],
  ["vehicle.aero.enabled", "啟用空力", "", null, null, true],
  ["vehicle.aero.coefficients.CL0", "模態基準 CL", ""],
  ["vehicle.aero.coefficients.CD0", "模態基準 CD", ""],
  ["vehicle.aero.coefficients.front_balance", "模態前軸下壓比例", "", 0, 1],
];

const brake = [
  ["vehicle.max_pedal_force_n", "最大踏板力", "N", 0],
  ["vehicle.front_brake_bias", "前軸平衡桿比例", "", 0, 1],
  ["vehicle.hydraulic_brake.pedal_ratio", "踏板比", "", .01],
  ["vehicle.hydraulic_brake.pad_mu", "來令片摩擦係數", "", 0],
  ...["front", "rear"].flatMap((axle, index) => {
    const label = index === 0 ? "前" : "後";
    return [
      [`vehicle.hydraulic_brake.${axle}_master_diameter_m`, `${label}總泵直徑`, "m", .001],
      [`vehicle.hydraulic_brake.${axle}_caliper_diameter_m`, `${label}卡鉗活塞直徑`, "m", .001],
      [`vehicle.hydraulic_brake.${axle}_disc_diameter_m`, `${label}碟盤有效直徑`, "m", .001],
      [`vehicle.hydraulic_brake.${axle}_pistons`, `${label}卡鉗活塞數`, "", 1, null, false, 1],
    ];
  }),
];

const corners = ["FL", "FR", "RL", "RR"];
function wheel(corner, index) {
  return [
    [`vehicle.tire_pressure_kpa.${index}`, "胎壓", "kPa", 1],
    [`vehicle.wheel_inertia_kg_m2.${index}`, "自轉慣量", "kg·m²", .01],
    [`vehicle.static_camber_deg.${index}`, "靜態外傾", "°", -20, 20],
    [`vehicle.drive_scale.${index}`, "驅動倍率", "", 0, 1],
    [`suspension.tires.${corner}.mu`, "簧下質量", "kg", .01],
    [`suspension.tires.${corner}.k`, "垂向剛性", "N/mm", .01],
    [`suspension.tires.${corner}.c`, "垂向阻尼", "N·s/m", 0],
  ];
}

const channelNames = {
  heave_f: "前軸升沉", heave_r: "後軸升沉", roll_f: "前軸側傾", roll_r: "後軸側傾",
  roll_c: "前後同向側傾", warp: "對角扭擺", FL: "FL 直推", FR: "FR 直推", RL: "RL 直推", RR: "RR 直推",
};

function channel(name) {
  return [
    [`suspension.channels.${name}.spring_scale`, "彈簧曲線倍率", "×", .001],
    [`suspension.channels.${name}.damper_scale`, "阻尼曲線倍率", "×", .001],
    [`suspension.channels.${name}.mr_scale`, "運動比倍率", "×", .001],
    [`suspension.channels.${name}.installed_mm`, "安裝預壓行程", "mm"],
    [`suspension.channels.${name}.jounce_mm`, "觸底行程", "mm", 0],
    [`suspension.channels.${name}.jounce_rate`, "觸底剛性", "N/mm", 0],
    [`suspension.channels.${name}.rebound_mm`, "伸長限位", "mm", 0],
    [`suspension.channels.${name}.rebound_rate`, "伸長限位剛性", "N/mm", 0],
  ];
}

function makeFields(specifications) {
  const grid = document.createElement("div");
  grid.className = "field-grid advanced";
  for (const [key, caption, unit, min, max, boolean, step] of specifications) {
    const label = document.createElement("label");
    label.append(caption);
    const input = document.createElement(boolean ? "select" : "input");
    input.dataset.setting = key;
    if (boolean) {
      input.append(new Option("沿用設定", ""), new Option("啟用", "true"), new Option("停用", "false"));
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
  const wheels = makeGroup("輪胎與輪端", [["vehicle.free_radius_m", "自由半徑", "m", .01]]);
  corners.forEach((corner, index) => wheels.append(makeGroup(corner, wheel(corner, index))));
  const suspension = makeGroup("懸吊與幾何", [
    ["vehicle.suspension_geometry.load_transfer_enabled", "幾何傳力", "", null, null, true],
    ["suspension.vehicle.h_rc_f", "前 Roll Center", "m"],
    ["suspension.vehicle.h_rc_r", "後 Roll Center", "m"],
  ]);
  for (const [name, label] of Object.entries(channelNames)) suspension.append(makeGroup(label, channel(name)));
  container.append(makeGroup("車身與重心", body), makeGroup("動力與空力", power), wheels, makeGroup("液壓煞車", brake), suspension);
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
    for (const input of inputs) {
      const value = input.dataset.setting.split(".").reduce((object, key) => object?.[key], reference);
      if (input.tagName === "SELECT") {
        input.options[0].textContent = typeof value === "boolean" ? `沿用設定（${value ? "啟用" : "停用"}）` : "沿用設定";
      } else {
        input.placeholder = value == null ? (input.dataset.setting.endsWith("_scale") ? "1" : "")
          : String(typeof value === "number" ? Number(value.toPrecision(7)) : value);
      }
    }
  }

  form.addEventListener("change", event => {
    if (["vehicle", "suspension"].includes(event.target.name)) refreshPlaceholders();
  });
  form.addEventListener("reset", () => queueMicrotask(refreshPlaceholders));
  refreshPlaceholders();
  return () => Object.fromEntries(inputs.filter(input => input.value !== "").map(input => [
    input.dataset.setting, input.tagName === "SELECT" ? input.value === "true" : input.valueAsNumber,
  ]));
}
