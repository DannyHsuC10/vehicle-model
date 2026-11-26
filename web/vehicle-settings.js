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

const axles = [
  { end: "front", label: "前輪", corner: "FL", index: 0 },
  { end: "rear", label: "後輪", corner: "RL", index: 2 },
];
const sharedWheels = [
  ["vehicle.tire_pressure_kpa.0", "胎壓", "kPa", 1],
  ["vehicle.wheel_inertia_kg_m2.0", "自轉慣量", "kg·m²", .01],
  ["suspension.tires.FL.k", "垂向剛性", "N/mm", .01],
  ["suspension.tires.FL.c", "垂向阻尼", "N·s/m", 0],
];

function wheel({ end, corner, index }) {
  return [
    [`vehicle.${end}_free_radius_m`, "自由半徑", "m", .01],
    [`vehicle.static_camber_deg.${index}`, "靜態外傾", "°", -20, 20],
    [`vehicle.drive_scale.${index}`, "驅動倍率", "", 0, 1],
    [`suspension.tires.${corner}.mu`, "簧下質量", "kg", .01],
  ];
}

const channelNames = {
  heave_f: "前軸升沉", heave_r: "後軸升沉", roll_f: "前軸側傾", roll_r: "後軸側傾",
  FL: "前軸直推", RL: "後軸直推",
};

function channel(name) {
  return [
    [`suspension.channels.${name}.main_rate`, "彈簧／主簧剛性", "N/mm", 0],
    [`suspension.channels.${name}.mr_fixed`, "固定運動比", "", .000001],
    [`suspension.channels.${name}.compression_coefficient`, "壓縮阻尼係數", "N·s/m", 0],
    [`suspension.channels.${name}.rebound_coefficient`, "回彈阻尼係數", "N·s/m", 0],
    [`suspension.channels.${name}.jounce_mm`, "觸底行程", "mm", 0],
    [`suspension.channels.${name}.rebound_mm`, "伸長限位", "mm", 0],
    [`suspension.channels.${name}.rebound_rate`, "伸長限位剛性", "N/mm", 0],
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
  const wheels = makeGroup("輪胎與輪端（左右對稱）", sharedWheels);
  const wheelNote = document.createElement("p");
  wheelNote.className = "field-note";
  wheelNote.textContent = "上方四項由四輪共用。匯入的設定若有差異，共用值採左前輪，同軸值採該軸左輪。";
  wheels.append(wheelNote);
  axles.forEach(axle => wheels.append(makeGroup(axle.label, wheel(axle))));
  const suspension = makeGroup("懸吊與幾何", [
    ["vehicle.suspension_geometry.load_transfer_enabled", "Roll Center／Anti 傳力", "", null, null, true],
    ["suspension.vehicle.h_rc_f", "前 Roll Center 高度", "mm", -10000, 10000, false, "any", 1000],
    ["suspension.vehicle.h_rc_r", "後 Roll Center 高度", "mm", -10000, 10000, false, "any", 1000],
    ["suspension.vehicle.dive_front", "煞車前 Anti-dive", "%", -200, 200, false, "any", 100],
    ["suspension.vehicle.lift_rear", "煞車後 Anti-lift", "%", -200, 200, false, "any", 100],
    ["suspension.vehicle.lift_front", "加速前 Anti-lift", "%", -200, 200, false, "any", 100],
    ["suspension.vehicle.squat_rear", "加速後 Anti-squat", "%", -200, 200, false, "any", 100],
  ]);
  const geometryNote = document.createElement("p");
  geometryNote.className = "field-note";
  geometryNote.textContent = "啟用傳力後會使用 Roll Center 與 Anti。Roll Center 為離地高度，Anti 為全車載重轉移百分比。";
  const camber = makeGroup("Camber Gain（每 25 mm 壓縮）", axles.map(({ label, index }) => [
    `vehicle.suspension_geometry.camber_gain_deg_per_25mm.${index}`, label, "°", -30, 30,
  ]));
  const camberNote = document.createElement("p");
  camberNote.className = "field-note";
  camberNote.textContent = "負值代表懸吊壓縮時輪頂往車內傾。";
  camber.append(camberNote);
  const tableNote = document.createElement("p");
  tableNote.className = "field-note";
  tableNote.hidden = true;
  const springNote = document.createElement("p");
  springNote.className = "field-note";
  springNote.textContent = "輸入彈簧剛性可調整主簧或取代彈簧查表。壓縮與回彈各可改用固定阻尼係數。設定只作用於已啟用的通道，留空則沿用原設定。";
  suspension.append(geometryNote, tableNote, camber, springNote);
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
    tableNote.textContent = locked.length ? `匯入的查表已接管 ${locked.join("、")}。` : "";
    for (const input of inputs) {
      const path = input.dataset.setting;
      const parts = path.split(".");
      const scale = Number(input.dataset.displayScale || 1);
      const isCamber = parts.includes("camber_gain_deg_per_25mm");
      const isMotion = parts.at(-1) === "mr_fixed";
      const isSpring = parts.at(-1) === "main_rate";
      const direction = { compression_coefficient: "compression", rebound_coefficient: "rebound" }[parts.at(-1)];
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
      if (isMotion) value = value?.length > 1 ? "沿用曲線" : value?.[0]?.[1];
      if (isSpring) {
        const kind = reference.suspension?.channels?.[parts[2]]?.spring_kind;
        input.min = kind === "tender" ? .000001 : 0;
        if (kind === "stages" || kind === "force") value = "沿用曲線";
      }
      if (direction) {
        const channel = reference.suspension?.channels?.[parts[2]];
        const curve = channel?.[direction];
        if (curve?.length) value = !channel.damper_ratio && curve.length === 1 ? curve[0][1] : "沿用設定";
      }
      if (input.tagName === "SELECT") {
        input.options[0].textContent = typeof value === "boolean" ? `沿用設定（${value ? "啟用" : "停用"}）` : "沿用設定";
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
