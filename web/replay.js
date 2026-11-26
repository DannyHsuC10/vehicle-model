const G = 9.81;
const radians = degrees => degrees * Math.PI / 180;
const corners = ["FL", "FR", "RL", "RR"];
const rimAngles = Array.from({ length: 16 }, (_, index) => index * Math.PI / 8);

function canvasContext(canvas) {
  const width = canvas.clientWidth, height = canvas.clientHeight;
  if (!width || !height) return null;
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const pixels = [Math.round(width * ratio), Math.round(height * ratio)];
  if (canvas.width !== pixels[0] || canvas.height !== pixels[1]) {
    [canvas.width, canvas.height] = pixels;
  }
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);
  return { context, width, height };
}

function polygon(ctx, vertices, fill, stroke) {
  ctx.beginPath();
  vertices.forEach(([x, y], index) => index ? ctx.lineTo(x, y) : ctx.moveTo(x, y));
  ctx.closePath();
  ctx.fillStyle = fill;
  ctx.fill();
  if (stroke) {
    ctx.strokeStyle = stroke;
    ctx.stroke();
  }
}

function courseSegments(points, width) {
  const segments = [];
  let distance = 0;
  for (let i = 1; i < points.length; i++) {
    const [x, y] = points[i - 1];
    const dx = points[i][0] - x, dy = points[i][1] - y;
    const length = Math.hypot(dx, dy);
    if (length === 0) continue;
    const count = Math.ceil(length / .45);
    const nx = -dy / length * width / 2, ny = dx / length * width / 2;
    for (let step = 0; step < count; step++) {
      const a = step / count, b = (step + 1) / count;
      const start = [x + dx * a, y + dy * a], end = [x + dx * b, y + dy * b];
      const edges = [start, end].map(([x, y]) => [[x + nx, y + ny], [x - nx, y - ny]]);
      segments.push({ start, end, edges, dash: Math.floor((distance + a * length) / 1.4) % 2 === 0 });
    }
    distance += length;
  }
  return segments;
}

function cameraFor(width, height, overhead) {
  const scale = Math.min(width / 6.2, height / 4.5);
  return overhead
    ? ([x, y, z]) => [width / 2 - scale * y, height * .54 - scale * x, -z]
    : ([x, y, z]) => {
      const forward = .84 * x - .54 * y;
      const side = -.54 * x - .84 * y;
      return [width / 2 + scale * side, height * .60 - scale * (.59 * forward + .81 * z), .81 * forward - .59 * z];
    };
}

function drawRoad(ctx, segments, project, x, y, yaw) {
  const cos = Math.cos(yaw), sin = Math.sin(yaw);
  const screen = ([wx, wy]) => {
    const dx = wx - x, dy = wy - y;
    return project([cos * dx + sin * dy, -sin * dx + cos * dy, 0]);
  };
  const near = segments.filter(({ start }) => Math.abs(start[0] - x) < 16 && Math.abs(start[1] - y) < 16);
  for (const { edges: [[a, b], [c, d]] } of near) polygon(ctx, [a, c, d, b].map(screen), "#253442");
  ctx.lineCap = "round";
  function line(a, b, color, width) {
    const from = screen(a), to = screen(b);
    ctx.beginPath();
    ctx.moveTo(from[0], from[1]);
    ctx.lineTo(to[0], to[1]);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.stroke();
  }
  for (const segment of near) {
    const { edges: [[a, b], [c, d]], start, end, dash } = segment;
    line(a, c, "#576778", 1.1);
    line(b, d, "#576778", 1.1);
    if (dash) line(start, end, "#91a2b0", 1.15);
  }
}

function carFaces(geometry, value, project) {
  const { axles, radius, cgHeight } = geometry;
  const front = axles[0][0], rear = axles[2][0];
  const halfTrack = Math.min(axles[0][1], axles[2][1]);
  const roll = radians(value("roll_deg")), pitch = radians(value("pitch_deg"));
  const cr = Math.cos(roll), sr = Math.sin(roll), cp = Math.cos(pitch), sp = Math.sin(pitch);
  const heave = value("heave_mm") / 1000;
  const bodyPoint = ([x, y, z]) => {
    const aroundRoll = sr * y + cr * z;
    return [cp * x + sp * aroundRoll, cr * y - sr * z, cgHeight + heave - sp * x + cp * aroundRoll];
  };
  const faces = [];
  const face = (points, color, stroke = "") => {
    const vertices = points.map(project);
    const depth = vertices.reduce((sum, point) => sum + point[2], 0) / vertices.length;
    faces.push({ vertices, depth, color, stroke });
  };
  function shell(a, b, wa, wb, floor, roof, top, side) {
    const ring = [[a, wa], [b, wb], [b, -wb], [a, -wa]];
    const bottom = ring.map(([x, y]) => bodyPoint([x, y, floor]));
    const above = ring.map(([x, y]) => bodyPoint([x, y, roof]));
    face(above, top, "#1b3237");
    for (let i = 0; i < 4; i++) {
      const next = (i + 1) % 4;
      face([bottom[i], bottom[next], above[next], above[i]], side, "#1b3237");
    }
  }
  const floor = .085 - cgHeight;
  shell(front * .42, rear - .13, halfTrack * .50, halfTrack * .63, floor, floor + .28, "#9adf69", "#447a4c");
  shell(front + .35, front * .42, halfTrack * .17, halfTrack * .50, floor + .02, floor + .15, "#baf584", "#568c53");
  shell(front + .46, front + .23, halfTrack * .88, halfTrack * .88, floor - .01, floor + .045, "#8ed3ba", "#325953");
  shell(rear - .17, rear - .44, halfTrack * .9, halfTrack * .9, floor + .42, floor + .48, "#89d0b5", "#2e5a55");
  shell(rear - .21, rear - .36, halfTrack * .11, halfTrack * .11, floor + .20, floor + .41, "#537b6c", "#233e3d");
  shell(front * .10, rear * .54, halfTrack * .28, halfTrack * .33, floor + .28, floor + .31, "#17252e", "#17252e");
  shell(rear * .10, rear * .34, halfTrack * .17, halfTrack * .17, floor + .315, floor + .42, "#ffd486", "#c4894c");

  corners.forEach((corner, index) => {
    const [x, y] = axles[index];
    const loaded = Math.max(radius * .55, value(`${corner}_rolling_radius_m`));
    const suspensionHub = bodyPoint([x, y, radius - cgHeight])[2] + value(`${corner}_wheel_travel_mm`) / 1000;
    const hub = [x, y, value(`${corner}_fz_n`) > 0 ? loaded : Math.max(loaded, suspensionHub)];
    const steer = radians(value(`${corner}_steer_deg`));
    const camber = radians(value(`${corner}_camber_deg`)) * Math.sign(y);
    const spin = value(`${corner}_wheel_spin_rad`);
    const tread = Math.min(.087, radius * .42);
    const wheelPoint = (angle, size, side) => {
      const fx = Math.cos(steer), fy = Math.sin(steer);
      const f = size * Math.cos(angle), up = size * Math.sin(angle);
      const lateral = Math.cos(camber) * side + Math.sin(camber) * up;
      return [hub[0] + f * fx - lateral * fy, hub[1] + f * fy + lateral * fx,
        hub[2] + Math.cos(camber) * up - Math.sin(camber) * side];
    };
    const sides = [-tread, tread].map(side => rimAngles.map(angle => wheelPoint(angle, loaded, side)));
    for (let i = 0; i < rimAngles.length; i++) {
      const j = (i + 1) % rimAngles.length;
      face([sides[0][i], sides[0][j], sides[1][j], sides[1][i]], i % 2 ? "#20313e" : "#2b3d48");
    }
    sides.forEach((outline, index) => {
      face(outline, "#14202a");
      const side = (index === 0 ? -1 : 1) * (tread + .004);
      face(rimAngles.map(angle => wheelPoint(angle, loaded * .64, side)), "#687e8d");
      for (let spoke = 0; spoke < 5; spoke++) {
        const angle = spoke * Math.PI * .4 - spin;
        face([wheelPoint(angle - .14, loaded * .14, side * 1.02),
          wheelPoint(angle - .09, loaded * .55, side * 1.02),
          wheelPoint(angle + .09, loaded * .55, side * 1.02),
          wheelPoint(angle + .14, loaded * .14, side * 1.02)], "#d7e3e9");
      }
    });
  });
  return faces.sort((a, b) => b.depth - a.depth);
}

function drawGG(canvas, data, index, limit) {
  const view = canvasContext(canvas);
  if (!view) return;
  const { context: ctx, width, height } = view;
  const x = width / 2, y = height / 2;
  const radius = Math.min(width, height) * .36;
  ctx.lineWidth = 1;
  ctx.strokeStyle = "#34495b";
  for (const fraction of [.25, .5, .75, 1]) {
    ctx.beginPath();
    ctx.arc(x, y, fraction * radius, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.beginPath();
  ctx.moveTo(x - radius, y); ctx.lineTo(x + radius, y);
  ctx.moveTo(x, y - radius); ctx.lineTo(x, y + radius);
  ctx.stroke();
  ctx.fillStyle = "#9eafc1";
  ctx.font = "11px system-ui";
  ctx.textAlign = "center";
  ctx.fillText(`+${limit} g 縱向`, x, y - radius - 5);
  ctx.fillText(`−${limit} g`, x, y + radius + 13);
  ctx.fillText("左", x - radius - 9, y + 4);
  ctx.fillText("右", x + radius + 9, y + 4);
  const point = row => [x - data.ay_mps2[row] / G / limit * radius, y - data.ax_mps2[row] / G / limit * radius];
  ctx.beginPath();
  for (let i = Math.max(0, index - 75); i <= index; i++) {
    const loc = point(i);
    if (i === Math.max(0, index - 75)) ctx.moveTo(...loc);
    else ctx.lineTo(...loc);
  }
  ctx.strokeStyle = "#6fcba3";
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(...point(index), 4.5, 0, Math.PI * 2);
  ctx.fillStyle = "#bafa78";
  ctx.fill();
}

export function replayView(scene, gg) {
  let previous, segments, ggLimit;
  return (result, index, overhead) => {
    if (!result) return;
    if (result !== previous) {
      segments = courseSegments(result.centerline, result.geometry.roadWidth);
      let peak = 1;
      result.series.ax_mps2.forEach((ax, i) => {
        peak = Math.max(peak, Math.hypot(ax, result.series.ay_mps2[i]) / G);
      });
      ggLimit = Math.ceil(peak * 2) / 2;
      previous = result;
    }
    const view = canvasContext(scene);
    if (view) {
      const { context: ctx, width, height } = view;
      const value = key => result.series[key][index];
      const project = cameraFor(width, height, overhead);
      const background = ctx.createLinearGradient(0, 0, 0, height);
      background.addColorStop(0, "#182833");
      background.addColorStop(1, "#0e1b25");
      ctx.fillStyle = background;
      ctx.fillRect(0, 0, width, height);
      drawRoad(ctx, segments, project, value("x_m"), value("y_m"), radians(value("yaw_deg")));
      const shadow = [[1.15, .47, 0], [-1.12, .52, 0], [-1.12, -.50, 0], [1.15, -.44, 0]];
      polygon(ctx, shadow.map(project), "#0d151eb3");
      ctx.lineWidth = .75;
      for (const { vertices, color, stroke } of carFaces(result.geometry, value, project)) polygon(ctx, vertices, color, stroke);
    }
    drawGG(gg, result.series, index, ggLimit);
  };
}
