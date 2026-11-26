const colors = ["#a8ea69", "#6fc2ff", "#ffab75", "#caadff"];
const wheels = ["FL", "FR", "RL", "RR"];
const four = key => wheels.map(wheel => [wheel, wheel + "_" + key]);

export const signals = {
  speed: { unit: "km/h", scale: 3.6, channels: [["車速", "speed_mps"]] },
  accel: { unit: "g", scale: 1 / 9.81, channels: [["縱向", "ax_mps2"], ["側向", "ay_mps2"]] },
  attitude: { unit: "°", channels: [["側傾", "roll_deg"], ["俯仰", "pitch_deg"]] },
  heave: { unit: "mm", channels: [["升沉", "heave_mm"]] },
  loads: { unit: "N", channels: four("fz_n") },
  travel: { unit: "mm", channels: four("wheel_travel_mm") },
  slip: { unit: "%", scale: 100, channels: four("slip_ratio") },
  angle: { unit: "°", channels: four("slip_angle_deg") },
  drive: { unit: "N·m", channels: four("drive_nm") },
  brake: { unit: "N·m", channels: four("brake_nm") },
  aero: { unit: "N", channels: [["下壓力", "downforce_n"], ["阻力", "drag_n"]] },
  cop: { unit: "m", channels: [["X", "cop_x_m"], ["Y", "cop_y_m"], ["Z", "cop_z_m"]] },
  pedals: { unit: "%", scale: 100, channels: [["油門", "throttle"], ["煞車", "brake"]] },
};

export function extent(values) {
  let min = Infinity, max = -Infinity;
  for (const value of values) {
    min = Math.min(min, value);
    max = Math.max(max, value);
  }
  return [min, max];
}

function surface(canvas) {
  const buffer = document.createElement("canvas");
  const context = canvas.getContext("2d");
  let previousKey;
  let projection;
  return (key, drawBase, drawOverlay) => {
    const { width, height } = canvas.getBoundingClientRect();
    if (!width || !height) return;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const pixelWidth = Math.round(width * ratio), pixelHeight = Math.round(height * ratio);
    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
      canvas.width = buffer.width = pixelWidth;
      canvas.height = buffer.height = pixelHeight;
      previousKey = undefined;
    }
    if (key !== previousKey) {
      buffer.width = pixelWidth;
      buffer.height = pixelHeight;
      const base = buffer.getContext("2d");
      base.scale(ratio, ratio);
      projection = drawBase(base, width, height);
      previousKey = key;
    }
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.drawImage(buffer, 0, 0);
    if (projection && drawOverlay) {
      context.save();
      context.scale(ratio, ratio);
      drawOverlay(context, projection);
      context.restore();
    }
  };
}

function speedColor(fraction) {
  const stops = [[73, 164, 255], [112, 219, 193], [243, 233, 109], [255, 147, 104]];
  const position = Math.min(3, Math.max(0, 3 * fraction));
  const index = Math.min(2, Math.floor(position)), blend = position - index;
  const rgb = stops[index].map((value, axis) => Math.round(value + blend * (stops[index + 1][axis] - value)));
  return `rgb(${rgb.join(",")})`;
}

function circle(ctx, x, y, radius, color) {
  ctx.beginPath();
  ctx.arc(x, y, radius, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

export function routeView(canvas) {
  const paint = surface(canvas);
  return (course, series, index) => {
    paint(course, (ctx, width, height) => {
      ctx.strokeStyle = "#1b2937";
      ctx.beginPath();
      for (let x = 18; x < width; x += 24) { ctx.moveTo(x, 0); ctx.lineTo(x, height); }
      for (let y = 18; y < height; y += 24) { ctx.moveTo(0, y); ctx.lineTo(width, y); }
      ctx.stroke();
      if (!course.length) return;
      const [minX, maxX] = extent(course.map(point => point[0]));
      const [minY, maxY] = extent(course.map(point => point[1]));
      const [fromX, toX] = series ? extent(series.x_m) : [minX, maxX];
      const [fromY, toY] = series ? extent(series.y_m) : [minY, maxY];
      const x0 = Math.min(minX, fromX), x1 = Math.max(maxX, toX);
      const y0 = Math.min(minY, fromY), y1 = Math.max(maxY, toY);
      const scale = Math.min((width - 66) / Math.max(x1 - x0, 1), (height - 64) / Math.max(y1 - y0, 1));
      const x = value => width / 2 + (value - (x0 + x1) / 2) * scale;
      const y = value => height / 2 - (value - (y0 + y1) / 2) * scale;
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      ctx.beginPath();
      course.forEach(([xValue, yValue], i) => i
        ? ctx.lineTo(x(xValue), y(yValue)) : ctx.moveTo(x(xValue), y(yValue)));
      ctx.lineWidth = 7;
      ctx.strokeStyle = "#344553";
      ctx.stroke();
      ctx.lineWidth = 1.4;
      ctx.strokeStyle = "#a2b5c8";
      ctx.stroke();
      if (series) {
        const [min, max] = extent(series.speed_mps);
        const stride = Math.max(1, Math.floor(series.x_m.length / 6000));
        ctx.lineWidth = 2.8;
        for (let i = 0; i < series.x_m.length - 1; i += stride) {
          const j = Math.min(i + stride, series.x_m.length - 1);
          ctx.beginPath();
          ctx.moveTo(x(series.x_m[i]), y(series.y_m[i]));
          ctx.lineTo(x(series.x_m[j]), y(series.y_m[j]));
          ctx.strokeStyle = speedColor((series.speed_mps[j] - min) / (max - min || 1));
          ctx.stroke();
        }
      }
      circle(ctx, x(course[0][0]), y(course[0][1]), 4.5, "#a8ea69");
      const maxLabel = 92 / scale;
      const unit = 10 ** Math.floor(Math.log10(maxLabel));
      const metres = [5, 2, 1].find(multiplier => multiplier * unit <= maxLabel) * unit;
      ctx.strokeStyle = "#9eafc1";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(18, height - 16);
      ctx.lineTo(18 + metres * scale, height - 16);
      ctx.stroke();
      ctx.fillStyle = "#9eafc1";
      ctx.font = "12px monospace";
      ctx.fillText(`${+metres.toPrecision(4)} m`, 18, height - 22);
      return { x, y };
    }, (ctx, { x, y }) => {
      if (!series) return;
      ctx.translate(x(series.x_m[index]), y(series.y_m[index]));
      ctx.rotate(-series.yaw_deg[index] * Math.PI / 180);
      ctx.fillStyle = "#f6fbff";
      ctx.strokeStyle = "#15314b";
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(8, 0);
      ctx.lineTo(-5, 5);
      ctx.lineTo(-3, 0);
      ctx.lineTo(-5, -5);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    });
  };
}

export function timeView(canvas, legend) {
  const paint = surface(canvas);
  return (key, series, signal, index, warmupEnd) => {
    paint(key, (ctx, width, height) => {
      legend.replaceChildren();
      if (!series) return;
      const { unit, scale = 1, channels } = signals[signal];
      const times = series.time_s;
      const duration = times.at(-1) || 1;
      const extrema = channels.map(([, channel]) => extent(series[channel]));
      const low = Math.min(0, ...extrema.map(([min]) => min * scale));
      const high = Math.max(0, ...extrema.map(([, max]) => max * scale));
      const range = high - low || 1;
      const lower = low < 0 ? low - .08 * range : 0;
      const upper = high + .12 * range;
      const left = 55, right = width - 15, top = 21, bottom = height - 27;
      const x = value => left + value / duration * (right - left);
      const y = value => bottom - (value - lower) / (upper - lower) * (bottom - top);
      ctx.font = "12px monospace";
      ctx.fillStyle = "#9eafc1";
      ctx.fillText(unit, 12, 15);
      ctx.strokeStyle = "#283747";
      ctx.beginPath();
      for (let tick = 0; tick <= 4; tick++) {
        const value = lower + tick / 4 * (upper - lower), row = y(value);
        ctx.moveTo(left, row);
        ctx.lineTo(right, row);
        ctx.textAlign = "right";
        ctx.fillText(Number(value.toPrecision(3)), left - 7, row + 4);
        const time = tick / 4 * duration, column = x(time);
        ctx.moveTo(column, top);
        ctx.lineTo(column, bottom);
        ctx.textAlign = tick === 4 ? "right" : tick === 0 ? "left" : "center";
        ctx.fillText(`${Number(time.toPrecision(3))} s`, column, height - 9);
      }
      ctx.stroke();
      if (warmupEnd > 0) {
        ctx.strokeStyle = "#7c8796";
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(x(warmupEnd), top);
        ctx.lineTo(x(warmupEnd), bottom);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      channels.forEach(([label, channel], lane) => {
        const marker = document.createElement("i"), item = document.createElement("span");
        marker.style.backgroundColor = colors[lane];
        item.append(marker, label);
        legend.append(item);
        ctx.strokeStyle = colors[lane];
        ctx.lineWidth = 1.7;
        ctx.beginPath();
        series[channel].forEach((value, i) => i
          ? ctx.lineTo(x(times[i]), y(value * scale)) : ctx.moveTo(x(0), y(value * scale)));
        ctx.stroke();
      });
      return { x, y, top, bottom, channels, scale };
    }, (ctx, { x, y, top, bottom, channels, scale }) => {
      const cursor = x(series.time_s[index]);
      ctx.strokeStyle = "#d8e7f5";
      ctx.beginPath();
      ctx.moveTo(cursor, top);
      ctx.lineTo(cursor, bottom);
      ctx.stroke();
      channels.forEach(([, channel], lane) => circle(ctx, cursor, y(series[channel][index] * scale), 3, colors[lane]));
    });
  };
}
