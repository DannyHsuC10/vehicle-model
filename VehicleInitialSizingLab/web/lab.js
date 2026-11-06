"use strict";

window.MathJax = {
  startup: {typeset: false},
  options: {enableAssistiveMml: false},
  tex: {displayMath: [["$$", "$$"]]},
  svg: {fontCache: "local"},
};

const byId = (id) => document.getElementById(id);
const form = byId("parameters");
const fields = byId("fields");
const solveButton = byId("solve");
const resetButton = byId("reset");
const status = byId("status");
const errorBox = byId("input-error");
const report = byId("report");
const chapter = byId("chapter");
const download = byId("download");
let lastMarkdown = "";

function element(tag, text, className) {
  const node = document.createElement(tag);
  if (text) node.textContent = text;
  if (className) node.className = className;
  return node;
}

function buildFields(groups) {
  for (const group of groups) {
    const details = element("details", "", "parameter-group");
    details.open = group.expanded;
    const rows = element("div", "", "group-fields");
    for (const field of group.fields) {
      const row = element("div", "", "field");
      const label = element("label", field.label);
      const input = element("input");
      input.type = field.kind;
      input.name = field.name;
      input.id = "input-" + field.name;
      label.htmlFor = input.id;
      if (field.hint) label.append(element("small", field.hint));

      if (field.kind === "checkbox") {
        input.defaultChecked = field.default;
      } else {
        input.step = "any";
        input.required = field.default !== null;
        input.defaultValue = field.default ?? "";
      }
      row.append(label, input, element("span", field.unit, "unit"));
      rows.append(row);
    }
    details.append(element("summary", group.title), rows);
    fields.append(details);
  }
}

function clearError() {
  errorBox.hidden = true;
  fields.querySelector('[aria-invalid="true"]')?.removeAttribute("aria-invalid");
}

function showError(message, field) {
  errorBox.textContent = message;
  errorBox.hidden = false;
  const input = field && form.elements.namedItem(field);
  if (input) {
    input.closest("details").open = true;
    input.setAttribute("aria-invalid", "true");
    input.focus();
  }
}

function markPending() {
  clearError();
  status.textContent = lastMarkdown
    ? "參數已變更。右側顯示上次求解結果。"
    : "就緒 · 按「求解」產生報告。";
}

function escapeHtml(text) {
  return text.replaceAll("&", "&amp;").replaceAll("<", "&lt;").replaceAll(">", "&gt;");
}

// The report emits a topologically ordered Mermaid flow with one decision branch.
function renderFlow(source) {
  const lines = source.trim().split("\n");
  if (lines.shift() !== "graph TD") return null;

  const nodes = new Map();
  const pattern = /^\s*([A-Z]+)(?:[\[{]"([^"]+)"[\]}])? -->\s*(?:\|"([^"]+)"\| )?([A-Z]+)(?:[\[{]"([^"]+)"[\]}])?$/;
  for (const line of lines) {
    const match = line.match(pattern);
    if (!match) return null;
    const [, from, fromLabel, branch, to, toLabel] = match;
    if (!nodes.has(from)) nodes.set(from, {label: fromLabel, level: 0});
    const target = nodes.get(to) ?? {label: toLabel, branch, level: 0};
    if (toLabel) target.label = toLabel;
    target.level = Math.max(target.level, nodes.get(from).level + 1);
    nodes.set(to, target);
  }
  const levels = [];
  for (const node of nodes.values()) {
    if (!node.label) return null;
    const branch = node.branch ? "<b>" + escapeHtml(node.branch) + "</b>" : "";
    (levels[node.level] ??= []).push('<span class="flow-node">' + branch + escapeHtml(node.label) + "</span>");
  }
  const rows = levels.map((level) => '<div class="flow-level">' + level.join("") + "</div>");
  return '<details class="flow"><summary>展開計算流程</summary>' + rows.join("") + "</details>";
}

marked.use({
  extensions: [{
    name: "equation",
    level: "block",
    tokenizer(source) {
      const match = /^\$\$\n([\s\S]*?)\n\$\$(?:\n|$)/.exec(source);
      if (match) return {type: "equation", raw: match[0], math: match[1]};
    },
    renderer({math}) {
      return '<div class="equation">$$\n' + escapeHtml(math) + '\n$$</div>\n';
    },
  }],
  renderer: {
    code({text, lang}) {
      return lang === "mermaid" ? (renderFlow(text) ?? false) : false;
    },
  },
});

async function renderResult(result) {
  const overview = byId("summary");
  const math = window.MathJax;
  await math.startup.promise;
  math.typesetClear([report]);

  // The Markdown comes only from the local solver's report generator.
  report.innerHTML = marked.parse(result.markdown);
  for (const table of report.querySelectorAll("table")) {
    const wrapper = element("div", "", "table-scroll");
    table.before(wrapper);
    wrapper.append(table);
  }
  chapter.options.length = 1;
  for (const [index, heading] of Array.from(report.querySelectorAll("h2")).entries()) {
    heading.id = "chapter-" + index;
    chapter.add(new Option(heading.textContent, heading.id));
  }

  const metrics = result.summary.map(({label, value, unit}) => {
    const card = element("div", "", "metric");
    const number = element("dd", value);
    number.append(element("span", unit));
    card.append(element("dt", label), number);
    return card;
  });
  overview.replaceChildren(...metrics);
  overview.hidden = false;
  byId("placeholder").hidden = true;
  byId("report-scroll").scrollTop = 0;
  await math.typesetPromise([report]);
}

function readInputs() {
  const values = {};
  for (const input of fields.querySelectorAll("input")) {
    if (input.type === "checkbox") {
      values[input.name] = input.checked;
    } else if (input.validity.badInput || (input.required && input.value === "")) {
      showError(input.labels[0].firstChild.textContent + "須填入數值。", input.name);
      return null;
    } else {
      values[input.name] = input.value === "" ? null : input.valueAsNumber;
    }
  }
  return values;
}

form.noValidate = true;
form.addEventListener("submit", async (event) => {
  event.preventDefault();
  if (solveButton.disabled) return;
  clearError();
  const values = readInputs();
  if (!values) {
    status.textContent = lastMarkdown ? "請修正參數。右側顯示上次求解結果。" : "請修正參數後再求解。";
    return;
  }

  solveButton.disabled = resetButton.disabled = fields.disabled = true;
  chapter.disabled = download.disabled = true;
  status.textContent = "正在求解並排版報告…";
  try {
    const response = await fetch("/api/solve", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(values),
    });
    const result = await response.json();
    if (!response.ok) {
      fields.disabled = false;
      showError(result.error, result.field);
      status.textContent = lastMarkdown ? "本次求解未完成。右側顯示上次求解結果。" : "請修正參數後再求解。";
      return;
    }
    await renderResult(result);
    lastMarkdown = result.markdown;
    status.textContent = "求解完成 · 顯示本次參數的報告。";
  } catch (error) {
    console.error(error);
    showError("無法完成求解或顯示報告。請確認啟動視窗仍在執行，再重新求解。");
    status.textContent = "本次求解未完成。";
  } finally {
    solveButton.disabled = resetButton.disabled = fields.disabled = false;
    chapter.disabled = download.disabled = !lastMarkdown;
  }
});
form.addEventListener("input", markPending);
form.addEventListener("reset", markPending);
document.addEventListener("keydown", (event) => {
  if (event.ctrlKey && event.key === "Enter" && !solveButton.disabled) {
    event.preventDefault();
    form.requestSubmit();
  }
});
chapter.addEventListener("change", () => {
  if (chapter.value) byId(chapter.value).scrollIntoView({block: "start"});
});
download.addEventListener("click", () => {
  const url = URL.createObjectURL(new Blob([lastMarkdown], {type: "text/markdown;charset=utf-8"}));
  const link = element("a");
  link.href = url;
  link.download = "vehicle_initial_sizing_report.md";
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});

async function initialize() {
  try {
    const response = await fetch("/api/fields");
    if (!response.ok) throw new Error("Failed to load parameters");
    buildFields(await response.json());
    solveButton.disabled = resetButton.disabled = false;
    markPending();
  } catch (error) {
    console.error(error);
    showError("無法載入參數。請重新啟動工具。");
    status.textContent = "載入失敗。";
  }
}

initialize();
