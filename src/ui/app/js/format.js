export function htmlEscape(value) {
  return String(value ?? "").replace(/[&<>'"]/g, (c) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    "'": "&#39;",
    '"': "&quot;"
  }[c] || c));
}

export function metricCellHtml({ label, value, decimals = 2, history = [] }) {
  const now = Number(value);
  const text = Number.isFinite(now) ? now.toFixed(decimals) : "--";
  const values = Array.isArray(history) ? history : [];
  const avg = values.length ? values.reduce((sum, v) => sum + v, 0) / values.length : 0;
  const sorted = values.slice().sort((a, b) => a - b);
  const p95 = sorted.length ? sorted[Math.min(sorted.length - 1, Math.max(0, Math.round((sorted.length - 1) * 0.95)))] : 0;
  return `<div><span>${htmlEscape(label)}</span><b>${text}</b><span>avg ${avg.toFixed(decimals)} / p95 ${p95.toFixed(decimals)}</span></div>`;
}

export function titleCaseCommand(command) {
  return String(command || "command")
    .replace(/([a-z0-9])([A-Z])/g, "$1 $2")
    .replace(/[_-]+/g, " ")
    .replace(/\s+/g, " ")
    .trim()
    .replace(/^./, (c) => c.toUpperCase());
}

export function esc(value) {
  return String(value ?? "").replace(/[&<>'"]/g, (c) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    "'": "&#39;",
    '"': "&quot;"
  }[c] || c));
}

export function fmt(value, decimals = 1) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(decimals) : "--";
}
