import fs from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { FileBlob, SpreadsheetFile, Workbook } from "@oai/artifact-tool";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const sourceCsv = path.join(root, "outputs", "battery_trade_study_20260725_final", "all_configuration_results.csv");
const outputDir = path.join(root, "outputs", "battery_trade_study_20260725_final", "workbooks");
const previewRoot = path.join(root, "outputs", "battery_trade_study_20260725_final", "workbook_previews");

function parseCsv(text) {
  const rows = [];
  let row = [], field = "", quoted = false;
  for (let i = 0; i < text.length; i += 1) {
    const c = text[i];
    if (quoted) {
      if (c === '"' && text[i + 1] === '"') { field += '"'; i += 1; }
      else if (c === '"') quoted = false;
      else field += c;
    } else if (c === '"') quoted = true;
    else if (c === ',') { row.push(field); field = ""; }
    else if (c === '\n') { row.push(field.replace(/\r$/, "")); rows.push(row); row = []; field = ""; }
    else field += c;
  }
  if (field.length || row.length) { row.push(field); rows.push(row); }
  const [headers, ...data] = rows;
  return data.filter((r) => r.length === headers.length).map((r) => Object.fromEntries(headers.map((h, i) => [h, r[i]])));
}

const n = (v) => (v === "" || v == null ? null : Number(v));
const b = (v) => v === "True";
const packHeaders = [
  "Candidate ID", "Manufacturer", "Cell model", "Form factor", "Series", "Parallel", "Total cells",
  "Nominal energy (kWh)", "Modeled usable energy (kWh)", "Pack mass (kg)", "Max charged pack voltage (V)", "Pack current limit (A)", "Power limit (kW)", "Initial pack resistance (mΩ)",
  "Simulation fidelity", "Target distance (m)", "Completed target", "Elapsed time (s)", "Equivalent avg lap (s)",
  "Time vs baseline (s)", "Final SOC", "Remaining usable energy (kWh)", "RMS cell current (A)", "Peak cell current (A)", "Peak pack current (A)",
  "Min terminal voltage (V)", "Pack heat (kWh)", "Battery efficiency (%)", "Adiabatic cell rise (°C)", "Overall rank", "Cell rank", "Pareto frontier", "Source URL",
];
const packFields = [
  "candidate_id", "manufacturer", "cell_model", "form_factor", "series_cells", "parallel_cells", "total_cells",
  "nominal_pack_energy_kwh", "model_usable_energy_kwh", "pack_mass_kg", "maximum_pack_voltage_v", "pack_current_limit_a", "power_limit_kw", "pack_resistance_initial_mohm",
  "simulation_fidelity", "target_distance_m", "completed_target_distance", "elapsed_time_s", "equivalent_average_lap_time_s",
  "time_difference_vs_baseline_s", "final_soc", "remaining_usable_chemical_kwh", "rms_cell_current_a", "peak_cell_current_a", "peak_pack_current_a",
  "minimum_terminal_voltage_v", "pack_resistive_heat_kwh", "battery_discharge_efficiency_pct", "adiabatic_cell_temperature_rise_c", "overall_rank", "cell_rank", "pareto_frontier", "source_url",
];
const numericFields = new Set([
  "series_cells", "parallel_cells", "total_cells", "nominal_pack_energy_kwh", "model_usable_energy_kwh", "pack_mass_kg", "maximum_pack_voltage_v",
  "pack_current_limit_a", "power_limit_kw", "pack_resistance_initial_mohm", "target_distance_m", "elapsed_time_s", "equivalent_average_lap_time_s",
  "time_difference_vs_baseline_s", "final_soc", "remaining_usable_chemical_kwh", "rms_cell_current_a", "peak_cell_current_a", "peak_pack_current_a",
  "minimum_terminal_voltage_v", "pack_resistive_heat_kwh", "battery_discharge_efficiency_pct", "adiabatic_cell_temperature_rise_c", "overall_rank", "cell_rank",
]);
const boolFields = new Set(["completed_target_distance", "pareto_frontier"]);
const themed = {
  dark: "#17365D", mid: "#1F4E78", light: "#D9EAF7", accent: "#5B9BD5", note: "#FFF2CC", green: "#E2F0D9", border: "#B7C9D6", white: "#FFFFFF",
};

function baseSheet(sheet) {
  sheet.showGridLines = false;
}
function title(sheet, range, text, subtitle) {
  sheet.mergeCells(range);
  const t = sheet.getRange(range);
  t.values = [[text]];
  t.format = { fill: themed.dark, font: { bold: true, color: themed.white, size: 16 }, horizontalAlignment: "left", verticalAlignment: "center" };
  t.format.rowHeight = 28;
  if (subtitle) {
    const endColumn = range.split(":")[1].replace(/[0-9]/g, "");
    const subtitleRange = `A2:${endColumn}2`;
    sheet.mergeCells(subtitleRange);
    const st = sheet.getRange(subtitleRange);
    st.values = [[subtitle]];
    st.format = { fill: themed.light, font: { italic: true, color: "#385723", size: 10 }, verticalAlignment: "center" };
    st.format.rowHeight = 22;
  }
}
function header(sheet, range) {
  sheet.getRange(range).format = { fill: themed.mid, font: { bold: true, color: themed.white }, horizontalAlignment: "center", verticalAlignment: "center", wrapText: true, borders: { preset: "outside", style: "thin", color: themed.border } };
  sheet.getRange(range).format.rowHeight = 30;
}
function numberFormats(sheet, startRow, endRow) {
  const sets = [
    ["E", "G", "#,##0"], ["H", "I", "0.000"], ["J", "M", "0.0"], ["N", "N", "0.000"], ["P", "P", "#,##0"],
    ["R", "T", "0.00"], ["U", "U", "0.0%"], ["V", "V", "0.000"], ["W", "Z", "0.0"], ["AA", "AA", "0.000"], ["AB", "AC", "0.0"], ["AD", "AE", "#,##0"],
  ];
  for (const [a, z, fmt] of sets) sheet.getRange(`${a}${startRow}:${z}${endRow}`).format.numberFormat = fmt;
}
function setWidths(sheet, widths) { for (const [range, width] of widths) sheet.getRange(range).format.columnWidth = width; }

function packSheet(workbook, rows, cellId) {
  const sheet = workbook.worksheets.add("Pack Configurations");
  baseSheet(sheet);
  title(sheet, "A1:AG1", `Battery Trade Study | ${cellId}`, "Source: all_configuration_results.csv | All configurations shown for this cell family.");
  sheet.getRange("A4:AG4").values = [packHeaders]; header(sheet, "A4:AG4");
  const values = rows.map((r) => packFields.map((f) => boolFields.has(f) ? b(r[f]) : numericFields.has(f) ? n(r[f]) : r[f]));
  const end = 4 + rows.length;
  sheet.getRange(`A5:AG${end}`).values = values;
  numberFormats(sheet, 5, end);
  sheet.tables.add(`A4:AG${end}`, true, "PackConfigurationsTable");
  sheet.freezePanes.freezeRows(4);
  sheet.freezePanes.freezeColumns(2);
  setWidths(sheet, [["A:A", 29], ["B:C", 18], ["D:D", 12], ["E:G", 10], ["H:N", 15], ["O:O", 19], ["P:T", 14], ["U:AC", 15], ["AD:AF", 12], ["AG:AG", 44]]);
  sheet.getRange(`A4:AG${end}`).format.borders = { preset: "outside", style: "thin", color: themed.border };
  return sheet;
}

const rankedColumns = [
  ["Candidate ID", "A"], ["Overall rank", "AD"], ["Elapsed time (s)", "R"], ["Time vs baseline (s)", "T"], ["Pack mass (kg)", "J"], ["Modeled usable energy (kWh)", "I"],
  ["RMS cell current (A)", "W"], ["Peak cell current (A)", "X"], ["Max charged pack voltage (V)", "K"], ["Pack heat (kWh)", "AA"], ["Battery efficiency (%)", "AB"], ["Fidelity", "O"], ["Pareto frontier", "AF"],
];
function rankedSheet(workbook, rows) {
  const sheet = workbook.worksheets.add("Ranked Results"); baseSheet(sheet);
  title(sheet, "A1:M1", "Ranked Results", "Formula-linked to Pack Configurations; lower elapsed time is better. Continuous green-yellow-red scale runs fastest to slowest.");
  sheet.getRange("A4:M4").values = [rankedColumns.map(([h]) => h)]; header(sheet, "A4:M4");
  const sorted = rows.map((r, i) => ({ r, i })).sort((a, b2) => Number(a.r.overall_rank) - Number(b2.r.overall_rank));
  const formulas = sorted.map(({ i }) => rankedColumns.map(([, source]) => `='Pack Configurations'!$${source}$${i + 5}`));
  const end = 4 + rows.length;
  sheet.getRange(`A5:M${end}`).formulas = formulas;
  sheet.getRange(`B5:B${end}`).format.numberFormat = "#,##0";
  sheet.getRange(`C5:D${end}`).format.numberFormat = "0.00";
  sheet.getRange(`E5:J${end}`).format.numberFormat = "0.0";
  sheet.getRange(`K5:K${end}`).format.numberFormat = "0.0";
  sheet.tables.add(`A4:M${end}`, true, "RankedResultsTable");
  sheet.freezePanes.freezeRows(4);
  sheet.freezePanes.freezeColumns(1);
  setWidths(sheet, [["A:A", 29], ["B:B", 11], ["C:D", 16], ["E:F", 17], ["G:J", 18], ["K:K", 16], ["L:L", 18], ["M:M", 14]]);
  const timeRange = sheet.getRange(`C5:C${end}`);
  timeRange.conditionalFormats.add("colorScale", {
    criteria: [
      { type: "lowestValue", color: "#63BE7B" },
      { type: "percentile", value: 50, color: "#FFEB84" },
      { type: "highestValue", color: "#F8696B" },
    ],
  });
  return sheet;
}

const chartSpecs = [
  ["Pack mass (kg)", "E", "Elapsed time vs pack mass", "Pack mass (kg)", "0.0", "pack_mass_kg"],
  ["Modeled usable energy (kWh)", "F", "Elapsed time vs modeled usable energy", "Modeled usable energy (kWh)", "0.000", "model_usable_energy_kwh"],
  ["RMS cell current (A)", "G", "Elapsed time vs RMS cell current", "RMS current (A)", "0.0", "rms_cell_current_a"],
  ["Peak cell current (A)", "H", "Elapsed time vs peak cell current", "Peak cell current (A)", "0.0", "peak_cell_current_a"],
  ["Max charged pack voltage (V)", "I", "Elapsed time vs max charged pack voltage", "Max charged pack voltage (V)", "0.0", "maximum_pack_voltage_v"],
  ["Pack heat (kWh)", "J", "Elapsed time vs pack heat", "Pack heat (kWh)", "0.000", "pack_resistive_heat_kwh"],
  ["Battery efficiency (%)", "K", "Elapsed time vs battery efficiency", "Battery efficiency (%)", "0.0", "battery_discharge_efficiency_pct"],
];
function summarySheet(workbook, rows, cellId) {
  const sheet = workbook.worksheets.add("Summary Charts"); baseSheet(sheet);
  title(sheet, "A1:Q1", `Summary Charts | ${cellId}`, "Seven scatter plots generated from the ranked configuration data. Each point is one completed pack configuration.");
  sheet.getRange("A4:H4").values = [["Fastest configuration", "Overall rank", "Elapsed time (s)", "Pack mass (kg)", "Modeled usable energy (kWh)", "RMS current (A)", "Peak current (A)", "Simulation fidelity"]];
  header(sheet, "A4:H4");
  sheet.getRange("A5:H5").formulas = [["='Ranked Results'!$A$5", "='Ranked Results'!$B$5", "='Ranked Results'!$C$5", "='Ranked Results'!$E$5", "='Ranked Results'!$F$5", "='Ranked Results'!$G$5", "='Ranked Results'!$H$5", "='Ranked Results'!$L$5"]];
  sheet.getRange("A5:H5").format = { fill: themed.green, font: { bold: true }, borders: { preset: "outside", style: "thin", color: themed.border } };
  sheet.getRange("B5:B5").format.numberFormat = "#,##0";
  sheet.getRange("C5:D5").format.numberFormat = "0.00"; sheet.getRange("E5:G5").format.numberFormat = "0.0";
  setWidths(sheet, [["A:A", 29], ["B:B", 12], ["C:G", 17], ["H:H", 19], ["I:I", 3], ["J:Q", 13]]);
  const sortedRows = rows.slice().sort((a, b2) => Number(a.overall_rank) - Number(b2.overall_rank));
  for (let i = 0; i < chartSpecs.length; i += 1) {
    const [, , chartTitle, xTitle, xFmt, sourceField] = chartSpecs[i];
    const chart = sheet.charts.add("scatter", {
      chartType: "scatter",
      title: chartTitle,
      hasLegend: false,
      series: [{
        name: "Configurations",
        xValues: sortedRows.map((r) => n(r[sourceField])),
        values: sortedRows.map((r) => n(r.elapsed_time_s)),
        fill: themed.accent,
      }],
    });
    chart.hasLegend = false;
    chart.xAxis = { numberFormatCode: xFmt };
    chart.yAxis = { numberFormatCode: "0.0" };
    chart.xAxis.title.text = xTitle;
    chart.yAxis.title.text = "Elapsed time (s)";
    const positions = [["A7", "H22"], ["J7", "Q22"], ["A24", "H39"], ["J24", "Q39"], ["A41", "H56"], ["J41", "Q56"], ["A58", "H73"]];
    chart.setPosition(...positions[i]);
  }
  return sheet;
}

function paretoSheet(workbook, rows, cellId) {
  const sheet = workbook.worksheets.add("Pareto Frontier"); baseSheet(sheet);
  title(sheet, "A1:L1", `Pareto Frontier | ${cellId}`, "Screening-level frontier with native results substituted where available. Simulation fidelity is shown for every configuration.");
  sheet.mergeCells("A3:L3");
  sheet.getRange("A3:L3").values = [["Interpretation: the Pareto flag is the study-level time-mass-heat screening flag. Treat screening_factor8 points as directionally useful only; compare their native_0p25m counterparts where present."]];
  sheet.getRange("A3:L3").format = { fill: themed.note, font: { italic: true, color: "#7F6000" }, wrapText: true, verticalAlignment: "center", borders: { preset: "outside", style: "thin", color: "#D6B656" } };
  sheet.getRange("A3:L3").format.rowHeight = 32;
  const cols = [["Candidate ID", "A"], ["Overall rank", "B"], ["Elapsed time (s)", "C"], ["Pack mass (kg)", "E"], ["Modeled usable energy (kWh)", "F"], ["Time vs baseline (s)", "D"], ["RMS current (A)", "G"], ["Peak current (A)", "H"], ["Max charged voltage (V)", "I"], ["Pack heat (kWh)", "J"], ["Fidelity", "L"], ["Pareto flag", "M"]];
  sheet.getRange("A5:L5").values = [cols.map(([h]) => h)]; header(sheet, "A5:L5");
  const formulas = rows.map((_, i) => cols.map(([, source]) => `='Ranked Results'!$${source}$${i + 5}`));
  const end = 5 + rows.length;
  sheet.getRange(`A6:L${end}`).formulas = formulas;
  sheet.getRange(`B6:B${end}`).format.numberFormat = "#,##0"; sheet.getRange(`C6:C${end}`).format.numberFormat = "0.00"; sheet.getRange(`D6:J${end}`).format.numberFormat = "0.0";
  sheet.tables.add(`A5:L${end}`, true, "ParetoScreeningTable");
  sheet.freezePanes.freezeRows(5); sheet.freezePanes.freezeColumns(1);
  setWidths(sheet, [["A:A", 29], ["B:B", 11], ["C:F", 17], ["G:J", 16], ["K:K", 19], ["L:L", 13]]);
  sheet.getRange(`L6:L${end}`).conditionalFormats.addCustom("=L6=TRUE", { fill: "#C6EFCE", font: { color: "#006100", bold: true } });
  return sheet;
}

async function buildOne(cellId, rows) {
  const workbook = Workbook.create();
  packSheet(workbook, rows, cellId);
  rankedSheet(workbook, rows);
  summarySheet(workbook, rows, cellId);
  paretoSheet(workbook, rows, cellId);
  const fileName = `${cellId}_battery_trade_study.xlsx`;
  const xlsxPath = path.join(outputDir, fileName);
  const xlsx = await SpreadsheetFile.exportXlsx(workbook);
  await xlsx.save(xlsxPath);
  // Import the exported artifact for final render/inspection rather than trusting only in-memory state.
  const imported = await SpreadsheetFile.importXlsx(await FileBlob.load(xlsxPath));
  const previewDir = path.join(previewRoot, cellId);
  await fs.mkdir(previewDir, { recursive: true });
  for (const sheetName of ["Pack Configurations", "Ranked Results", "Summary Charts", "Pareto Frontier"]) {
    const image = await imported.render({ sheetName, autoCrop: "all", scale: 1, format: "png" });
    await fs.writeFile(path.join(previewDir, `${sheetName.replaceAll(" ", "_")}.png`), new Uint8Array(await image.arrayBuffer()));
  }
  const summary = await imported.inspect({ kind: "workbook,sheet,table,drawing", maxChars: 4000, tableMaxRows: 4, tableMaxCols: 6 });
  const key = await imported.inspect({ kind: "table", range: "Ranked Results!A4:M9", include: "values,formulas", tableMaxRows: 6, tableMaxCols: 13, maxChars: 4000 });
  const errors = await imported.inspect({ kind: "match", searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A", options: { useRegex: true, maxResults: 100 }, summary: "formula error scan", maxChars: 4000 });
  return { cellId, xlsxPath, previewDir, summary: summary.ndjson, key: key.ndjson, errors: errors.ndjson };
}

async function main() {
  await fs.mkdir(outputDir, { recursive: true });
  await fs.mkdir(previewRoot, { recursive: true });
  const allRows = parseCsv(await fs.readFile(sourceCsv, "utf8"));
  const groups = new Map();
  for (const r of allRows) { if (!groups.has(r.cell_id)) groups.set(r.cell_id, []); groups.get(r.cell_id).push(r); }
  const results = [];
  for (const [cellId, rows] of [...groups.entries()].sort(([a], [b2]) => a.localeCompare(b2))) results.push(await buildOne(cellId, rows));
  await fs.writeFile(path.join(outputDir, "workbook_build_qa.json"), JSON.stringify(results, null, 2));
  console.log(JSON.stringify(results.map(({ cellId, xlsxPath, previewDir, errors }) => ({ cellId, xlsxPath, previewDir, errors })), null, 2));
}

main().catch((error) => { console.error(error); process.exitCode = 1; });
