// Runs the docs runtime under Node the way the site does, and checks a G-code program end to end:
// moves come back in millimetres with their source lines, and an error points at its line and column.
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { loadRuntime } from "../site/src/scripts/runtime.ts";

const bytes = readFileSync(fileURLToPath(new URL("docs-runtime.wasm", import.meta.url)));
const runtime = await loadRuntime(bytes);
const ok = runtime.gcode("G21 G90\nG0 X0 Y0 Z5\nG1 Z-1 F300\nG2 X20 Y0 I10 J0\nM30\n");
const kinds = ok.moves.map((move) => move.kind).join(",");
if (ok.diagnostics.length || kinds !== "rapid,cut,cut") throw new Error(`unexpected result: ${JSON.stringify(ok)}`);
const arc = ok.moves[2];
const end = arc.points.at(-1);
if (arc.line !== 4 || arc.points.length < 10 || Math.abs(end[0] - 20) > 1e-6 || Math.abs(end[1]) > 1e-6)
	throw new Error(`arc not sampled as expected: ${JSON.stringify(arc)}`);
const bad = runtime.gcode("G1 X10 F100\nG5 X3\n");
const error = bad.diagnostics.find((d) => d.severity === "error");
if (!error || error.line !== 2) throw new Error(`no positioned error: ${JSON.stringify(bad.diagnostics)}`);
console.log(`docs runtime: ${ok.moves.length} moves; error "${error.code}" at line ${error.line}, column ${error.column}`);
// A heavier carriage accelerates less; a bigger pulley trades acceleration for speed.
const light = runtime.belt(2, 0.45, 60).rows, heavy = runtime.belt(10, 0.45, 60).rows;
const at = (rows, teeth) => rows.find((row) => row.teeth === teeth);
if (light.length !== 25 || !(at(heavy, 20).acceleration < at(light, 20).acceleration))
	throw new Error(`belt limits do not fall with mass: ${JSON.stringify([at(light, 20), at(heavy, 20)])}`);
if (!(at(light, 40).velocity > at(light, 20).velocity && at(light, 40).acceleration < at(light, 20).acceleration))
	throw new Error(`pulley size does not trade acceleration for speed: ${JSON.stringify([at(light, 20), at(light, 40)])}`);
const row = at(light, 20);
console.log(`belt axis, 20 teeth, 2 kg: ${row.velocity.toFixed(2)} m/s, ${row.acceleration.toFixed(1)} m/s²`);
