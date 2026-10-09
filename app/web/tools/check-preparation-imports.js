#!/usr/bin/env node
"use strict";
// Preparation must remain portable: no rendering, native libraries, callbacks or filesystem host.
const fs = require("fs");
const moduleBytes = new WebAssembly.Module(fs.readFileSync(process.argv[2]));
const allowed = new Set(["materia_preparation", "std", "haxeon_host", "haxeon_runtime", "env"]);
for (const entry of WebAssembly.Module.imports(moduleBytes)) {
  if (!allowed.has(entry.module) || (entry.kind !== "function" && !(entry.module === "env" && entry.name === "memory")))
    throw new Error(`Nonportable preparation import: ${entry.module}.${entry.name}`);
  if ((entry.module === "haxeon_runtime" && !entry.name.startsWith("__math_")) ||
      (entry.module === "haxeon_host" && entry.name.startsWith("callback_")) ||
      (entry.module === "std" && !["sys_time", "sys_cpu_time", "sys_exit"].includes(entry.name)))
    throw new Error(`Unsupported preparation runtime dependency: ${entry.module}.${entry.name}`);
}
console.log("Preparation guest imports are portable");
