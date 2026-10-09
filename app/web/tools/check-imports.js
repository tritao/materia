#!/usr/bin/env node
"use strict";
require("../../../tools/web/check-imports.js");

// Native editor features must resolve at build time. Only CadKit is optional;
// runtime/host services below are implemented by the JavaScript bridge.
const fs = require("fs");
const [guestPath, , hostScript] = process.argv.slice(2);
const services = new Set(["std", "env", "haxeon_runtime", "haxeon_host", "materia_web_files"]);
const hostExports = new Set([...fs.readFileSync(hostScript, "utf8").matchAll(/Module\["_(\w+)"\]\s*=\s*wasmExports\["([\w$]+)"\]/g)].map(match => match[1]));
const missing = WebAssembly.Module.imports(new WebAssembly.Module(fs.readFileSync(guestPath)))
  .filter(entry => entry.kind === "function" && !services.has(entry.module) &&
    !(entry.module === "cadkit-core" && !process.env.MATERIA_WEB_OCCT_DIR) && !hostExports.has(entry.name));
if (missing.length) throw new Error("Missing required browser native functions: " + missing.map(entry => `${entry.module}.${entry.name}`).join(", "));
console.log("Required browser native functions are exported");
