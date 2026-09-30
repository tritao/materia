#!/usr/bin/env node
// Checks that every function the guest imports from the host has the same Wasm type as the host
// export it will be bound to, and lists every mismatch at once (instantiation reports only the first).
//   node check-imports.js GUEST.wasm HOST.wasm HOST.js
"use strict";

const fs = require("fs");

const valueTypes = {0x7f: "i32", 0x7e: "i64", 0x7d: "f32", 0x7c: "f64", 0x7b: "v128", 0x70: "funcref", 0x6f: "externref"};

function readModule(path) {
  const bytes = fs.readFileSync(path);
  let offset = 8;
  const u32 = () => {
    let result = 0, shift = 0, byte;
    do {
      byte = bytes[offset++];
      result |= (byte & 0x7f) << shift;
      shift += 7;
    } while (byte & 0x80);
    return result >>> 0;
  };
  const name = () => {
    const length = u32();
    const text = bytes.toString("utf8", offset, offset + length);
    offset += length;
    return text;
  };
  const skipLimits = () => {
    const flags = u32();
    u32();
    if (flags & 1) u32();
  };
  const types = [], functionTypes = [], imports = [], exports = [];
  while (offset < bytes.length) {
    const id = bytes[offset++], size = u32(), end = offset + size;
    if (id === 1) {
      for (let count = u32(); count > 0; count--) {
        if (bytes[offset] !== 0x60) throw new Error(`${path}: unsupported type form ${bytes[offset]}`);
        offset++;
        const parameters = [], results = [];
        for (let n = u32(); n > 0; n--) parameters.push(valueTypes[bytes[offset++]] || "?");
        for (let n = u32(); n > 0; n--) results.push(valueTypes[bytes[offset++]] || "?");
        types.push(`(${parameters.join(",")})->(${results.join(",")})`);
      }
    } else if (id === 2) {
      for (let count = u32(); count > 0; count--) {
        const module = name(), field = name(), kind = bytes[offset++];
        if (kind === 0) {
          const type = types[u32()];
          imports.push({module, field, type});
          functionTypes.push(type);
        } else if (kind === 1) {
          offset++;
          skipLimits();
        } else if (kind === 2) skipLimits();
        else if (kind === 3) offset += 2;
        else if (kind === 4) offset += 1 + 1 + 1;
      }
    } else if (id === 3) {
      for (let count = u32(); count > 0; count--) functionTypes.push(types[u32()]);
    } else if (id === 7) {
      for (let count = u32(); count > 0; count--) {
        const field = name(), kind = bytes[offset++], index = u32();
        if (kind === 0) exports.push({field, type: functionTypes[index]});
      }
    }
    offset = end;
  }
  return {imports, exports: new Map(exports.map(entry => [entry.field, entry.type]))};
}

const [guestPath, hostPath, hostScript] = process.argv.slice(2);
const guest = readModule(guestPath), host = readModule(hostPath);
// Emscripten minifies export names; its JS binds Module["_name"] = wasmExports["xy"].
const hostNames = new Map();
for (const match of fs.readFileSync(hostScript, "utf8").matchAll(/Module\["_(\w+)"\]\s*=\s*wasmExports\["(\w+)"\]/g))
  hostNames.set(match[1], match[2]);
const mismatches = [];
for (const entry of guest.imports) {
  const exported = hostNames.get(entry.field);
  if (exported === undefined) continue;
  const hostType = host.exports.get(exported);
  if (hostType !== entry.type)
    mismatches.push(`${entry.module}.${entry.field}: guest ${entry.type}, host ${hostType}`);
}
if (mismatches.length > 0) {
  console.error(`check-imports: ${mismatches.length} guest imports differ from the host's C functions:`);
  for (const line of mismatches) console.error("  " + line);
  process.exit(1);
}
console.log(`check-imports: ${guest.imports.length} guest imports, all host-bound ones match`);
