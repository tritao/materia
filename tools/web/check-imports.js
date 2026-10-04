#!/usr/bin/env node
// Checks that every function the guest imports from the host has the same Wasm type as the host
// export it will be bound to, and lists every mismatch at once (instantiation reports only the first).
//   node check-imports.js GUEST.wasm HOST.wasm HOST.js
"use strict";

const fs = require("fs");

const valueTypes = {0x7f: "i32", 0x7e: "i64", 0x7d: "f32", 0x7c: "f64", 0x7b: "v128", 0x78: "i8", 0x77: "i16"};
const heapTypes = {0x73: "nofunc", 0x72: "noextern", 0x71: "none", 0x70: "func", 0x6f: "extern", 0x6e: "any", 0x6d: "eq",
  0x6c: "i31", 0x6b: "struct", 0x6a: "array", 0x69: "exn", 0x74: "noexn"};

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
  // A heap type is an abstract type byte or a non-negative type index (s33).
  const heapType = () => {
    if (heapTypes[bytes[offset]] !== undefined) return heapTypes[bytes[offset++]];
    return String(u32());
  };
  const valueType = () => {
    const code = bytes[offset++];
    if (valueTypes[code]) return valueTypes[code];
    if (code === 0x64 || code === 0x63) return `(ref ${code === 0x63 ? "null " : ""}${heapType()})`;
    if (heapTypes[code]) return `${heapTypes[code]}ref`;
    throw new Error(`${path}: unsupported value type ${code}`);
  };
  // One type-section entry: a function, struct or array type, optionally declared as a subtype.
  const subtype = () => {
    if (bytes[offset] === 0x50 || bytes[offset] === 0x4f) {
      offset++;
      for (let n = u32(); n > 0; n--) u32();
    }
    const form = bytes[offset++];
    if (form === 0x60) {
      const parameters = [], results = [];
      for (let n = u32(); n > 0; n--) parameters.push(valueType());
      for (let n = u32(); n > 0; n--) results.push(valueType());
      return `(${parameters.join(",")})->(${results.join(",")})`;
    }
    if (form === 0x5f) {
      for (let n = u32(); n > 0; n--) {
        valueType();
        offset++;
      }
      return "struct";
    }
    if (form === 0x5e) {
      valueType();
      offset++;
      return "array";
    }
    throw new Error(`${path}: unsupported type form ${form}`);
  };
  const types = [], functionTypes = [], imports = [], exports = [];
  while (offset < bytes.length) {
    const id = bytes[offset++], size = u32(), end = offset + size;
    if (id === 1) {
      for (let count = u32(); count > 0; count--) {
        if (bytes[offset] === 0x4e) {
          offset++;
          for (let n = u32(); n > 0; n--) types.push(subtype());
        } else types.push(subtype());
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
