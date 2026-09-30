// Starts the browser editor: the Emscripten host (materia_web.js/.wasm) owns the
// linear memory and the NativeKit, UIKit and SceneKit C ABIs; the Haxeon guest
// (materia_guest.wasm) is the editor and imports that memory and those functions.
// window.materia reports progress for tests: {state, frames, error}.
"use strict";

const canvas = document.getElementById("canvas");
const statusLine = document.getElementById("status");
const report = window.materia = {state: "loading", frames: 0, error: null, unavailable: []};
const callbacks = new Map();
const dates = [];
let hostMemory = null;
let guest = null;

function fail(message) {
  report.state = "failed";
  report.error = message;
  statusLine.textContent = message;
  statusLine.hidden = false;
  console.error(message);
}

function nativeError() {
  const read = Module["_nk_last_error"];
  const pointer = read ? read() : 0;
  if (!pointer) return "";
  const heap = new Uint8Array(hostMemory.buffer);
  let end = pointer;
  while (heap[end]) end++;
  return new TextDecoder().decode(heap.subarray(pointer, end));
}

// Haxeon wasm32 strings and bytes store their length at +8 and data at +16.
function guestBytes(pointer) {
  if (!pointer) return new Uint8Array(0);
  const length = new DataView(hostMemory.buffer).getInt32(pointer + 8, true);
  return new Uint8Array(hostMemory.buffer, pointer + 16, Math.max(0, length));
}

function hostContract() {
  const read = name => Module["_nkui_haxeon_memory_contract_" + name]() >>> 0;
  const contract = {
    version: read("version"), pageSize: read("page_size"), hostBase: read("host_base"),
    hostLimit: read("host_limit"), guestBase: read("guest_base"), guestLimit: read("guest_limit"),
    memorySize: read("memory_size")
  };
  if (Module["_nkui_haxeon_memory_contract_status"]() !== 0)
    throw new Error("the native heap crossed into the guest's memory");
  if (hostMemory.buffer.byteLength !== contract.memorySize)
    throw new Error("the host memory does not have the contract's size");
  return contract;
}

function checkGuestContract(module, host) {
  const sections = WebAssembly.Module.customSections(module, "haxeon.memory.contract");
  if (sections.length !== 1) throw new Error("the guest has no memory contract");
  const view = new DataView(sections[0]);
  const guestContract = {
    version: view.getUint8(3), pageSize: view.getUint32(4, true), hostBase: view.getUint32(8, true),
    hostLimit: view.getUint32(12, true), guestBase: view.getUint32(16, true),
    guestLimit: view.getUint32(20, true), memorySize: view.getUint32(24, true)
  };
  for (const field in host)
    if (guestContract[field] !== host[field])
      throw new Error(`guest and host memory contracts differ in ${field}`);
}

// Native callbacks: the guest passes a closure and the C signature descriptor
// ("arguments>result", codes as HxiHaxeEmitter writes them); the host gets a
// function-table entry that follows the Wasm32 C ABI and calls back into the guest.
function splitTopLevel(text) {
  const result = [];
  let depth = 0, start = 0;
  for (let index = 0; index < text.length; index++) {
    if (text[index] === "{") depth++;
    else if (text[index] === "}") depth--;
    else if (text[index] === "," && depth === 0) {
      result.push(text.slice(start, index));
      start = index + 1;
    }
  }
  if (start < text.length) result.push(text.slice(start));
  return result.filter(token => token.length > 0);
}

// How one scalar code travels: its Wasm signature letter and DataView accessors.
const scalarCodes = {
  "1": ["i", "Int8"], "2": ["i", "Uint8"], "3": ["i", "Int16"], "4": ["i", "Uint16"], "5": ["i", "Int32"],
  "6": ["i", "Uint32"], "7": ["j", "BigInt64"], "8": ["j", "BigUint64"], "9": ["f", "Float32"],
  "10": ["d", "Float64"], "11": ["i", "Uint32"], "13": ["i", "Uint32"], "14": ["i", "Uint32"]
};

// A record holding one scalar travels as that scalar; any other record as a pointer to its bytes.
function callbackCode(token) {
  token = token.trim();
  if (token[0] !== "{") return {signature: (scalarCodes[token] || ["i"])[0]};
  const [size, , elements] = token.slice(1, -1).split(";");
  const single = elements.includes(",") || elements.includes("{") ? null : scalarCodes[elements];
  return {signature: single ? single[0] : "i", recordSize: Number(size), direct: single ? single[1] : null};
}

function createCallback(signaturePointer, _pointerSizes, _pointerNullable, closure) {
  const descriptor = new TextDecoder().decode(guestBytes(signaturePointer)).split("@")[0];
  const split = descriptor.lastIndexOf(">");
  const argumentCodes = splitTopLevel(descriptor.slice(0, split)).map(callbackCode);
  const resultCode = descriptor.slice(split + 1) === "0" ? null : callbackCode(descriptor.slice(split + 1));
  const indirectResult = resultCode && resultCode.recordSize && !resultCode.direct;
  const wasmSignature = (indirectResult ? "vi" : resultCode ? resultCode.signature : "v") +
    argumentCodes.map(code => code.signature).join("");
  const state = {error: "", errorKind: 0};
  const callback = (...raw) => {
    const resultPointer = indirectResult ? raw.shift() : 0;
    try {
      const args = argumentCodes.map((code, index) => {
        if (!code.recordSize) return raw[index];
        const bytes = guest["__haxeon_callback_alloc_bytes"](code.recordSize);
        if (code.direct)
          new DataView(hostMemory.buffer)["set" + code.direct](bytes + 16, raw[index], true);
        else
          new Uint8Array(hostMemory.buffer, bytes + 16, code.recordSize)
            .set(new Uint8Array(hostMemory.buffer, raw[index], code.recordSize));
        return bytes;
      });
      const result = invokeClosure(closure, args);
      if (!resultCode || !resultCode.recordSize) return result;
      if (!result) throw new Error("the callback returned a null record");
      if (resultCode.direct) return new DataView(hostMemory.buffer)["get" + resultCode.direct](result + 16, true);
      new Uint8Array(hostMemory.buffer, resultPointer, resultCode.recordSize)
        .set(new Uint8Array(hostMemory.buffer, result + 16, resultCode.recordSize));
    } catch (error) {
      state.errorKind = 1;
      state.error = error && (error.stack || error.message) || String(error);
      if (resultPointer) new Uint8Array(hostMemory.buffer, resultPointer, resultCode.recordSize).fill(0);
      return resultCode && resultCode.signature === "j" ? 0n : 0;
    }
  };
  const pointer = Module.addFunction(callback, wasmSignature);
  callbacks.set(pointer, state);
  return pointer;
}

function invokeClosure(closure, args) {
  const value = closure | 0;
  if (!value || !guest || !guest.table) throw new Error("the callback has no closure");
  if (value & 1) return guest.table.get(value >>> 1)(...args);
  const view = new DataView(hostMemory.buffer);
  return guest.table.get(view.getInt32(value + 8, true))(view.getInt32(value + 12, true), ...args);
}

function runtimeImports() {
  return {
    __math_ceil: Math.ceil, __math_floor: Math.floor, __math_round: Math.round,
    __math_is_finite: Number.isFinite, __math_is_nan: Number.isNaN,
    __math_fmod: (left, right) => left % right, __math_pow: Math.pow, __math_sqrt: Math.sqrt,
    __math_sin: Math.sin, __math_cos: Math.cos, __math_tan: Math.tan, __math_atan2: Math.atan2,
    __sys_print: pointer => console.log(new TextDecoder().decode(guestBytes(pointer)).replace(/\n$/, "")),
    // Dates are opaque handles to a millisecond timestamp.
    __date_now: () => dates.push(Date.now()),
    __date_get_time: handle => dates[handle - 1],
    native_callback_create: createCallback,
    native_callback_close: pointer => {
      if (!callbacks.delete(pointer)) return 0;
      Module.removeFunction(pointer);
      return 1;
    },
    native_callback_error_kind: pointer => callbacks.get(pointer)?.errorKind ?? 0,
    native_callback_take_error: pointer => {
      const state = callbacks.get(pointer);
      if (!state || !state.errorKind) return 0;
      const encoded = new TextEncoder().encode(state.error);
      const bytes = guest["__haxeon_callback_alloc_bytes"](encoded.length);
      new Uint8Array(hostMemory.buffer, bytes + 16, encoded.length).set(encoded);
      state.error = "";
      state.errorKind = 0;
      return bytes;
    }
  };
}

// Every C function the guest imports comes from the host when it links that
// library. Kits the browser build does not link yet (CadKit, SimKit, RobotKit,
// AnimKit, StockKit) get stubs that throw, so only the features using them fail.
function guestImports(module) {
  const imports = {
    env: {memory: hostMemory},
    std: {
      sys_exit: () => {},
      sys_time: () => performance.now() / 1000,
      sys_getpid: () => 1
    },
    haxeon_runtime: runtimeImports()
  };
  const unavailable = new Set();
  for (const entry of WebAssembly.Module.imports(module)) {
    if (entry.kind !== "function" || imports[entry.module]?.[entry.name]) continue;
    const table = imports[entry.module] ??= {};
    const hosted = Module["_" + entry.name];
    if (typeof hosted === "function" && entry.module !== "std" && entry.module !== "haxeon_runtime") {
      table[entry.name] = hosted;
      continue;
    }
    unavailable.add(entry.module);
    table[entry.name] = () => {
      throw new Error(`${entry.module}.${entry.name} is not available in the browser build`);
    };
  }
  report.unavailable = [...unavailable].sort();
  return imports;
}

function frame(time) {
  try {
    const result = guest["app.MainWeb.frame"](time);
    if (result < 0) {
      fail("The editor stopped with an error; see the console. " + nativeError());
      return;
    }
    report.frames++;
    if (report.state === "loading") {
      report.state = "running";
      statusLine.hidden = true;
    }
    if (result > 0) requestAnimationFrame(frame);
    else report.state = "stopped";
  } catch (error) {
    fail("The editor failed: " + (error.stack || error.message));
  }
}

async function startGuest() {
  const host = hostContract();
  const module = await WebAssembly.compileStreaming(fetch("materia_guest.wasm"));
  checkGuestContract(module, host);
  guest = (await WebAssembly.instantiate(module, guestImports(module))).exports;
  const dark = matchMedia("(prefers-color-scheme: dark)").matches ? 1 : 0;
  if (guest["app.MainWeb.configure"](canvas.clientWidth, canvas.clientHeight, dark) !== 0)
    throw new Error("the editor rejected the canvas size");
  if (guest["app.MainWeb.main"]() !== 0)
    throw new Error("the editor did not start; see the console. " + nativeError());
  requestAnimationFrame(frame);
}

var Module = {
  canvas,
  instantiateWasm(imports, receiveInstance) {
    WebAssembly.instantiateStreaming(fetch("materia_web.wasm"), imports)
      .then(result => {
        hostMemory = Object.values(result.instance.exports).find(value => value instanceof WebAssembly.Memory);
        receiveInstance(result.instance);
      })
      .catch(error => fail("The NativeKit host failed to load: " + error.message));
    return {};
  },
  onRuntimeInitialized() {
    if (!hostMemory) {
      fail("The NativeKit host did not export its memory");
      return;
    }
    startGuest().catch(error => fail("The editor failed to start: " + (error.stack || error.message)));
  },
  print: text => console.log(text),
  printErr: text => console.error(text)
};
