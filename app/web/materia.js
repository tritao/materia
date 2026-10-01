// Starts the browser editor: the Emscripten host (materia_web.js/.wasm) owns the linear memory and the
// NativeKit, UIKit and SceneKit C ABIs; the Haxeon guest (materia_guest.wasm, wasm32 or wasm-gc) is the editor.
// haxeon-host.js connects them. window.materia reports progress for tests: {state, frames, error, unavailable, restoredFiles}, and
// window.materia.inspect() prints the editor's state as a `materia-report` console line (app.MainWeb.report).
"use strict";

const canvas = document.getElementById("canvas");
const statusLine = document.getElementById("status");
const report = window.materia = {state: "loading", frames: 0, error: null, unavailable: [], restoredFiles: 0,
  // The host heap in bytes (NativeKit's allocator): {arena, inUse, peak, largestFree, freeBlocks}.
  memory: () => {
    const statistic = Module._nk_wasm_host_allocator_statistic;
    if (!statistic) return null;
    return {arena: statistic(0) >>> 0, inUse: statistic(1) >>> 0, peak: statistic(2) >>> 0,
      largestFree: statistic(3) >>> 0, freeBlocks: statistic(4) >>> 0};
  },
  inspect: () => guest ? guest["app.MainWeb.report"]() : -1};
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
  const pointer = Module["_nk_last_error"] ? Module["_nk_last_error"]() : 0;
  if (!pointer) return "";
  const bytes = new Uint8Array(hostMemory.buffer);
  let end = pointer;
  while (bytes[end] !== 0) end++;
  return new TextDecoder().decode(bytes.subarray(pointer, end));
}

// The host's half of the linear-memory partition the guest was compiled against.
function hostContract() {
  const read = name => Module["_nkui_haxeon_memory_contract_" + name]() >>> 0;
  if (Module["_nkui_haxeon_memory_contract_status"]() !== 0)
    throw new Error("the native heap crossed into the guest's memory");
  const contract = {
    version: read("version"), pageSize: read("page_size"), hostBase: read("host_base"), hostLimit: read("host_limit"),
    guestBase: read("guest_base"), guestLimit: read("guest_limit"), memorySize: read("memory_size")
  };
  if (hostMemory.buffer.byteLength !== contract.memorySize)
    throw new Error("the host memory does not have the contract's size");
  return contract;
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

// The editor's files across page loads (MateriaWebFiles.hxi, app.BrowserFiles): `load` reads what earlier sessions
// stored in the origin private file system before the guest starts, the guest pulls it once, and every later change
// is queued and applied to OPFS in order. Storage failures are logged; the session keeps working in memory.
const files = (() => {
  const ROOT = "materia-files";
  const decoder = new TextDecoder(), encoder = new TextEncoder();
  let restored = [], queue = Promise.resolve();
  const memoryBytes = () => new Uint8Array(hostMemory.buffer);
  const cString = pointer => {
    const bytes = memoryBytes();
    let end = pointer;
    while (bytes[end] !== 0) end++;
    return decoder.decode(bytes.subarray(pointer, end));
  };
  const names = path => path.split("/").filter(name => name !== "");
  const root = async () => (await navigator.storage.getDirectory()).getDirectoryHandle(ROOT, {create: true});
  const directory = async (path, create) => {
    let handle = await root();
    for (const name of path) handle = await handle.getDirectoryHandle(name, {create});
    return handle;
  };
  const write = async (parent, name, data) => {
    const writable = await (await parent.getFileHandle(name, {create: true})).createWritable();
    await writable.write(data);
    await writable.close();
  };
  const copy = async (source, parent, name) => {
    if (source.kind === "file") return write(parent, name, await source.getFile());
    const target = await parent.getDirectoryHandle(name, {create: true});
    for await (const [child, handle] of source.entries()) await copy(handle, target, child);
  };
  const enqueue = (action, operation) => {
    queue = queue.then(operation).catch(error => console.warn(`materia: could not ${action}: ${error.message}`));
  };
  // Answers a guest out-buffer call: a null pointer asks for the size, then the guest passes a buffer that big.
  const answer = (value, pointer, sizePointer) => {
    const view = new DataView(hostMemory.buffer);
    if (pointer === 0) {
      view.setUint32(sizePointer, value.length, true);
      return;
    }
    const size = Math.min(view.getUint32(sizePointer, true), value.length);
    memoryBytes().set(value.subarray(0, size), pointer);
    view.setUint32(sizePointer, size, true);
  };
  async function load() {
    if (!navigator.storage?.getDirectory) return;
    const walk = async (handle, prefix) => {
      const children = [];
      for await (const entry of handle.entries()) children.push(entry);
      children.sort(([left], [right]) => left < right ? -1 : left > right ? 1 : 0);
      for (const [name, child] of children) {
        const path = prefix + "/" + name;
        if (child.kind === "directory") {
          restored.push({path, directory: true});
          await walk(child, path);
        } else {
          restored.push({path, directory: false, bytes: new Uint8Array(await (await child.getFile()).arrayBuffer())});
        }
      }
    };
    try {
      await walk(await root(), "");
    } catch (error) {
      console.warn("materia: could not read stored files: " + error.message);
      restored = [];
    }
    report.restoredFiles = restored.filter(entry => !entry.directory).length;
  }
  const imports = {
    _materia_files_restore_count: () => restored.length,
    _materia_files_restore_path: (index, pointer, sizePointer) => {
      const entry = restored[index];
      if (!entry) return -1;
      answer(encoder.encode(entry.path), pointer, sizePointer);
      return entry.directory ? 0 : 1;
    },
    _materia_files_restore_content: (index, pointer, sizePointer) => {
      const entry = restored[index];
      if (!entry || entry.directory) return -1;
      answer(entry.bytes, pointer, sizePointer);
      return 0;
    },
    _materia_files_restore_done: () => { restored = []; },
    _materia_files_directory_created: pointer => {
      const path = names(cString(pointer));
      enqueue("create /" + path.join("/"), () => directory(path, true));
    },
    _materia_files_file_written: (pointer, content, length) => {
      const path = names(cString(pointer)), data = memoryBytes().slice(content, content + length);
      enqueue("save /" + path.join("/"), async () => write(await directory(path.slice(0, -1), true), path.at(-1), data));
    },
    _materia_files_removed: pointer => {
      const path = names(cString(pointer));
      enqueue("remove /" + path.join("/"), async () =>
        (await directory(path.slice(0, -1), false)).removeEntry(path.at(-1)));
    },
    // OPFS has no portable move, so a rename copies the file or tree and removes the original.
    _materia_files_renamed: (from, to) => {
      const source = names(cString(from)), target = names(cString(to));
      enqueue(`move /${source.join("/")} to /${target.join("/")}`, async () => {
        const sourceParent = await directory(source.slice(0, -1), false), targetParent = await directory(target.slice(0, -1), true);
        let handle;
        try {
          handle = await sourceParent.getFileHandle(source.at(-1));
        } catch {
          handle = await sourceParent.getDirectoryHandle(source.at(-1));
        }
        await targetParent.removeEntry(target.at(-1), {recursive: true}).catch(() => {});
        await copy(handle, targetParent, target.at(-1));
        await sourceParent.removeEntry(source.at(-1), {recursive: true});
      });
    }
  };
  // Resolves once every queued change is in storage (tests and page unloads can wait on it).
  const settled = () => queue;
  return {load, imports, settled};
})();
report.filesSettled = () => files.settled();

async function startGuest() {
  await files.load();
  const started = await HaxeonWasmHost.instantiate(fetch("materia_guest.wasm"),
    {emscripten: Module, memory: hostMemory, contract: hostContract()});
  guest = started.exports;
  report.unavailable = started.unavailable;
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
  printErr: text => console.error(text),
  // haxeon-host.js binds the guest's materia_web_files imports to these.
  ...files.imports
};
