"use strict";
importScripts("haxeon-host.js");
const VERSION = 1;
let started;
let input, job;
let memory;
const encoder = new TextEncoder(), decoder = new TextDecoder();
function text(pointer) {
  const bytes = new Uint8Array(memory.buffer);
  let end = pointer;
  while (bytes[end]) end++;
  return decoder.decode(bytes.subarray(pointer, end));
}
function answer(bytes, pointer, sizePointer) {
  const view = new DataView(memory.buffer);
  if (!pointer) { view.setUint32(sizePointer, bytes.length, true); return; }
  const size = Math.min(view.getUint32(sizePointer, true), bytes.length);
  new Uint8Array(memory.buffer, pointer, size).set(bytes.subarray(0, size));
  view.setUint32(sizePointer, size, true);
}
const send = (type, fields = {}, transfer = []) => postMessage({version: VERSION, id: job, type, ...fields}, transfer);
async function initialize() {
  // Private fixed memory: no SharedArrayBuffer, native libraries, UI host or filesystem.
  memory = new WebAssembly.Memory({initial: 8192, maximum: 8192});
  return HaxeonWasmHost.instantiate(fetch("materia_preparation.wasm"), {
    memory, emscripten: {
      _materia_prepare_input: (pointer, size) => { answer(input, pointer, size); return 0; },
      _materia_prepare_phase: pointer => send("progress", {phase: text(pointer)}),
      _materia_prepare_complete: (pointer, length) => {
        const bytes = new Uint8Array(memory.buffer).slice(pointer, pointer + length);
        send("complete", {bytes: bytes.buffer}, [bytes.buffer]);
      },
      _materia_prepare_error: pointer => send("error", {error: text(pointer)})
    },
    print: line => console.log(line)
  });
}
onmessage = async ({data}) => {
  if (data.version !== VERSION || data.type !== "prepare" || input) return;
  job = data.id;
  input = new Uint8Array(data.artifact);
  try {
    started ??= initialize();
    const guest = await started;
    guest.exports["app.PreparationWorkerMain.main"]();
  } catch (error) { send("error", {error: error.stack || error.message || String(error)}); }
  finally { input = null; }
};
