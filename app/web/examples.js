"use strict";

// Network adapter only. The guest owns persisted files and decides which verified resources are missing.
const MateriaExamples = {
  create(memory) {
    const encoder = new TextEncoder(), decoder = new TextDecoder();
    let catalog = {version: 1, examples: []}, catalogBytes = encoder.encode(JSON.stringify(catalog));
    let base, active = null, nextId = 1;
    const requests = [];
    const string = pointer => {
      const bytes = new Uint8Array(memory().buffer);
      let end = pointer;
      while (bytes[end]) end++;
      return decoder.decode(bytes.subarray(pointer, end));
    };
    const answer = (bytes, pointer, sizePointer) => {
      const view = new DataView(memory().buffer);
      if (!pointer) { view.setUint32(sizePointer, bytes.length, true); return; }
      const size = Math.min(view.getUint32(sizePointer, true), bytes.length);
      new Uint8Array(memory().buffer, pointer, size).set(bytes.subarray(0, size));
      view.setUint32(sizePointer, size, true);
    };
    async function load() {
      const url = new URL(window.MATERIA_EXAMPLES_URL || "examples/catalog.json", location.href);
      try {
        const response = await fetch(url);
        if (!response.ok) throw new Error(`catalog HTTP ${response.status}`);
        const text = await response.text();
        if (text.length > 2000000) throw new Error("catalog is too large");
        const value = JSON.parse(text), ids = new Set();
        if (value.version !== 1 || !Array.isArray(value.examples) || value.examples.length > 100)
          throw new Error("unsupported catalog");
        for (const entry of value.examples) {
          if (typeof entry.id !== "string" || !/^[a-z0-9-]+$/.test(entry.id) || ids.has(entry.id) ||
              typeof entry.source !== "string" || !Array.isArray(entry.files) || entry.files.length > 20)
            throw new Error("invalid example entry");
          ids.add(entry.id);
          const paths = new Set();
          for (const file of entry.files) {
            if (!/^[a-f0-9]{64}$/.test(file.sha256) ||
                ![".mtrg", ".materia", ".glb"].some(extension => file.url === `objects/${file.sha256}${extension}`) ||
                !Number.isInteger(file.bytes) || file.bytes <= 0 || file.bytes > 150000000 ||
                typeof file.path !== "string" || file.path.includes("..") || paths.has(file.path) ||
                !["/files/examples/", "/app/examples/", "/animkit/assets/"].some(prefix => file.path.startsWith(prefix)))
              throw new Error("invalid example resource");
            paths.add(file.path);
          }
          if (entry.source && !paths.has(entry.source)) throw new Error("example source is missing");
        }
        catalog = value;
        catalogBytes = encoder.encode(JSON.stringify(value));
        base = new URL(".", url);
      } catch (error) {
        console.warn("materia: examples unavailable: " + error.message);
      }
    }
    async function download(job, files) {
      try {
        let loaded = 0;
        const total = files.reduce((sum, file) => sum + file.bytes, 0);
        const progress = () => { job.phase = `Downloading example · ${(loaded / 1000000).toFixed(1)} / ${(total / 1000000).toFixed(1)} MB`; };
        for (const file of files) {
          if (active !== job) return;
          const request = {id: job.example, url: file.url, status: "downloading"};
          requests.push(request);
          if (requests.length > 200) requests.shift();
          const response = await fetch(new URL(file.url, base), {signal: job.controller.signal});
          if (!response.ok) throw new Error(`Example download failed (HTTP ${response.status}). Try again.`);
          const bytes = new Uint8Array(file.bytes), reader = response.body.getReader();
          let offset = 0;
          try {
            for (;;) {
              const {done, value} = await reader.read();
              if (done) break;
              if (offset + value.length > bytes.length) throw new Error("Example download exceeds its declared size");
              bytes.set(value, offset); offset += value.length; loaded += value.length; progress();
            }
          } finally { reader.releaseLock(); }
          if (offset !== bytes.length) throw new Error("Example download is incomplete. Try again.");
          // The guest verifies SHA-256 before publishing into its filesystem.
          request.status = "downloaded";
          job.contents.push(bytes);
        }
        if (active === job) { job.status = 1; job.phase = "Checking downloaded example"; }
      } catch (error) {
        if (active === job) { job.status = -1; job.error = error.message; }
      }
    }
    const imports = {
      _materia_examples_catalog: (pointer, size) => { answer(catalogBytes, pointer, size); return 0; },
      _materia_example_start: (idPointer, pathsPointer) => {
        if (active) return -1;
        const example = string(idPointer), entry = catalog.examples.find(item => item.id === example);
        if (!entry) return -1;
        const paths = JSON.parse(string(pathsPointer));
        if (!Array.isArray(paths) || new Set(paths).size !== paths.length) return -1;
        const files = paths.map(path => entry.files.find(file => file.path === path));
        if (files.some(file => !file)) return -1;
        const job = active = {id: nextId++, example, status: 0, phase: "Downloading example", contents: [],
          controller: new AbortController(), error: "Example download failed"};
        download(job, files);
        return job.id;
      },
      _materia_example_status: id => active?.id === id ? active.status : -1,
      _materia_example_phase: (id, pointer, size) => {
        if (active?.id !== id) return -1;
        answer(encoder.encode(active.phase), pointer, size); return 0;
      },
      _materia_example_error: (id, pointer, size) => {
        if (active?.id !== id) return -1;
        answer(encoder.encode(active.error), pointer, size); return 0;
      },
      _materia_example_content: (id, index, pointer, size) => {
        const bytes = active?.id === id && active.contents[index];
        if (!bytes) return -1;
        answer(bytes, pointer, size); return 0;
      },
      _materia_example_cancel: id => {
        if (active?.id !== id) return;
        active.controller.abort(); active = null;
      }
    };
    return {load, imports, inspect: () => ({catalog: catalog.examples.map(entry => entry.id), requests: requests.slice(),
      active: active ? {id: active.id, example: active.example, phase: active.phase, status: active.status} : null})};
  }
};
