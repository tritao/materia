/**
 * Loader for the docs runtime (docs/runtime): real Materia kits compiled to Wasm GC.
 *
 * The guest's exports take scalars only. A call sends its text one character code at a time, runs the
 * operation, and reads back the one JSON line the guest prints.
 */
export interface GcodeDiagnostic {
	severity: "error" | "warning";
	code: string;
	line: number;
	column: number;
	message: string;
}

export interface GcodeMove {
	kind: string;
	line: number;
	/** Points in millimetres. */
	points: [number, number, number][];
}

export interface DocsRuntime {
	gcode(source: string): { diagnostics: GcodeDiagnostic[]; moves: GcodeMove[] };
}

export async function loadRuntime(source: BufferSource | Response | Promise<Response>): Promise<DocsRuntime> {
	let memory: WebAssembly.Memory | undefined;
	let printed = "";
	const decoder = new TextDecoder();
	const imports = {
		haxeon_host: {
			print(address: number) {
				const bytes = new Uint8Array(memory!.buffer);
				let end = address;
				while (bytes[end] !== 0) end++;
				printed += decoder.decode(bytes.subarray(address, end));
			},
		},
		haxeon_runtime: {
			__math_pow: Math.pow,
			__math_cos: Math.cos,
			__math_sin: Math.sin,
			__math_atan2: Math.atan2,
			__math_round: (value: number) => Math.floor(value + 0.5),
		},
	};
	let instance: WebAssembly.Instance;
	if (source instanceof Response || source instanceof Promise) {
		const response = await source;
		if (!response.ok) throw new Error(`docs runtime: HTTP ${response.status}`);
		// Streaming needs the application/wasm type; fall back to bytes when a server sends another.
		instance = (
			response.headers.get("content-type")?.startsWith("application/wasm")
				? await WebAssembly.instantiateStreaming(response, imports)
				: await WebAssembly.instantiate(await response.arrayBuffer(), imports)
		).instance;
	} else instance = (await WebAssembly.instantiate(source, imports)).instance;
	const exports = instance.exports as Record<string, any>;
	memory = exports.memory;
	exports.main();

	function call(operation: string, text: string): any {
		printed = "";
		for (const character of text) exports["DocsRuntime.char"](character.codePointAt(0));
		exports[operation]();
		const line = printed.trim().split("\n").at(-1);
		if (!line) throw new Error(`docs runtime: ${operation} printed nothing`);
		return JSON.parse(line);
	}

	return { gcode: (text) => call("DocsRuntime.gcode", text) };
}
