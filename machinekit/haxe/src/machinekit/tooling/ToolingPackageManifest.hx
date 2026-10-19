package machinekit.tooling;

import materia.tooling.export.ToolingFabrication.FabricationPlan;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingValidation.ToolingReadiness;
import materia.tooling.ToolingCodec;
import materia.tooling.export.ToolingManifest;

/** Deterministic inventory of a freshly generated directory, excluding this manifest itself. */
class ToolingPackageManifest {
	static function digest(bytes:haxe.io.Bytes):String {
		var value = haxe.crypto.Sha256.make(bytes), hex = new StringBuf();
		for (i in 0...value.length)
			hex.add(StringTools.hex(value.get(i), 2).toLowerCase());
		return hex.toString();
	}

	public static function write(dir:String, f:FabricationPlan, checks:Array<ToolingValidation>):Void {
		var records:Array<ToolingArtifact> = [];
		function visit(relative:String):Void {
			var names = sys.FileSystem.readDirectory(dir + (relative == "" ? "" : "/" + relative));
			names.sort(Reflect.compare);
			for (name in names) {
				var path = relative == "" ? name : relative + "/" + name;
				if (path == "manifest.json")
					continue;
				if (sys.FileSystem.isDirectory(dir + "/" + path))
					visit(path);
				else {
					if (StringTools.endsWith(path, ".step"))
						reproducibleStep(dir + "/" + path);
					var bytes = sys.io.File.getBytes(dir + "/" + path);
					records.push({path: path, bytes: bytes.length, sha256: digest(bytes)});
				}
			}
		}
		var all = f.validation.concat(checks);
		all.sort((a, b) -> Reflect.compare(a.rule, b.rule));
		var seen = new Map<String, Bool>();
		for (check in all) {
			if (seen.exists(check.rule))
				throw "Duplicate fabrication rule";
			seen.set(check.rule, true);
		}
		sys.io.File.saveContent(dir + "/fabrication-validation.json", ToolingCodec.canonical(all) + "\n");
		sys.io.File.saveContent(dir + "/fabrication-diagnostics.json", ToolingCodec.canonical(ToolingDiagnostics.fromValidation(all).items) + "\n");
		// The two final reports also participate in the hash inventory.
		records = [];
		visit("");
		records.sort((a, b) -> Reflect.compare(a.path, b.path));
		var manifest:ToolingManifest = {
			format: "materia.tooling-manifest",
			version: 1,
			phase: 5,
			lengthUnit: materia.tooling.ToolingEnums.ToolingLengthUnit.Millimetres,
			sourceFingerprint: f.sourceFingerprint,
			fabricationFingerprint: f.fingerprint,
			packageFingerprint: haxe.crypto.Sha256.encode(ToolingCodec.canonical({
				input: f.fingerprint,
				files: records
			})),
			files: records,
			fabrication: ToolingReadiness.evaluate(all),
			validation: all,
			replacementGroups: f.replacementGroups
		};
		sys.io.File.saveContent(dir + "/manifest.json", ToolingCodec.canonical(manifest) + "\n");
	}

	/** Normalize only the generated STEP timestamp, never its geometry. */
	static function reproducibleStep(path:String):Void {
		var s = sys.io.File.getContent(path),
			prefix = "FILE_NAME('Open CASCADE Shape Model','";
		var start = s.indexOf(prefix);
		if (start < 0)
			throw "Generated STEP has an unsupported header";
		start += prefix.length;
		var end = s.indexOf("'", start);
		if (end < start)
			throw "Generated STEP has an invalid header";
		var normalized = s.substr(0, start) + "1970-01-01T00:00:00" + s.substr(end);
		if (normalized != s)
			sys.io.File.saveContent(path, normalized);
	}

	public static function verify(dir:String):Void {
		var manifest:Dynamic = haxe.Json.parse(sys.io.File.getContent(dir + "/manifest.json"));
		if (manifest.format != "materia.tooling-manifest" || manifest.version != 1)
			throw "Unsupported tooling manifest";
		if (!Std.isOfType(manifest.files, Array))
			throw "Invalid tooling manifest files";
		var files:Array<Dynamic> = cast manifest.files, previous = "";
		for (file in files) {
			var name:String = file.path;
			if (name == null || name.length == 0 || name.indexOf("\\") >= 0 || name == "manifest.json" || Reflect.compare(previous, name) >= 0)
				throw "Invalid tooling manifest path/order";
			for (part in name.split("/"))
				if (part == "" || part == "." || part == "..")
					throw "Invalid tooling manifest path";
			previous = name;
			var path = dir + "/" + name;
			if (!sys.FileSystem.exists(path) || sys.FileSystem.isDirectory(path))
				throw "Missing tooling artifact: " + name;
			var bytes = sys.io.File.getBytes(path);
			if (bytes.length != file.bytes || digest(bytes) != file.sha256)
				throw "Tooling artifact hash differs: " + name;
		}
		var expected = haxe.crypto.Sha256.encode(ToolingCodec.canonical({input: manifest.fabricationFingerprint, files: files}));
		if (expected != manifest.packageFingerprint)
			throw "Tooling package fingerprint differs";
	}
}
