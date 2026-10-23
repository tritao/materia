package machinekit.tooling;

import materia.tooling.export.ToolingFabrication.FabricationPlan;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingValidation.ToolingReadiness;
import materia.tooling.ToolingCodec;
import materia.tooling.export.ToolingManifest;

/** Deterministic inventory of a freshly generated directory, excluding this manifest itself. */
class ToolingPackageManifest {
	static function fingerprint(manifest:Dynamic):String {
		return haxe.crypto.Sha256.encode(ToolingCodec.canonical({
			format: manifest.format,
			version: manifest.version,
			phase: manifest.phase,
			lengthUnit: manifest.lengthUnit,
			sourceFingerprint: manifest.sourceFingerprint,
			fabricationFingerprint: manifest.fabricationFingerprint,
			files: manifest.files,
			fabrication: manifest.fabrication,
			validation: manifest.validation,
			replacementGroups: manifest.replacementGroups
		}));
	}

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
		var all:Array<ToolingValidation>=[],byRule:Map<String,ToolingValidation>=[];
		for (check in f.validation.concat(checks)) {
			var existing=byRule.get(check.rule);
			if (existing!=null) {
				if (ToolingCodec.canonical(existing)!=ToolingCodec.canonical(check))
					throw "Conflicting fabrication validation: "+check.rule;
				continue;
			}
			byRule.set(check.rule,check);all.push(check);
		}
		all.sort((a, b) -> Reflect.compare(a.rule, b.rule));
		var seen = new Map<String, Bool>();
		for (check in all) {
			if (seen.exists(check.rule))
				throw "Duplicate fabrication rule";
			seen.set(check.rule, true);
		}
		sys.io.File.saveContent(dir + "/fabrication-validation.json", ToolingCodec.canonical(all) + "\n");
		// The detailed report participates in the single hash inventory.
		records = [];
		visit("");
		records.sort((a, b) -> Reflect.compare(a.path, b.path));
		var manifest:ToolingManifest = {
			format: "materia.tooling-manifest",
			version: 12,
			phase: 5,
			lengthUnit: materia.tooling.ToolingEnums.ToolingLengthUnit.Millimetres,
			sourceFingerprint: f.sourceFingerprint,
			fabricationFingerprint: f.fingerprint,
			packageFingerprint: "",
			files: records,
			fabrication: ToolingReadiness.evaluate(all),
			validation: "fabrication-validation.json",
			replacementGroups: f.replacementGroups
		};
		manifest.packageFingerprint = fingerprint(manifest);
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

	static function readValidation(raw:Dynamic):Array<ToolingValidation> {
		if (!Std.isOfType(raw,Array)) throw "Invalid tooling validation records";
		var rows:Array<Dynamic>=cast raw,result:Array<ToolingValidation>=[];
		function strings(value:Dynamic):Array<String> {
			if (!Std.isOfType(value,Array)) throw "Invalid tooling validation list";
			var values:Array<Dynamic>=cast value;
			return [for (entry in values) {
				if (!Std.isOfType(entry,String)) throw "Invalid tooling validation text";
				var text:String=cast entry;text;
			}];
		}
		for (row in rows) {
			materia.casting.CastingFeatureCodec.fields(row,["rule","affectedIds","status","severity","message","evidence"]);
			var status:materia.tooling.ToolingDiagnostic.ToolingCheckStatus=switch materia.casting.CastingRecordCodec.string(row,"status") {
				case "pass":materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass;case "pending":materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending;case "fail":materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail;case "unsupported":materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported;
				default:throw "Invalid tooling validation status";
			};
			var severity:materia.tooling.ToolingDiagnostic.ToolingSeverity=switch materia.casting.CastingRecordCodec.string(row,"severity") {
				case "info":materia.tooling.ToolingDiagnostic.ToolingSeverity.Info;case "warning":materia.tooling.ToolingDiagnostic.ToolingSeverity.Warning;case "error":materia.tooling.ToolingDiagnostic.ToolingSeverity.Error;
				default:throw "Invalid tooling validation severity";
			};
			var check:ToolingValidation={rule:materia.casting.CastingRecordCodec.string(row,"rule"),
				affectedIds:strings(Reflect.field(row,"affectedIds")),status:status,severity:severity,
				message:materia.casting.CastingRecordCodec.string(row,"message"),evidence:strings(Reflect.field(row,"evidence"))};
			ToolingReadiness.validate(check);result.push(check);
		}
		return result;
	}

	public static function verify(dir:String):Void {
		var manifest:Dynamic = haxe.Json.parse(sys.io.File.getContent(dir + "/manifest.json"));
		var keys = [
			"format",
			"version",
			"phase",
			"lengthUnit",
			"sourceFingerprint",
			"fabricationFingerprint",
			"packageFingerprint",
			"files",
			"fabrication",
			"validation",
			"replacementGroups"
		];
		for (key in Reflect.fields(manifest))
			if (keys.indexOf(key) < 0)
				throw "Unknown tooling manifest field";
		for (key in keys)
			if (!Reflect.hasField(manifest, key))
				throw "Missing tooling manifest field";
		if (manifest.format != "materia.tooling-manifest" || manifest.version != 12)
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
		if (manifest.validation!="fabrication-validation.json"
			|| files.filter(file -> file.path==manifest.validation).length!=1)
			throw "Missing hashed fabrication validation report";
		readValidation(haxe.Json.parse(sys.io.File.getContent(dir+"/"+manifest.validation)));
		var expected = fingerprint(manifest);
		if (expected != manifest.packageFingerprint)
			throw "Tooling package fingerprint differs";
	}
}
