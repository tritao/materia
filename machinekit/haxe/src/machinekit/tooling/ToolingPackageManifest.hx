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

	/** Rebuild made-insert jobs and drawing checks from the hashed source plan. */
	static function sourceChecks(dir:String,sourceFingerprint:String):Array<ToolingValidation> {
		var path=dir+"/mold-plan.json";
		if (!sys.FileSystem.exists(path)) throw "Missing source mold plan";
		var plan=materia.tooling.casting.CastingMoldCodec.decode(sys.io.File.getContent(path));
		if (materia.tooling.casting.CastingMoldCodec.fingerprint(plan)!=sourceFingerprint)
			throw "Source mold plan differs from fabrication source";
		var jobs:Array<materia.tooling.casting.InsertManufacturingPlan.InsertManufacturingJob> = [];
		var checks:Array<ToolingValidation> = [];
		for (insert in plan.document.input.inserts) switch insert.origin {
			case Catalogue(_):
			case Fabricated(_):
				var job=materia.tooling.casting.InsertManufacturingPlan.build(insert);
				jobs.push(job);checks=checks.concat(job.validation);
				checks.push(InsertDrawing.review(insert).validation);
		}
		var expected={format:"materia.tooling.insert-manufacturing",version:2,
			sourceFingerprint:sourceFingerprint,inputFingerprint:ToolingCodec.inputFingerprint(plan.document.input),jobs:jobs};
		path=dir+"/insert-manufacturing.json";
		if (!sys.FileSystem.exists(path)
			|| ToolingCodec.canonical(haxe.Json.parse(sys.io.File.getContent(path)))!=ToolingCodec.canonical(expected))
			throw "Insert manufacturing report differs from source";
		var replication=ToolingFabricationExport.replicationReport(plan,sourceFingerprint);
		path=dir+"/replication-jigs.json";
		if (!sys.FileSystem.exists(path)
			|| ToolingCodec.canonical(haxe.Json.parse(sys.io.File.getContent(path)))!=ToolingCodec.canonical(replication))
			throw "Replication report differs from source";
		var replicationChecks:Array<ToolingValidation> =cast replication.validation;
		checks=checks.concat(replicationChecks);
		var jointReport=ToolingFabricationExport.foundationJointReport(plan,sourceFingerprint);
		path=dir+"/foundation-joints.json";
		if (!sys.FileSystem.exists(path)
			|| ToolingCodec.canonical(haxe.Json.parse(sys.io.File.getContent(path)))!=ToolingCodec.canonical(jointReport))
			throw "Foundation joint source or process validation omitted or changed";
		var jointChecks:Array<ToolingValidation> =cast jointReport.validation;
		checks=checks.concat(jointChecks);
		var anchorReport=ToolingFabricationExport.insertAnchorageReport(plan,sourceFingerprint);
		path=dir+"/insert-anchorage.json";
		if (!sys.FileSystem.exists(path)
			|| ToolingCodec.canonical(haxe.Json.parse(sys.io.File.getContent(path)))!=ToolingCodec.canonical(anchorReport))
			throw "Insert anchorage source validation omitted or changed";
		var anchorChecks:Array<ToolingValidation> =cast anchorReport.validation;
		checks=checks.concat(anchorChecks);
		return checks;
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
		var all = f.validation.concat(checks);
		var hardwareReport = dir + "/hardware-validation.json";
		if (!sys.FileSystem.exists(hardwareReport))
			throw "Missing hardware validation report";
		for (row in readValidation(haxe.Json.parse(sys.io.File.getContent(hardwareReport)))) {
			var existing = all.filter(v -> v.rule == row.rule);
			if (existing.length == 0)
				all.push(row);
			else if (existing.length != 1 || ToolingCodec.canonical(existing[0]) != ToolingCodec.canonical(row))
				throw "Conflicting hardware validation";
		}
		for (row in sourceChecks(dir,f.sourceFingerprint)) {
			var existing=all.filter(check -> check.rule==row.rule);
			if (existing.length==0) all.push(row);
			else if (existing.length!=1 || ToolingCodec.canonical(existing[0])!=ToolingCodec.canonical(row))
				throw "Conflicting source manufacturing validation";
		}
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
			version: 7,
			phase: 5,
			lengthUnit: materia.tooling.ToolingEnums.ToolingLengthUnit.Millimetres,
			sourceFingerprint: f.sourceFingerprint,
			fabricationFingerprint: f.fingerprint,
			packageFingerprint: "",
			files: records,
			fabrication: ToolingReadiness.evaluate(all),
			validation: all,
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
		if (!Std.isOfType(raw, Array))
			throw "Invalid tooling validation records";
		var rawValidation:Array<Dynamic> = cast raw;
		return [
			for (row in rawValidation) {
				var ids:Array<Dynamic> = cast row.affectedIds;
				var evidence:Array<Dynamic> = cast row.evidence;
				{
					rule: row.rule,
					affectedIds: [for (id in ids) Std.string(id)],
					status: cast row.status,
					severity: cast row.severity,
					message: row.message,
					evidence: [for (item in evidence) Std.string(item)]
				};
			}
		];
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
		if (manifest.format != "materia.tooling-manifest" || manifest.version != 7)
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
		if (!Std.isOfType(manifest.validation, Array))
			throw "Invalid tooling validation records";
		var validation = readValidation(manifest.validation);
		if (files.filter(file -> file.path == "hardware-validation.json").length != 1)
			throw "Missing hashed hardware validation";
		for (record in readValidation(haxe.Json.parse(sys.io.File.getContent(dir + "/hardware-validation.json"))))
			if (validation.filter(v -> ToolingCodec.canonical(v) == ToolingCodec.canonical(record)).length != 1)
				throw "Hardware validation omitted from readiness";
		for (name in ["mold-plan.json","insert-manufacturing.json","insert-anchorage.json"])
			if (files.filter(file -> file.path==name).length!=1) throw "Missing hashed source manufacturing artifact: "+name;
		for (record in sourceChecks(dir,manifest.sourceFingerprint))
			if (validation.filter(check -> ToolingCodec.canonical(check)==ToolingCodec.canonical(record)).length!=1)
				throw "Source manufacturing validation omitted from readiness";
		var readiness = ToolingReadiness.evaluate(validation);
		if (ToolingCodec.canonical(readiness) != ToolingCodec.canonical(manifest.fabrication))
			throw "Tooling readiness differs from validation";
		var report = haxe.Json.parse(sys.io.File.getContent(dir + "/fabrication-validation.json"));
		if (ToolingCodec.canonical(validation) != ToolingCodec.canonical(report))
			throw "Tooling validation differs from its hashed artifact";
		var expected = fingerprint(manifest);
		if (expected != manifest.packageFingerprint)
			throw "Tooling package fingerprint differs";
	}
}
