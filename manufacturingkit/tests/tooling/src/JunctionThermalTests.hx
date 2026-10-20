import materia.tooling.process.JunctionThickness;
import materia.tooling.process.OrthogonalSections;
import materia.tooling.process.ThermalCoverage;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.ProcessSpecCodec;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.casting.BoxAlgebra;
import materia.tooling.ToolingCodec;

class JunctionThermalTests {
	static var assertions = 0;

	static function check(ok:Bool, message:String):Void {
		assertions++;
		if (!ok)
			throw message;
	}

	static function near(a:Float, b:Float, t:Float, message:String):Void
		check(Math.abs(a - b) <= t, message + ": " + a + " vs " + b);

	static function rejects(fn:Void->Void, expected:String):Void {
		var message = "";
		try
			fn()
		catch (e:Dynamic)
			message = Std.string(e);
		check(message.indexOf(expected) >= 0, "Expected " + expected + ", got " + message);
	}

	static function b(x:Float, y:Float, z:Float, w:Float, d:Float, h:Float):MoldBox
		return {
			x: x,
			y: y,
			z: z,
			width: w,
			depth: d,
			height: h
		};

	static function bounded(r:JunctionThicknessReport, expected:Float):Void {
		check(r.lowerDiameterMm <= expected + 1e-7
			&& r.upperDiameterMm >= expected - 1e-7, "analytic diameter lies in certified interval");
		check(r.converged
			&& r.upperDiameterMm - r.lowerDiameterMm <= r.settings.diameterToleranceMm, "declared geometry tolerance is satisfied");
	}

	static function state(r:ThermalCoverageReport, rule:String):String
		return r.validation.filter(v -> v.rule == rule)[0].status;

	public static function run(raw:CastingProcessSpec):Void {
		var settings:JunctionSearchSettings = {diameterToleranceMm: 0.01, maximumCells: 50000};
		var block = [b(0, 0, 0, 100, 60, 20)];
		var rectangle = JunctionThickness.analyze("body", "source", block, [], settings);
		bounded(rectangle, 20);
		near(rectangle.witness.z, 10, 1e-7, "rectangular witness at shortest-axis midplane");
		var cross = [b(-5, -1, -4, 10, 2, 8), b(-1, -5, -4, 2, 10, 8)];
		var junction = JunctionThickness.analyze("body", "source", cross, [], settings);
		bounded(junction, 2 * Math.sqrt(2));
		check(junction.lowerDiameterMm > 2.8, "junction thicker than either arm's 2 mm wall");
		var before = ToolingCodec.canonical(junction);
		cross.reverse();
		check(ToolingCodec.canonical(JunctionThickness.analyze("body", "source", cross, [], settings)) == before,
			"input permutation leaves exact report unchanged");
		var l = [b(0, 0, -4, 4, 2, 8), b(0, 0, -4, 2, 4, 8)];
		bounded(JunctionThickness.analyze("body", "source", l, [], settings), 8 - 4 * Math.sqrt(2));
		var disconnected = JunctionThickness.analyze("body", "source", [b(0, 0, 0, 2, 2, 2), b(10, 0, 0, 6, 6, 6)], [], settings);
		bounded(disconnected, 6);
		var cavity = BoxAlgebra.subtract([b(-5, -5, -5, 10, 10, 10)], [b(-1, -1, -1, 2, 2, 2)]);
		var withHole = JunctionThickness.analyze("body", "source", cavity, [{id: "hole", box: b(-1, -1, -1, 2, 2, 2)}], settings);
		check(withHole.upperDiameterMm < 10, "internal cavity limits inscribed diameter");
		check(withHole.faces.filter(f -> f.sourceIds.indexOf("hole") >= 0).length == 6, "internal cavity has six owned exposed faces");
		var faceArea = 0.0;
		for (face in junction.faces)
			faceArea += face.axis == X ? face.region.depth * face.region.height : face.axis == Y ? face.region.width * face.region.height : face.region.width * face.region.depth;
		near(faceArea, 392, 1e-7, "cross surface excludes artificial internal partitions");
		var coarse = JunctionThickness.analyze("body", "source", cross, [], {diameterToleranceMm: 1e-8, maximumCells: 20});
		check(!coarse.converged, "exhausted work budget never claims convergence");
		check(coarse.lowerDiameterMm <= 2 * Math.sqrt(2) + 1e-7 && coarse.upperDiameterMm >= 2 * Math.sqrt(2) - 1e-7,
			"budget exhaustion retains valid global bounds");
		rejects(() -> JunctionThickness.analyze("body", "s", block, [], {diameterToleranceMm: 0, maximumCells: 20}), "Invalid junction search settings");
		rejects(() -> JunctionThickness.analyze("body", "s", [b(0, 0, 0, -1, 1, 1)], []), "Invalid junction box");
		var source = haxe.crypto.Sha256.encode("source");
		var sections = OrthogonalSections.analyze("body", source, block, []),
			spec = ProcessSpecCodec.snapshot(raw);
		spec.sections = [];
		var empty = ThermalCoverage.analyze("body", sections, spec, "process");
		near(empty.uncoveredVolumeMm3, 120000, 1e-7, "no section definitions means no thermal geometric coverage");
		check(state(empty, "process.cure.geometric-coverage") == "pending", "empty section coverage stays pending");
		var boundary:ThermalBoundary = {
			kind: Insulated,
			temperatureC: null,
			heatTransferWPerM2K: null,
			evidence: ["fixture"]
		};
		var span = sections.spans.filter(s -> s.axis == Z)[0];
		spec.sections = [
			OrthogonalSections.cureSection(span, boundary, boundary, ["fixture"], sections.geometryFingerprint)
		];
		var full = ThermalCoverage.analyze("body", sections, spec, "process");
		near(full.coveredVolumeMm3, 120000, 1e-7, "explicit slab span covers exact block");
		check(state(full, "process.cure.geometric-coverage") == "pass" && state(full, "process.cure.junction-model") == "pass",
			"rectangular geometric model coverage passes separately from thermal solver");
		var otherStamp = OrthogonalSections.analyze("body", "different-report-stamp", block, []);
		check(otherStamp.geometryFingerprint == sections.geometryFingerprint,
			"thermal bindings depend on exact geometry, not the enclosing report's plan/document stamp");
		check(state(ThermalCoverage.analyze("body", otherStamp, spec, "p"), "process.cure.geometric-coverage") == "pass",
			"same-geometry orientation reports retain an explicit binding");
		spec.sections[0].geometryFingerprint = haxe.crypto.Sha256.encode("stale");
		check(state(ThermalCoverage.analyze("body", sections, spec, "p"), "process.cure.geometric-coverage") == "fail",
			"stale geometry fingerprint cannot qualify unchanged span IDs");
		spec.sections[0].geometryFingerprint = sections.geometryFingerprint;
		var split = OrthogonalSections.analyze("body", source, block, [{id: "core", box: b(0, 0, 20, 50, 60, 10)}]);
		var splitSpan = split.spans.filter(s -> s.axis == Z && s.upperSourceIds.indexOf("core") >= 0)[0];
		var partialSpec = ProcessSpecCodec.snapshot(spec);
		partialSpec.sections = [
			OrthogonalSections.cureSection(splitSpan, boundary, boundary, [], split.geometryFingerprint)
		];
		var partial = ThermalCoverage.analyze("body", split, partialSpec, "p");
		near(partial.coveredVolumeMm3, 60000, 1e-7, "one source-owned half-slab covers half the block");
		near(partial.uncoveredVolumeMm3, 60000, 1e-7, "remaining half is explicitly uncovered");
		near(BoxAlgebra.volume(partial.uncoveredRegions), partial.uncoveredVolumeMm3, 1e-7, "uncovered-region geometry matches coverage accounting");
		spec.sections[0].spanIds = [];
		check(state(ThermalCoverage.analyze("body", sections, spec, "p"), "process.cure.geometric-coverage") == "pending",
			"source IDs alone cannot establish coverage");
		spec.sections[0].spanIds = ["missing-span"];
		check(state(ThermalCoverage.analyze("body", sections, spec, "p"), "process.cure.geometric-coverage") == "fail",
			"unknown span rejected as geometric coverage failure");
		spec.sections[0].spanIds = [span.id];
		spec.sections[0].thicknessMm = 19;
		check(state(ThermalCoverage.analyze("body", sections, spec, "p"), "process.cure.geometric-coverage") == "fail",
			"incorrect slab thickness cannot cover a different region");
		spec.sections[0].thicknessMm = 20;
		spec.sections[0].sourceIds = ["wrong-face"];
		check(state(ThermalCoverage.analyze("body", sections, spec, "p"), "process.cure.geometric-coverage") == "fail", "bound source ownership must match");
		spec.sections = [
			OrthogonalSections.cureSection(span, boundary, boundary, [], sections.geometryFingerprint)
		];
		var duplicate = ProcessSpecCodec.snapshot(spec).sections[0];
		duplicate.id = "duplicate";
		spec.sections.push(duplicate);
		check(state(ThermalCoverage.analyze("body", sections, spec, "p"), "process.cure.geometric-coverage") == "fail",
			"duplicate span assignments cannot qualify coverage");
		sections = OrthogonalSections.analyze("body", source, cross, []);
		spec.sections = [
			for (s in sections.spans)
				if (s.axis == Z) OrthogonalSections.cureSection(s, boundary, boundary, [], sections.geometryFingerprint)
		];
		var all = ThermalCoverage.analyze("body", sections, spec, "p");
		near(all.uncoveredVolumeMm3, 0, 1e-7, "all cross extrusion slabs cover the exact geometric region");
		check(state(all, "process.cure.junction-model") == "pending", "complete span volume does not turn a junction into 1D thermal coverage");
		spec.sections = [];
		var encoded = ProcessSpecCodec.encode(spec),
			old:Dynamic = haxe.Json.parse(encoded);
		old.version = 2;
		rejects(() -> ProcessSpecCodec.decode(haxe.Json.stringify(old)), "Unsupported casting process format/version");
		Sys.println('JunctionThermalTests: $assertions assertions passed');
	}
}
