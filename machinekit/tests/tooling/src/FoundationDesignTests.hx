import machinekit.structural.MineralCasting;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.assembly.PosedParts;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.casting.CastingCavities;
import materia.casting.CastingPatterns;

/** Independent frozen pre-migration CAD plus exact neutral/CAD intersection checks. */
class FoundationDesignTests {
	static var assertions = 0;

	static function same(a:Part, b:Part, id:String):Void {
		var added = a.subtract(b),
			removed = b.subtract(a),
			difference = added.volume() + removed.volume();
		added.close();
		removed.close();
		assertions++;
		if (difference > 1e-3)
			throw id + " symmetric difference: " + difference;
		Sys.println(id + " symmetric difference: " + difference);
	}

	static function prism(p:materia.casting.CastingPrism):Part {
		var local = Part.prism([for (v in p.profile) new Vector(v.x, v.y)], 0, p.height);
		try {
			var out = PosedParts.moved(local, p.pose);
			local.close();
			return out;
		} catch (e:Dynamic) {
			local.close();
			throw e;
		}
	}

	static function slopedFoot():Void {
		var values = new ComponentValues().setNumber("width", 100)
			.setNumber("depth", 80)
			.setNumber("height", 120)
			.setNumber("footWidth", 160)
			.setNumber("footHeight", 60)
			.setNumber("footShoulder", 20);
		var source = new MineralCasting(values), body = source.geometry();
		assertions++;
		if (source.collisionHullQualification.status != machinekit.structural.CastingCollisionQualification.CastingCollisionStatus.Unsupported
			|| !source.collisionHullQualification.diagnostics("foot-fixture").hasErrors())
			throw "Non-box casting collision hull must report Unsupported explicitly";
		var feature = CastingPatterns.box("foot-crossing", materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,
			[CastingPatterns.face("foot.side-2")], 60, 30, 30, 65, 0, 25);
		var cavity = CastingCavities.derive(source.castingEnvelope(), [feature])[0];
		var input:materia.tooling.casting.CastingToolingInput = {
			id: "foot-fixture", revision: 1, lengthUnit: materia.tooling.ToolingEnums.ToolingLengthUnit.Millimetres,
			envelope: source.castingEnvelope(), features: [feature], inserts: []
		};
		for (checks in [
			materia.tooling.process.FillAnalysis.sourceSupport(input),
			materia.tooling.process.OrthogonalSections.sourceSupport(input),
			materia.tooling.casting.CastingMoldPlanner.sourceSupport(input)
		]) {
			assertions++;
			if (checks.filter(check -> check.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported
				&& check.affectedIds.indexOf(feature.id) >= 0).length != 1)
				throw "Box-only consumer must report the exact non-box source feature";
		}
		var localTool = Part.box(60, 30, 30),
			tool = localTool.translated(new Vector(65, 0, 25));
		localTool.close();
		var reference = tool.intersect(body);
		var actual = Solids.union([for (p in cavity.pieces) prism(p)]);
		same(reference, actual, "foot-crossing cut/body intersection");
		reference.close();
		actual.close();
		tool.close();
		body.close();
		var message = "";
		try
			CastingCavities.boxes(cavity)
		catch (e:Dynamic)
			message = Std.string(e);
		assertions++;
		if (message.indexOf("Unsupported non-box finished cavity: foot-crossing") < 0)
			throw "Box consumer lost feature ID";
	}

	public static function main():Void {

		var spec = machinekit.standard.SocketHeadCapScrew.metric("M12", 35).thread.mating();
		var limits = machinekit.design.ThreadedHolePolicy.resolve(spec, Steel);
		assertions++;
		if (limits.minEngagement != 12 || limits.minTipGap != spec.pitch || limits.minBack != 3
			|| limits.minWall != 6 || Math.abs(limits.minClearance - 1.2) > 1e-9)
			throw "Steel hole policy must resolve dimensional design minima";
		var custom = machinekit.design.ThreadedHolePolicy.resolve(spec, Steel, {minEngagement: 18});
		assertions++;
		if (custom.minEngagement != 18 || custom.overrides.minEngagement != 18
			|| custom.minWall != limits.minWall || limits.overrides.minEngagement != null)
			throw "Hole limit overrides must be recorded without changing policy defaults";
		var rejected = false;
		try machinekit.design.ThreadedHolePolicy.resolve(spec, Steel, {minWall: 0})
		catch (e:Dynamic) rejected = Std.string(e).indexOf("finite and positive: minWall") >= 0;
		assertions++;
		if (!rejected)
			throw "Hole policy accepted a placeholder wall limit";
		var pattern = materia.casting.InsertHoleGeometry.through(6, 20);
		pattern[0].id = "left";
		pattern[0].frame.x = -10;
		var right = materia.casting.InsertHoleGeometry.through(6, 20)[0];
		right.id = "right";
		right.frame.x = 10;
		pattern.push(right);
		var encoded = machinekit.structural.InsertHoles.fromNeutral(pattern);
		var restored = machinekit.structural.InsertHoles.records(encoded);
		var interfaces = machinekit.structural.InsertHoles.facet(restored, 50, 40, 20);
		var copied = machinekit.structural.InsertHoles.copy(interfaces);
		assertions++;
		if (copied.length != 2 || copied[0].frame.x != -10 || copied[1].frame.x != 10
			|| machinekit.structural.InsertHoles.fromNeutral(copied) != encoded)
			throw "Insert hole pattern must retain frames and resolved policy through the facet";
		copied[1].counterbore = {diameter: 12, depth: 3};
		var analytic = materia.casting.InsertHoleGeometry.massProperties(50, 40, 20, copied);
		var insert = new machinekit.structural.CastInsert(new ComponentValues()
			.setToken("role", "generic-mount").setNumber("width", 50).setNumber("depth", 40).setNumber("height", 20)
			.setToken("holes", machinekit.structural.InsertHoles.fromNeutral(copied)));
		var solid = insert.geometry(), physical = solid.massProperties();
		solid.close();
		assertions++;
		if (Math.abs(analytic.volume - physical.volume) > 1e-6
			|| Math.abs(analytic.center.x - physical.centerOfMass.x) > 1e-7
			|| Math.abs(analytic.center.z - physical.centerOfMass.z) > 1e-7)
			throw "Multi-hole insert mass and centroid must agree with actual drilled steel CAD";
		var blind = materia.casting.InsertHoleGeometry.through(6, 20)[0];
		blind.extent = materia.casting.InsertHole.HoleExtent.Blind;
		blind.depth = 8;
		blind.counterbore = {diameter: 12, depth: 3};
		var blindInsert = new machinekit.structural.CastInsert(new ComponentValues()
			.setToken("role", "generic-mount").setNumber("width", 50).setNumber("depth", 40).setNumber("height", 20)
			.setToken("holes", machinekit.structural.InsertHoles.fromNeutral([blind])));
		var drilled = blindInsert.geometry(), actualBlind = drilled.massProperties();
		var pin = machinekit.tooling.ToolingPin.boreMask(blind).geometry(), mask = PosedParts.moved(pin, blind.frame);
		pin.close();
		var common = mask.intersect(drilled), overlapVolume = common.volume();
		common.close();
		mask.close();
		drilled.close();
		var expectedBlind = materia.casting.InsertHoleGeometry.massProperties(50, 40, 20, [blind]);
		assertions++;
		if (overlapVolume > 1e-7 || Math.abs(expectedBlind.volume - actualBlind.volume) > 1e-6)
			throw "Blind counterbore depth and stepped protection must agree with actual CAD";
		var tapped = machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.tappedThrough(
			machinekit.structural.InsertHoles.metricThread("M12"), 20));
		tapped[0].limits = machinekit.design.ThreadedHolePolicy.neutral(custom);
		var overridden = machinekit.structural.InsertHoles.fromNeutral(tapped);
		var recovered = machinekit.structural.InsertHoles.copy(machinekit.structural.InsertHoles.facet(
			machinekit.structural.InsertHoles.records(overridden), 50, 40, 20));
		assertions++;
		if (recovered[0].limits.policyId != custom.policyId || recovered[0].limits.overrides[0].value != 18
			|| machinekit.structural.InsertHoles.fromNeutral(recovered) != overridden)
			throw "Insert facet lost resolved limits or their recorded overrides";
		var target = new machinekit.structural.CastInsert(new ComponentValues()
			.setToken("role", "generic-mount").setNumber("width", 50).setNumber("depth", 40).setNumber("height", 20)
			.setToken("holes", machinekit.structural.InsertHoles.tappedThrough(machinekit.structural.InsertHoles.metricThread("M12"), 20)));
		var clamp = new machinekit.structural.CastInsert(new ComponentValues()
			.setToken("role", "generic-mount").setNumber("width", 50).setNumber("depth", 40).setNumber("height", 5)
			.setToken("holes", machinekit.structural.InsertHoles.through(14, 5)));
		var screw = machinekit.standard.SocketHeadCapScrew.metric("M12", 30);
		var head = materia.assembly.AssemblyFrames.fromRotationMatrix(0, 0, 25, [1, 0, 0, 0, -1, 0, 0, 0, -1]);
		var joint = new machinekit.assembly.MachineAssembly();
		joint.addComponent("target", target);
		joint.addComponent("clamp", clamp, materia.assembly.AssemblyFrames.translation(0, 0, 20));
		joint.addComponent("screw", screw, materia.assembly.AssemblyFrames.compose(head,
			materia.assembly.AssemblyFrames.inverse(screw.bearingFrame())));
		joint.addDesignRequirement(machinekit.design.DesignRequirement.FastenerJoint("through-fixing", "screw",
			{member: "target", feature: "fixing"}, [{member: "clamp", feature: "fixing"}]));
		var throughAudit = machinekit.design.AssemblyDesignChecks.check(joint);
		throughAudit.throwIfErrors();
		assertions++;
		joint.replaceComponent("screw", machinekit.standard.SocketHeadCapScrew.metric("M12", 10));
		var shortAudit = machinekit.design.AssemblyDesignChecks.check(joint);
		assertions++;
		if (shortAudit.items.filter(item -> item.subject == "through-fixing thread engagement").length != 1)
			throw "Through-hole fasteners must still enforce numeric engagement limits";
		rejected = false;
		try machinekit.structural.InsertHoles.records('{"version":1,"holes":[]}')
		catch (e:Dynamic) rejected = Std.string(e).indexOf("Unsupported insert hole recipe") >= 0;
		assertions++;
		if (!rejected)
			throw "Legacy insert hole encoding must be rejected";
		var cwd = Sys.getCwd(),
			root = (sys.FileSystem.exists(cwd + "/machinekit") ? cwd + "/machinekit/tests/tooling" : cwd) + "/fixtures/foundation-design";
		var frame = MillMoldSetup.frame();
		var mold = materia.tooling.casting.CastingMoldPlanner.plan(MillMoldSetup.document(frame), MillMoldSetup.options(frame));
		assertions++;
		if (mold.coreUnits.length != 2 || mold.coreUnits.filter(unit -> unit.coreIds.length == 3 && unit.featureIds.length == 3).length != 2)
			throw "Retained mill rail beams must preserve two authored core units with three foam fragments each";
		var reference:Dynamic = haxe.Json.parse(sys.io.File.getContent(root + "/reference.json"));
		if (Sys.args().indexOf("--audit-openings") >= 0)
			for (source in [frame.base.body, frame.column.body])
				for (feature in source.castingFeatures())
					Sys.println(feature.id + " " + haxe.Json.stringify(feature.openings));
		var foundations:Array<{id:String, part:machinekit.structural.MineralFoundation}> = [
			{id: "mill-base", part: frame.base}, {id: "mill-column", part: frame.column},
			{id: "hobber-bed", part: new machinekit.hobbing.MineralCastBed()},
			{id: "hobber-column", part: new machinekit.hobbing.MineralCastColumn()}
		];
		var baseline:Dynamic = haxe.Json.parse(sys.io.File.getContent(root + "/../../../../examples/flexspline-hobber/BASELINE_FOUNDATIONS.json"));
		if (baseline.format != "machinekit.foundation-baseline" || baseline.version != 2)
			throw "Unsupported foundation migration baseline";
		var frozenRows:Array<Dynamic> = cast baseline.currentBeforeMigration;
		for (row in foundations) {
			var process = sys.io.Process.run("gzip", ["-dc", root + "/" + row.id + ".step.gz"]);
			var text = process.readStdout(), status = process.exitCode();
			process.close();
			if (status != 0)
				throw "Cannot decode frozen migration CAD fixture";
			var records:Array<Dynamic> = cast reference.files;
			var matches = records.filter(record -> record.id == row.id);
			assertions++;
			if (matches.length != 1 || haxe.crypto.Sha256.encode(text) != matches[0].stepSha256
				|| haxe.io.Bytes.ofString(text).length != matches[0].stepBytes)
				throw "Frozen migration CAD fixture fingerprint mismatch: " + row.id;
			var previous = new Part(cadkit.Shape.importStepText(text)),
				actual = row.part.body.geometry();
			same(previous, actual, row.id);
			previous.close();
			actual.close();
			if (StringTools.startsWith(row.id, "hobber-")) {
				var kind = row.id.substr(7), frozen = frozenRows.filter(before -> before.id == kind)[0];
				var boxes:Array<Dynamic> = cast frozen.cells;
				var previousCells:Array<materia.casting.Box3> = [for (box in boxes) {
					x: materia.casting.CastingRecordCodec.number(box, "x"), y: materia.casting.CastingRecordCodec.number(box, "y"),
					z: materia.casting.CastingRecordCodec.number(box, "z"), width: materia.casting.CastingRecordCodec.number(box, "width"),
					depth: materia.casting.CastingRecordCodec.number(box, "depth"), height: materia.casting.CastingRecordCodec.number(box, "height")
				}];
				var cells = row.part.body.materialBoxes();
				var difference = materia.casting.BoxAlgebra.volume(materia.casting.BoxAlgebra.subtract(cells, previousCells))
					+ materia.casting.BoxAlgebra.volume(materia.casting.BoxAlgebra.subtract(previousCells, cells));
				assertions++;
				if (difference > 1e-6)
					throw "Shared algebra changes frozen hobber material: " + row.id;
				assertions++;
				if (Math.abs(materia.casting.BoxAlgebra.volume(cells) - frozen.volume) > 1e-6)
					throw "Shared algebra changes frozen hobber volume: " + row.id;
				var half:Float = frozen.halfSpan, expected:Float = frozen.minimumIy;
				var iy = row.part.body.minimumSectionIy(-half, half);
				assertions++;
				if (Math.abs(iy - expected) > expected * 1e-10)
					throw "Shared algebra changes frozen hobber minimum section Iy: " + row.id + ": " + iy + " vs " + expected;
				var features = row.part.body.castingFeatures();
				assertions++;
				if (features.filter(feature -> feature.derivedFrom != null).length == 0)
					throw "Hobber retained supports must preserve authored parent IDs";
				if (kind == "bed") {
					assertions++;
					if (features.filter(feature -> feature.kind == InterfaceRecess && feature.owner != null
						&& StringTools.startsWith(feature.owner, "footSeat")).length != 3)
						throw "Removable foot plate registers must name their owners";
				}
				assertions++;
				if (row.part.insertLocations().filter(insert -> insert.id == "driveSeat")[0].part.parameterToken("role") != "drive-mount")
					throw "Hobber drive seats require their explicit source role";
			}
		}
		slopedFoot();
		Sys.println("FoundationDesignTests: " + assertions + " assertions passed");
	}
}
