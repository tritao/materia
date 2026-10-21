import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;
import materia.assembly.AssemblyFrames as F;
import haxe.Json;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.BoxMoldPlanner;
import materia.tooling.casting.BoxMoldCodec;
import materia.tooling.panel.PanelCodec;
import materia.tooling.casting.CoreSupports;
import materia.tooling.casting.CoreSupport.CoreSupportMode;
import materia.tooling.casting.CoreSupport.CoreSupportKind;

/** Planning/persistence remain buildable with ProjectKit alone, without native CAD. */
class BoxPlannerTests {
	static var assertions = 0;

	static function check(ok:Bool, label:String):Void {
		assertions++;
		if (!ok)
			throw label;
	}

	static function rejects(fn:Void->Void, label:String):Void {
		var caught = false;
		try
			fn()
		catch (e:Dynamic)
			caught = true;
		check(caught, label);
	}

	public static function run(d:ToolingDocument):Void {
		d = ToolingCodec.snapshot(d);
		d.recipe.minimumLigament = 2;
		var insertSize = InsertBodies.box(d.input.inserts[0].body, "seat");
		d.input.inserts[0].body = materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Box(insertSize.width, insertSize.depth, 15));
		d.input.inserts[0].pose.z = 85;
		d.input.inserts[0].holes = TestInsertHoles.through(5,InsertBodies.height(d.input.inserts[0].body));
		var hole = d.input.inserts[0].holes[0];
		hole.thread = {designation: "M6", majorDiameter: 6, pitch: 1, starts: 1, hand: Right,
			internal: true, toleranceClass: "6H", profile: Metric};
		hole.tapDrill = 5;
		hole.threadDepth = hole.depth;
		hole.clearanceDiameter = 6.6;
		hole.limits = {policyId: "steel-threaded-hole-v1", threaded: {minEngagement: 6, minTipGap: 1},
			minBack: 1.5, minWall: 3, minClearance: 0.6, overrides: []};
		var recess = d.input.features.filter(f -> f.kind == "insert-recess")[0];
		recess.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Insert(d.input.inserts[0].body);
		recess.geometry.pose.z = 85;
		d.setup.sourceFingerprint = ToolingCodec.inputFingerprint(d.input);
		var pattern = ToolingCodec.snapshot(d);
		var second = materia.casting.InsertHole.InsertHoleCodec.snapshot([hole])[0];
		pattern.input.inserts[0].holes[0].frame.x = -8;
		second.id = "second";
		second.frame.x = 8;
		pattern.input.inserts[0].holes.push(second);
		pattern.setup.sourceFingerprint = ToolingCodec.inputFingerprint(pattern.input);
		var patterned = BoxMoldPlanner.plan(pattern);
		check(patterned.fixtures[0].fixings.length == 2, "both source threads produce independent fixings");
		var shallow = materia.casting.InsertHole.InsertHoleCodec.snapshot([hole])[0];
		shallow.depth = materia.casting.InsertHole.InsertHoleCodec.threadLimits(shallow).minEngagement
			+ materia.casting.InsertHole.InsertHoleCodec.threadLimits(shallow).minTipGap + pattern.recipe.hardware.plugDepth - 0.1;
		check(materia.tooling.casting.InsertFixturePlanner.select(shallow,
			F.compose(pattern.setup.sourceToMold, pattern.input.inserts[0].pose), patterned.fixtures[0].mountingFace,
			pattern.recipe.hardware) == null, "screw and far-end plug cannot consume the same depth");
		check(patterned.penetrations.filter(p -> p.insertId == "seat" && p.sourceHoleId != null).length == 2,
			"each insert penetration traces to its source hole");
		check(BoxMoldCodec.encode(BoxMoldCodec.decode(BoxMoldCodec.encode(patterned))) == BoxMoldCodec.encode(patterned),
			"multi-hole fixing regeneration");
		patterned.fixtures[0].fixings[0].engagement = 0.001;
		rejects(() -> BoxMoldPlanner.validate(patterned), "engagement must match the numeric source policy");
		var noThread = ToolingCodec.snapshot(d);
		noThread.input.inserts[0].holes[0].thread = null;
		noThread.input.inserts[0].holes[0].tapDrill = null;
		noThread.input.inserts[0].holes[0].threadDepth = null;
		noThread.input.inserts[0].holes[0].limits.threaded = null;
		noThread.setup.sourceFingerprint = ToolingCodec.inputFingerprint(noThread.input);
		var blocked = BoxMoldPlanner.plan(noThread);
		check(blocked.fixtures[0].fixings.length == 0 && blocked.changeRequests.filter(r -> r.kind == "insert-fixture-hole").length == 1
			&& !blocked.fabrication.ready, "plain bore never becomes an invented thread");
		var p = BoxMoldPlanner.plan(d), encoded = BoxMoldCodec.encode(p);
		check(BoxMoldCodec.encode(BoxMoldCodec.decode(encoded)) == encoded, "mold canonical persistence");
		check(p.panels.members.length == 5 && p.penetrations.length == 9, "all sockets/lands and fixing hole classified");
		check(p.documentFingerprint == ToolingCodec.documentFingerprint(d), "mold provenance");
		var raw:Dynamic = Json.parse(encoded);
		Reflect.setField(raw, "version", 8);
		rejects(() -> BoxMoldCodec.decode(Json.stringify(raw)), "previous mold schema rejected");
		raw = Json.parse(encoded);
		Reflect.setField(raw, "version", 99);
		rejects(() -> BoxMoldCodec.decode(Json.stringify(raw)), "unknown mold version");
		raw = Json.parse(encoded);
		Reflect.setField(Reflect.field(raw, "resolved"), "id", "corrupt");
		rejects(() -> BoxMoldCodec.decode(Json.stringify(raw)), "generated records cannot be edited behind source");
		p.cores[0].cutting.wireDirection = {x: 1, y: 1, z: 0};
		rejects(() -> BoxMoldPlanner.validate(p), "unsupported wire direction rejected");
		p = BoxMoldPlanner.plan(d);
		p.penetrations.pop();
		rejects(() -> BoxMoldPlanner.validate(p), "missing penetration classification");
		var panel:Dynamic = Json.parse(PanelCodec.encode(p.panels.members[0].panel));
		Reflect.setField(panel, "version", 1);
		rejects(() -> PanelCodec.decode(Json.stringify(panel)), "old panel encoding rejected");
		panel = Json.parse(PanelCodec.encode(p.panels.members[0].panel));
		Reflect.setField(Reflect.field(panel, "panel"), "contactOpenings", null);
		rejects(() -> PanelCodec.decode(Json.stringify(panel)), "null opening list rejected");
		d.recipe.panel.measuredThickness = 18.2;
		check(BoxMoldCodec.fingerprint(BoxMoldPlanner.plan(d)) != BoxMoldCodec.fingerprint(BoxMoldCodec.decode(encoded)), "stock changes invalidate package");
		var bridge = BoxMoldPlanner.plan(d);
		check(bridge.cores[0].support.kind == CoreSupportKind.Bridge
			&& bridge.supportPanels.length == 4, "suspended core receives carrier and pinned bridge");
		var old:Dynamic = Json.parse(BoxMoldCodec.encode(bridge));
		Reflect.setField(old, "version", 1);
		rejects(() -> BoxMoldCodec.decode(Json.stringify(old)), "old mold encoding rejected");
		var o = CoreSupports.defaults(d.recipe);
		o.mode = CoreSupportMode.BedSocket;
		rejects(() -> BoxMoldPlanner.plan(d, o), "bed socket cannot alter a solid casting floor");
		var bedDoc = ToolingCodec.snapshot(d);
		bedDoc.input.features.filter(f -> f.kind != "insert-recess")[0].geometry.pose.z = 40;
        bedDoc.input.features.filter(f -> f.kind != "insert-recess")[0].openings=[materia.casting.CastingPatterns.face("body.z-max")];
		bedDoc.setup.sourceFingerprint = ToolingCodec.inputFingerprint(bedDoc.input);
		var bed = BoxMoldPlanner.plan(bedDoc), core = bed.cores[0];
		check(core.support.kind == CoreSupportKind.BedSocket && bed.supportPanels.length == 0, "bed opening selects routed socket");
		check(bed.penetrations.length == 10 && core.support.foot != null, "bed socket classified and separate foot defined");
		var foot = core.support.foot;
		if (foot == null)
			throw "Missing foot";
		check(Math.abs(core.foam.z - bed.cavity.z) < 1e-7 && foot.foam.z < bed.cavity.z, "main core height preserved and foot buried");
		check(BoxMoldCodec.encode(BoxMoldCodec.decode(BoxMoldCodec.encode(bed))) == BoxMoldCodec.encode(bed), "bed support roundtrip");
		o = CoreSupports.defaults(bedDoc.recipe);
		o.mode = CoreSupportMode.Bridge;
		rejects(() -> BoxMoldPlanner.plan(bedDoc, o), "bridge cannot pass a closed top");
		o = CoreSupports.defaults(bedDoc.recipe);
		o.fitClearance = 30;
		rejects(() -> BoxMoldPlanner.plan(bedDoc, o), "socket clearance cannot escape core sealing land");
		o = CoreSupports.defaults(bedDoc.recipe);
		o.adhesiveThickness = 0;
		rejects(() -> BoxMoldPlanner.plan(bedDoc, o), "zero adhesive thickness rejected");
		rejects(() -> CoreSupports.options({
			mode: "guess",
			embedDepth: 6,
			fitClearance: 0.2,
			adhesiveThickness: 0.2
		}), "unknown support mode rejected");
		foot.cutting.wireDirection = {x: 1, y: 0, z: 0};
		rejects(() -> BoxMoldPlanner.validate(bed), "foot cutting direction validated");
		var constrained = ToolingCodec.snapshot(d);
		constrained.recipe.jointClearance = 0.1;
		rejects(() -> ToolingCodec.snapshot(constrained), "clearance must include stock variation and allowances");
		var sealPlan = BoxMoldPlanner.plan(d);
		sealPlan.penetrations.filter(x -> x.kind == "insert-sealed")[0].sealMargin = 100;
		rejects(() -> BoxMoldPlanner.validate(sealPlan), "seal margin rejected before native CAD");
		rejects(() -> BoxMoldCodec.encode(sealPlan), "invalid seal cannot be encoded");
		for (axis in [
			materia.tooling.ToolingRecipe.CoreWireAxis.X,
			materia.tooling.ToolingRecipe.CoreWireAxis.Y,
			materia.tooling.ToolingRecipe.CoreWireAxis.Z
		]) {
			var direction = ToolingCodec.snapshot(d);
			direction.recipe.coreWireAxis = axis;
			var cut = BoxMoldPlanner.plan(direction).cores[0].cutting;
			check(cut.wireDirection.x * cut.wireDirection.x + cut.wireDirection.y * cut.wireDirection.y + cut.wireDirection.z * cut.wireDirection.z == 1,
				"declared wire axis and faces");
		}
		var packed:Dynamic = Json.parse(encoded);
		check(!Reflect.hasField(Reflect.field(packed, "resolved"), "document"), "source document stored once");
		check(!p.fabrication.ready && p.validation.length > 0, "readiness separate from successful planning");
		check(p.changeRequests.filter(x -> x.kind == "lifting-points").length == 1, "inverted box handling requires lifting points");
		var longId = "";
		for (i in 0...128)
			longId += "a";
		var derived = materia.tooling.ToolingIds.derive("core-", longId, "-socket");
		materia.tooling.panel.PanelCodec.identifier(derived);
		check(derived.length <= 128, "derived source IDs always satisfy shared vocabulary");
		var oversized = ToolingCodec.snapshot(d);
		oversized.recipe.hardware.locatingPinDiameter = 4;
		rejects(() -> BoxMoldPlanner.plan(oversized), "support obround must fit the selected router");
		var padPlan = BoxMoldPlanner.plan(d);
		var pad = padPlan.supportPanels.filter(x -> StringTools.endsWith(x.panel.id, "-north-pad"))[0];
		pad.pose.y = padPlan.cores[0].foam.y;
		rejects(() -> BoxMoldPlanner.validate(padPlan), "carrier pad overlap rejected by neutral planner");
		Sys.println('BoxPlannerTests: $assertions assertions passed');
	}
}
