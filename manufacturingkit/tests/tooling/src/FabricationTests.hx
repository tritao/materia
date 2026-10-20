import materia.tooling.export.FabricationSpec;
import materia.tooling.export.FabricationSpecCodec;
import materia.tooling.export.ToolingFabrication;
import materia.tooling.export.ToolingLoads;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.ToolingCodec;

class FabricationTests {
	static var assertions = 0;

	static function check(ok:Bool, message:String):Void {
		assertions++;
		if (!ok)
			throw message;
	}

	static function rejects(fn:Void->Void, message:String):Void {
		var actual = "";
		try
			fn()
		catch (e:Dynamic)
			actual = Std.string(e);
		check(actual.indexOf(message) >= 0, 'Expected "$message", got "$actual"');
	}

	public static function spec():FabricationSpec
		return {
			revision: 1,
			sheetMaterialId: "tooling-coated-board",
			solvent: Unknown,
			adhesiveContainsEPS: null,
			resinReference: "test-only resin",
			compatibilityEvidence: [],
			gravityMps2: 9.80665,
			loadFactor: 1.5,
			compactionPressurePa: null,
			compactionEvidence: [],
			plates: [],
			braces: [],
			fasteners: [],
			coreRestraints: [],
			rigidRestraints: [],
			blankMarginMm: 10,
			blankKerfMm: 3.2,
			couponClearancesMm: [0.2, 0.3, 0.4]
		};

	public static function run(p:CastingMoldPlan):Void {
		var s = spec(), f = ToolingFabrication.plan(p, s);
		check(!f.fabrication.ready, "missing process/fixture evidence blocks readiness");
		var requirement = f.panelRequirements[0], stock = f.sheetStock;
		var blankPlan:materia.sheet.SheetCutPlan = {
			schemaVersion: 1,
			id: "blank-bridge-test",
			revision: 1,
			stockSpecId: stock.id,
			stockSpecRevision: stock.revision,
			stockSpecFingerprint: materia.sheet.SheetPlanValidator.stockFingerprint(stock),
			lengthUnit: "mm",
			kerf: s.blankKerfMm,
			edgeMargin: 0,
			requirements: materia.sheet.SheetPlanValidator.captureRequirements([requirement]),
			placements: [
				{
					id: "blank",
					requirementId: requirement.id,
					x: 0,
					y: 0,
					width: requirement.blankWidth,
					height: requirement.blankHeight,
					rotation: 0,
					regionId: "part"
				}
			],
			cuts: [
				{
					id: "column-cut",
					regionId: "stock",
					axis: "vertical",
					position: requirement.blankWidth,
					firstRegionId: "column",
					secondRegionId: "right"
				},
				{
					id: "part-cut",
					regionId: "column",
					axis: "horizontal",
					position: requirement.blankHeight,
					firstRegionId: "part",
					secondRegionId: "top"
				}
			],
			retainedRegionIds: ["right", "top"]
		};
		var blankReview = materia.sheet.SheetPlanValidator.validate(blankPlan, stock, [requirement]);
		check(blankReview.valid, "blank requirements are accepted by the existing sheet planner: " + blankReview.messages.join("; "));
		check(f.panelRequirements.length == p.panels.members.length + p.supportPanels.length + 4, "panel and fit/locator coupon blanks");
		check(f.validation.filter(c -> c.rule == "fabrication.loads.compaction"
			&& c.status == "pending").length == 1, "undeclared compaction cannot pass");
		check(f.validation.filter(c -> StringTools.startsWith(c.rule, "fabrication.core-restraint.")
			&& c.status == "pending").length == p.cores.length,
			"missing restraint is per-core pending");
		check(ToolingCodec.canonical(f) == ToolingCodec.canonical(ToolingFabrication.plan(p, s)), "fabrication generation is deterministic");
		var encoded = FabricationSpecCodec.encode(s);
		check(FabricationSpecCodec.encode(FabricationSpecCodec.decode(encoded)) == encoded, "fabrication spec versioned round-trip");
		var raw:Dynamic = haxe.Json.parse(encoded);
		raw.version = 99;
		rejects(() -> FabricationSpecCodec.decode(haxe.Json.stringify(raw)), "Unsupported fabrication spec");
		raw = haxe.Json.parse(encoded);
		Reflect.setField(raw.spec, "freeTextPending", new Array<String>());
		rejects(() -> FabricationSpecCodec.decode(haxe.Json.stringify(raw)), "Unknown fabrication field");
		s.solvent = StyreneBearing;
		check(ToolingFabrication.plan(p, s).validation.filter(c -> c.rule == "fabrication.materials.compatibility"
			&& c.status == "fail").length == 1,
			"styrene plus EPS fails outright");
		check(f.validation.filter(c -> c.rule == "fabrication.materials.cure" && c.status == "pending").length == 1,
			"fabrication cannot invent an assessed cure peak without a CureReport");
		var core = p.cores[0],
			load = f.loads.estimates.filter(v -> v.id == core.id)[0];
		var expected = core.cavity.width * core.cavity.depth * core.cavity.height * 1e-9 * p.document.recipe.mixDensityKgM3 * s.gravityMps2 * s.loadFactor;
		check(Math.abs(load.forceN - expected) < 1e-7, "core gross buoyancy uses actual displaced volume and recipe density");
		s = spec();
		s.coreRestraints = [
			{
				coreId: core.id,
				allowableN: 1,
				bearingAreaMm2: 1,
				allowableBearingPa: 1,
				evidence: ["test-only inadequate restraint"]
			}
		];
		check(ToolingFabrication.plan(p, s).validation.filter(c -> c.rule == "fabrication.core-restraint." + core.id
			&& c.status == "fail").length == 1,
			"inadequate attachment/bearing fails");
		s = spec();
		s.fasteners = [
			{
				id: "test-fasteners",
				loadIds: [core.id],
				hardwareIds: [p.hardware[0].id],
				allowableN: 0.01,
				evidence: ["test-only capacity"]
			}
		];
		check(ToolingFabrication.plan(p, s).validation.filter(c -> c.rule == "fabrication.fastener.test-fasteners"
			&& c.status == "fail").length == 1,
			"fastener reaction exceeds capacity");
		var rigid = [for (v in p.formers) v.volume].concat([for (v in p.fixtures) v.datum]);
		check(rigid.length > 0, "fixture includes rigid tooling");
		for (member in rigid) {
			check(f.loads.estimates.filter(v -> v.id == member.id).length == 1, "every rigid former and datum has an uplift estimate");
			check(f.loads.estimates.filter(v -> v.id == member.id + "-lateral-x").length == 1, "every rigid former and datum has a lateral estimate");
		}
		var wetPlan = materia.tooling.casting.CastingMoldCodec.decode(materia.tooling.casting.CastingMoldCodec.encode(p));
		var wetMembers = [for (v in wetPlan.formers) v.volume].concat([for (v in wetPlan.fixtures) v.datum]);
		var wetMember = wetMembers[0], envelope = wetPlan.envelope;
		wetMember.profiles = [];
		wetMember.cuts = [];
		wetMember.pieces = [
			{
				x: envelope.x,
				y: envelope.y,
				z: envelope.z,
				width: 10,
				depth: 20,
				height: 10
			}
		];
		s = spec();
		s.rigidRestraints = [
			{
				memberId: rigid[0].id,
				upliftCapacityN: 1e12,
				lateralXCapacityN: 1e-12,
				lateralYCapacityN: 1e12,
				evidence: ["test restraint"]
			}
		];
		var rigidLoads = ToolingLoads.report(wetPlan, s);
		var lateral = rigidLoads.estimates.filter(v -> v.id == rigid[0].id + "-lateral-x")[0];
		check(lateral.forceN > 1e-12, "rigid fixture is immersed and exercises lateral loading");
		check(rigidLoads.validation.filter(v -> v.rule == "fabrication.rigid-restraint." + rigid[0].id && v.status == "fail").length == 1,
			"lateral weakness fails despite adequate uplift capacity");
		s.rigidRestraints.push(s.rigidRestraints[0]);
		rejects(() -> ToolingLoads.report(wetPlan, s), "Duplicate rigid tooling restraint");
		s.rigidRestraints = [
			{
				memberId: "unknown-member",
				upliftCapacityN: 1,
				lateralXCapacityN: 1,
				lateralYCapacityN: 1,
				evidence: []
			}
		];
		rejects(() -> ToolingLoads.report(wetPlan, s), "Unknown rigid tooling restraint member");
		for (member in p.panels.members)
			if (member.panel.operations.length > 1)
				check(f.validation.filter(v -> v.rule == "fabrication.plate." + member.panel.id && v.status == "unsupported").length == 1,
					"each machined panel declares its structural model gap");
		var plate = ToolingLoads.plate(1000, 1, 1, 0.02, 3e9, 0.3),
			d = 3e9 * Math.pow(0.02, 3) / (12 * (1 - 0.3 * 0.3));
		check(Math.abs(plate.deflectionMm * 0.001 * d / 1000 - 0.00406235) < 1e-7, "Navier simply-supported square uniform-load coefficient");
		var half = ToolingLoads.plate(1000, 1, 1, 0.02, 6e9, 0.3);
		check(Math.abs(plate.deflectionMm / 2 - half.deflectionMm) < 1e-10, "plate stiffness scaling");
		var beam = ToolingLoads.beam(1000, 1, 0.2, 0.05, 0.1, 10e9);
		check(Math.abs(beam.stressPa - 3e5) < 1e-6, "uniform simply-supported brace bending stress");
		var reversed = f.panelRequirements.copy();
		reversed.reverse();
		check(ToolingCodec.canonical(ToolingFabrication.layout(reversed, p.document.recipe.panel.width, p.document.recipe.panel.height,
			s.blankKerfMm)) == ToolingCodec.canonical(f.blankPlacements),
			"blank layout independent of requirement order");
		for (v in f.blankPlacements)
			check(v.xMm >= 0
				&& v.yMm >= 0
				&& v.xMm + v.widthMm <= p.document.recipe.panel.width
				&& v.yMm + v.heightMm <= p.document.recipe.panel.height,
				"blank lies in stock");
		check(ToolingFabrication.csv([["a,b", "quoted\"field", "line\nbreak"]]) == '"a,b","quoted""field","line\nbreak"\n',
			"CSV escapes commas quotes and newlines");
		Sys.println('FabricationTests: $assertions assertions passed');
	}
}
