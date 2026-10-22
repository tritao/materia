import materia.casting.BoxAlgebra;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.ProcessSpecCodec;
import materia.tooling.process.FillAnalysis;
import materia.tooling.process.CureAnalysis;
import materia.tooling.process.CastingProcessAnalysis;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.CastingMoldPlan;

class ProcessTests {
	static var assertions = 0;

	static function items(v:Dynamic):Array<Dynamic>
		return cast v;

	static function check(ok:Bool, label:String):Void {
		assertions++;
		if (!ok)
			throw label;
	}

	static function near(actual:Null<Float>, expected:Float, tolerance:Float, label:String):Void
		check(actual != null && Math.abs(actual - expected) <= tolerance, label + ": " + actual + " vs " + expected);

	static function rejects(fn:Void->Void, expected:String):Void {
		var error = "";
		try
			fn()
		catch (e:Dynamic)
			error = Std.string(e);
		check(error.indexOf(expected) >= 0, 'Expected "$expected", got "$error"');
	}

	static function box(x:Float, z:Float, width:Float, height:Float):MoldBox
		return {
			x: x,
			y: 0,
			z: z,
			width: width,
			depth: 2,
			height: height
		};

	static function boundary(kind:ThermalBoundaryKind):ThermalBoundary
		return {
			kind: kind,
			temperatureC: kind == Insulated ? null : 20,
			heatTransferWPerM2K: kind == Convection ? 10 : null,
			evidence: ["fixture-boundary"]
		};

	public static function spec():CastingProcessSpec
		return {
			id: "process-test",
			revision: 1,
			recipeFingerprint: haxe.crypto.Sha256.encode("test-recipe"),
			initialTemperatureC: 20,
			ambientTemperatureC: 20,
			temperatureMarginC: 5,
			pourRateMm3PerSecond: null,
			maximumAirEscapeWidthMm: null,
			mixComposition: null,
			mixer: null,
			faceContacts: [],
			openFaceContact: null,
			castability: {
				maximumAggregateSizeMm: null,
				minimumPassageAggregateRatio: null,
				maximumSectionThicknessMm: null,
				evidence: []
			},
			cure: {
				reference: "test-only resin",
				resinMassFraction: 0.1,
				heatOfReactionJPerKgResin: 500000,
				mixSpecificHeatJPerKgK: 1000,
				mixConductivityWPerMK: 1,
				resinMaximumTemperatureC: 120,
				gelMaximumTemperatureC: 120,
				kinetics: {
					kind: FirstOrderArrhenius,
					preExponentialPerSecond: 0.01,
					activationEnergyJPerMol: 0,
					initialConversion: 0,
					autocatalyticPreExponentialPerSecond: 0,
					autocatalyticExponentM: 1,
					reactionExponentN: 1
				},
				evidence: ["analytic-fixture"]
			},
			sections: [
				{
					id: "wall",
					sourceIds: ["body"],
					spanIds: [],
					geometryFingerprint: null,
					thicknessMm: 20,
					left: boundary(Insulated),
					right: boundary(Insulated),
					evidence: ["analytic-slab"]
				}
			],
			solver: {
				cells: 41,
				timeStepSeconds: 1,
				durationSeconds: 1000,
				maximumExtensionSeconds: 0,
				convergenceToleranceC: 1e-8,
				maximumIterations: 100,
				remainingHeatToleranceC: 0.01
			}
		};

	static function status(report:CureReport, rule:String):String {
		var rows = report.validation.filter(r -> r.rule == rule);
		if (rows.length != 1)
			throw "Missing/duplicate cure rule: " + rule;
		return rows[0].status;
	}

	/** Independent dense-height reference: explicit face graph, no event-plane
	 * solver or BoxAlgebra.groups. Its first sampled trapped volume approaches
	 * the analytic right-hand limit as the height step shrinks.
	 */
	static function sampledTraps(region:Array<MoldBox>, step:Float):Float {
		var prior:Array<Array<Int>> = [], total = 0.0, index = 0;
		while ((index + 0.37) * step < 10) {
			var height = (index + 0.37) * step;
			var cells:Array<{id:Int, box:MoldBox}> = [];
			for (i in 0...region.length) {
				var b = region[i], z = Math.max(height, b.z), top = b.z + b.height;
				if (top > z)
					cells.push({
						id: i,
						box: {
							x: b.x,
							y: b.y,
							z: z,
							width: b.width,
							depth: b.depth,
							height: top - z
						}
					});
			}
			while (cells.length > 0) {
				var group = [cells.shift()], cursor = 0;
				while (cursor < group.length) {
					var a = group[cursor++].box, j = cells.length - 1;
					while (j >= 0) {
						var b = cells[j].box;
						var overlap = [
							Math.min(a.x + a.width, b.x + b.width) - Math.max(a.x, b.x),
							Math.min(a.y + a.depth, b.y + b.depth) - Math.max(a.y, b.y),
							Math.min(a.z + a.height, b.z + b.height) - Math.max(a.z, b.z)
						];
						if (overlap.filter(v -> v < -1e-8).length == 0 && overlap.filter(v -> v > 1e-8).length >= 2)
							group.push(cells.splice(j, 1)[0]);
						j--;
					}
				}
				if (group.filter(v -> v.box.z + v.box.height >= 10 - 1e-8).length > 0)
					continue;
				var known = false;
				for (old in prior)
					for (cell in group)
						if (old.indexOf(cell.id) >= 0)
							known = true;
				if (!known) {
					prior.push([for (cell in group) cell.id]);
					for (cell in group)
						total += cell.box.width * cell.box.depth * cell.box.height;
				}
			}
			index++;
		}
		return total;
	}

	public static function run(p:CastingMoldPlan):Void {
		var airRecord=materia.tooling.ToolingCodec.canonical(materia.tooling.process.FillAnalysis.FillPocketGeometry.CadAirAbove(2,"region-example"));
		check(airRecord.indexOf("cad-air-above")>=0 && airRecord.indexOf("boxes")<0,
			"native air references never masquerade as box regions");
		rejects(() -> materia.tooling.ToolingCodec.canonical(materia.tooling.process.FillAnalysis.FillPocketGeometry.CadAirAbove(Math.NaN,"region-example")),
			"Invalid CAD air-region height");
		rejects(() -> materia.tooling.ToolingCodec.canonical(materia.tooling.process.FillAnalysis.FillPocketGeometry.CadAirAbove(2,"region invalid")),
			"Invalid tooling identifier");
		rejects(() -> materia.tooling.ToolingCodec.canonical(materia.tooling.process.FillAnalysis.FillOpeningGeometry.CadOpening(Math.POSITIVE_INFINITY,[])),
			"Invalid CAD opening height");
		rejects(() -> materia.tooling.ToolingCodec.canonical(materia.tooling.process.FillAnalysis.FillOpeningGeometry.CadOpening(2,["cover","cover"])),
			"Duplicate CAD opening blocker ID");

		var region = [box(0, 0, 2, 10), box(2, 0, 2, 2), box(4, 0, 4, 6)];
		var boundaries = [{id: "ceiling-feature", box: box(4, 6, 4, 4)}];
		var cores = [{id: "foam", geometry: materia.tooling.casting.MoldRegions.box(box(10, 1, 2, 3))}];
		function fill(cells:Array<MoldBox>, ?extra:Array<Float>):FillReport
			return FillAnalysis.analyze("body", "source", "process", cells, boundaries, cores, 10, 2400, 10, 2, null, extra);
		var trapped = fill(region);
		check(trapped.trappedPockets.length == 1, "downward-ceiling pocket caught");
		var pocket = trapped.trappedPockets[0];
		near(pocket.sealedAtMm, 2, 1e-10, "exact neck closure event");
		near(pocket.volumeMm3, 32, 1e-10, "analytic trapped pocket volume");
		near(pocket.proposedVentPoint.z, 6, 1e-10, "vent proposal at ceiling");
		check(pocket.sourceIds.indexOf("ceiling-feature") >= 0, "pocket has source feature ownership");
		check(trapped.validation.filter(r -> r.status == "fail").length == 1, "trapped pocket blocks readiness");
		var dense = fill(region, [for (i in 0...1001) i / 100.0]);
		for (level in dense.levels) {
			var h = level.heightMm;
			var analytic = 4 * Math.min(h, 10) + 4 * Math.min(h, 2) + 8 * Math.min(h, 6) - 8 * Math.max(0, Math.min(h, 6) - 2);
			near(level.mixVolumeMm3, analytic, 1e-9, "dense level curve excludes air above its side-neck seal");
		}
		near(trapped.totalMixVolumeMm3, 64, 1e-9, "trapped air is excluded from final mix quantity");
		near(trapped.geometricResinVolumeMm3, 96, 1e-9, "geometric cavity volume remains separately available");
		var multiple = fill(region.concat([box(-6, 0, 4, 8), box(-2, 0, 2, 3)]));
		check(multiple.trappedPockets.length == 2, "two independently side-sealed pockets");
		near(multiple.trappedPockets[0].volumeMm3 + multiple.trappedPockets[1].volumeMm3, 72, 1e-9, "multiple pocket volumes counted once");
		var nested = fill(region.concat([box(8, 0, 2, 8)]));
		check(nested.trappedPockets.length == 1
			&& switch nested.trappedPockets[0].geometry {case Rectilinear(_,faces):faces.length>=2;default:false;}, "one trapped component retains nested ceiling levels");
		near(nested.trappedPockets[0].volumeMm3, 56, 1e-9, "nested ceiling limiting volume");
		near(sampledTraps(region, 0.01), 32, 0.1, "dense independent face graph approaches side-neck limiting volume");
		near(sampledTraps(region.concat([box(-6, 0, 4, 8), box(-2, 0, 2, 3)]), 0.01), 72, 0.1, "dense independent graph agrees on multiple pockets");
		near(sampledTraps(region.concat([box(8, 0, 2, 8)]), 0.01), 56, 0.1, "dense independent graph agrees on nested ceilings");
		var twoTops = [box(0, 0, 2, 10), box(2, 0, 2, 2), box(4, 0, 2, 10)];
		var covered = FillAnalysis.analyze("body", "s", "p", twoTops, [], [], 10, 2400, 10, null, null, [5], [box(4, 0, 2, 1)],
			[{id: "low-wall", bottomMm: 0}, {id: "high-wall", bottomMm: 4}]);
		near(covered.openFillAreaMm2, 4, 1e-9, "bridge footprint is subtracted from open fill area");
		check(covered.trappedPockets.length == 1, "tool-covered top cannot vent an isolated pocket");
		near(covered.trappedPockets[0].sealedAtMm, 2, 1e-9, "covered top seals at side neck");
		var halfway = covered.levels.filter(v -> v.heightMm == 5)[0];
		near(halfway.panelHeads.filter(v -> v.id == "low-wall")[0].headPa, 120, 1e-9, "per-level low panel head");
		near(halfway.panelHeads.filter(v -> v.id == "high-wall")[0].headPa, 24, 1e-9, "per-level elevated panel head");
		rejects(() -> FillAnalysis.analyze("body", "s", "p", twoTops, [], [], 10, 2400, 10, null, null, null, [box(0, 0, 6, 1)]),
			"Fill face has zero open area");
		region.reverse();
		check(ToolingCodec.canonical(fill(region)) == ToolingCodec.canonical(trapped), "fill independent of member order");
		var vented = region.concat([box(6, 6, 2, 4)]),
			ventedReport = fill(vented);
		check(ventedReport.trappedPockets.length == 0, "real open vent path clears trapped pocket");
		check(ventedReport.validation.filter(r -> r.rule == "process.fill.connected-air" && r.status == "pass")
			.length == 1, "vented geometry passes topology");
		check(trapped.validation.filter(r -> r.rule == "process.fill.air-escape-width" && r.status == "pending").length == 1,
			"unknown ceiling width stays pending");
		var widthFail = FillAnalysis.analyze("body", "s", "p", region, boundaries, [], 10, 2400, 10, null, 1);
		check(widthFail.validation.filter(r -> r.rule == "process.fill.air-escape-width" && r.status == "fail").length == 1,
			"wide ceiling fails declared limit");
		var widthPass = FillAnalysis.analyze("body", "s", "p", region, boundaries, [], 10, 2400, 10, null, 100);
		check(widthPass.validation.filter(r -> r.rule == "process.fill.air-escape-width" && r.status == "pass").length == 1,
			"ceiling passes declared large limit");
		for (level in trapped.levels) {
			near(level.mixMassKg, level.mixVolumeMm3 * 2400e-9, 1e-12, "level mass units");
			near(level.timeSeconds, level.mixVolumeMm3 / 2, 1e-10, "declared pour-rate time");
		}
		var atSubmersion = trapped.levels.filter(r -> r.heightMm == 4)[0],
			finalLevel = trapped.levels[trapped.levels.length - 1];
		near(atSubmersion.cores[0].grossUpliftN, 12 * 2400 * 10e-9, 1e-12, "gross uplift units");
		near(atSubmersion.cores[0].grossUpliftN, finalLevel.cores[0].grossUpliftN, 1e-12, "uplift peaks at submersion");
		near(finalLevel.maximumHeadPa, 240, 1e-10, "final liquid head pressure");
		rejects(() -> fill(region, [11]), "Invalid extra fill height");
		rejects(() -> FillAnalysis.analyze("body", "s", "p", region, boundaries, [], 5, 2400, 10, null, null), "insufficient fill height");
		var edgeOnly:Array<MoldBox> = [
			box(0, 0, 2, 10),
			{
				x: 2,
				y: 2,
				z: 0,
				width: 2,
				depth: 2,
				height: 6
			}
		];
		check(fill(edgeOnly).trappedPockets.length == 1, "edge-only contact cannot vent an air region");
		var ceilingFragments:Array<MoldBox> = [
			{
				x: 0,
				y: 0,
				z: 0,
				width: 1,
				depth: 4,
				height: 10
			},
			{
				x: 1,
				y: 0,
				z: 0,
				width: 1,
				depth: 4,
				height: 6
			},
			{
				x: 2,
				y: 0,
				z: 0,
				width: 1,
				depth: 4,
				height: 6
			},
			{
				x: 3,
				y: 0,
				z: 0,
				width: 1,
				depth: 4,
				height: 6
			}
		];
		var combinedCeiling = FillAnalysis.analyze("body", "s", "p", ceilingFragments, [], [], 10, 2400, 10, null, 2.5);
		check(combinedCeiling.validation.filter(v -> v.rule == "process.fill.air-escape-width" && v.status == "fail").length == 1,
			"artificial cell fragments cannot make a wide connected ceiling pass");
		var overlapCore = FillAnalysis.analyze("body", "s", "p", region, boundaries, [{id: "overlap", geometry: materia.casting.CastingRegion.Union([
			materia.tooling.casting.MoldRegions.box(box(10, 1, 2, 3)),materia.tooling.casting.MoldRegions.box(box(10, 2, 2, 2))])}], 10, 2400,
			10, null, null);
		near(overlapCore.levels[overlapCore.levels.length - 1].cores[0].submergedVolumeMm3, 12, 1e-10, "overlapping core displacement counted once");
		var s = spec(), encoded = ProcessSpecCodec.encode(s);
		check(ProcessSpecCodec.encode(ProcessSpecCodec.decode(encoded)) == encoded, "strict process roundtrip");
		var raw:Dynamic = haxe.Json.parse(encoded);
		raw.version = 1;
		rejects(() -> ProcessSpecCodec.decode(haxe.Json.stringify(raw)), "Unsupported casting process");
		raw = haxe.Json.parse(encoded);
		raw.version = 8;
		rejects(() -> ProcessSpecCodec.decode(haxe.Json.stringify(raw)), "Unsupported casting process");
		raw = haxe.Json.parse(encoded);
		Reflect.deleteField(raw.spec, "mixer");
		rejects(() -> ProcessSpecCodec.decode(haxe.Json.stringify(raw)), "Missing casting process field: mixer");
		raw = haxe.Json.parse(encoded);
		Reflect.setField(raw.spec, "typo", 1);
		rejects(() -> ProcessSpecCodec.decode(haxe.Json.stringify(raw)), "Unknown casting process field: typo");
		raw = haxe.Json.parse(encoded);
		raw.spec.cure.mixSpecificHeatJPerKgK = 0;
		rejects(() -> ProcessSpecCodec.decode(haxe.Json.stringify(raw)), "Invalid casting process number");
		raw = haxe.Json.parse(encoded);
		raw.spec.cure.kinetics.kind = "guessed-model";
		rejects(() -> ProcessSpecCodec.decode(haxe.Json.stringify(raw)), "Unsupported cure kinetics");
		raw = haxe.Json.parse(encoded);
		items(raw.spec.sections)[0].left.temperatureC = 20;
		rejects(() -> ProcessSpecCodec.decode(haxe.Json.stringify(raw)), "Inconsistent thermal boundary");
		raw = haxe.Json.parse(encoded);
		raw.spec.solver.durationSeconds = 1e9;
		rejects(() -> ProcessSpecCodec.decode(haxe.Json.stringify(raw)), "work limit");
		var nulText = StringTools.replace(encoded, '"test-only resin"', '"bad\\u0000reference"');
		rejects(() -> ProcessSpecCodec.decode(nulText), "Invalid casting process text");
		near(CureAnalysis.adiabaticRise(s.cure), 50, 1e-12, "adiabatic resin vs mix mass convention");
		var insulated = CureAnalysis.section(s.sections[0], s, 2400);
		var expectedAlpha = 1 - Math.pow(1.01, -1000);
		near(insulated.peakTemperatureC, 20 + 50 * expectedAlpha, 1e-6, "insulated solver matches exact backward-Euler reaction energy");
		for (t in insulated.finalTemperaturesC)
			near(t, insulated.peakTemperatureC, 1e-6, "insulated temperature uniform");
		near(insulated.futurePeakEstimateC, 70, 1e-6, "insulated future bound retains all reaction energy");
		check(insulated.peakTemperatureC <= 70 + 1e-6, "reaction cannot exceed adiabatic upper bound");
		s.cure.kinetics.initialConversion = 0.25;
		var partial = CureAnalysis.section(s.sections[0], s, 2400);
		near(partial.futurePeakEstimateC, 57.5, 1e-6, "initial conversion reduces remaining reaction energy");
		s = spec();
		s.cure.kinetics = null;
		var safe = CureAnalysis.analyze("body", "s", "p", s, 2400, 100, true);
		check(status(safe, "process.cure.lumped-estimate") == "pass", "lumped screening estimate can pass its margin rule without kinetics");
		check(safe.sections.length == 0, "missing kinetics leave the section tier unresolved");
		var uncertain = CureAnalysis.analyze("body", "s", "p", s, 2400, 60, true);
		check(status(uncertain, "process.cure.lumped-estimate") == "pending", "high upper bound is not proof of overheating");
		check(status(uncertain, "process.cure.section.wall") == "pending"
			&& uncertain.sections.length == 0, "missing kinetics cannot pass section tier");
		s.cure.resinMassFraction = null;
		check(CureAnalysis.analyze("body", "s", "p", s, 2400, 60, true).lumpedPeakEstimateC == null, "missing material property produces no invented bound");
		s = spec();
		var overheated = CureAnalysis.analyze("body", "s", "p", s, 2400, 60, true);
		check(status(overheated, "process.cure.section.wall") == "fail", "predicted section peak above EPS limit fails");
		s.solver.durationSeconds = 1;
		var short = CureAnalysis.analyze("body", "s", "p", s, 2400, 60, true);
		check(status(short, "process.cure.section.wall") == "pending", "early termination cannot hide future heat");
		check(short.sections[0].peakTemperatureC < 60
			&& short.sections[0].futurePeakEstimateC > 60, "short run reports residual exotherm bound");
		// Arrhenius coupling changes rate as the reaction heats the slab.
		s = spec();
		s.cure.kinetics.activationEnergyJPerMol = 40000;
		s.cure.kinetics.preExponentialPerSecond = 100000;
		s.solver.durationSeconds = 200;
		s.solver.timeStepSeconds = 0.2;
		var coupled = CureAnalysis.section(s.sections[0], s, 2400);
		for (i in 0...coupled.finalConversions.length) {
			check(coupled.finalConversions[i] >= 0 && coupled.finalConversions[i] <= 1, "Arrhenius conversion remains bounded");
			near(coupled.finalTemperaturesC[i], 20 + 50 * coupled.finalConversions[i], 1e-6, "coupled heat/reaction energy balance");
		}
		s.solver.timeStepSeconds = 100;
		s.solver.maximumIterations = 2;
		rejects(() -> CureAnalysis.section(s.sections[0], s, 2400), "did not converge");
		check(status(CureAnalysis.analyze("body", "s", "p", s, 2400, 60, true), "process.cure.section.wall") == "unsupported",
			"nonconverged solve becomes a blocking model record");
		s = spec();
		s.ambientTemperatureC = null;
		check(status(CureAnalysis.analyze("body", "s", "p", s, 2400, 100, true), "process.cure.lumped-estimate") == "pending",
			"unknown ambient temperature cannot qualify upper bound");
		s.ambientTemperatureC = 100;
		check(status(CureAnalysis.analyze("body", "s", "p", s, 2400, 100, true), "process.cure.lumped-estimate") == "pending",
			"hot environment included in upper bound");
		// Inert slab cooling: exact Fourier sine series at the midplane.
		s = spec();
		s.initialTemperatureC = 60;
		s.cure.heatOfReactionJPerKgResin = 0;
		s.cure.kinetics = null;
		s.sections[0].left = boundary(FixedTemperature);
		s.sections[0].right = boundary(FixedTemperature);
		s.solver.cells = 81;
		s.solver.durationSeconds = 40;
		s.solver.timeStepSeconds = 0.5;
		var coarse = CureAnalysis.section(s.sections[0], s, 1000);
		s.solver.timeStepSeconds = 0.05;
		var fine = CureAnalysis.section(s.sections[0], s, 1000), exact = 0.0;
		for (m in 0...100) {
			var odd = 2 * m + 1;
			exact += (m % 2 == 0 ? 1 : -1) / odd * Math.exp(-odd * odd * Math.PI * Math.PI * 1e-6 * 40 / (0.02 * 0.02));
		}
		exact = 20 + 40 * 4 / Math.PI * exact;
		near(fine.finalTemperaturesC[40], exact, 0.025, "inert slab agrees with analytic cooling series");
		check(Math.abs(fine.finalTemperaturesC[40] - exact) < Math.abs(coarse.finalTemperaturesC[40] - exact), "timestep refinement approaches analytic slab");
		for (t in fine.finalTemperaturesC)
			check(t >= 20 - 1e-8 && t <= 60 + 1e-8, "inert slab maximum principle");
		s.sections[0].left = boundary(Convection);
		s.sections[0].right = boundary(Convection);
		var convection = CureAnalysis.section(s.sections[0], s, 1000);
		check(convection.finalTemperaturesC[40] > fine.finalTemperaturesC[40], "finite convection cools slower than fixed-temperature faces");
		var reactive = spec();
		reactive.sections[0].left = boundary(Convection);
		reactive.sections[0].right = boundary(Convection);
		reactive.cure.kinetics.kind = AutocatalyticArrhenius;
		reactive.cure.kinetics.preExponentialPerSecond = 2000;
		reactive.cure.kinetics.autocatalyticPreExponentialPerSecond = 10000;
		reactive.cure.kinetics.activationEnergyJPerMol = 30000;
		reactive.solver.timeStepSeconds = 0.1;
		reactive.solver.durationSeconds = 1200;
		var refinements:Array<CureSectionResult> = [];
		for (cells in [11, 21, 41]) {
			reactive.solver.cells = cells;
			var result = CureAnalysis.section(reactive.sections[0], reactive, 2400);
			refinements.push(result);
			check(result.reactionHeatJPerM2 > 0 && result.boundaryHeatOutJPerM2 > 0, "convective reactive fixture exercises heat generation and heat loss");
			near(result.energyResidualJPerM2, 0, 1e-4, "global reactive/convection energy balance");
			check(result.reactionComplete, "reactive run reaches declared remaining-heat tolerance");
		}
		check(Math.abs(refinements[2].peakTemperatureC - refinements[1].peakTemperatureC) < Math.abs(refinements[1].peakTemperatureC
			- refinements[0].peakTemperatureC),
			"spatial refinement reduces convective reactive peak change");
		near(refinements[2].peakTemperatureC, refinements[1].peakTemperatureC, 0.2, "refined reactive peak agreement");
		reactive.solver.durationSeconds = 1;
		var incomplete = CureAnalysis.analyze("body", "s", "p", reactive, 2400, 100, true);
		check(!incomplete.sections[0].reactionComplete && status(incomplete, "process.cure.section.wall") == "pending",
			"short reactive run cannot pass on a lumped screening estimate");
		reactive.cure.gelMaximumTemperatureC = 21;
		reactive.solver.durationSeconds = 1200;
		check(status(CureAnalysis.analyze("body", "s", "p", reactive, 2400, 100, true), "process.cure.section.wall") == "fail",
			"gel-temperature limit is independently enforced");
		s = spec();
		s.recipeFingerprint = ToolingCodec.recipeFingerprint(p.document.recipe);
		s.sections = [];
		var integrated = CastingProcessAnalysis.analyze(p, s, 9.80665);
		if (integrated.pour==null) throw "Box process fixture requires a pour assessment";
		var integratedPour:materia.tooling.process.PourPlanning.PourAssessment=cast integrated.pour;
		check(integratedPour.sourceFingerprint == integrated.sourceFingerprint
			&& integratedPour.fillProcessFingerprint == integrated.processFingerprint,
			"pour plan consumes the same fingerprinted fill analysis");
		check(integrated.validation.filter(v -> v.rule == "process.pour.batch-schedule" && v.status == "pending").length == 1,
			"missing mixer data reaches aggregate readiness");
		var batchSpec = ProcessSpecCodec.snapshot(s);
		batchSpec.pourRateMm3PerSecond = 1e9;
		batchSpec.mixComposition = {reference: "synthetic mix", cureReference: batchSpec.cure.reference,
			cureMassIngredientIds: ["resin"], ingredients: [
				{id: "resin", kind: materia.tooling.process.PourPlanning.MixIngredientKind.Resin, materialReference: "synthetic resin", massFraction: 0.1},
				{id: "hardener", kind: materia.tooling.process.PourPlanning.MixIngredientKind.Hardener, materialReference: "synthetic hardener", massFraction: 0.05},
				{id: "aggregate", kind: materia.tooling.process.PourPlanning.MixIngredientKind.Aggregate, materialReference: "synthetic aggregate", massFraction: 0.85}
			]};
		batchSpec.mixer = {reference: "synthetic equipment", capacityKg: integratedPour.totalMixMassKg / 3,
			mixingSeconds: 0.001, transferSeconds: 0.001, workingTimeSeconds: 1000, workingTimeReference: "synthetic timing"};
		var batchEncoded = ProcessSpecCodec.encode(batchSpec);
		check(ProcessSpecCodec.encode(ProcessSpecCodec.decode(batchEncoded)) == batchEncoded,
			"mix ingredients, heat denominator and mixer survive strict codec roundtrip");
		var batched = CastingProcessAnalysis.analyze(p, batchSpec, 9.80665);
		if (batched.pour==null) throw "Box process fixture requires a batch assessment";
		var batchedPour:materia.tooling.process.PourPlanning.PourAssessment=cast batched.pour;
		var batchPlan = batchedPour.schedule;
		check(batchPlan != null && batchPlan.batches.length == 3, "real process pipeline includes the declared batch schedule");
		check(batched.validation.filter(v -> v.rule == "process.pour.cure-mass-basis" && v.status == "pass").length == 1,
			"weighed resin and cure heat denominator agree explicitly");
		check(batched.validation.filter(v -> v.rule == "process.pour.cure-sequence" && v.status == "pending").length == 1,
			"a uniform cure model cannot silently qualify a new multi-batch history");
		for (row in batchedPour.validation)
			check(batched.validation.filter(v -> v.rule == row.rule && ToolingCodec.canonical(v) == ToolingCodec.canonical(row)).length == 1,
				"aggregate report retains pour rule " + row.rule);
		batchSpec.cure.resinMassFraction = 0.2;
		var wrongBasis = CastingProcessAnalysis.analyze(p, batchSpec, 9.80665);
		check(wrongBasis.validation.filter(v -> v.rule == "process.pour.cure-mass-basis"
			&& v.status == "fail").length == 1, "inconsistent weighing and thermal models fail in the real pipeline");
		check(wrongBasis.cure.validation.filter(v -> v.rule == "process.pour.cure-mass-basis" && v.status == "fail").length == 1,
			"cure report retains its weighed-mass failure for downstream fabrication");
		var basisFabrication = FabricationTests.spec();
		basisFabrication.adhesiveContainsEPS = true;
		basisFabrication.resinReference = batchSpec.cure.reference;
		check(materia.tooling.export.ToolingFabrication.plan(p, basisFabrication, wrongBasis.cure).validation.filter(v ->
			v.rule == "fabrication.materials.cure" && v.status == "fail").length == 1,
			"fabrication cannot pass cure qualification with the wrong heat denominator");
		var resinMismatch = FabricationTests.spec();
		resinMismatch.resinReference = "another resin";
		check(CastingProcessAnalysis.analyze(p, s, 9.80665, resinMismatch)
			.validation.filter(v -> v.rule == "process.cure.resin-identity" && v.status == "fail")
			.length == 1,
			"cure and solvent resin identity must agree");
		var adhesiveDocument = ToolingCodec.snapshot(p.document);
		for (strategy in adhesiveDocument.setup.features)
			if (strategy.strategy == materia.tooling.casting.CastingSetup.CoreStrategy.SacrificialEPS) {
				strategy.strategy = materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer;
				strategy.withdrawal = {x: 0, y: 0, z: 1};
			}
		var adhesivePlan = materia.tooling.casting.CastingMoldPlanner.plan(adhesiveDocument, p.options);
		resinMismatch.adhesiveContainsEPS = true;
		check(adhesivePlan.cores.length == 0
			&& CastingProcessAnalysis.analyze(adhesivePlan, s, 9.80665, resinMismatch).cure.limits.filter(v -> v.id == "eps-core-stock").length == 1,
			"EPS adhesive is checked even without foam cores");
		for (v in p.validation)
			check(integrated.validation.filter(r -> r.rule == v.rule
				&& ToolingCodec.canonical(r) == ToolingCodec.canonical(v)).length == 1,
				"process report retains exact planner rule " + v.rule);
		check(!integrated.fabrication.ready, "analysis cannot clear existing mold blockers");
		var safeDocument = ToolingCodec.snapshot(p.document);
		safeDocument.recipe.core.maximumTemperatureC = 1000;
		var safePlan = materia.tooling.casting.CastingMoldPlanner.plan(safeDocument, p.options);
		var safeSpec = spec();
		safeSpec.recipeFingerprint = ToolingCodec.recipeFingerprint(safeDocument.recipe);
		safeSpec.cure.heatOfReactionJPerKgResin = 0;
		safeSpec.sections = [];
		var safeReport = CastingProcessAnalysis.analyze(safePlan, safeSpec, 9.80665);
		check(safeReport.cure.validation.filter(v -> v.rule == "process.cure.lumped-estimate" && v.status == "pass").length == 1,
			"inert lumped screening does not require a cooling-model geometry proof");
		check(safeReport.validation.filter(v -> v.rule == "process.cure.geometric-coverage"
			|| v.rule == "process.cure.junction-model").length > 0,
			"lumped screening never drops geometric coverage or junction model blockers");
		safeSpec.sections = [
			{
				id: "stale-bound-section",
				sourceIds: [p.document.input.id],
				spanIds: ["missing-span"],
				geometryFingerprint: haxe.crypto.Sha256.encode("stale"),
				thicknessMm: 20,
				left: boundary(Insulated),
				right: boundary(Insulated),
				evidence: []
			}
		];
		check(CastingProcessAnalysis.analyze(safePlan, safeSpec, 9.80665)
			.validation.filter(v -> v.rule == "process.cure.binding.stale-bound-section" && v.status == "fail")
			.length == 1,
			"lumped screening never hides an invalid authored geometry binding");
		s.recipeFingerprint = haxe.crypto.Sha256.encode("stale");
		rejects(() -> CastingProcessAnalysis.analyze(p, s, 9.80665), "Stale process recipe");
		JunctionThermalTests.run(spec());
		CastabilityTests.run(p, spec());
		OrientationSectionTests.run(p, spec());
		Sys.println('ProcessTests: $assertions assertions passed');
	}
}
