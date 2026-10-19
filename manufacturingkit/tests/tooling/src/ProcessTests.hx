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

	static function spec():CastingProcessSpec
		return {
			id: "process-test",
			revision: 1,
			recipeFingerprint: haxe.crypto.Sha256.encode("test-recipe"),
			initialTemperatureC: 20,
			ambientTemperatureC: 20,
			temperatureMarginC: 5,
			pourRateMm3PerSecond: null,
			maximumAirEscapeWidthMm: null,
			cure: {
				reference: "test-only resin",
				resinMassFraction: 0.1,
				heatOfReactionJPerKgResin: 500000,
				mixSpecificHeatJPerKgK: 1000,
				mixConductivityWPerMK: 1,
				resinMaximumTemperatureC: 120,
				kinetics: {
					kind: FirstOrderArrhenius,
					preExponentialPerSecond: 0.01,
					activationEnergyJPerMol: 0,
					initialConversion: 0
				},
				evidence: ["analytic-fixture"]
			},
			sections: [
				{
					id: "wall",
					sourceIds: ["body"],
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
				convergenceToleranceC: 1e-8,
				maximumIterations: 100
			}
		};

	static function status(report:CureReport, rule:String):String {
		var rows = report.validation.filter(r -> r.rule == rule);
		if (rows.length != 1)
			throw "Missing/duplicate cure rule: " + rule;
		return rows[0].status;
	}

	public static function run(p:CastingMoldPlan):Void {
		var region = [box(0, 0, 2, 10), box(2, 0, 2, 2), box(4, 0, 4, 6)];
		var boundaries = [{id: "ceiling-feature", box: box(4, 6, 4, 4)}];
		var cores = [{id: "foam", boxes: [box(10, 1, 2, 3)]}];
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
		check(ToolingCodec.canonical(fill(region, [1, 3, 5, 9]).trappedPockets) == ToolingCodec.canonical(trapped.trappedPockets),
			"noncritical output levels do not change pocket IDs or volumes");
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
		var overlapCore = FillAnalysis.analyze("body", "s", "p", region, boundaries, [{id: "overlap", boxes: [box(10, 1, 2, 3), box(10, 2, 2, 2)]}], 10, 2400,
			10, null, null);
		near(overlapCore.levels[overlapCore.levels.length - 1].cores[0].submergedVolumeMm3, 12, 1e-10, "overlapping core displacement counted once");
		var s = spec(), encoded = ProcessSpecCodec.encode(s);
		check(ProcessSpecCodec.encode(ProcessSpecCodec.decode(encoded)) == encoded, "strict process roundtrip");
		var raw:Dynamic = haxe.Json.parse(encoded);
		raw.version = 2;
		rejects(() -> ProcessSpecCodec.decode(haxe.Json.stringify(raw)), "Unsupported casting process");
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
		near(insulated.futurePeakBoundC, 70, 1e-6, "insulated future bound retains all reaction energy");
		check(insulated.peakTemperatureC <= 70 + 1e-6, "reaction cannot exceed adiabatic upper bound");
		s.cure.kinetics.initialConversion = 0.25;
		var partial = CureAnalysis.section(s.sections[0], s, 2400);
		near(partial.futurePeakBoundC, 57.5, 1e-6, "initial conversion reduces remaining reaction energy");
		s = spec();
		s.cure.kinetics = null;
		var safe = CureAnalysis.analyze("body", "s", "p", s, 2400, 100, true);
		check(status(safe, "process.cure.adiabatic-bound") == "pass", "adiabatic bound can pass without kinetics");
		check(safe.sections.length == 0, "passing upper bound skips section tier");
		var uncertain = CureAnalysis.analyze("body", "s", "p", s, 2400, 60, true);
		check(status(uncertain, "process.cure.adiabatic-bound") == "pending", "high upper bound is not proof of overheating");
		check(status(uncertain, "process.cure.section.wall") == "pending"
			&& uncertain.sections.length == 0, "missing kinetics cannot pass section tier");
		s.cure.resinMassFraction = null;
		check(CureAnalysis.analyze("body", "s", "p", s, 2400, 60, true).adiabaticPeakBoundC == null, "missing material property produces no invented bound");
		s = spec();
		var overheated = CureAnalysis.analyze("body", "s", "p", s, 2400, 60, true);
		check(status(overheated, "process.cure.section.wall") == "fail", "predicted section peak above EPS limit fails");
		s.solver.durationSeconds = 1;
		var short = CureAnalysis.analyze("body", "s", "p", s, 2400, 60, true);
		check(status(short, "process.cure.section.wall") == "pending", "early termination cannot hide future heat");
		check(short.sections[0].peakTemperatureC < 60
			&& short.sections[0].futurePeakBoundC > 60, "short run reports residual exotherm bound");
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
		check(status(CureAnalysis.analyze("body", "s", "p", s, 2400, 100, true), "process.cure.adiabatic-bound") == "pending",
			"unknown ambient temperature cannot qualify upper bound");
		s.ambientTemperatureC = 100;
		check(status(CureAnalysis.analyze("body", "s", "p", s, 2400, 100, true), "process.cure.adiabatic-bound") == "pending",
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
		s = spec();
		s.recipeFingerprint = ToolingCodec.recipeFingerprint(p.document.recipe);
		s.sections = [];
		var integrated = CastingProcessAnalysis.analyze(p, s, 9.80665);
		for (v in p.validation)
			check(integrated.validation.filter(r -> r.rule == v.rule
				&& ToolingCodec.canonical(r) == ToolingCodec.canonical(v)).length == 1,
				"process report retains exact planner rule " + v.rule);
		check(!integrated.fabrication.ready, "analysis cannot clear existing mold blockers");
		s.recipeFingerprint = haxe.crypto.Sha256.encode("stale");
		rejects(() -> CastingProcessAnalysis.analyze(p, s, 9.80665), "Stale process recipe");
		Sys.println('ProcessTests: $assertions assertions passed');
	}
}
