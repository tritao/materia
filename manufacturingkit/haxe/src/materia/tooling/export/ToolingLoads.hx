package materia.tooling.export;

import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.BoxVolumes;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;
import materia.tooling.export.FabricationSpec;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelGeometry;
import materia.assembly.AssemblyFrames as F;

enum abstract LoadEstimateKind(String) to String {
	var PanelPeakBound = "panel-peak-bound";
	var CoreGrossUplift = "core-gross-uplift";
	var BraceUniformBeam = "brace-uniform-beam";
	var FastenerEqualShare = "fastener-equal-share";
	var SupportUplift = "support-uplift";
	var RigidGrossUpliftBound = "rigid-gross-uplift-bound";
	var RigidLateralXBound = "rigid-lateral-x-bound";
	var RigidLateralYBound = "rigid-lateral-y-bound";
}

typedef LoadEstimate = {
	var id:String;
	var kind:LoadEstimateKind;
	var pressurePa:Float;
	var forceN:Float;
	var displacedVolumeMm3:Float;
	var deflectionMm:Null<Float>;
	var stressPa:Null<Float>;
}

typedef LoadReport = {var estimates:Array<LoadEstimate>; var validation:Array<ToolingValidation>;}

/** Baseline liquid pressure; conservative peak-pressure structural demand, not paste flow. */
class ToolingLoads {
	static function value(map:Map<String, Float>, id:String):Float {
		var v = map.get(id);
		if (v == null)
			throw "Missing load estimate";
		return v;
	}

	public static function positive(v:Float):Void {
		if (!Math.isFinite(v) || v <= 0)
			throw "Load input must be finite and positive";
	}

	public static function check(rule:String, ids:Array<String>, status:ToolingCheckStatus, message:String, evidence:Array<String>):ToolingValidation {
		return {
			rule: rule,
			affectedIds: ids,
			status: status,
			severity: status == Pass ? Info : status == Pending ? Warning : Error,
			message: message,
			evidence: evidence.copy()
		};
	}

	/** Navier odd Fourier series for an isotropic, four-edge simply-supported rectangle under uniform p.
	 * Returns centre deflection and maximum centre bending stress. Cutouts/orthotropy are unsupported.
	 */
	public static function plate(p:Float, a:Float, b:Float, t:Float, e:Float, nu:Float, terms:Int = 63):{deflectionMm:Float, stressPa:Float} {
		for (v in [a, b, t, e])
			positive(v);
		if (!Math.isFinite(p) || p < 0 || !Math.isFinite(nu) || nu < 0 || nu >= 0.5 || terms < 3 || terms > 127)
			throw "Invalid plate model input";
		var d = e * t * t * t / (12 * (1 - nu * nu)), w = 0.0, mx = 0.0, my = 0.0;
		for (i in 0...terms)
			for (j in 0...terms) {
				var m = 2 * i + 1, n = 2 * j + 1, aa = m * m / (a * a), bb = n * n / (b * b);
				var sign = (i + j) % 2 == 0 ? 1.0 : -1.0;
				var coeff = 16 * p * sign / (Math.pow(Math.PI, 6) * d * m * n * Math.pow(aa + bb, 2));
				w += coeff;
				mx += d * Math.PI * Math.PI * (aa + nu * bb) * coeff;
				my += d * Math.PI * Math.PI * (nu * aa + bb) * coeff;
			}
		if (!Math.isFinite(w) || !Math.isFinite(mx) || !Math.isFinite(my))
			throw "Plate calculation exceeds numeric range";
		return {deflectionMm: Math.abs(w) * 1000, stressPa: 6 * Math.max(Math.abs(mx), Math.abs(my)) / (t * t)};
	}

	public static function beam(p:Float, span:Float, tributary:Float, width:Float, depth:Float, e:Float):{deflectionMm:Float, stressPa:Float} {
		for (v in [span, tributary, width, depth, e])
			positive(v);
		if (!Math.isFinite(p) || p < 0)
			throw "Invalid beam pressure";
		var q = p * tributary, i = width * Math.pow(depth, 3) / 12;
		return {deflectionMm: 5 * q * Math.pow(span, 4) / (384 * e * i) * 1000, stressPa: q * span * span * depth / (16 * i)};
	}

	public static function report(p:CastingMoldPlan, s:FabricationSpec):LoadReport {
		positive(s.gravityMps2);
		positive(s.loadFactor);
		if (s.loadFactor < 1)
			throw "Load factor must be at least one";
		if (s.compactionPressurePa != null && (!Math.isFinite(s.compactionPressurePa) || s.compactionPressurePa < 0))
			throw "Invalid compaction pressure";
		var r:LoadReport = {estimates: [], validation: []};
		var declared = p.options.compaction.method != "undeclared";
		var compact = declared
			&& (p.options.compaction.method == "none" || (s.compactionPressurePa != null && s.compactionEvidence.length > 0));
		r.validation.push(check("fabrication.loads.compaction", [p.document.input.id], compact ? Pass : Pending,
			"Additional peak pressure requires a declared compaction method and evidence; hydrostatic alone does not qualify vibration.",
			s.compactionEvidence));
		var extra = s.compactionPressurePa == null ? 0.0 : s.compactionPressurePa;
		var demands = new Map<String, LoadEstimate>();
		var cores = p.cores.copy();
		cores.sort((a, b) -> Reflect.compare(a.id, b.id));
		var occupied:Array<materia.tooling.casting.MoldPlan.MoldBox> = [];
		var coreVolumes = new Map<String, Float>(),
			coreForces = new Map<String, Float>();
		for (c in cores) {
			var volume = BoxVolumes.volume(BoxVolumes.subtract([c.cavity], occupied));
			occupied.push(c.cavity);
			coreVolumes.set(c.id, volume);
			coreForces.set(c.id,
				(p.document.recipe.mixDensityKgM3 * s.gravityMps2 * volume * 1e-9 + extra * c.cavity.width * c.cavity.depth * 1e-6) * s.loadFactor);
		}
		for (m in p.panels.members.concat(p.supportPanels)) {
			var b = PanelGeometry.bounds(PanelContours.outside(m.panel));
			var zmin = 1e30, zmax = -1e30;
			for (x in [b.x, b.x + b.width])
				for (y in [b.y, b.y + b.height]) {
					var q = F.transformPoint(m.pose, x, y, 0);
					zmin = Math.min(zmin, q.z);
					zmax = Math.max(zmax, q.z);
				}
			var head = Math.max(0, p.document.setup.fillHeightMm - Math.max(p.cavity.z, zmin)) * 0.001;
			var pressure = p.document.recipe.mixDensityKgM3 * s.gravityMps2 * head;
			// Bounding face area and maximum pressure are conservative; external supports have zero fluid contact.
			var isWall = p.panels.members.indexOf(m) >= 0;
			if (!isWall)
				pressure = 0;
			var design = (pressure + (isWall ? extra : 0)) * s.loadFactor;
			var demand:LoadEstimate = {
				id: m.panel.id,
				kind: PanelPeakBound,
				pressurePa: design,
				forceN: design * b.width * b.height * 1e-6,
				displacedVolumeMm3: 0,
				deflectionMm: null,
				stressPa: null
			};
			if (!isWall) {
				demand.kind = SupportUplift;
				for (c in cores)
					if (c.bridgeId == m.panel.id || c.carrierId == m.panel.id)
						demand.forceN += value(coreForces, c.id);
				demand.pressurePa = demand.forceN / (b.width * b.height * 1e-6);
				r.validation.push(check("fabrication.support-loadpath." + m.panel.id, [m.panel.id], Pending,
					"Core forces reach the carrier/bridge as local loads. Rail reactions, adhesive distribution and connections need a qualified load-path model.",
					[]));
			}
			demands.set(demand.id, demand);
			r.estimates.push(demand);
			var outline = PanelContours.outside(m.panel);
			var plain = m.panel.operations.length == 1
				&& outline.length == 4
				&& outline.filter(e -> e.center != null).length == 0
				&& outline.filter(e -> Math.abs(e.end.x - e.start.x) > 1e-7 && Math.abs(e.end.y - e.start.y) > 1e-7).length == 0;
			if (!plain || !isWall) {
				r.validation.push(check("fabrication.plate." + m.panel.id, [m.panel.id], Unsupported,
					"This panel has rebates/holes/tabs, a nonrectangular boundary or local support loads. No structural model for its actual geometry is implemented; rectangular plate capacity is not credited.",
					[]));
				continue;
			}
			var supports = s.plates.filter(v -> v.panelId == m.panel.id);
			if (supports.length == 0) {
				r.validation.push(check("fabrication.plate." + m.panel.id, [m.panel.id], Pending,
					"Panel support spans, isotropic material properties and deflection/stress limits are not qualified.", []));
				continue;
			}
			if (supports.length != 1)
				throw "Duplicate plate support";
			var v = supports[0];
			for (n in [v.spanXmm, v.spanYmm, v.youngPa, v.allowableStressPa, v.maximumDeflectionMm])
				positive(n);
			if (v.spanXmm > b.width || v.spanYmm > b.height)
				throw "Plate span exceeds panel blank";
			var a = plate(design, v.spanXmm * 0.001, v.spanYmm * 0.001, m.panel.stock.measuredThickness * 0.001, v.youngPa, v.poisson);
			demand.deflectionMm = a.deflectionMm;
			demand.stressPa = a.stressPa;
			var over = a.stressPa > v.allowableStressPa || a.deflectionMm > v.maximumDeflectionMm;
			r.validation.push(check("fabrication.plate." + m.panel.id, [m.panel.id], over ? Fail : compact && v.evidence.length > 0 ? Pass : Pending,
				"Uniform peak-pressure isotropic simply-supported rectangular plate; support/material evidence is required.", v.evidence));
		}
		for (c in cores) {
			var volume = value(coreVolumes, c.id),
				design = value(coreForces, c.id);
			var load:LoadEstimate = {
				id: c.id,
				kind: CoreGrossUplift,
				pressurePa: 0,
				forceN: design,
				displacedVolumeMm3: volume,
				deflectionMm: null,
				stressPa: null
			};
			demands.set(c.id, load);
			r.estimates.push(load);
			var matches = s.coreRestraints.filter(v -> v.coreId == c.id);
			if (matches.length > 1)
				throw "Duplicate core restraint";
			var status:ToolingCheckStatus = Pending,
				evidence:Array<String> = [];
			if (matches.length == 1) {
				var v = matches[0];
				for (n in [v.allowableN, v.bearingAreaMm2, v.allowableBearingPa])
					positive(n);
				if (v.bearingAreaMm2 > c.bondAreaMm2)
					throw "Core restraint area exceeds planned bond area";
				evidence = v.evidence;
				status = design > v.allowableN
					|| design / (v.bearingAreaMm2 * 1e-6) > v.allowableBearingPa ? Fail : compact && evidence.length > 0 ? Pass : Pending;
			}
			r.validation.push(check("fabrication.core-restraint." + c.id, [c.id], status,
				"Gross displaced mix uplift, without EPS weight credit; attachment and dimensional bearing limits require evidence.", evidence));
		}
		var rigid = [for (former in p.formers) former.volume].concat([for (fixture in p.fixtures) fixture.datum]);
		rigid.sort((a, b) -> Reflect.compare(a.id, b.id));
		for (member in rigid) {
			var clipped:Array<materia.tooling.casting.MoldPlan.MoldBox> = [];
			for (box in materia.tooling.casting.MoldVolumeBounds.pieces(member)) {
				var wet = BoxVolumes.intersection(box, p.envelope);
				if (wet != null) {
					wet.height = Math.min(wet.height, p.document.setup.fillHeightMm - wet.z);
					if (wet.height > 1e-7)
						clipped.push(wet);
				}
			}
			var volume = BoxVolumes.volume(clipped),
				zmin = p.document.setup.fillHeightMm;
			for (box in clipped)
				zmin = Math.min(zmin, box.z);
			function projection(axis:Int):Float {
				var footprints:Array<materia.tooling.casting.MoldPlan.MoldBox> = [
					for (box in clipped)
						{
							x: axis == 0 ? box.y : box.x,
							y: axis == 2 ? box.y : box.z,
							z: 0,
							width: axis == 0 ? box.depth : box.width,
							depth: axis == 2 ? box.depth : box.height,
							height: 1
						}
				];
				return BoxVolumes.volume(BoxVolumes.subtract(footprints, []));
			}
			var pressure = (p.document.recipe.mixDensityKgM3 * s.gravityMps2 * Math.max(0, p.document.setup.fillHeightMm - zmin) * 0.001
				+ extra) * s.loadFactor;
			var uplift = (p.document.recipe.mixDensityKgM3 * s.gravityMps2 * volume * 1e-9 + extra * projection(2) * 1e-6) * s.loadFactor;
			var lateralX = pressure * projection(0) * 1e-6,
				lateralY = pressure * projection(1) * 1e-6;
			for (demand in [
				{id: member.id, kind: RigidGrossUpliftBound, force: uplift},
				{id: materia.tooling.ToolingIds.derive("", member.id, "-lateral-x"), kind: RigidLateralXBound, force: lateralX},
				{id: materia.tooling.ToolingIds.derive("", member.id, "-lateral-y"), kind: RigidLateralYBound, force: lateralY}
			]) {
				var load:LoadEstimate = {
					id: demand.id,
					kind: demand.kind,
					pressurePa: pressure,
					forceN: demand.force,
					displacedVolumeMm3: volume,
					deflectionMm: null,
					stressPa: null
				};
				if (demands.exists(load.id))
					throw "Duplicate load estimate ID";
				r.estimates.push(load);
				demands.set(load.id, load);
			}
			var restraints = s.rigidRestraints.filter(v -> v.memberId == member.id);
			if (restraints.length > 1)
				throw "Duplicate rigid tooling restraint";
			var status:ToolingCheckStatus = volume == 0 ? Pass : Pending,
				evidence:Array<String> = [];
			if (restraints.length == 1) {
				var restraint = restraints[0];
				for (n in [restraint.upliftCapacityN, restraint.lateralXCapacityN, restraint.lateralYCapacityN])
					positive(n);
				evidence = restraint.evidence;
				status = uplift > restraint.upliftCapacityN
					|| lateralX > restraint.lateralXCapacityN
					|| lateralY > restraint.lateralYCapacityN ? Fail : compact && evidence.length > 0 ? Pass : Pending;
			}
			r.validation.push(check("fabrication.rigid-restraint." + member.id, [member.id], status,
				"Gross displaced-volume uplift and unbalanced X/Y peak-pressure bounds, with no self-weight credit. Profile bounds and ignored holes are conservative; uplift and lateral attachment capacities need evidence.",
				evidence));
		}
		for (restraint in s.rigidRestraints)
			if (rigid.filter(v -> v.id == restraint.memberId).length != 1)
				throw "Unknown rigid tooling restraint member";
		for (v in s.plates)
			if (!demands.exists(v.panelId))
				throw "Unknown plate panel";
		for (v in s.coreRestraints)
			if (cores.filter(c -> c.id == v.coreId).length != 1)
				throw "Unknown restraint core";
		var ids = new Map<String, Bool>();
		for (v in s.braces) {
			materia.tooling.ToolingIds.validate(v.id);
			if (ids.exists(v.id))
				throw "Duplicate brace ID";
			ids.set(v.id, true);
			positive(v.allowableStressPa);
			positive(v.maximumDeflectionMm);
			var pressure = 0.0;
			if (v.panelIds.length == 0)
				throw "Brace needs panel references";
			for (id in v.panelIds) {
				if (!demands.exists(id) || p.panels.members.concat(p.supportPanels).filter(m -> m.panel.id == id).length != 1)
					throw "Unknown brace panel";
				pressure = Math.max(pressure, demands.get(id).pressurePa);
			}
			var value = beam(pressure, v.spanMm * 0.001, v.tributaryWidthMm * 0.001, v.widthMm * 0.001, v.depthMm * 0.001, v.youngPa);
			r.estimates.push({
				id: v.id,
				kind: BraceUniformBeam,
				pressurePa: pressure,
				forceN: pressure * v.spanMm * v.tributaryWidthMm * 1e-6,
				displacedVolumeMm3: 0,
				deflectionMm: value.deflectionMm,
				stressPa: value.stressPa
			});
			r.validation.push(check("fabrication.brace." + v.id,
				v.panelIds, value.stressPa > v.allowableStressPa || value.deflectionMm > v.maximumDeflectionMm ? Fail : compact
				&& v.evidence.length > 0 ? Pass : Pending,
				"Simply-supported rectangular brace, uniform tributary peak pressure; connection geometry remains separately qualified.", v.evidence));
		}
		if (s.braces.length == 0)
			r.validation.push(check("fabrication.braces", [p.document.input.id], Pending, "No qualified brace geometry/capacity supplied.", []));
		for (v in s.fasteners) {
			materia.tooling.ToolingIds.validate(v.id);
			if (ids.exists(v.id))
				throw "Duplicate structural ID";
			ids.set(v.id, true);
			positive(v.allowableN);
			if (v.hardwareIds.length < 1 || v.hardwareIds.length > 10000 || v.loadIds.length == 0)
				throw "Invalid fastener capacity group";
			for (id in v.hardwareIds)
				if (p.hardware.filter(h -> h.id == id).length != 1)
					throw "Unknown structural fastener hardware";
			var force = 0.0, seen = new Map<String, Bool>();
			for (id in v.loadIds) {
				if (!demands.exists(id) || seen.exists(id))
					throw "Invalid fastener load reference";
				seen.set(id, true);
				force += demands.get(id).forceN;
			}
			r.estimates.push({
				id: v.id,
				kind: FastenerEqualShare,
				pressurePa: 0,
				forceN: force / v.hardwareIds.length,
				displacedVolumeMm3: 0,
				deflectionMm: null,
				stressPa: null
			});
			r.validation.push(check("fabrication.fastener." + v.id, v.loadIds,
				force / v.hardwareIds.length > v.allowableN ? Fail : compact && v.evidence.length > 0 ? Pass : Pending,
				"Equal reaction shares require connection/load-distribution evidence; diameter alone is not capacity.", v.evidence));
		}
		if (s.fasteners.length == 0)
			r.validation.push(check("fabrication.fasteners", [p.document.input.id], Pending, "No qualified closure/fastener capacities supplied.", []));
		r.estimates.sort((a, b) -> Reflect.compare(a.id, b.id));
		r.validation.sort((a, b) -> Reflect.compare(a.rule, b.rule));
		return r;
	}
}
