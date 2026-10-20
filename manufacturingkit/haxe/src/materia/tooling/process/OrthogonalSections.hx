package materia.tooling.process;

import materia.tooling.casting.BoxVolumes;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingIds;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.process.FillAnalysis.FillBoundary;

enum abstract SectionAxis(String) to String {
	var X = "x";
	var Y = "y";
	var Z = "z";
}

typedef PlanarSectionSpan = {
	var id:String;
	var axis:SectionAxis;
	var startMm:Float;
	var endMm:Float;
	var thicknessMm:Float;
	var areaMm2:Float;
	var volumeMm3:Float;
	var lowerSourceIds:Array<String>;
	var upperSourceIds:Array<String>;

	/** Exact material strips, not the arbitrary boxes left by subtraction. */
	var strips:Array<MoldBox>;
}

typedef OrthogonalSectionReport = {
	var version:Int;
	var sourceFingerprint:String;
	var geometryFingerprint:String;
	var spans:Array<PlanarSectionSpan>;
	var mixVolumeMm3:Float;
	var coveredVolumeByAxisMm3:Array<{axis:SectionAxis, volumeMm3:Float}>;

	/** Exact scalar for a rectangular solid. Nonrectangular geometry uses the junction interval. */
	var maximumInscribedThicknessMm:Null<Float>;

	var junction:JunctionThickness.JunctionThicknessReport;

	var validation:Array<ToolingValidation>;
}

/** Coordinate-compressed rays span the entire material interval between opposite
 * physical faces. Boolean partition sizes are never mistaken for wall thickness.
 */
class OrthogonalSections {
	static inline var EPS = 1e-7;

	static function lo(b:MoldBox, a:Int):Float
		return a == 0 ? b.x : a == 1 ? b.y : b.z;

	static function size(b:MoldBox, a:Int):Float
		return a == 0 ? b.width : a == 1 ? b.depth : b.height;

	static function values(boxes:Array<MoldBox>, axis:Int):Array<Float> {
		var all:Array<Float> = [];
		for (b in boxes) {
			all.push(lo(b, axis));
			all.push(lo(b, axis) + size(b, axis));
		}
		all.sort(Reflect.compare);
		var out:Array<Float> = [];
		for (v in all)
			if (out.length == 0 || v - out[out.length - 1] > EPS)
				out.push(v);
		return out;
	}

	static function owners(boundaries:Array<FillBoundary>, axis:Int, plane:Float, u:Int, v:Int, u0:Float, u1:Float, v0:Float, v1:Float, lower:Bool,
			sourceId:String):Array<String> {
		var ids:Array<String> = [];
		for (b in boundaries) {
			var face = lo(b.box, axis) + (lower ? size(b.box, axis) : 0);
			if (Math.abs(face - plane) > EPS
				|| lo(b.box, u) > u0 + EPS
				|| lo(b.box, u) + size(b.box, u) < u1 - EPS
				|| lo(b.box, v) > v0 + EPS
				|| lo(b.box, v) + size(b.box, v) < v1 - EPS)
				continue;
			if (ids.indexOf(b.id) < 0)
				ids.push(b.id);
		}
		if (ids.length == 0)
			ids.push(sourceId);
		ids.sort(Reflect.compare);
		return ids;
	}

	/** Convert an extracted span only after the caller declares its two actual thermal boundaries.
	 * This supplies geometric thickness/source IDs; it does not qualify 3D junction coverage.
	 */
	public static function cureSection(span:PlanarSectionSpan, left:CastingProcessSpec.ThermalBoundary, right:CastingProcessSpec.ThermalBoundary,
			evidence:Array<String>, geometryFingerprint:String):CastingProcessSpec.CureSection {
		ToolingIds.validate(span.id);
		if (!Math.isFinite(span.thicknessMm) || span.thicknessMm <= 0 || Math.abs(span.endMm - span.startMm - span.thicknessMm) > EPS)
			throw "Invalid extracted cure section";
		var ids:Array<String> = [];
		for (id in span.lowerSourceIds.concat(span.upperSourceIds))
			if (ids.indexOf(id) < 0) {
				ToolingIds.validate(id);
				ids.push(id);
			}
		ids.sort(Reflect.compare);
		return {
			id: span.id,
			sourceIds: ids,
			spanIds: [span.id],
			geometryFingerprint: geometryFingerprint,
			thicknessMm: span.thicknessMm,
			left: left,
			right: right,
			evidence: evidence.copy()
		};
	}

	public static function analyze(sourceId:String, sourceFingerprint:String, region:Array<MoldBox>, boundaries:Array<FillBoundary>):OrthogonalSectionReport {
		ToolingIds.validate(sourceId);
		if (region.length == 0 || region.length > 2048 || boundaries.length > 512)
			throw "Invalid section geometry count";
		var canonical = region.copy();
		canonical.sort((a, b) -> Reflect.compare(ToolingCodec.canonical(a), ToolingCodec.canonical(b)));
		for (boundary in boundaries)
			ToolingIds.validate(boundary.id);
		for (b in canonical.concat([for (boundary in boundaries) boundary.box])) {
			for (value in [b.x, b.y, b.z, b.width, b.depth, b.height])
				if (!Math.isFinite(value))
					throw "Invalid section box";
			if (b.width <= EPS || b.depth <= EPS || b.height <= EPS)
				throw "Invalid section box";
		}
		var boxes = BoxVolumes.subtract(canonical, []),
			result:Array<PlanarSectionSpan> = [],
			coverage:Array<{axis:SectionAxis, volumeMm3:Float}> = [];
		var volume = BoxVolumes.volume(boxes),
			axes:Array<SectionAxis> = [X, Y, Z];
		for (axis in 0...3) {
			var u = (axis + 1) % 3,
				v = (axis + 2) % 3,
				us = values(boxes.concat([for (boundary in boundaries) boundary.box]), u),
				vs = values(boxes.concat([for (boundary in boundaries) boundary.box]), v);
			if ((us.length - 1.0) * (vs.length - 1) * boxes.length > 10000000)
				throw "Orthogonal section work limit exceeded";
			var spans = new Map<String, PlanarSectionSpan>(), total = 0.0;
			for (i in 0...us.length - 1)
				for (j in 0...vs.length - 1) {
					var u0 = us[i], u1 = us[i + 1], v0 = vs[j], v1 = vs[j + 1];
					var intervals:Array<{start:Float, end:Float}> = [];
					for (b in boxes)
						if (lo(b, u) <= u0 + EPS && lo(b, u) + size(b, u) >= u1 - EPS && lo(b, v) <= v0 + EPS && lo(b, v) + size(b, v) >= v1 - EPS)
							intervals.push({start: lo(b, axis), end: lo(b, axis) + size(b, axis)});
					intervals.sort((a, b) -> a.start == b.start ? Reflect.compare(a.end, b.end) : Reflect.compare(a.start, b.start));
					var runs:Array<{start:Float, end:Float}> = [];
					for (interval in intervals) {
						if (runs.length == 0 || interval.start > runs[runs.length - 1].end + EPS)
							runs.push({start: interval.start, end: interval.end});
						else
							runs[runs.length - 1].end = Math.max(runs[runs.length - 1].end, interval.end);
					}
					for (run in runs) {
						var lower = owners(boundaries, axis, run.start, u, v, u0, u1, v0, v1, true, sourceId);
						var upper = owners(boundaries, axis, run.end, u, v, u0, u1, v0, v1, false, sourceId);
						var key = ToolingCodec.canonical({
							axis: axes[axis],
							start: run.start,
							end: run.end,
							lower: lower,
							upper: upper
						});
						var span = spans.get(key);
						if (span == null) {
							span = {
								id: ToolingIds.derive("section-", haxe.crypto.Sha256.encode(key).substr(0, 20)),
								axis: axes[axis],
								startMm: run.start,
								endMm: run.end,
								thicknessMm: run.end - run.start,
								areaMm2: 0,
								volumeMm3: 0,
								lowerSourceIds: lower,
								upperSourceIds: upper,
								strips: []
							};
							spans.set(key, span);
						}
						var origin = [0.0, 0.0, 0.0], extent = [0.0, 0.0, 0.0];
						origin[axis] = run.start;
						origin[u] = u0;
						origin[v] = v0;
						extent[axis] = run.end - run.start;
						extent[u] = u1 - u0;
						extent[v] = v1 - v0;
						var area = (u1 - u0) * (v1 - v0),
							amount = area * (run.end - run.start);
						span.areaMm2 += area;
						span.volumeMm3 += amount;
						total += amount;
						span.strips.push({
							x: origin[0],
							y: origin[1],
							z: origin[2],
							width: extent[0],
							depth: extent[1],
							height: extent[2]
						});
					}
				}
			if (Math.abs(total - volume) > Math.max(1e-6, volume * 1e-10))
				throw "Section rays do not cover the mix volume";
			coverage.push({axis: axes[axis], volumeMm3: total});
			for (span in spans)
				result.push(span);
		}
		result.sort((a, b) -> Reflect.compare(a.id, b.id));
		var minimum = [for (a in 0...3) values(boxes, a)[0]], maximum = [
			for (a in 0...3) {
				var x = values(boxes, a);
				x[x.length - 1];
			}
		];
		var w = maximum[0] - minimum[0],
			d = maximum[1] - minimum[1],
			h = maximum[2] - minimum[2];
		var rectangular = Math.abs(w * d * h - volume) <= Math.max(1e-6, volume * 1e-10);
		var junction = JunctionThickness.analyze(sourceId, sourceFingerprint, boxes, boundaries);
		return {
			version: 2,
			sourceFingerprint: sourceFingerprint,
			geometryFingerprint: haxe.crypto.Sha256.encode(ToolingCodec.canonical({spans: result, volumeMm3: volume})),
			spans: result,
			mixVolumeMm3: volume,
			coveredVolumeByAxisMm3: coverage,
			maximumInscribedThicknessMm: rectangular ? Math.min(w, Math.min(d, h)) : null,
			junction: junction,
			validation: [
				FillAnalysis.check("process.sections.planar-coverage", [sourceId], ToolingCheckStatus.Pass,
					"Opposing-face material intervals cover the exact resin volume independently on all three axes.", [sourceFingerprint]),
				FillAnalysis.check("process.sections.junction-thickness", [sourceId],
					junction.converged ? ToolingCheckStatus.Pass : ToolingCheckStatus.Pending,
					"Largest inscribed diameter bounded by exposed-face distance: "
					+ junction.lowerDiameterMm
					+ ".."
					+ junction.upperDiameterMm
					+ " mm. Numerical geometry convergence does not qualify junction heat flow.",
					[sourceFingerprint])
			]
		};
	}
}
