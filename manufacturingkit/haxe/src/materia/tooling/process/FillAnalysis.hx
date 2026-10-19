package materia.tooling.process;

import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingIds;
import materia.tooling.casting.BoxVolumes;
import materia.tooling.casting.MoldPlan.MoldBox;

typedef FillBoundary = {var id:String; var box:MoldBox;}
typedef FillCore = {var id:String; var boxes:Array<MoldBox>;}
typedef FillCeiling = {var sourceIds:Array<String>; var box:MoldBox;}

typedef AirPocket = {
	var id:String;
	var sealedAtMm:Float;
	var volumeMm3:Float;
	var ceilingFaces:Array<FillCeiling>;
	var sourceIds:Array<String>;

	/** Candidate only: routing/sealing a vent is separate tooling/design work. */
	var proposedVentPoint:{x:Float, y:Float, z:Float};
}

typedef FillLevel = {
	var heightMm:Float;
	var mixVolumeMm3:Float;
	var mixMassKg:Float;
	var timeSeconds:Null<Float>;
	var maximumHeadPa:Float;
	var cores:Array<{id:String, submergedVolumeMm3:Float, grossUpliftN:Float}>;
}

typedef FillReport = {
	var sourceFingerprint:String;
	var processFingerprint:String;
	var model:String;
	var criticalHeightsMm:Array<Float>;
	var totalMixVolumeMm3:Float;
	var levels:Array<FillLevel>;
	var trappedPockets:Array<AirPocket>;
	var validation:Array<ToolingValidation>;
}

/** Exact rectilinear event-plane analysis. Air connectivity is evaluated in the
 * right-hand limit at each critical height: boxes ending there disappear,
 * surviving boxes retain their limiting volume. No offset height or pose sampling.
 * Positive-area faces connect cells; edge/point contacts never vent a pocket.
 */
class FillAnalysis {
	static inline var EPS = 1e-7;

	public static function check(rule:String, ids:Array<String>, status:ToolingCheckStatus, message:String, evidence:Array<String>):ToolingValidation
		return {
			rule: rule,
			affectedIds: ids.copy(),
			status: status,
			severity: status == ToolingCheckStatus.Fail ? Error : status == ToolingCheckStatus.Pass ? Info : Warning,
			message: message,
			evidence: evidence.copy()
		};

	static function sorted(boxes:Array<MoldBox>):Array<MoldBox> {
		var out = [for (b in boxes) BoxVolumes.copy(b)];
		out.sort((a, b) -> Reflect.compare(ToolingCodec.canonical(a), ToolingCodec.canonical(b)));
		return out;
	}

	static function above(boxes:Array<MoldBox>, h:Float):Array<MoldBox> {
		var out:Array<MoldBox> = [];
		for (b in boxes)
			if (b.z + b.height > h + EPS) {
				var c = BoxVolumes.copy(b);
				c.z = Math.max(h, b.z);
				c.height = b.z + b.height - c.z;
				out.push(c);
			}
		return out;
	}

	static function submerged(boxes:Array<MoldBox>, h:Float):Float {
		var volume = 0.0;
		for (b in boxes)
			volume += b.width * b.depth * Math.max(0, Math.min(b.height, h - b.z));
		return volume;
	}

	static function unique(values:Array<Float>):Array<Float> {
		values.sort(Reflect.compare);
		var out:Array<Float> = [];
		for (v in values)
			if (out.length == 0 || v - out[out.length - 1] > EPS)
				out.push(v);
		return out;
	}

	static function owners(face:MoldBox, boundaries:Array<FillBoundary>, sourceId:String):Array<String> {
		var ids:Array<String> = [];
		for (b in boundaries)
			if (Math.abs(b.box.z - face.z) <= EPS
				&& Math.min(b.box.x + b.box.width, face.x + face.width) - Math.max(b.box.x, face.x) > EPS
				&& Math.min(b.box.y + b.box.depth, face.y + face.depth) - Math.max(b.box.y, face.y) > EPS)
				if (ids.indexOf(b.id) < 0)
					ids.push(b.id);
		if (ids.length == 0)
			ids.push(sourceId);
		ids.sort(Reflect.compare);
		return ids;
	}

	/** A face is represented by its exact XY footprint and zero height. */
	static function ceilings(group:Array<MoldBox>, all:Array<MoldBox>, fill:Float, boundaries:Array<FillBoundary>, sourceId:String):Array<FillCeiling> {
		var faces:Array<FillCeiling> = [];
		for (b in group) {
			var z = b.z + b.height;
			if (z >= fill - EPS)
				continue;
			var footprint = {
				x: b.x,
				y: b.y,
				z: 0.0,
				width: b.width,
				depth: b.depth,
				height: 1.0
			};
			var cover:Array<MoldBox> = [];
			for (c in all)
				if (c.z <= z + EPS && c.z + c.height > z + EPS)
					cover.push({
						x: c.x,
						y: c.y,
						z: 0,
						width: c.width,
						depth: c.depth,
						height: 1
					});
			for (face in BoxVolumes.subtract([footprint], cover)) {
				face.z = z;
				face.height = 0;
				faces.push({box: face, sourceIds: owners(face, boundaries, sourceId)});
			}
		}
		faces.sort((a, b) -> Reflect.compare(ToolingCodec.canonical(a), ToolingCodec.canonical(b)));
		return faces;
	}

	public static function analyze(sourceId:String, sourceFingerprint:String, processFingerprint:String, region:Array<MoldBox>,
			boundaries:Array<FillBoundary>, cores:Array<FillCore>, fillHeightMm:Float, densityKgM3:Float, gravityMps2:Float, pourRate:Null<Float>,
			airEscapeWidth:Null<Float>, ?extraLevels:Array<Float>):FillReport {
		ToolingIds.validate(sourceId);
		if (region.length == 0
			|| region.length > 2048
			|| boundaries.length > 512
			|| cores.length > 512
			|| extraLevels != null
			&& extraLevels.length > 4096
			|| !Math.isFinite(fillHeightMm)
			|| !Math.isFinite(densityKgM3)
			|| densityKgM3 <= 0
			|| !Math.isFinite(gravityMps2)
			|| gravityMps2 <= 0
			|| pourRate != null
			&& (!Math.isFinite(pourRate) || pourRate <= 0)
			|| airEscapeWidth != null
			&& (!Math.isFinite(airEscapeWidth) || airEscapeWidth <= 0))
			throw "Invalid fill analysis inputs";
		for (boundary in boundaries)
			ToolingIds.validate(boundary.id);
		var geometry = region.copy();
		for (boundary in boundaries)
			geometry.push(boundary.box);
		for (core in cores)
			for (box in core.boxes)
				geometry.push(box);
		for (b in geometry) {
			for (n in [b.x, b.y, b.z, b.width, b.depth, b.height])
				if (!Math.isFinite(n))
					throw "Nonfinite fill box";
			if (b.width <= EPS || b.depth <= EPS || b.height <= EPS)
				throw "Invalid fill box";
		}
		for (b in region) {
			for (n in [b.x, b.y, b.z, b.width, b.depth, b.height])
				if (!Math.isFinite(n))
					throw "Nonfinite fill box";
			if (b.width <= EPS || b.depth <= EPS || b.height <= EPS || b.z + b.height > fillHeightMm + EPS)
				throw "Invalid fill box or insufficient fill height";
		}
		var boxes = sorted(BoxVolumes.subtract(sorted(region), []));
		var heights:Array<Float> = [fillHeightMm];
		for (b in boxes) {
			heights.push(b.z);
			heights.push(b.z + b.height);
		}
		var critical = unique(heights), floor = critical[0];
		var pockets:Array<AirPocket> = [], prior:Array<Array<MoldBox>> = [];
		for (h in critical) {
			var air = above(boxes, h);
			for (component in BoxVolumes.groups(air)) {
				if (component.filter(b -> b.z + b.height >= fillHeightMm - EPS).length > 0)
					continue;
				var known = false;
				for (old in prior)
					for (a in component)
						for (b in old)
							if (BoxVolumes.intersection(a, b) != null)
								known = true;
				if (known)
					continue;
				var faces = ceilings(component, boxes, fillHeightMm, boundaries, sourceId);
				if (faces.length == 0)
					throw "Trapped air component has no ceiling";
				var highest = faces[0];
				for (face in faces)
					if (face.box.z > highest.box.z)
						highest = face;
				var ids:Array<String> = [];
				for (face in faces)
					for (id in face.sourceIds)
						if (ids.indexOf(id) < 0)
							ids.push(id);
				ids.sort(Reflect.compare);
				var hash = haxe.crypto.Sha256.encode(ToolingCodec.canonical(sorted(component))).substr(0, 16);
				pockets.push({
					id: ToolingIds.derive("air-", hash, ""),
					sealedAtMm: h,
					volumeMm3: BoxVolumes.volume(component),
					ceilingFaces: faces,
					sourceIds: ids,
					proposedVentPoint: {
						x: highest.box.x + highest.box.width / 2,
						y: highest.box.y + highest.box.depth / 2,
						z: highest.box.z
					}
				});
				prior.push(component);
			}
		}
		pockets.sort((a, b) -> Reflect.compare(a.id, b.id));
		var checks:Array<ToolingValidation> = [];
		if (pockets.length == 0)
			checks.push(check("process.fill.connected-air", [sourceId], ToolingCheckStatus.Pass,
				"Every air component reaches the open fill plane at every critical height.", [sourceFingerprint]));
		for (p in pockets)
			checks.push(check("process.fill.trap."
				+ p.id, p.sourceIds, ToolingCheckStatus.Fail,
				"Air pocket closes at Z="
				+ p.sealedAtMm
				+ " mm; limiting trapped volume="
				+ p.volumeMm3
				+ " mm3. Candidate vent requires routing or another setup.",
				[sourceFingerprint]));
		var broad = ceilings(boxes, boxes, fillHeightMm, boundaries, sourceId);
		if (airEscapeWidth == null)
			checks.push(check("process.fill.air-escape-width", [sourceId], ToolingCheckStatus.Pending,
				"No recipe air-escape-width limit declared; connected air alone does not qualify paste filling.", []));
		else {
			var offending:Array<String> = [];
			// Connect coplanar ceiling fragments before applying a width limit.
			// A connected face's bounding width is conservative for L-shaped faces;
			// artificial Boolean cell boundaries cannot manufacture an escape route.
			var planes = unique([for (face in broad) face.box.z]);
			for (z in planes) {
				var faces = broad.filter(f -> Math.abs(f.box.z - z) <= EPS);
				var footprints:Array<MoldBox> = [
					for (face in faces) {
						var b = BoxVolumes.copy(face.box);
						b.z = 0;
						b.height = 1;
						b;
					}
				];
				for (group in BoxVolumes.groups(footprints)) {
					var minX = group[0].x, minY = group[0].y, maxX = minX, maxY = minY;
					for (b in group) {
						minX = Math.min(minX, b.x);
						minY = Math.min(minY, b.y);
						maxX = Math.max(maxX, b.x + b.width);
						maxY = Math.max(maxY, b.y + b.depth);
					}
					if (Math.min(maxX - minX, maxY - minY) <= airEscapeWidth)
						continue;
					for (face in faces)
						for (b in group)
							if (face.box.x == b.x && face.box.y == b.y && face.box.width == b.width && face.box.depth == b.depth)
								for (id in face.sourceIds)
									if (offending.indexOf(id) < 0)
										offending.push(id);
				}
			}
			offending.sort(Reflect.compare);
			checks.push(check("process.fill.air-escape-width", offending.length > 0 ? offending : [sourceId],
				offending.length > 0 ? ToolingCheckStatus.Fail : ToolingCheckStatus.Pass,
				"Connected coplanar ceiling bounding widths compared with declared maximum air-escape width " + airEscapeWidth +
				" mm; no fluid/compaction simulation.",
				[processFingerprint]));
		}
		// Additional output levels never participate in pocket detection or IDs.
		var outputLevels = critical.copy();
		for (c in cores)
			for (b in c.boxes) {
				outputLevels.push(b.z);
				outputLevels.push(b.z + b.height);
			}
		if (extraLevels != null)
			for (h in extraLevels) {
				if (!Math.isFinite(h) || h < floor || h > fillHeightMm)
					throw "Invalid extra fill height";
				outputLevels.push(h);
			}
		cores = cores.copy();
		cores.sort((a, b) -> Reflect.compare(a.id, b.id));
		var seen = new Map<String, Bool>();
		for (c in cores) {
			ToolingIds.validate(c.id);
			if (seen.exists(c.id))
				throw "Duplicate fill core ID";
			seen.set(c.id, true);
		}
		var levels:Array<FillLevel> = [];
		for (h in unique(outputLevels))
			if (h >= floor - EPS && h <= fillHeightMm + EPS) {
				var v = submerged(boxes, h);
				levels.push({
					heightMm: h,
					mixVolumeMm3: v,
					mixMassKg: v * densityKgM3 * 1e-9,
					timeSeconds: pourRate == null ? null : v / pourRate,
					maximumHeadPa: Math.max(0, h - floor) * 0.001 * densityKgM3 * gravityMps2,
					cores: [
						for (c in cores) {
							var displacement = submerged(BoxVolumes.subtract(sorted(c.boxes), []), h);
							{id: c.id, submergedVolumeMm3: displacement, grossUpliftN: displacement * densityKgM3 * gravityMps2 * 1e-9};
						}
					]
				});
			}
		return {
			sourceFingerprint: sourceFingerprint,
			processFingerprint: processFingerprint,
			model: "exact-orthogonal-flat-free-surface",
			criticalHeightsMm: critical,
			totalMixVolumeMm3: BoxVolumes.volume(boxes),
			levels: levels,
			trappedPockets: pockets,
			validation: checks
		};
	}
}
