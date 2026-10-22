package materia.tooling.process;

import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingIds;
import materia.casting.BoxAlgebra;
import materia.tooling.casting.MoldPlan.MoldBox;

typedef FillBoundary = {var id:String; var box:MoldBox;}
typedef FillCore = {var id:String; var geometry:materia.casting.CastingRegion;}
typedef FillPanel = {var id:String; var bottomMm:Float;}
typedef FillCeiling = {var sourceIds:Array<String>; var box:MoldBox;}

/** Native references are reproducible from the report's bound source plan.
 * The component key selects the exact connected solid above the sealing plane;
 * a native region never supplies an invented array of boxes.
 */
enum FillPocketGeometry {
	Rectilinear(boxes:Array<MoldBox>, ceilings:Array<FillCeiling>);
	CadAirAbove(heightMm:Float, componentKey:String);
}

enum FillOpeningGeometry {
	RectilinearOpening(footprints:Array<MoldBox>);
	CadOpening(heightMm:Float, blockerIds:Array<String>);
}

class FillGeometryRecords {
	public static function pocket(value:FillPocketGeometry):Dynamic return switch value {
		case Rectilinear(boxes,ceilings): {kind:"rectilinear",boxes:boxes,ceilingFaces:ceilings};
		case CadAirAbove(height,key):
			if (!Math.isFinite(height)) throw "Invalid CAD air-region height";
			ToolingIds.validate(key);
			{kind:"cad-air-above",heightMm:height,componentKey:key};
	};
	public static function opening(value:FillOpeningGeometry):Dynamic return switch value {
		case RectilinearOpening(boxes): {kind:"rectilinear",footprints:boxes};
		case CadOpening(height,ids):
			if (!Math.isFinite(height)) throw "Invalid CAD opening height";
			var seen=new Map<String,Bool>();
			for (id in ids) {
				ToolingIds.validate(id);
				if (seen.exists(id)) throw "Duplicate CAD opening blocker ID";
				seen.set(id,true);
			}
			{kind:"cad-opening",heightMm:height,blockerIds:ids};
	};
}

typedef AirPocket = {
	var id:String;
	var sealedAtMm:Float;
	var volumeMm3:Float;
	var geometry:FillPocketGeometry;
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
	var panelHeads:Array<{id:String, headPa:Float}>;
	var cores:Array<{id:String, submergedVolumeMm3:Float, grossUpliftN:Float}>;
}

enum abstract FillModel(String) to String {
	var Orthogonal = "exact-orthogonal-flat-free-surface";
	var Brep = "exact-brep-flat-free-surface";
}

typedef FillReport = {
	var sourceFingerprint:String;
	var processFingerprint:String;
	var model:FillModel;
	var criticalHeightsMm:Array<Float>;
	var totalMixVolumeMm3:Float;
	var geometricResinVolumeMm3:Float;
	var openFillAreaMm2:Float;
	var openFillGeometry:FillOpeningGeometry;
	var levels:Array<FillLevel>;
	var trappedPockets:Array<AirPocket>;
	var validation:Array<ToolingValidation>;
}

typedef FillSourceSettings = {
	var sourceFingerprint:String;
	var processFingerprint:String;
	var cores:Array<FillCore>;
	var densityKgM3:Float;
	var gravityMps2:Float;
	var pourRateMm3PerSecond:Null<Float>;
	var maximumAirEscapeWidthMm:Null<Float>;
	var extraLevels:Array<Float>;
	var fillObstructions:Array<MoldBox>;
	var panels:Array<FillPanel>;
}

typedef FillAssessment = {var report:Null<FillReport>; var validation:Array<ToolingValidation>;}

/** Exact rectilinear event-plane analysis. Air connectivity is evaluated in the
 * right-hand limit at each critical height: boxes ending there disappear,
 * surviving boxes retain their limiting volume. No offset height or pose sampling.
 * Positive-area faces connect cells; edge/point contacts never vent a pocket.
 */
class FillAnalysis {
	public static function coreBoxes(core:FillCore):Array<MoldBox>
		return materia.tooling.casting.MoldCores.boxes(core.geometry,core.id);

	public static function sourceSupport(input:materia.tooling.casting.CastingToolingInput):Array<ToolingValidation>
		return materia.tooling.casting.CastingBoxSupport.check(input, "process.fill.geometry");
	public static function assessSource(document:materia.tooling.ToolingDocument, settings:FillSourceSettings):FillAssessment
		return assessRegions(materia.tooling.casting.CastingSourceRegions.derive(document), document.setup.fillHeightMm, settings);

	public static function assessRegions(geometry:materia.tooling.casting.CastingSourceRegions.CastingBoxRegions,
			fillHeightMm:Float, settings:FillSourceSettings):FillAssessment {
		var checks = materia.tooling.casting.CastingSourceRegions.checks(geometry, "process.fill.geometry");
		if (geometry.regions == null) return {report: null, validation: checks};
		var regions:{resin:Array<MoldBox>, boundaries:Array<FillBoundary>} = cast geometry.regions;
		var report = analyze(geometry.sourceId, settings.sourceFingerprint, settings.processFingerprint,
			regions.resin, regions.boundaries, settings.cores, fillHeightMm, settings.densityKgM3, settings.gravityMps2,
			settings.pourRateMm3PerSecond, settings.maximumAirEscapeWidthMm, settings.extraLevels, settings.fillObstructions, settings.panels);
		report.validation = report.validation.concat(checks);
		return {report: report, validation: report.validation.copy()};
	}

	static inline var EPS = 1e-7;

	public static function check(rule:String, ids:Array<String>, status:ToolingCheckStatus, message:String, evidence:Array<String>):ToolingValidation
		return {
			rule: rule,
			affectedIds: ids.copy(),
			status: status,
			severity: status == ToolingCheckStatus.Pass ? Info : status == ToolingCheckStatus.Pending ? Warning : Error,
			message: message,
			evidence: evidence.copy()
		};

	static function sorted(boxes:Array<MoldBox>):Array<MoldBox> {
		var out = [for (b in boxes) BoxAlgebra.copy(b)];
		out.sort((a, b) -> Reflect.compare(ToolingCodec.canonical(a), ToolingCodec.canonical(b)));
		return out;
	}

	static function above(boxes:Array<MoldBox>, h:Float):Array<MoldBox> {
		var out:Array<MoldBox> = [];
		for (b in boxes)
			if (b.z + b.height > h + EPS) {
				var c = BoxAlgebra.copy(b);
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
	static function ceilings(group:Array<MoldBox>, all:Array<MoldBox>, fill:Float, boundaries:Array<FillBoundary>, sourceId:String,
			?open:Array<MoldBox>):Array<FillCeiling> {
		var faces:Array<FillCeiling> = [];
		for (b in group) {
			var z = b.z + b.height;
			if (z >= fill - EPS && open == null)
				continue;
			var footprint = {
				x: b.x,
				y: b.y,
				z: 0.0,
				width: b.width,
				depth: b.depth,
				height: 1.0
			};
			var cover:Array<MoldBox> = z >= fill - EPS && open != null ? open.copy() : [];
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
			for (face in BoxAlgebra.subtract([footprint], cover)) {
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
			airEscapeWidth:Null<Float>, ?extraLevels:Array<Float>, ?fillObstructions:Array<MoldBox>, ?panels:Array<FillPanel>):FillReport {
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
			for (box in coreBoxes(core))
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
		var boxes = sorted(BoxAlgebra.subtract(sorted(region), []));
		var footprint:Array<MoldBox> = [
			for (box in boxes)
				if (box.z + box.height >= fillHeightMm - EPS) {
					x: box.x,
					y: box.y,
					z: 0.0,
					width: box.width,
					depth: box.depth,
					height: 1.0
				}
		];
		var open = BoxAlgebra.subtract(footprint, fillObstructions == null ? [] : fillObstructions);
		var openArea = BoxAlgebra.volume(open);
		if (openArea <= EPS)
			throw "Fill face has zero open area";
		var heights:Array<Float> = [fillHeightMm];
		for (b in boxes) {
			heights.push(b.z);
			heights.push(b.z + b.height);
		}
		var critical = unique(heights), floor = critical[0];
		var pockets:Array<AirPocket> = [], prior:Array<Array<MoldBox>> = [];
		for (h in critical) {
			var air = above(boxes, h);
			for (component in BoxAlgebra.groups(air)) {
				var vented = false;
				for (box in component)
					if (box.z + box.height >= fillHeightMm - EPS)
						for (opening in open)
							if (Math.min(box.x + box.width, opening.x + opening.width) - Math.max(box.x, opening.x) > EPS
								&& Math.min(box.y + box.depth, opening.y + opening.depth) - Math.max(box.y, opening.y) > EPS)
								vented = true;
				if (vented)
					continue;
				var known = false;
				for (old in prior)
					for (a in component)
						for (b in old)
							if (BoxAlgebra.intersection(a, b) != null)
								known = true;
				if (known)
					continue;
				var faces = ceilings(component, boxes, fillHeightMm, boundaries, sourceId, open);
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
					volumeMm3: BoxAlgebra.volume(component),
					geometry:Rectilinear(sorted(component),faces),
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
		var broad = ceilings(boxes, boxes, fillHeightMm, boundaries, sourceId, open);
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
						var b = BoxAlgebra.copy(face.box);
						b.z = 0;
						b.height = 1;
						b;
					}
				];
				for (group in BoxAlgebra.groups(footprints)) {
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
			for (b in coreBoxes(c)) {
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
		var trapped = BoxAlgebra.subtract([for (group in prior) for (box in group) box], []);
		var panelRecords = panels == null ? [] : panels.copy();
		panelRecords.sort((a, b) -> Reflect.compare(a.id, b.id));
		var levels:Array<FillLevel> = [];
		for (h in unique(outputLevels))
			if (h >= floor - EPS && h <= fillHeightMm + EPS) {
				var v = submerged(boxes, h) - submerged(trapped, h);
				levels.push({
					heightMm: h,
					mixVolumeMm3: v,
					mixMassKg: v * densityKgM3 * 1e-9,
					timeSeconds: pourRate == null ? null : v / pourRate,
					maximumHeadPa: Math.max(0, h - floor) * 0.001 * densityKgM3 * gravityMps2,
					panelHeads: [
						for (panel in panelRecords)
							{
								id: panel.id,
								headPa: Math.max(0, h - panel.bottomMm) * 0.001 * densityKgM3 * gravityMps2
							}
					],
					cores: [
						for (c in cores) {
							var displacement = submerged(BoxAlgebra.subtract(sorted(coreBoxes(c)), []), h);
							{id: c.id, submergedVolumeMm3: displacement, grossUpliftN: displacement * densityKgM3 * gravityMps2 * 1e-9};
						}
					]
				});
			}
		return {
			sourceFingerprint: sourceFingerprint,
			processFingerprint: processFingerprint,
			model: Orthogonal,
			criticalHeightsMm: critical,
			totalMixVolumeMm3: BoxAlgebra.volume(boxes) - BoxAlgebra.volume(trapped),
			geometricResinVolumeMm3: BoxAlgebra.volume(boxes),
			openFillAreaMm2: openArea,
			openFillGeometry:RectilinearOpening(sorted(open)),
			levels: levels,
			trappedPockets: pockets,
			validation: checks
		};
	}
}
