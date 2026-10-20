package materia.tooling.process;

import materia.tooling.ToolingCodec;
import materia.tooling.ToolingIds;
import materia.tooling.casting.BoxVolumes;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.process.FillAnalysis.FillBoundary;
import materia.tooling.process.OrthogonalSections.SectionAxis;

typedef ThicknessPoint = {var x:Float; var y:Float; var z:Float;}
typedef ThicknessFace = {var axis:SectionAxis; var planeMm:Float; var region:MoldBox; var sourceIds:Array<String>;}
typedef JunctionSearchSettings = {var diameterToleranceMm:Float; var maximumCells:Int;}

typedef JunctionThicknessReport = {
	var version:Int;
	var sourceFingerprint:String;
	var settings:JunctionSearchSettings;
	var lowerDiameterMm:Float;
	var upperDiameterMm:Float;
	var witness:ThicknessPoint;
	var witnessSourceIds:Array<String>;
	var converged:Bool;
	var evaluatedCells:Int;
	var faces:Array<ThicknessFace>;
}

private typedef SearchCell = {var box:MoldBox; var radius:Float; var upper:Float; var point:ThicknessPoint;}

/** Largest Euclidean inscribed sphere in an exact orthogonal union. Exposed
 * rectangles are exact. Branch-and-bound uses the 1-Lipschitz distance field;
 * every live cell retains an upper bound, including when the budget is exhausted.
 * Numerical diameter tolerance is not a recipe/process acceptance threshold.
 */
class JunctionThickness {
	static inline var EPS = 1e-7;

	static function lo(b:MoldBox, a:Int):Float
		return a == 0 ? b.x : a == 1 ? b.y : b.z;

	static function size(b:MoldBox, a:Int):Float
		return a == 0 ? b.width : a == 1 ? b.depth : b.height;

	static function point(p:ThicknessPoint, a:Int):Float
		return a == 0 ? p.x : a == 1 ? p.y : p.z;

	public static function boundary(sourceId:String, boxes:Array<MoldBox>, owners:Array<FillBoundary>):Array<ThicknessFace> {
		var faces:Array<ThicknessFace> = [];
		for (b in boxes)
			for (axis in 0...3)
				for (side in 0...2) {
					var plane = lo(b, axis) + (side == 0 ? 0 : size(b, axis)),
						u = (axis + 1) % 3,
						v = (axis + 2) % 3;
					var footprint:MoldBox = {
						x: lo(b, u),
						y: lo(b, v),
						z: 0,
						width: size(b, u),
						depth: size(b, v),
						height: 1
					};
					var cover:Array<MoldBox> = [];
					for (c in boxes)
						if (side == 0 ? lo(c, axis) < plane - EPS
							&& lo(c, axis) + size(c, axis) >= plane - EPS : lo(c, axis) <= plane + EPS && lo(c, axis) + size(c, axis) > plane + EPS)
							cover.push({
								x: lo(c, u),
								y: lo(c, v),
								z: 0,
								width: size(c, u),
								depth: size(c, v),
								height: 1
							});
					for (f in BoxVolumes.subtract([footprint], cover)) {
						// Split by owner footprints so no face receives an adjacent feature's ID.
						var pieces = [{box: f, ids: new Array<String>()}];
						for (owner in owners) {
							var ownerPlane = lo(owner.box, axis) + (side == 0 ? size(owner.box, axis) : 0);
							if (Math.abs(ownerPlane - plane) > EPS)
								continue;
							var mask:MoldBox = {
								x: lo(owner.box, u),
								y: lo(owner.box, v),
								z: 0,
								width: size(owner.box, u),
								depth: size(owner.box, v),
								height: 1
							};
							var next = [];
							for (piece in pieces) {
								for (rest in BoxVolumes.difference(piece.box, mask))
									next.push({box: rest, ids: piece.ids.copy()});
								var hit = BoxVolumes.intersection(piece.box, mask);
								if (hit != null) {
									var ids = piece.ids.copy();
									if (ids.indexOf(owner.id) < 0)
										ids.push(owner.id);
									next.push({box: hit, ids: ids});
								}
							}
							pieces = next;
						}
						for (piece in pieces) {
							var origin = [0.0, 0.0, 0.0],
								extent = [0.0, 0.0, 0.0];
							origin[axis] = plane;
							origin[u] = piece.box.x;
							origin[v] = piece.box.y;
							extent[u] = piece.box.width;
							extent[v] = piece.box.depth;
							if (piece.ids.length == 0)
								piece.ids.push(sourceId);
							piece.ids.sort(Reflect.compare);
							faces.push({
								axis: axis == 0 ? X : axis == 1 ? Y : Z,
								planeMm: plane,
								sourceIds: piece.ids,
								region: {
									x: origin[0],
									y: origin[1],
									z: origin[2],
									width: extent[0],
									depth: extent[1],
									height: extent[2]
								}
							});
						}
					}
				}
		faces.sort((a, b) -> Reflect.compare(ToolingCodec.canonical(a), ToolingCodec.canonical(b)));
		return faces;
	}

	public static function distance(p:ThicknessPoint, f:ThicknessFace):Float {
		var squared = 0.0;
		for (a in 0...3) {
			var x = point(p, a),
				low = lo(f.region, a),
				high = low + size(f.region, a);
			var d = x < low ? low - x : x > high ? x - high : 0;
			squared += d * d;
		}
		return Math.sqrt(squared);
	}

	static function cell(b:MoldBox, faces:Array<ThicknessFace>, cap:Float):SearchCell {
		var p:ThicknessPoint = {x: b.x + b.width / 2, y: b.y + b.depth / 2, z: b.z + b.height / 2};
		var radius = Math.POSITIVE_INFINITY, nearest = faces[0];
		for (f in faces) {
			var d = distance(p, f);
			if (d < radius) {
				radius = d;
				nearest = f;
			}
		}
		var diagonal = Math.sqrt(b.width * b.width + b.depth * b.depth + b.height * b.height) / 2;
		// Distance to a fixed convex rectangle is convex: its maximum over a box
		// occurs at a vertex. This tightens, but never replaces, the Lipschitz bound.
		var faceUpper = 0.0;
		for (x in 0...2)
			for (y in 0...2)
				for (z in 0...2)
					faceUpper = Math.max(faceUpper, distance({x: b.x + x * b.width, y: b.y + y * b.depth, z: b.z + z * b.height}, nearest));
		return {
			box: b,
			point: p,
			radius: radius,
			upper: Math.max(radius, Math.min(cap, Math.min(radius + diagonal, faceUpper)) + 1e-9)
		};
	}

	static function better(a:SearchCell, b:SearchCell):Bool {
		if (a.upper != b.upper)
			return a.upper > b.upper;
		for (axis in 0...3) {
			var order = Reflect.compare(lo(a.box, axis), lo(b.box, axis));
			if (order != 0)
				return order < 0;
		}
		return ToolingCodec.canonical(a.box) < ToolingCodec.canonical(b.box);
	}

	static function push(heap:Array<SearchCell>, c:SearchCell):Void {
		heap.push(c);
		var i = heap.length - 1;
		while (i > 0) {
			var parent = Std.int((i - 1) / 2);
			if (!better(heap[i], heap[parent]))
				break;
			var temp = heap[i];
			heap[i] = heap[parent];
			heap[parent] = temp;
			i = parent;
		}
	}

	static function pop(heap:Array<SearchCell>):SearchCell {
		var top = heap[0], last = heap.pop();
		if (heap.length == 0)
			return top;
		heap[0] = last;
		var i = 0;
		while (2 * i + 1 < heap.length) {
			var child = 2 * i + 1;
			if (child + 1 < heap.length && better(heap[child + 1], heap[child]))
				child++;
			if (!better(heap[child], heap[i]))
				break;
			var temp = heap[i];
			heap[i] = heap[child];
			heap[child] = temp;
			i = child;
		}
		return top;
	}

	public static function analyze(sourceId:String, fingerprint:String, region:Array<MoldBox>, owners:Array<FillBoundary>, ?settings:JunctionSearchSettings,
			?heatFaces:Array<ThicknessFace>):JunctionThicknessReport {
		ToolingIds.validate(sourceId);
		if (settings == null)
			settings = {diameterToleranceMm: 0.5, maximumCells: 20000};
		if (!Math.isFinite(settings.diameterToleranceMm)
			|| settings.diameterToleranceMm <= 0
			|| settings.maximumCells < 1
			|| settings.maximumCells > 1000000)
			throw "Invalid junction search settings";
		if (region.length == 0 || region.length > 2048 || owners.length > 512)
			throw "Invalid junction geometry count";
		for (b in region.concat([for (o in owners) o.box])) {
			for (n in [b.x, b.y, b.z, b.width, b.depth, b.height])
				if (!Math.isFinite(n))
					throw "Invalid junction box";
			if (b.width <= EPS || b.depth <= EPS || b.height <= EPS)
				throw "Invalid junction box";
		}
		var ordered = [for (b in region) BoxVolumes.copy(b)];
		ordered.sort((a, b) -> Reflect.compare(ToolingCodec.canonical(a), ToolingCodec.canonical(b)));
		var sortedOwners = owners.copy();
		sortedOwners.sort((a, b) -> Reflect.compare(a.id, b.id));
		for (o in sortedOwners)
			ToolingIds.validate(o.id);
		var boxes = BoxVolumes.subtract(ordered, []),
			faces = heatFaces == null ? boundary(sourceId, boxes, sortedOwners) : heatFaces;
		if (faces.length == 0)
			throw "Thermal thickness needs at least one heat-removing face";
		if (boxes.length > settings.maximumCells)
			throw "Junction search budget below initial cell count";
		var lower = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY],
			upper = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
		for (b in boxes)
			for (a in 0...3) {
				lower[a] = Math.min(lower[a], lo(b, a));
				upper[a] = Math.max(upper[a], lo(b, a) + size(b, a));
			}
		var cap = Math.min(upper[0] - lower[0], Math.min(upper[1] - lower[1], upper[2] - lower[2])) / 2;
		if (heatFaces != null)
			cap = Math.sqrt(Math.pow(upper[0] - lower[0], 2) + Math.pow(upper[1] - lower[1], 2) + Math.pow(upper[2] - lower[2], 2))
				+ distance({x: lower[0], y: lower[1], z: lower[2]}, faces[0]);
		var heap:Array<SearchCell> = [], best:SearchCell = null, count = 0;
		for (b in boxes) {
			var c = cell(b, faces, cap);
			push(heap, c);
			count++;
			if (best == null || c.radius > best.radius)
				best = c;
			if (heatFaces != null)
				for (x in [b.x, b.x + b.width])
					for (y in [b.y, b.y + b.depth])
						for (z in [b.z, b.z + b.height]) {
							var point = {x: x, y: y, z: z},
								radius = Math.POSITIVE_INFINITY;
							for (face in faces)
								radius = Math.min(radius, distance(point, face));
							if (radius > best.radius)
								best = {
									box: b,
									radius: radius,
									upper: c.upper,
									point: point
								};
						}
		}
		while (heap.length > 0 && 2 * (heap[0].upper - best.radius) > settings.diameterToleranceMm && count + 2 <= settings.maximumCells) {
			var current = pop(heap);
			if (current.upper <= best.radius)
				continue;
			var axis = 0;
			for (a in 1...3)
				if (size(current.box, a) > size(current.box, axis))
					axis = a;
			var a = BoxVolumes.copy(current.box),
				b = BoxVolumes.copy(current.box);
			if (axis == 0) {
				a.width /= 2;
				b.x += a.width;
				b.width = a.width;
			} else if (axis == 1) {
				a.depth /= 2;
				b.y += a.depth;
				b.depth = a.depth;
			} else {
				a.height /= 2;
				b.z += a.height;
				b.height = a.height;
			}
			for (part in [a, b]) {
				var c = cell(part, faces, cap);
				count++;
				if (c.radius > best.radius)
					best = c;
				if (c.upper > best.radius)
					push(heap, c);
			}
		}
		var maxRadius = heap.length == 0 ? best.radius : Math.max(best.radius, heap[0].upper),
			ids:Array<String> = [];
		for (f in faces)
			if (distance(best.point, f) <= best.radius + settings.diameterToleranceMm / 2)
				for (id in f.sourceIds)
					if (ids.indexOf(id) < 0)
						ids.push(id);
		ids.sort(Reflect.compare);
		return {
			version: 1,
			sourceFingerprint: fingerprint,
			settings: {diameterToleranceMm: settings.diameterToleranceMm, maximumCells: settings.maximumCells},
			lowerDiameterMm: 2 * best.radius,
			upperDiameterMm: 2 * maxRadius,
			witness: best.point,
			witnessSourceIds: ids,
			converged: 2 * (maxRadius - best.radius) <= settings.diameterToleranceMm,
			evaluatedCells: count,
			faces: faces
		};
	}
}
