package machinekit.component;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Authored convex pieces in the component's local CAD frame, preserving routed gaps. */
class CollisionHullFacet implements ComponentFacet {
	public final hulls:Array<Array<Float>>;
	public function new(hulls:Array<Array<Float>>) {
		this.hulls = [for (hull in hulls) hull.copy()];
	}
	public static function fromBoxes(boxes:Array<Array<Float>>):CollisionHullFacet {
		var hulls:Array<Array<Float>> = [];
		for (box in boxes) {
			if (box.length != 6) throw "Collision box needs six bounds";
			for (axis in 0...3) if (!(box[axis] < box[axis + 3])) throw "Collision box needs positive volume";
			var hull:Array<Float> = [];
			for (x in [box[0], box[3]]) for (y in [box[1], box[4]]) for (z in [box[2], box[5]]) {
				hull.push(x); hull.push(y); hull.push(z);
			}
			hulls.push(hull);
		}
		return new CollisionHullFacet(hulls);
	}
	/** Rectangular collision proxy with circumscribed square cutouts around round openings. */
	public static function aroundCircularOpenings(width:Float, height:Float, lower:Float, upper:Float,
			centers:Array<Vector>, radius:Float):CollisionHullFacet {
		if (!Math.isFinite(width) || !Math.isFinite(height) || width <= 0 || height <= 0 ||
			!Math.isFinite(lower) || !Math.isFinite(upper) || lower >= upper ||
			!Math.isFinite(radius) || radius <= 0 || centers == null || centers.length == 0)
			throw "Collision plate needs positive dimensions, depth, and round openings";
		var minX = -width/2, maxX = width/2, minY = -height/2, maxY = height/2;
		var xs = [minX,maxX], ys = [minY,maxY];
		for (center in centers) {
			if (center == null || !Math.isFinite(center.x) || !Math.isFinite(center.y) ||
				center.x-radius < minX || center.x+radius > maxX ||
				center.y-radius < minY || center.y+radius > maxY)
				throw "Collision plate opening does not fit inside its authored bounds";
			xs.push(center.x-radius); xs.push(center.x+radius);
			ys.push(center.y-radius); ys.push(center.y+radius);
		}
		xs.sort(compareCoordinate); ys.sort(compareCoordinate);
		xs = uniqueCoordinates(xs); ys = uniqueCoordinates(ys);
		var boxes:Array<Array<Float>> = [];
		for (xi in 0...(xs.length-1)) for (yi in 0...(ys.length-1)) {
			var x0 = xs[xi], x1 = xs[xi+1], y0 = ys[yi], y1 = ys[yi+1], cutout = false;
			for (center in centers) if (x0 >= center.x-radius-1e-9 && x1 <= center.x+radius+1e-9 &&
				y0 >= center.y-radius-1e-9 && y1 <= center.y+radius+1e-9) {
				cutout = true; break;
			}
			if (!cutout) boxes.push([x0,y0,lower,x1,y1,upper]);
		}
		if (boxes.length == 0) throw "Collision plate openings leave no authored material";
		// Grid lines locate the holes, not physical seams. Coalesce adjacent rectangles without
		// changing their union so a flush seat does not create a contact manifold per grid cell.
		var merged = true;
		while (merged) {
			merged = false;
			for (i in 0...boxes.length) {
				for (j in (i+1)...boxes.length) {
					var a = boxes[i], b = boxes[j];
					for (axis in 0...2) {
						var other = 1-axis;
						if (a[other] != b[other] || a[other+3] != b[other+3] ||
							a[2] != b[2] || a[5] != b[5]) continue;
						if (a[axis+3] != b[axis] && b[axis+3] != a[axis]) continue;
						a[axis] = Math.min(a[axis], b[axis]);
						a[axis+3] = Math.max(a[axis+3], b[axis+3]);
						boxes.splice(j, 1); merged = true; break;
					}
					if (merged) break;
				}
				if (merged) break;
			}
		}
		return fromBoxes(boxes);
	}
	static function compareCoordinate(a:Float, b:Float):Int return a < b ? -1 : a > b ? 1 : 0;
	static function uniqueCoordinates(values:Array<Float>):Array<Float> {
		var unique:Array<Float> = [];
		for (value in values) if (unique.length == 0 || Math.abs(value-unique[unique.length-1]) > 1e-9) unique.push(value);
		return unique;
	}
	/** A bounded convex profile, in CAD millimetres; no union hull fills a neighbouring gap. */
	public static function prism(points:Array<Vector>, lower:Float, upper:Float):Array<Float> {
		if (points.length < 3 || points.length > 32 || !Math.isFinite(lower) || !Math.isFinite(upper) || lower >= upper)
			throw "Collision prism needs a bounded convex profile and positive depth";
		var hull:Array<Float> = [];
		for (z in [lower,upper]) for (point in points) { hull.push(point.x); hull.push(point.y); hull.push(z); }
		return hull;
	}
	/** Inscribed cylinder proxy, retaining the exact axial limits and cardinal radii. */
	public static function cylinder(radius:Float, lower:Float, upper:Float):Array<Float> {
		if (!Math.isFinite(radius) || radius <= 0) throw "Collision cylinder needs a positive radius";
		return prism([for (i in 0...32) new Vector(radius*Math.cos(i*2*Math.PI/32),radius*Math.sin(i*2*Math.PI/32))],lower,upper);
	}
	/** Extrude an authored constant section from its actual top cap, retaining every hole. */
	public static function fromExtrusion(part:Part, lower:Float, upper:Float,
			linearDeflection:Float = 0.1, angularDeflection:Float = 0.5):CollisionHullFacet {
		if (!Math.isFinite(linearDeflection) || linearDeflection <= 0 ||
			!Math.isFinite(angularDeflection) || angularDeflection <= 0)
			throw "Collision extrusion needs positive finite tessellation deflections";
		var mesh = part.shape.tessellate(linearDeflection,angularDeflection), hulls:Array<Array<Float>> = [];
		var vertices:Array<Vector> = [], vertexIds = new Map<String, Int>();
		var regions:Array<Array<Int>> = [];
		for (i in 0...Std.int(mesh.indices.length/12)) {
			var triangle:Array<Vector> = [];
			for (j in 0...3) {
				var offset = mesh.indices.getInt32((i*3+j)*4)*24;
				triangle.push(new Vector(mesh.vertices.getDouble(offset),mesh.vertices.getDouble(offset+8),mesh.vertices.getDouble(offset+16)));
			}
			if (Math.abs(triangle[0].z-upper) > 1e-6 || Math.abs(triangle[1].z-upper) > 1e-6 ||
				Math.abs(triangle[2].z-upper) > 1e-6) continue;
			var ids = [for (point in triangle) collisionVertex(point, vertices, vertexIds)];
			var region = convexHullIndices(ids, vertices);
			if (region.length == 3 && polygonArea(region, vertices) > 1e-10) regions.push(region);
		}
		if (regions.length == 0) throw "Collision extrusion has no triangles on its authored top cap";
		regions = mergeConvexRegions(regions, vertices);
		for (region in regions) {
			if (region.length <= 32) {
				hulls.push(prism([for (index in region) vertices[index]],lower,upper));
				continue;
			}
			// Split unusually detailed convex outlines into exact fan wedges; each remains
			// a convex prism and stays within the native 64-vertex hull limit.
			var start = 1;
			while (start < region.length - 1) {
				var end = start + 30;
				if (end >= region.length) end = region.length - 1;
				var wedge = [region[0]];
				for (index in start...(end + 1)) wedge.push(region[index]);
				hulls.push(prism([for (index in wedge) vertices[index]],lower,upper));
				start = end;
			}
		}
		return new CollisionHullFacet(hulls);
	}

	static function collisionVertex(point:Vector, vertices:Array<Vector>, ids:Map<String, Int>):Int {
		var key = Std.string(Math.round(point.x * 1e6)) + ":" + Std.string(Math.round(point.y * 1e6));
		var existing = ids.get(key);
		if (existing != null) return existing;
		var index = vertices.length;
		vertices.push(new Vector(point.x,point.y));
		ids.set(key,index);
		return index;
	}

	/** Greedily merge adjacent cap triangles when their union remains exactly convex. */
	static function mergeConvexRegions(regions:Array<Array<Int>>, vertices:Array<Vector>):Array<Array<Int>> {
		while (true) {
			var sharedEdges = new Map<String, Int>(), merged = false;
			for (regionIndex in 0...regions.length) {
				var region = regions[regionIndex];
				for (edgeIndex in 0...region.length) {
					var first = region[edgeIndex], second = region[(edgeIndex + 1) % region.length];
					var key = first < second ? '$first:$second' : '$second:$first';
					var previous = sharedEdges.get(key);
					if (previous == null) {
						sharedEdges.set(key,regionIndex);
						continue;
					}
					if (previous == regionIndex) continue;
					var union = regions[previous].copy();
					for (index in region) if (union.indexOf(index) < 0) union.push(index);
					var hull = convexHullIndices(union,vertices);
					var combinedArea = polygonArea(regions[previous],vertices) + polygonArea(region,vertices);
					var hullArea = polygonArea(hull,vertices);
					if (hull.length >= 3 && Math.abs(hullArea-combinedArea) <= (1+combinedArea)*1e-9) {
						regions[previous] = hull;
						regions.splice(regionIndex,1);
						merged = true;
						break;
					}
				}
				if (merged) break;
			}
			if (!merged) break;
		}
		return regions;
	}

	/** Stable monotone-chain hull of unique cap vertices, counter-clockwise. */
	static function convexHullIndices(indices:Array<Int>, vertices:Array<Vector>):Array<Int> {
		var sorted:Array<Int> = [];
		for (index in indices) if (sorted.indexOf(index) < 0) sorted.push(index);
		sorted.sort(function(left, right) {
			var a = vertices[left], b = vertices[right];
			if (a.x < b.x) return -1;
			if (a.x > b.x) return 1;
			if (a.y < b.y) return -1;
			if (a.y > b.y) return 1;
			return 0;
		});
		if (sorted.length <= 2) return sorted;
		var lower:Array<Int> = [];
		for (index in sorted) {
			while (lower.length >= 2 && cross(vertices[lower[lower.length-2]],vertices[lower[lower.length-1]],vertices[index]) <= 1e-10)
				lower.pop();
			lower.push(index);
		}
		var upper:Array<Int> = [];
		var cursor = sorted.length - 1;
		while (cursor >= 0) {
			var index = sorted[cursor];
			while (upper.length >= 2 && cross(vertices[upper[upper.length-2]],vertices[upper[upper.length-1]],vertices[index]) <= 1e-10)
				upper.pop();
			upper.push(index);
			cursor--;
		}
		lower.pop();
		upper.pop();
		return lower.concat(upper);
	}

	static function cross(origin:Vector, first:Vector, second:Vector):Float
		return (first.x-origin.x)*(second.y-origin.y)-(first.y-origin.y)*(second.x-origin.x);

	static function polygonArea(region:Array<Int>, vertices:Array<Vector>):Float {
		if (region.length < 3) return 0;
		var origin = vertices[region[0]], twice = 0.0;
		for (index in 1...(region.length-1))
			twice += cross(origin,vertices[region[index]],vertices[region[index+1]]);
		return Math.abs(twice)*0.5;
	}
	public function check(component:MachineComponent):Void {
		if (hulls.length == 0 || !component.hasGeometry()) throw "Collision pieces need component geometry";
		for (hull in hulls) {
			if (hull.length < 12 || hull.length > 192 || hull.length % 3 != 0) throw "Invalid collision hull";
			for (coordinate in hull) if (!Math.isFinite(coordinate)) throw "Collision hull must be finite";
		}
	}
	public function describe():String return '${hulls.length} convex collision pieces';
	public static function of(component:MachineComponent):Null<CollisionHullFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, CollisionHullFacet)) return cast facet;
		return null;
	}
}
