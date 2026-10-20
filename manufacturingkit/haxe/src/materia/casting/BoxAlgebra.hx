package materia.casting;

import materia.casting.Box3;
import materia.casting.CastingFeature;

/** Exact orthogonal box set operations. Outputs are disjoint finished volumes, not meshes. */
class BoxAlgebra {
    // Millimetres: suppress arithmetic residue at coincident cut planes, not design clearances.
    static inline final PLANE_EPSILON = 1e-9;
	public static function intersection(a:Box3, b:Box3):Null<Box3> {
		var x = Math.max(a.x, b.x),
			y = Math.max(a.y, b.y),
			z = Math.max(a.z, b.z);
		var w = Math.min(a.x + a.width, b.x + b.width) - x,
			d = Math.min(a.y + a.depth, b.y + b.depth) - y,
			h = Math.min(a.z + a.height, b.z + b.height) - z;
		return w > PLANE_EPSILON && d > PLANE_EPSILON && h > PLANE_EPSILON ? {
			x: x,
			y: y,
			z: z,
			width: w,
			depth: d,
			height: h
		} : null;
	}

	public static function difference(a:Box3, b:Box3):Array<Box3> {
		var i = intersection(a, b);
		if (i == null)
			return [copy(a)];
		var out:Array<Box3> = [];
		function piece(x:Float, y:Float, z:Float, w:Float, d:Float, h:Float):Void {
			if (w > PLANE_EPSILON && d > PLANE_EPSILON && h > PLANE_EPSILON)
				out.push({
					x: x,
					y: y,
					z: z,
					width: w,
					depth: d,
					height: h
				});
		}
		piece(a.x, a.y, a.z, i.x - a.x, a.depth, a.height);
		piece(i.x + i.width, a.y, a.z, a.x + a.width - i.x - i.width, a.depth, a.height);
		piece(i.x, a.y, a.z, i.width, i.y - a.y, a.height);
		piece(i.x, i.y + i.depth, a.z, i.width, a.y + a.depth - i.y - i.depth, a.height);
		piece(i.x, i.y, a.z, i.width, i.depth, i.z - a.z);
		piece(i.x, i.y, i.z + i.height, i.width, i.depth, a.z + a.height - i.z - i.height);
		return out;
	}

	public static function subtract(add:Array<Box3>, cuts:Array<Box3>):Array<Box3> {
		var out:Array<Box3> = [];
		for (box in add) {
			var next = [copy(box)];
			for (c in out.concat(cuts)) {
				var pieces:Array<Box3> = [];
				for (p in next)
					for (q in difference(p, c))
						pieces.push(q);
				next = pieces;
			}
			for (p in next)
				out.push(p);
			if (out.length > 2048)
				throw "Too many orthogonal volume pieces";
		}
		return out;
	}

	public static function groups(boxes:Array<Box3>):Array<Array<Box3>> {
		var remaining = boxes.copy(), out:Array<Array<Box3>> = [];
		remaining.sort((a, b) -> a.z != b.z ? Reflect.compare(a.z, b.z) : a.y != b.y ? Reflect.compare(a.y, b.y) : Reflect.compare(a.x, b.x));
		function connected(a:Box3, b:Box3):Bool {
			var x = Math.min(a.x + a.width, b.x + b.width) - Math.max(a.x, b.x),
				y = Math.min(a.y + a.depth, b.y + b.depth) - Math.max(a.y, b.y),
				z = Math.min(a.z + a.height, b.z + b.height) - Math.max(a.z, b.z);
			return x >= -1e-7 && y >= -1e-7 && z >= -1e-7 && ((x > 1e-7 ? 1 : 0) + (y > 1e-7 ? 1 : 0) + (z > 1e-7 ? 1 : 0) >= 2);
		}
		while (remaining.length > 0) {
			var group = [remaining.shift()], changed = true;
			while (changed) {
				changed = false;
				var i = remaining.length - 1;
				while (i >= 0) {
					var match = false;
					for (a in group)
						if (connected(a, remaining[i]))
							match = true;
					if (match) {
						group.push(remaining[i]);
						remaining.splice(i, 1);
						changed = true;
					}
					i--;
				}
			}
			out.push(group);
		}
		return out;
	}

	public static function copy(b:Box3):Box3
		return {
			x: b.x,
			y: b.y,
			z: b.z,
			width: b.width,
			depth: b.depth,
			height: b.height
		};

	public static function volume(boxes:Array<Box3>):Float {
		var v = 0.0;
		for (b in boxes)
			v += b.width * b.depth * b.height;
		return v;
	}

	/** Remove decomposition seams using exact coordinate equality. */
	public static function coalesce(input:Array<Box3>):Array<Box3> {
		var boxes = [for (b in input) copy(b)];
		boxes.sort(compare);
		var changed = true;
		while (changed) {
			changed = false;
			for (i in 0...boxes.length) {
				for (j in i + 1...boxes.length) {
					var a = limits(boxes[i]), b = limits(boxes[j]);
					for (axis in 0...3) {
						var same = true;
						for (k in 0...3)
							if (k != axis && (a[k] != b[k] || a[k + 3] != b[k + 3]))
								same = false;
						if (!same || (a[axis + 3] != b[axis] && b[axis + 3] != a[axis]))
							continue;
						a[axis] = Math.min(a[axis], b[axis]);
						a[axis + 3] = Math.max(a[axis + 3], b[axis + 3]);
						boxes[i] = fromLimits(a);
						boxes.splice(j, 1);
						changed = true;
						break;
					}
					if (changed)
						break;
				}
				if (changed)
					break;
			}
		}
		boxes.sort(compare);
		return boxes;
	}

	public static function limits(b:Box3):Array<Float>
		return [b.x, b.y, b.z, b.x + b.width, b.y + b.depth, b.z + b.height];

	public static function fromLimits(b:Array<Float>):Box3 {
		if (b.length != 6)
			throw "Box needs six bounds";
		for (v in b)
			if (!Math.isFinite(v))
				throw "Nonfinite box";
		if (b[3] <= b[0] || b[4] <= b[1] || b[5] <= b[2])
			throw "Nonpositive box";
		return {
			x: b[0],
			y: b[1],
			z: b[2],
			width: b[3] - b[0],
			depth: b[4] - b[1],
			height: b[5] - b[2]
		};
	}

	public static function compare(a:Box3, b:Box3):Int {
		var aa = limits(a), bb = limits(b);
		for (i in 0...6)
			if (aa[i] != bb[i])
				return Reflect.compare(aa[i], bb[i]);
		return 0;
	}

	public static function remove(boxes:Array<Box3>, cut:Box3):Array<Box3>
		return subtract(boxes, [cut]);

	public static function material(body:Array<CastingPrism>, features:Array<CastingFeature>):Array<Box3> {
		var cavities = CastingCavities.derive(body, features);
		var region = [
			for (p in body) for (b in CastingCavities.boxes({featureId: p.id, parentId: p.id, pieces: [p]})) b
		];
		return coalesce(subtract(region, [for (c in cavities) for (b in CastingCavities.boxes(c)) b]));
	}

	/** Stable fragment IDs depend on exact geometry, never traversal/array order. */
	public static function retain(features:Array<CastingFeature>, supports:Array<Box3>):Array<CastingFeature> {
		var result:Array<CastingFeature> = [];
		for (f in features) {
			var cells = coalesce(subtract([CastingBounds.box(f.geometry)], supports));
			if (cells.length == 1 && compare(cells[0], CastingBounds.box(f.geometry)) == 0) {
				result.push(f);
				continue;
			}
			for (b in cells) {
				var fragment:CastingFeature = {
					id: f.id,
					kind: f.kind,
					geometry: f.geometry,
					openings: [for (o in f.openings) {kind: o.kind, target: o.target}],
					draft: {degrees: f.draft.degrees, axis: {x: f.draft.axis.x, y: f.draft.axis.y, z: f.draft.axis.z}},
					cornerRadius: f.cornerRadius,
					owner: f.owner,
					adjustable: f.adjustable.copy(),
					derivedFrom: f.derivedFrom
				};
				fragment.derivedFrom = f.derivedFrom == null ? f.id : f.derivedFrom;
				fragment.id = fragment.derivedFrom + "-fragment-" + haxe.crypto.Sha256.encode(haxe.Json.stringify(limits(b))).substr(0, 20);
				fragment.geometry = {
					kind: Box,
					width: b.width,
					depth: b.depth,
					height: b.height,
					pose: materia.assembly.AssemblyFrames.translation(b.x + b.width / 2, b.y + b.depth / 2, b.z)
				};
				// A retained support may close a formerly open face. Keep only openings that survive.
				var before = CastingBounds.box(f.geometry),
					old = limits(before),
					now = limits(b);
				fragment.openings = fragment.openings.filter(o -> {
					if (o.kind != BodyFace)
						return true;
					var names = ["x-min", "y-min", "z-min", "x-max", "y-max", "z-max"];
					var suffix = o.target.substr(o.target.lastIndexOf(".") + 1),
						index = names.indexOf(suffix);
					return index < 0 || old[index] == now[index];
				});
				result.push(fragment);
			}
		}
		return CastingFeatureCodec.snapshot(result);
	}

	/** Exact minimum centroidal second moment about Y over X, in mm^4. */
	public static function minimumIy(cells:Array<Box3>, x0:Float, x1:Float):Float {
		if (!Math.isFinite(x0 + x1) || x0 >= x1)
			throw "Invalid section interval";
		var material = subtract(cells, []), cuts = [x0, x1];
		for (b in material)
			for (x in [b.x, b.x + b.width])
				if (x > x0 && x < x1 && cuts.indexOf(x) < 0)
					cuts.push(x);
		cuts.sort(Reflect.compare);
		var minimum = Math.POSITIVE_INFINITY;
		for (i in 0...cuts.length - 1) {
			var x = (cuts[i] + cuts[i + 1]) / 2, area = 0.0, first = 0.0, second = 0.0;
			for (b in material)
				if (x > b.x && x < b.x + b.width) {
					var a = b.depth * b.height, z = b.z + b.height / 2;
					area += a;
					first += a * z;
					second += b.depth * b.height * b.height * b.height / 12 + a * z * z;
				}
			if (area <= 0)
				throw "Casting has no continuous beam section";
			minimum = Math.min(minimum, second - first * first / area);
		}
		return minimum;
	}

	static function compactLimits(input:Array<Array<Float>>):Array<Array<Float>> {
		if (input.length < 100)
			return [for (b in input) b.copy()];
		var planes:Array<Array<Float>> = [[], [], []];
		for (b in input)
			for (axis in 0...3)
				for (v in [b[axis], b[axis + 3]])
					if (planes[axis].filter(p -> Math.abs(p-v)<=PLANE_EPSILON).length == 0)
						planes[axis].push(v);
		for (p in planes)
			p.sort((a, b) -> a < b ? -1 : a > b ? 1 : 0);
		var dims = [for (p in planes) p.length - 1], nx = dims[0], ny = dims[1], nz = dims[2], total = nx * ny * nz;
		if (total <= 0 || total > 200000)
			return [for (b in input) b.copy()];
		var occupied = [for (i in 0...total) false];
		for (b in input) {
			function indexAt(axis:Int,value:Float):Int {
                for(i in 0...planes[axis].length)if(Math.abs(planes[axis][i]-value)<=PLANE_EPSILON)return i;
                throw "Missing casting partition plane";
            }
            var lo = [for (a in 0...3) indexAt(a,b[a])],
                hi = [for (a in 0...3) indexAt(a,b[a+3])];
			for (z in lo[2]...hi[2])
				for (y in lo[1]...hi[1])
					for (x in lo[0]...hi[0])
						occupied[(z * ny + y) * nx + x] = true;
		}
		var best = [for (b in input) b.copy()];
		for (order in [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]]) {
			var remaining = occupied.copy(), boxes:Array<Array<Float>> = [];
			for (z0 in 0...nz)
				for (y0 in 0...ny)
					for (x0 in 0...nx) {
						if (!remaining[(z0 * ny + y0) * nx + x0])
							continue;
						var lo = [x0, y0, z0], hi = [x0 + 1, y0 + 1, z0 + 1];
						for (axis in order)
							while (hi[axis] < dims[axis]) {
								var faceLo = lo.copy(), faceHi = hi.copy();
								faceLo[axis] = hi[axis];
								faceHi[axis]++;
								var full = true;
								for (z in faceLo[2]...faceHi[2])
									for (y in faceLo[1]...faceHi[1])
										for (x in faceLo[0]...faceHi[0])
											if (!remaining[(z * ny + y) * nx + x])
												full = false;
								if (!full)
									break;
								hi[axis]++;
							}
						for (z in lo[2]...hi[2])
							for (y in lo[1]...hi[1])
								for (x in lo[0]...hi[0])
									remaining[(z * ny + y) * nx + x] = false;
						boxes.push([
							planes[0][lo[0]],
							planes[1][lo[1]],
							planes[2][lo[2]],
							planes[0][hi[0]],
							planes[1][hi[1]],
							planes[2][hi[2]]
						]);
					}
			boxes = [for (b in coalesce([for (v in boxes) fromLimits(v)])) limits(b)];
			if (boxes.length < best.length)
				best = boxes;
		}
		if (Math.abs(volume([for (v in best) fromLimits(v)]) - volume([for (v in input) fromLimits(v)])) > Math.max(1e-6,
			volume([for (v in input) fromLimits(v)]) * 1e-10))
			throw "Casting cell compaction changed volume";
		return best;
	}

	public static function compact(input:Array<Box3>):Array<Box3>
		return [
			for (b in compactLimits([for (v in subtract(input, [])) limits(v)])) fromLimits(b)
		];
}
