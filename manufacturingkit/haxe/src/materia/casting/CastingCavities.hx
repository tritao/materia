package materia.casting;

import materia.casting.CastingFeature;
import materia.casting.CastingPrism;
import materia.casting.CastingPrism.CastingPoint2;
import materia.assembly.AssemblyFrames as F;

typedef FinishedCastingCavity = {var featureId:String; var parentId:String; var pieces:Array<CastingPrism>;}
typedef CastingFace3 = {var id:String; var points:Array<{x:Float, y:Float, z:Float}>;}

/** Single source of cut ∩ body. Supported envelopes are disjoint convex prisms
 * extruded along a coordinate axis (box, chamfered body, flared foot).
 */
class CastingCavities {
	public static function derive(body:Array<CastingPrism>, features:Array<CastingFeature>):Array<FinishedCastingCavity> {
		var records = CastingFeatureCodec.snapshot(features),
			faces = boundary(body),
			result:Array<FinishedCastingCavity> = [];
		for (f in records) {
			var b = CastingBounds.box(f.geometry),
				pieces:Array<CastingPrism> = [];
			for (prism in body) {
				try
					CastingBounds.box({
						kind: Box,
						width: 1,
						depth: 1,
						height: 1,
						pose: prism.pose
					})
				catch (error:Dynamic)
					throw "Unsupported envelope orientation for feature: " + f.id;
				var inverse = F.inverse(prism.pose);
				var corners = [
					for (x in [b.x, b.x + b.width]) for (y in [b.y, b.y + b.depth]) for (z in [b.z, b.z + b.height]) F.transformPoint(inverse, x, y, z)
				];
				var xx = [for (p in corners) p.x],
					yy = [for (p in corners) p.y],
					zz = [for (p in corners) p.z];
				xx.sort(Reflect.compare);
				yy.sort(Reflect.compare);
				zz.sort(Reflect.compare);
				var low = Math.max(0, zz[0]),
					high = Math.min(prism.height, zz[7]);
				if (high - low <= 1e-9)
					continue;
				var profile = CastingPolygons.halfPlane(prism.profile, 1, 0, xx[7]);
				profile = CastingPolygons.halfPlane(profile, -1, 0, -xx[0]);
				profile = CastingPolygons.halfPlane(profile, 0, 1, yy[7]);
				profile = CastingPolygons.halfPlane(profile, 0, -1, -yy[0]);
				if (profile.length < 3 || CastingPolygons.area(profile) <= 1e-9)
					continue;
				pieces.push({
					id: f.id + "-" + prism.id,
					profile: profile,
					height: high - low,
					pose: F.compose(prism.pose, F.translation(0, 0, low))
				});
			}
			if (pieces.length == 0)
				throw "Casting cut does not reach body: " + f.id;
			var reached:Array<String> = [];
			for (face in faces)
				if (faceMeetsBox(face, b) && reached.indexOf(face.id) < 0)
					reached.push(face.id);
			for (o in f.openings) {
				if (o.kind == BodyFace && reached.indexOf(o.target) < 0)
					throw "Unreached declared opening: " + f.id + " " + o.target;
				if (o.kind == Feature) {
					var other = records.filter(v -> v.id == o.target)[0];
					if (!boxesMeet(b, CastingBounds.box(other.geometry)))
						throw "Unreached feature opening: " + f.id + " " + o.target;
				}
			}
			for (id in reached)
				if (f.openings.filter(o -> o.kind == BodyFace && o.target == id).length == 0)
					throw "Undeclared casting breakout: " + f.id + " " + id;
			result.push({featureId: f.id, parentId: f.derivedFrom == null ? f.id : f.derivedFrom, pieces: pieces});
		}
		return result;
	}

	public static function boxes(cavity:FinishedCastingCavity):Array<Box3> {
		var result:Array<Box3> = [];
		for (p in cavity.pieces) {
			var xs = [for (v in p.profile) v.x],
				ys = [for (v in p.profile) v.y];
			xs.sort(Reflect.compare);
			ys.sort(Reflect.compare);
			if (p.profile.length != 4)
				throw "Unsupported non-box finished cavity: " + cavity.featureId;
			for (v in p.profile)
				if ((Math.abs(v.x - xs[0]) > 1e-8 && Math.abs(v.x - xs[3]) > 1e-8)
					|| (Math.abs(v.y - ys[0]) > 1e-8 && Math.abs(v.y - ys[3]) > 1e-8))
					throw "Unsupported non-box finished cavity: " + cavity.featureId;
			result.push(CastingBounds.box({
				kind: Box,
				width: xs[3] - xs[0],
				depth: ys[3] - ys[0],
				height: p.height,
				pose: F.compose(p.pose, F.translation((xs[0] + xs[3]) / 2, (ys[0] + ys[3]) / 2, 0))
			}));
		}
		return BoxAlgebra.coalesce(result);
	}

	public static function reachedOpenings(body:Array<CastingPrism>, geometry:CastingGeometry):Array<CastingOpening> {
		var b = CastingBounds.box(geometry), ids:Array<String> = [];
		for (face in boundary(body))
			if (faceMeetsBox(face, b) && ids.indexOf(face.id) < 0)
				ids.push(face.id);
		ids.sort(Reflect.compare);
		return [for (id in ids) {kind: BodyFace, target: id}];
	}

	static function boxesMeet(a:Box3, b:Box3):Bool
		return Math.min(a.x + a.width, b.x + b.width) >= Math.max(a.x, b.x) - 1e-8
			&& Math.min(a.y + a.depth, b.y + b.depth) >= Math.max(a.y, b.y) - 1e-8
			&& Math.min(a.z + a.height, b.z + b.height) >= Math.max(a.z, b.z) - 1e-8;

	static function faceMeetsBox(face:CastingFace3, box:Box3):Bool {
		var points = face.points;
		for (axis in 0...3)
			for (side in [0, 1]) {
				var lo = [box.x, box.y, box.z],
					hi = [box.x + box.width, box.y + box.depth, box.z + box.height],
					out:Array<{x:Float, y:Float, z:Float}> = [];
				for (i in 0...points.length) {
					var p = points[i], q = points[(i + 1) % points.length], a = [p.x, p.y, p.z], b = [q.x, q.y, q.z];
					var u = side == 0 ? lo[axis] - a[axis] : a[axis] - hi[axis],
						v = side == 0 ? lo[axis] - b[axis] : b[axis] - hi[axis];
					if (u <= 1e-9)
						out.push(p);
					if ((u < -1e-9 && v > 1e-9) || (u > 1e-9 && v < -1e-9)) {
						var t = u / (u - v);
						out.push({x: p.x + t * (q.x - p.x), y: p.y + t * (q.y - p.y), z: p.z + t * (q.z - p.z)});
					}
				}
				points = out;
				if (points.length < 3)
					return false;
			}
		var a = points[0], area = 0.0;
		for (i in 1...points.length - 1) {
			var b = points[i], c = points[i + 1], x = (b.y - a.y) * (c.z - a.z) - (b.z - a.z) * (c.y - a.y),
				y = (b.z - a.z) * (c.x - a.x) - (b.x - a.x) * (c.z - a.z), z = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
			area += Math.sqrt(x * x + y * y + z * z) / 2;
		}
		return area > 1e-8;
	}

	public static function boundary(body:Array<CastingPrism>):Array<CastingFace3> {
		var result:Array<CastingFace3> = [];
		for (prism in body)
			for (face in prismFaces(prism)) {
				var pieces = [face.points];
				for (other in body)
					if (other.id != prism.id) {
						var next:Array<Array<{x:Float, y:Float, z:Float}>> = [];
						for (piece in pieces) {
							var inside = piece;
							for (planeFace in prismFaces(other)) {
								var plane = planeOf(planeFace.points);
								var outside = clip3(inside, -plane.x, -plane.y, -plane.z, -plane.d);
								// Points on the same plane belong to the intersection, not both partitions.
								if (outside.length >= 3
									&& outside.filter(p -> plane.x * p.x + plane.y * p.y + plane.z * p.z > plane.d + 1e-8).length > 0)
									next.push(outside);
								inside = clip3(inside, plane.x, plane.y, plane.z, plane.d);
								if (inside.length < 3)
									break;
							}
						}
						pieces = next;
					}
				for (points in pieces)
					if (points.length >= 3)
						result.push({id: face.id, points: points});
			}
		return result;
	}

	static function prismFaces(prism:CastingPrism):Array<CastingFace3> {
		var low = [for (p in prism.profile) F.transformPoint(prism.pose, p.x, p.y, 0)],
			high = [for (p in prism.profile) F.transformPoint(prism.pose, p.x, p.y, prism.height)];
		var bottom = low.copy();
		bottom.reverse();
		var faces:Array<CastingFace3> = [{id: "", points: bottom}, {id: "", points: high}];
		for (i in 0...low.length) {
			var j = (i + 1) % low.length;
			faces.push({id: prism.id + ".side-" + i, points: [low[i], low[j], high[j], high[i]]});
		}
		for (i in 0...faces.length) {
			var face = faces[i],
				plane = planeOf(face.points),
				components = [plane.x, plane.y, plane.z],
				names = ["x", "y", "z"];
			var named = false;
			for (axis in 0...3)
				if (Math.abs(components[axis]) > 1 - 1e-8) {
					face.id = "body." + names[axis] + (components[axis] < 0 ? "-min" : "-max");
					named = true;
				}
			if (!named && face.id == "")
				face.id = prism.id + (i == 0 ? ".start" : ".end");
		}
		return faces;
	}

	static function planeOf(points:Array<{x:Float, y:Float, z:Float}>):{
		x:Float,
		y:Float,
		z:Float,
		d:Float
	} {
		var a = points[0], b = points[1], c = points[2], x = (b.y - a.y) * (c.z - a.z) - (b.z - a.z) * (c.y - a.y),
			y = (b.z - a.z) * (c.x - a.x) - (b.x - a.x) * (c.z - a.z), z = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x),
			length = Math.sqrt(x * x + y * y + z * z);
		if (length < 1e-12)
			throw "Degenerate casting boundary face";
		x /= length;
		y /= length;
		z /= length;
		return {
			x: x,
			y: y,
			z: z,
			d: x * a.x + y * a.y + z * a.z
		};
	}

	static function clip3(points:Array<{x:Float, y:Float, z:Float}>, x:Float, y:Float, z:Float, d:Float):Array<{x:Float, y:Float, z:Float}> {
		var result:Array<{x:Float, y:Float, z:Float}> = [];
		for (i in 0...points.length) {
			var a = points[i], b = points[(i + 1) % points.length], u = x * a.x + y * a.y + z * a.z - d, v = x * b.x + y * b.y + z * b.z - d;
			if (u <= 1e-9)
				result.push(a);
			if ((u < -1e-9 && v > 1e-9) || (u > 1e-9 && v < -1e-9)) {
				var t = u / (u - v);
				result.push({x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y), z: a.z + t * (b.z - a.z)});
			}
		}
		return result;
	}
}
