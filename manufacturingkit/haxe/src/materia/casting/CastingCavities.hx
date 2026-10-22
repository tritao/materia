package materia.casting;

import materia.casting.CastingFeature;
import materia.casting.CastingPrism;
import materia.casting.CastingPrism.CastingPoint2;
import materia.assembly.AssemblyFrames as F;

enum FinishedCavityGeometry {
	Prisms(pieces:Array<CastingPrism>);
	/** Exact cut/body expression; native realization must also validate openings. */
	SolidIntersection(body:Array<CastingPrism>, cut:CastingFeature, features:Array<CastingFeature>);
	UnsupportedGeometry(reason:String);
}

typedef FinishedCastingCavity = {
	var featureId:String;
	var parentId:String;
	var geometry:FinishedCavityGeometry;
}
typedef CastingFace3 = {var id:String; var points:Array<{x:Float, y:Float, z:Float}>;}

/** Single source of cut ∩ body. Supported envelopes are disjoint convex prisms
 * extruded along a coordinate axis (box, chamfered body, flared foot).
 */
class CastingCavities {
	/** Rebuildable exact expression for the region already derived here. */
	public static function region(cavity:FinishedCastingCavity):CastingRegion return switch cavity.geometry {
		case Prisms(pieces): envelopeRegion(pieces);
		case SolidIntersection(body,feature,_): CastingRegion.Intersection([envelopeRegion(body),cutRegion(body,feature)]);
		case UnsupportedGeometry(reason): throw "Unsupported casting cavity geometry: "+cavity.featureId+": "+reason;
	};

	public static function envelopeRegion(body:Array<CastingPrism>):CastingRegion {
		if (body==null || body.length==0) throw "Casting region needs an envelope";
		return CastingRegion.Union([for (prism in body) CastingRegion.Placed({
			solid:CastingFeature.CastingGeometrySolid.Prism(CastingProfiles.polygon(prism.profile),prism.height),pose:prism.pose})]);
	}

	/** Tool realization of source intent, shared with the finished-region expression. */
	public static function cutRegion(body:Array<CastingPrism>,feature:CastingFeature):CastingRegion {
		if (feature.draft.degrees!=0) {
			var section=CastingDraftSections.derive(body,feature);
			return CastingRegion.Drafted(feature.id,section.opening,section.pose,section.depth,feature.draft.degrees);
		}
		return switch feature.geometry.solid {
			case Insert(_):
				if (feature.cornerRadius!=0) throw "Insert recess cannot modify its source exterior: "+feature.id;
				CastingRegion.Placed(feature.geometry);
			default: CastingRegion.Placed({solid:CastingFeature.CastingGeometrySolid.Prism(CastingFeatureContours.contour(feature),
				CastingFeature.CastingGeometrySolids.height(feature.geometry.solid)),pose:feature.geometry.pose});
		};
	}

	public static function bodyFaceNormal(body:Array<CastingPrism>, id:String):{x:Float, y:Float, z:Float} {
		var faces = boundary(body).filter(face -> face.id == id);
		if (faces.length == 0)
			throw "Unknown casting body face: " + id;
		var plane = planeOf(faces[0].points);
		return {x: plane.x, y: plane.y, z: plane.z};
	}
	public static function derive(body:Array<CastingPrism>, features:Array<CastingFeature>):Array<FinishedCastingCavity> {
		var records = CastingFeatureCodec.snapshot(features),
			faces = boundary(body),
			result:Array<FinishedCastingCavity> = [];
		for (f in records) {
			for (opening in f.openings)
				if (opening.kind == BodyFace && faces.filter(face -> face.id == opening.target).length == 0)
					throw "Unknown casting body face: " + opening.target;
			var connectedNonBox=f.openings.filter(opening -> opening.kind==Feature
				&& CastingFeatureCodec.openingTargets(records,opening.target).filter(other ->
					!CastingFeature.CastingGeometrySolids.isBox(other.geometry.solid)
					|| other.draft.degrees!=0 || other.cornerRadius!=0).length>0).length>0;
			if (!CastingFeature.CastingGeometrySolids.isBox(f.geometry.solid) || f.draft.degrees!=0 || f.cornerRadius!=0 || connectedNonBox) {
					var envelope = [for (prism in body) {
						id: prism.id, profile: [for (point in prism.profile) {x: point.x, y: point.y}],
						height: prism.height, pose: CastingRecordCodec.frame(prism.pose)
					}];
					result.push({featureId: f.id, parentId: f.derivedFrom == null ? f.id : f.derivedFrom,
						geometry: SolidIntersection(envelope, f, records)});
					continue;
			}
			var b = CastingBounds.feature(f),
				pieces:Array<CastingPrism> = [];
			for (prism in body) {
				try
					CastingBounds.box({
						solid: Cut(CastingSolid.Box(1, 1, 1)),
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
					var others = CastingFeatureCodec.openingTargets(records, o.target);
					if (others.filter(other -> boxesMeet(b, CastingBounds.box(other.geometry))).length == 0)
						throw "Unreached feature opening: " + f.id + " " + o.target;
				}
			}
			for (id in reached)
				if (f.openings.filter(o -> o.kind == BodyFace && o.target == id).length == 0)
					throw "Undeclared casting breakout: " + f.id + " " + id;
			result.push({featureId: f.id, parentId: f.derivedFrom == null ? f.id : f.derivedFrom, geometry: Prisms(pieces)});
		}
		return result;
	}

	/** CAD and all consumers share the same realization gate; no empty stand-in. */
	public static function prisms(cavity:FinishedCastingCavity):Array<CastingPrism> return switch cavity.geometry {
		case Prisms(pieces): pieces;
		case SolidIntersection(_, _, _): throw "Unsupported non-prismatic finished cavity: " + cavity.featureId;
		case UnsupportedGeometry(reason): throw "Unsupported casting cavity geometry: " + cavity.featureId + ": " + reason;
	};

	public static function boxes(cavity:FinishedCastingCavity):Array<Box3> {
		if (!supportsBoxes(cavity)) throw "Unsupported non-box finished cavity: " + cavity.featureId;
		var result:Array<Box3> = [];
		for (p in prisms(cavity)) {
			if (!isBoxPrism(p))
				throw "Unsupported non-box finished cavity: " + cavity.featureId;
			var xs = [for (v in p.profile) v.x],
				ys = [for (v in p.profile) v.y];
			xs.sort(Reflect.compare);
			ys.sort(Reflect.compare);
			result.push(CastingBounds.box({
				solid: Cut(CastingSolid.Box(xs[3] - xs[0], ys[3] - ys[0], p.height)),
				pose: F.compose(p.pose, F.translation((xs[0] + xs[3]) / 2, (ys[0] + ys[3]) / 2, 0))
			}));
		}
		return BoxAlgebra.coalesce(result);
	}

	public static function supportsBoxes(cavity:FinishedCastingCavity):Bool
		return switch cavity.geometry {
			case Prisms(pieces): pieces.length > 0 && pieces.filter(piece -> !isBoxPrism(piece)).length == 0;
			case SolidIntersection(_, _, _): false;
			case UnsupportedGeometry(_): false;
		};

	static function isBoxPrism(p:CastingPrism):Bool {
		if (p.profile.length != 4)
			return false;
		var xs = [for (point in p.profile) point.x], ys = [for (point in p.profile) point.y];
		xs.sort(Reflect.compare);
		ys.sort(Reflect.compare);
		for (point in p.profile)
			if ((Math.abs(point.x - xs[0]) > 1e-8 && Math.abs(point.x - xs[3]) > 1e-8)
				|| (Math.abs(point.y - ys[0]) > 1e-8 && Math.abs(point.y - ys[3]) > 1e-8))
				return false;
		for (axis in [F.transformVector(p.pose, 1, 0, 0), F.transformVector(p.pose, 0, 1, 0), F.transformVector(p.pose, 0, 0, 1)])
			if (CastingBounds.axis(axis) == null)
				return false;
		return true;
	}

	/** Diagnostic only. Source callers declare openings; this must not populate intent. */
	public static function reachedOpenings(body:Array<CastingPrism>, geometry:CastingGeometry):Array<CastingOpening> {
		var b = CastingBounds.box(geometry), ids:Array<String> = [];
		for (face in boundary(body))
			if (faceMeetsBox(face, b) && ids.indexOf(face.id) < 0)
				ids.push(face.id);
		ids.sort(Reflect.compare);
		return [for (id in ids) {kind: BodyFace, target: id}];
	}

	static function boxesMeet(a:Box3, b:Box3):Bool {
		var overlaps = [
			Math.min(a.x + a.width, b.x + b.width) - Math.max(a.x, b.x),
			Math.min(a.y + a.depth, b.y + b.depth) - Math.max(a.y, b.y),
			Math.min(a.z + a.height, b.z + b.height) - Math.max(a.z, b.z)
		];
		if (overlaps.filter(length -> length < -1e-9).length > 0)
			return false;
		overlaps.sort((a, b) -> Reflect.compare(b, a));
		return overlaps[0] * overlaps[1] > 1e-9;
	}

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
				if (CastingBounds.axis({x: plane.x, y: plane.y, z: plane.z}) == axis) {
					face.id = prism.id + "." + names[axis] + (components[axis] < 0 ? "-min" : "-max");
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
