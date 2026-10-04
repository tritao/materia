package cadkit.sketch;

import cadkit.modeling.Curve;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;

/** Validated exact regions; native profiles and downstream models consume this same solved geometry. */
class ProfileGeometry {
	public final plane:Plane;
	final regionData:Array<ProfileRegion>;
	public function new(plane:Plane, regions:Array<ProfileRegion>) {
		this.plane = plane;
		regionData = regions.copy();
	}
	public function regions():Array<ProfileRegion> { return regionData.copy(); }

	public function build():Sketch {
		var curves:Array<Curve> = [];
		var result:Null<Sketch> = null;
		try {
			for (region in regionData) {
				var outer = wire(region.outer());
				curves.push(outer);
				var holes:Array<Curve> = [];
				for (hole in region.holes()) {
					var curve = wire(hole);
					curves.push(curve);
					holes.push(curve);
				}
				var face = Sketch.named(Sketch.face(outer, holes, plane), region.id);
				if (result == null) result = face;
				else {
					try {
						var combined = result.combine(face);
						result.close(); face.close(); result = combined;
					} catch (error:Dynamic) { face.close(); throw error; }
				}
			}
			if (result == null) throw new ProfileError("ambiguous", [], "profile has no outer boundary");
			for (curve in curves) curve.close();
			return result;
		} catch (error:Dynamic) {
			for (curve in curves) curve.close();
			if (result != null) result.close();
			if (Std.isOfType(error, ProfileError)) throw error;
			var ids:Array<String> = [];
			for (region in regionData) {
				for (curve in region.outer()) ids.push(curve.entityId);
				for (hole in region.holes()) for (curve in hole) ids.push(curve.entityId);
			}
			throw new ProfileError("invalid", ids, "native profile construction failed: " + Std.string(error));
		}
	}
	function wire(data:Array<ProfileCurve>):Curve {
		var parts:Array<Curve> = [];
		try {
			for (curve in data) {
				var part:Curve;
				if (curve.center == null)
					part = Curve.line(plane.toWorld(curve.start), plane.toWorld(curve.end));
				else {
					var center:Vector = cast curve.center;
					if (Math.abs(curve.sweep) == 2 * Math.PI)
						part = Curve.circle(curve.radius, new Plane(plane.toWorld(center), plane.xDirection, plane.normal));
					else {
						var a = curve.startAngle + curve.sweep / 2;
						var middle = new Vector(center.x + curve.radius * Math.cos(a), center.y + curve.radius * Math.sin(a));
						part = Curve.arc(plane.toWorld(curve.start), plane.toWorld(middle), plane.toWorld(curve.end));
					}
				}
				parts.push(Curve.named(part, "e." + curve.entityId));
			}
			var result = Curve.wire(parts);
			for (part in parts) part.close();
			return result;
		} catch (error:Dynamic) { for (part in parts) part.close(); throw error; }
	}
}
