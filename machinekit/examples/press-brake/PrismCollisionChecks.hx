import cadkit.modeling.Axis;
import cadkit.modeling.Location;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.SheetDiagnosis;
import machinekit.pressbrake.planning.ConvexPrism;
import machinekit.pressbrake.planning.PrismSatQuery;
import machinekit.pressbrake.planning.SheetCollisionQuery;

/** Independent contact, containment and skew-edge fixtures for the interim convex-prism query. */
class PrismCollisionChecks {
	static var checks:Int = 0;
	public static function main():Void run();
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function box(id:String, x:Float, y:Float, z:Float):ConvexPrism {
		return new ConvexPrism(id, [new Vector(-x, -y, -z), new Vector(x, -y, -z),
			new Vector(x, y, -z), new Vector(-x, y, -z)], new Vector(0, 0, 2 * z));
	}
	static function invalid(action:Void->Void):Void {
		checks++;
		try { action(); }
		catch (failure:SheetDiagnosis) { if (failure.code == "press.convex-prism") return; throw failure; }
		throw "Expected named invalid prism diagnosis";
	}
	// This fixture guard deliberately omits edge axes: it proves face tests cannot resolve the pair.
	static function faceProjectionsOverlap(a:ConvexPrism, b:ConvexPrism):Bool {
		var axes = a.faceNormals().concat(b.faceNormals());
		for (axis in axes) {
			var lowA = Math.POSITIVE_INFINITY, highA = Math.NEGATIVE_INFINITY;
			var lowB = Math.POSITIVE_INFINITY, highB = Math.NEGATIVE_INFINITY;
			for (point in a.vertices()) { lowA = Math.min(lowA, point.dot(axis)); highA = Math.max(highA, point.dot(axis)); }
			for (point in b.vertices()) { lowB = Math.min(lowB, point.dot(axis)); highB = Math.max(highB, point.dot(axis)); }
			if (highA <= lowB || highB <= lowA) return false;
		}
		return true;
	}
	public static function run():Void {
		checks = 0;
		var query:SheetCollisionQuery = new PrismSatQuery();
		var rod = box("rod", 2, 0.2, 0.2), contained = box("contained", 0.1, 0.1, 0.1);
		var rodInset = rod.inset(0.1);
		check(rodInset != null, "Inset retains a sufficiently thick prism");
		if (rodInset == null) throw "Missing retained inset fixture";
		for (point in rodInset.vertices())
			check(Math.abs(point.x) <= 1.9 + 1e-8 && Math.abs(point.y) <= 0.1 + 1e-8 && Math.abs(point.z) <= 0.1 + 1e-8,
				"Euclidean inset moves every box face inward by the declared distance");
		check(rod.inset(0.2) == null, "Collapsed thickness has no inset material");
		check(rod.inset(0.2-1e-9) == null, "Subresolution cap thickness does not construct an invalid prism");
		var longThin = box("long-thin-inset",10000,0.2,1);
		check(longThin.inset(0.2-1e-9) == null, "Large area does not hide subresolution side edges");
		check(rod.inset(0) == rod, "Zero inset preserves immutable material");
		var insetFixture = new ConvexPrism("oblique-inset",[new Vector(-1,-1,0),new Vector(1,-1,0),new Vector(1,1,0),new Vector(-1,1,0)],new Vector(1,0,2));
		var obliqueInner = insetFixture.inset(0.1);
		check(obliqueInner != null, "Oblique extrusion retains its face inset");
		if (obliqueInner == null) throw "Missing oblique inset fixture";
		for (point in obliqueInner.vertices())
			check(point.z >= 0.1 - 1e-8 && point.z <= 1.9 + 1e-8 && Math.abs(point.y) <= 0.9 + 1e-8
				&& Math.abs(2*point.x-point.z) <= 2-0.1*Math.sqrt(5)+1e-8,
				"Oblique inset respects Euclidean side planes rather than shrinking vertex coordinates");
		check(query.overlap(rod, rod), "A prism overlaps itself");
		check(query.overlap(rod, contained), "Containment is overlap");
		check(query.overlap(contained, rod), "Containment is symmetric");
		var touching = rod.placed(Location.translation(new Vector(4, 0, 0)));
		var penetrating = rod.placed(Location.translation(new Vector(3.9, 0, 0)));
		var separated = rod.placed(Location.translation(new Vector(4.1, 0, 0)));
		check(!query.overlap(rod, touching), "Face contact has no positive penetration");
		check(query.overlap(rod, penetrating), "Positive penetration is overlap");
		check(!query.overlap(rod, separated), "Separated prisms do not overlap");
		check(Math.abs(query.separation(rod, separated) - 0.1) < 1e-8, "Support gap has physical mm units");
		check(Math.abs(query.separation(separated, rod) - 0.1) < 1e-8, "Support gap is symmetric");
		check(Math.abs(query.separation(rod, touching)) < 1e-8, "Touching has zero support gap");
		check(query.separation(rod, penetrating) < 0, "Overlapping prisms have negative support gaps");
		check(!query.overlap(rod, rod.placed(Location.translation(new Vector(4, 0.4, 0.4)))), "Vertex contact is permitted");
		var tiny = rod.placed(Location.translation(new Vector(3.995, 0, 0)));
		check(query.overlap(rod, tiny), "Small penetration above default tolerance is detected");
		check(!new PrismSatQuery(0.01).overlap(rod, tiny), "Declared mm penetration tolerance is applied");
		var rotation = Location.rotation(new Axis(new Vector(), new Vector(1, 2, 3)), 0.9);
		var skew = rod.placed(Location.translation(new Vector(0, 0, 0.8)).compose(rotation));
		check(faceProjectionsOverlap(rod, skew), "Skew fixture overlaps on every face-normal projection");
		check(!query.overlap(rod, skew), "Edge-cross-product axis separates the skew rods");
		check(!query.overlap(skew, rod), "Skew separation is symmetric");
		check(query.separation(rod, skew) > 0, "Edge-axis separation also certifies a positive distance bound");
		var common = Location.translation(new Vector(1000, 2000, 3000)).compose(
			Location.rotation(new Axis(new Vector(), new Vector(3, 1, 2)), 0.7));
		check(query.overlap(rod.placed(common), penetrating.placed(common)), "Rigid transform preserves penetration");
		check(!query.overlap(rod.placed(common), skew.placed(common)), "Rigid transform preserves edge-axis separation");
		check(Math.abs(query.separation(rod.placed(common), separated.placed(common)) - 0.1) < 1e-8, "Rigid transform preserves the metric support gap");
		check(query.overlap(rod.placed(common), contained.placed(common)), "Rigid transform preserves containment");
		var almostParallel = separated.placed(Location.rotation(Axis.Z(), 1e-13));
		check(!query.overlap(rod, almostParallel), "Nearly parallel edges retain a stable separation");
		var oblique = new ConvexPrism("oblique", [new Vector(0, 0), new Vector(0, 2),
			new Vector(2, 2), new Vector(2, 0)], new Vector(1, 0.3, 2));
		var inner = contained.placed(Location.translation(new Vector(1.5, 1.15, 1)));
		check(query.overlap(oblique, inner), "Oblique extrusion with clockwise winding contains its centre");
		check(!query.overlap(oblique, inner.placed(Location.translation(new Vector(10, 0, 0)))), "Oblique face axes reject separation");
		var polygon = [new Vector(0, 0, 0), new Vector(2, 0, 0), new Vector(2, 2, 0), new Vector(0, 2, 0)];
		var owned = new ConvexPrism("owned", polygon, Vector.Z()); polygon[0] = new Vector(999, 0, 0);
		check(owned.vertices()[0].x == 0, "Prism owns its polygon array");
		var exposed = owned.vertices(); var removed = exposed.pop();
		check(owned.vertices().length == 8, "Returned vertex arrays cannot change the prism");
		invalid(function() {
			var result = new ConvexPrism("concave", [new Vector(0, 0), new Vector(2, 0),
				new Vector(1, 1), new Vector(2, 2), new Vector(0, 2)], Vector.Z());
		});
		invalid(function() {
			var result = new ConvexPrism("nonplanar", [new Vector(0, 0), new Vector(2, 0),
				new Vector(2, 2), new Vector(0, 2, 0.1)], Vector.Z());
		});
		invalid(function() {
			var result = new ConvexPrism("flat", [new Vector(0, 0), new Vector(2, 0), new Vector(0, 2)], Vector.X());
		});
		invalid(function() {
			var result = new ConvexPrism("duplicate", [new Vector(0, 0), new Vector(2, 0), new Vector(2, 0), new Vector(0, 2)], Vector.Z());
		});
		trace("Prism collision checks passed (" + checks + " assertions)");
	}
}
