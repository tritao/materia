import cadkit.PlanarRegion;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Planar regions through the Haxe layer: sections, offsets, booleans and exact boundary loops. */
class PlanarRegionSmoke {
	static function check(ok:Bool, message:String):Void {
		if (!ok) throw message;
	}

	static function near(actual:Float, expected:Float, message:String, ?tolerance:Float = 1e-6):Void
		check(Math.abs(actual - expected) < tolerance, '$message: $actual against $expected');

	static function main():Int {
		run();
		return 0;
	}

	public static function run():Void {
		// A 20 mm cube with a 6 mm hole through it.
		var cube = Part.box(20, 20, 20);
		var hole = Part.cylinderSpan(3, -5, 25, 0, 0);
		var part = cube.subtract(hole);
		cube.close(); hole.close();

		var section = PlanarRegion.section(part.shape, 7.5);
		check(section.faceCount() == 1, "one face at mid height");
		near(section.area(), 400.0 - Math.PI * 9.0, "section area");
		var loops = section.loops();
		check(loops.length == 2, "an outer loop and a hole");
		var outer = loops[0].outer ? loops[0] : loops[1], inner = loops[0].outer ? loops[1] : loops[0];
		check(outer.outer && !inner.outer, "one of each");
		check(outer.segments.length == 4 && !outer.segments[0].arc, "the outer loop is four lines");
		check(inner.segments.length == 1 && inner.segments[0].arc, "the hole is one arc");
		near(inner.segments[0].radius, 3.0, "the hole's radius", 1e-9);
		near(Math.abs(inner.segments[0].sweep), 2.0 * Math.PI, "a full circle", 1e-9);
		near(outer.signedArea(), 400.0, "the outer loop runs counter-clockwise", 1e-9);
		near(inner.signedArea(), -Math.PI * 9.0, "the hole runs clockwise", 1e-9);
		near(outer.z, 0.0, "sections are brought down to z = 0", 1e-9);
		near(outer.length(), 80.0, "the square's perimeter", 1e-9);
		// Each loop closes up.
		for (loop in loops) for (index in 0...loop.segments.length) {
			var segment = loop.segments[index], next = loop.segments[(index + 1) % loop.segments.length];
			near(segment.x1, next.x0, "loops close in x", 1e-7);
			near(segment.y1, next.y0, "loops close in y", 1e-7);
		}

		// Shrinking: perimeters are offsets of the section, and the hole's loop grows exactly.
		var inset = section.offset(-0.225);
		near(inset.area(), 19.55 * 19.55 - Math.PI * 3.225 * 3.225, "inset area", 1e-6);
		var ring = inset.loops();
		var radii = [for (loop in ring) if (!loop.outer) loop.segments[0].radius];
		check(radii.length == 1, "one hole loop");
		near(radii[0], 3.225, "the hole's loop is a circle of radius 3 + d", 1e-9);
		// A band is the section less its inset, and the three booleans agree on areas.
		var band = section.cut(inset);
		near(band.area(), section.area() - inset.area(), "band", 1e-6);
		var inside = section.common(inset);
		near(inside.area(), inset.area(), "common", 1e-6);
		var whole = band.fuse(inset);
		near(whole.area(), section.area(), "fuse", 1e-6);
		check(whole.faceCount() == 1, "the pieces merge back into one face");
		// Vanishing and empty regions.
		var gone = section.offset(-11.0);
		check(gone.isEmpty() && gone.loops().length == 0 && gone.area() == 0.0, "an exhausted region is empty");
		check(section.common(gone).isEmpty(), "nothing in common with nothing");
		var top = PlanarRegion.section(part.shape, 25.0);
		check(top.isEmpty(), "a plane above the part misses it");
		for (region in [top, gone, whole, inside, band, inset, section])
			region.close();
		part.close();
		Sys.println("Planar region tests passed");
	}
}
