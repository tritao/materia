import CadKit;
import cadkit.Shape;
import cadkit.Geometry;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.Axis;
import cadkit.threads.ThreadSpec;
import cadkit.threads.Threads;
import cadkit.threads.ThreadedFace.ThreadStart;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.HoleFeature;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.LinearPatternFeature;

class ThreadSmoke {
	static function check(value:Bool, message:String):Void { if (!value) throw message; }
	static function near(value:Float, expected:Float):Void
		check(Math.abs(value - expected) < 1e-5, 'thread: expected $expected, got $value');

	public static function main():Void run();
	public static function run():Void {
		var spec = new ThreadSpec("M6", 6, 1, 1, RightHand, true);
		var top = new SelectionRecipe("face", "plane", Vector.Z(), "max", Vector.Z(), 1);
		var d = new Document();
		var plate = d.add(new BoxFeature(60, 40, 30));
		var a = d.add(HoleFeature.tapped(plate, top, Vector.X(), "blind", -15, 0, 5, 25, spec, 20));
		var b = d.add(HoleFeature.tapped(plate, top, Vector.X(), "blind", 0, 0, 5, 25, spec, 12));
		var plain = d.add(HoleFeature.plain(plate, top, Vector.X(), "blind", 15, 0, 5, 25));
		var cutA = d.add(new BooleanFeature(plate, a, BooleanOperation.Cut));
		var cutB = d.add(new BooleanFeature(cutA, b, BooleanOperation.Cut));
		var cut = d.add(new BooleanFeature(cutB, plain, BooleanOperation.Cut));
		d.setOutput(cut); d.recompute();
		var initial = Threads.ofShape(d.result());
		check(initial.length == 2, "only two equal-diameter holes have threads");
		near(initial[0].length, 20); near(initial[1].length, 12);
		near(initial[0].frame.origin.z, 30); near(initial[0].frame.normal.z, -1);
		var originalName = initial[0].attribute.faceName;
		a.x.set(-12); d.recompute();
		var moved = Threads.ofShape(d.result());
		check(moved[0].attribute.faceName == originalName, "position edit preserves named thread");
		near(moved[0].frame.origin.x, initial[0].frame.origin.x + 3);
		var restored = DocumentCodec.decode(DocumentCodec.encode(d));
		var saved = Threads.ofShape(restored.result());
		near(saved[0].length, 20); near(saved[1].length, 12);
		check(saved[0].attribute.spec.matches(spec), "thread specification survives saved feature");
		restored.close(); d.close();

		var patterned = new Document();
		var target = patterned.add(new BoxFeature(40, 40, 8));
		var tool = patterned.add(HoleFeature.tapped(target, top, Vector.X(), "through-all", 0, 0, 5, 1, spec, 20));
		var pattern = patterned.add(new LinearPatternFeature(tool, 2, 12, Vector.X(), 2, 12, Vector.Y()));
		var finalCut = patterned.add(new BooleanFeature(target, pattern, BooleanOperation.Cut));
		patterned.setOutput(finalCut); patterned.recompute();
		var threads = Threads.ofShape(patterned.result());
		check(threads.length == 4, "four patterned threads retain separate names");
		for (thread in threads) near(thread.length, 8);
		check(threads[0].attribute.faceName != threads[1].attribute.faceName, "pattern names distinguish instances");
		var transformed = Location.translation(new Vector(10, 20, 30)).compose(Location.rotation(Axis.Y(), Math.PI / 2)).apply(patterned.result());
		var placed = Threads.ofShape(transformed);
		near(placed[0].length, 8); near(placed[0].frame.normal.x, -1);
		transformed.close(); patterned.close();

		var shank = Part.cylinderSpan(3, -40, 0);
		var external = Threads.attach(shank.shape, "cyl.side", spec.mating(), AxisMaximum, 18, 22);
		var externalThreads = Threads.ofShape(external);
		near(externalThreads[0].frame.origin.z, -22); near(externalThreads[0].length, 18);
		check(externalThreads[0].partial, "partial shank retains plain upper length");
		var cutterBase = Part.box(20, 20, 30);
		var cutter = cutterBase.translated(new Vector(0, 0, -10));
		cutterBase.close();
		var shortened = external.cut(cutter.shape);
		near(Threads.ofShape(shortened)[0].frame.origin.z, -22);
		var splitBase = Part.box(20, 20, 4);
		var splitTool = splitBase.translated(new Vector(0, 0, -32));
		splitBase.close();
		var split = external.cut(splitTool.shape);
		var splitThreads = Threads.ofShape(split);
		check(splitThreads.length == 2, "split named face carries clipped thread intervals");
		near(splitThreads[0].length + splitThreads[1].length, 14);
		var mirrored = external.mirror(Geometry.vec3(0, 0, 0), Vector.Z().native());
		var mirrorThread = Threads.ofShape(mirrored)[0];
		near(mirrorThread.frame.origin.z, 22); near(mirrorThread.frame.normal.z, 1);
		check(mirrorThread.attribute.spec.hand == RightHand, "manufacturing mirror retains specified hand");
		mirrored.close();
		split.close(); splitTool.close(); shortened.close(); cutter.close(); external.close(); shank.close();

		var cone = Part.revolve([{r: 0.0, z: 0.0}, {r: 3.0, z: 0.0}, {r: 4.0, z: 10.0}, {r: 0.0, z: 10.0}]);
		var coneNames:Array<String> = [];
		for (index in 0...cone.shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face = cone.shape.subshape(CadKit.ShapeKind.Face, index);
			coneNames.push(face.surfaceKind() == CadKit.SurfaceKind.Cone ? "thread-wall" : "");
			face.close();
		}
		var namedCone = cone.shape.withElementNames(CadKit.ShapeKind.Face, coneNames);
		var threadedCone = Threads.attach(namedCone, "thread-wall", spec.mating(), AxisMinimum, 7);
		near(Threads.ofShape(threadedCone)[0].length, 7);
		threadedCone.close(); namedCone.close(); cone.close();
		var missing = false;
		var simple = Shape.cylinder(3, 10);
		try Threads.attach(simple, "missing-wall", spec, AxisMinimum) catch (_:Dynamic) missing = true;
		check(missing, "missing name reports instead of matching radius"); simple.close();

		var stock = Part.box(20, 20, 10);
		var boreBase = Threads.tappedBore(spec, 5, 10);
		var bore = boreBase.translated(new Vector(0, 0, 10));
		boreBase.close();
		var tapped = stock.subtract(bore);
		var larger = Part.cylinderSpan(4, -1, 11);
		var removed = tapped.subtract(larger);
		check(Threads.of(removed).length == 0, "proven deleted thread is removed, never reassigned");
		removed.close(); larger.close(); tapped.close(); bore.close(); stock.close();
		var rejected = false;
		try new HoleFeature(plate, top, Vector.X(), "counterbore", "blind", 0, 0, 5, 10, 9, 2, 0, false, Math.PI / 2, spec, 5)
		catch (_:Dynamic) rejected = true;
		check(rejected, "counterbored tapped hole rejected");
		Sys.println("Named thread tests passed");
	}
}
