import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.Feature;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterExpression;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.features.BoxFeature;

private class TypedParameterProbeFeature extends Feature {
	public final permissive:Parameter;
	public final restrictive:Parameter;
	public final looseCount:Parameter;

	public function new() {
		super();
		permissive = new Parameter(this, "probe.permissive", 10, 0, false, 1e300, ParameterKind.Length);
		restrictive = new Parameter(this, "probe.restrictive", 10, 4, false, 1e300, ParameterKind.Length);
		looseCount = new Parameter(this, "probe.count", 2, 0, false, 1e300, ParameterKind.Count);
	}
}

class TypedParameterSmoke {
	static function check(value:Bool, message:String):Void {
		if (!value)
			throw message;
	}

	static function near(value:Float, expected:Float):Void {
		check(Math.abs(value - expected) < 1e-7 * Math.max(1, Math.abs(expected)), 'expected $expected, got $value');
	}

	public static function run():Void {
		var document = new Document();
		var base = document.defineTypedParameter("baseLevel.elevation", 1, ParameterKind.Length, "m");
		var upper = document.defineTypedParameter("upperLevel.elevation", 3.5, ParameterKind.Length, "m");
		var width = document.defineTypedParameter("wall.width", 2, ParameterKind.Length, "m");
		var depth = document.defineTypedParameter("wall.depth", 30, ParameterKind.Length, "cm");
		var height = document.defineExpression("wall.height", ParameterKind.Length, "m",
			"upperLevel.elevation - baseLevel.elevation");
		var area = document.defineExpression("wall.area", ParameterKind.Area, "m2", "wall.width * wall.depth");
		var volume = document.defineExpression("wall.volume", ParameterKind.Volume, "m3", "wall.area * wall.height");
		var angle = document.defineTypedParameter("roof.angle", 90, ParameterKind.Angle, "deg");
		var count = document.defineTypedParameter("bay.count", 4, ParameterKind.Count, "count");
		var doubledCount = document.defineExpression("bay.double-count", ParameterKind.Count, "count", "bay.count * 2");

		near(base.value, 1000);
		near(upper.valueIn("ft"), 3.5 / 0.3048);
		near(height.value, 2500);
		near(height.valueIn("m"), 2.5);
		near(area.valueIn("m2"), 0.6);
		near(volume.valueIn("m3"), 1.5);
		near(angle.value, Math.PI / 2);
		near(count.value, 4);
		near(doubledCount.value, 8);

		var wall = document.add(new BoxFeature(width.value, depth.value, height.value));
		width.bind(wall.width);
		depth.bind(wall.depth);
		height.bind(wall.height);
		document.setOutput(wall);
		document.recompute();
		near(document.result().volume(), 2000 * 300 * 2500);
		var failed = false;
		try wall.height.set(1000) catch (error:Dynamic) failed = true;
		check(failed && height.value == 2500, "bound expression parameters are read-only");

		upper.set(4, "m");
		document.recompute();
		near(height.valueIn("m"), 3);
		near(document.result().bounds().get_max().get_z(), 3000);
		check(document.undo(), "typed standalone parameter undo");
		document.recompute();
		near(document.result().bounds().get_max().get_z(), 2500);
		check(document.redo(), "typed standalone parameter redo");
		document.recompute();
		near(document.result().bounds().get_max().get_z(), 3000);
		var committedWall = document.result();
		upper.set(0.5, "m");
		failed = false;
		try document.recompute() catch (error:Dynamic) failed = true;
		check(failed && document.result() == committedWall, "invalid expression result preserves committed geometry");
		check(document.undo(), "invalid typed edit undo");
		document.recompute();
		near(document.result().bounds().get_max().get_z(), 3000);

		failed = false;
		try count.set(2.5) catch (error:Dynamic) failed = true;
		check(failed && count.value == 4, "count conversion rejects fractional values");
		failed = false;
		try document.defineExpression("bad.area", ParameterKind.Area, "m2", "wall.width + wall.area") catch (error:Dynamic) failed = true;
		check(failed, "expression addition validates dimensions");

		var first = document.defineTypedParameter("cycle.first", 1, ParameterKind.Length, "m");
		document.defineExpression("cycle.second", ParameterKind.Length, "m", "cycle.first + wall.depth");
		failed = false;
		try document.setExpression("cycle.first", "cycle.second - wall.depth") catch (error:Dynamic) failed = true;
		check(failed && first.expression == null, "expression dependency cycles are rejected atomically");
		var earlier = document.defineTypedParameter("forward.earlier", 1, ParameterKind.Length, "m");
		document.defineTypedParameter("forward.later", 2, ParameterKind.Length, "m");
		document.setExpression("forward.earlier", "forward.later + wall.depth");
		near(earlier.valueIn("m"), 2.3);
		check(document.undo(), "expression definition undo");
		near(earlier.valueIn("m"), 1);
		check(document.redo(), "expression definition redo");
		near(earlier.valueIn("m"), 2.3);

		var historyDocument = new Document();
		var historyBox = historyDocument.add(new BoxFeature(10, 8, 6));
		var historyWidth = historyDocument.defineTypedParameter("history.width", 10, ParameterKind.Length, "mm");
		historyDocument.defineTypedParameter("history.other", 20, ParameterKind.Length, "mm");
		historyWidth.bind(historyBox.width);
		historyDocument.setExpression("history.width", "history.other");
		near(historyBox.width.value, 20);
		check(historyDocument.undo(), "bound expression undo");
		near(historyBox.width.value, 10);
		check(historyWidth.expression == null, "bound expression undo restores the expression");
		check(historyDocument.redo(), "bound expression redo");
		near(historyBox.width.value, 20);
		check(historyDocument.undo(), "bound expression second undo");
		var cancelled = historyDocument.beginTransaction();
		historyDocument.setExpression("history.width", "history.other");
		cancelled.cancel();
		near(historyBox.width.value, 10);
		check(historyWidth.expression == null, "cancelled bound expression restores the expression");
		historyDocument.close();

		var validationDocument = new Document();
		var probe = validationDocument.add(new TypedParameterProbeFeature());
		var sharedLength = validationDocument.defineTypedParameter("validation.length", 10, ParameterKind.Length, "mm");
		validationDocument.defineTypedParameter("validation.small", 3, ParameterKind.Length, "mm");
		sharedLength.bind(probe.permissive);
		sharedLength.bind(probe.restrictive);
		failed = false;
		try validationDocument.setExpression("validation.length", "validation.small") catch (error:Dynamic) failed = true;
		check(failed, "all expression binding targets are validated");
		near(probe.permissive.value, 10);
		near(probe.restrictive.value, 10);
		check(sharedLength.expression == null && !validationDocument.undo(), "failed expression change has no history entry");

		var typedCount = validationDocument.defineTypedParameter("validation.count", 2, ParameterKind.Count, "count");
		typedCount.bind(probe.looseCount);
		failed = false;
		try probe.looseCount.set(2.5) catch (error:Dynamic) failed = true;
		check(failed && probe.looseCount.value == 2, "direct bound slot edits enforce the named count type");
		var typedAngle = validationDocument.defineTypedParameter("validation.angle", 10, ParameterKind.Angle, "rad");
		var typedBox = validationDocument.add(new BoxFeature(10, 10, 10));
		failed = false;
		try typedAngle.bind(typedBox.depth) catch (error:Dynamic) failed = true;
		check(failed, "typed feature bindings require compatible quantities");
		validationDocument.close();

		var parserDocument = new Document();
		parserDocument.defineTypedParameter("parser.other", 20, ParameterKind.Length, "mm");
		parserDocument.defineTypedParameter("parser.width", 10, ParameterKind.Length, "mm");
		var ratio = parserDocument.defineExpression("parser.ratio", ParameterKind.Scalar, "1",
			"parser.other / (parser.other - parser.width)");
		near(ratio.value, 2);
		failed = false;
		try new ParameterExpression("1.2.3") catch (error:Dynamic) failed = true;
		check(failed, "malformed numeric literals are rejected");
		near(parserDocument.defineExpression("parser.scientific", ParameterKind.Scalar, "1", "1.2e3").value, 1200);
		parserDocument.close();

		var encoded = DocumentCodec.encode(document);
		var restored = DocumentCodec.decode(encoded);
		near(restored.parameter("wall.height").valueIn("m"), 3);
		near(restored.parameter("wall.area").valueIn("m2"), 0.6);
		near(restored.parameter("wall.volume").valueIn("m3"), 1.8);
		near(restored.parameter("forward.earlier").valueIn("m"), 2.3);
		near(restored.result().volume(), document.result().volume());
		restored.parameter("upperLevel.elevation").set(4.5, "m");
		restored.recompute();
		near(restored.parameter("wall.height").valueIn("m"), 3.5);
		restored.close();
		document.close();

		var example = new ExpressionDrivenWall();
		near(example.wall.currentShape().bounds().get_max().get_z(), 3000);
		example.setLevels(0.5, 4);
		near(example.wall.currentShape().bounds().get_max().get_z(), 3500);
		var restoredExample = DocumentCodec.decode(DocumentCodec.encode(example.document));
		near(restoredExample.parameter("wall.height").valueIn("m"), 3.5);
		restoredExample.close();
		example.close();
	}
}
