import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ParametricError;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.SketchFeature;
import cadkit.parametric.recording.DocumentBuilder;

class RecordingBuilderSmoke {
	static function check(value:Bool, message:String):Void {
		if (!value)
			throw message;
	}

	static function near(actual:Float, expected:Float):Void {
		check(Math.abs(actual - expected) < 0.00001, "recording measurement mismatch " + actual + " / " + expected);
	}

	public static function run():Void {
		var cube:Null<BoxFeature> = null;
		var document = DocumentBuilder.build(function(builder) {
			var size = builder.dimension("cube.size", 5);
			var feature = builder.box(size, size, size);
			cube = feature;
			builder.output(feature);
			// Output selection is explicit and need not be the last recorded feature.
			builder.polyline([new Vector(), new Vector(1, 0, 0)]);
		});
		var cubeFeature:BoxFeature = cast cube;
		check(document.namedParameters().length == 1, "one shared dimension");
		check(document.parameter("cube.size").bindings().length == 3, "shared dimension bindings");
		near(document.result().volume(), 125);
		document.parameter("cube.size").set(7);
		check(cubeFeature.width.value == 7 && cubeFeature.depth.value == 7 && cubeFeature.height.value == 7, "shared direct edit propagation");
		document.recompute();
		near(document.result().volume(), 343);
		check(document.undo(), "shared dimension undo");
		document.recompute();
		near(document.parameter("cube.size").value, 5);
		near(document.result().volume(), 125);
		check(document.redo(), "shared dimension redo");
		document.recompute();
		near(document.result().volume(), 343);
		cubeFeature.width.set(8);
		check(cubeFeature.depth.value == 8 && cubeFeature.height.value == 8, "bound feature edit propagation");
		document.recompute();
		near(document.result().volume(), 512);
		check(document.undo(), "bound feature edit undo");
		document.recompute();
		near(document.result().volume(), 343);
		check(document.redo(), "bound feature edit redo");
		document.recompute();
		near(document.result().volume(), 512);

		var loaded = DocumentCodec.decode(DocumentCodec.encode(document));
		check(loaded.parameter("cube.size").bindings().length == 3, "shared binding persistence");
		check(loaded.outputFeature().id.toInt() == 1, "output persistence");
		loaded.parameter("cube.size").set(9);
		loaded.recompute();
		near(loaded.result().volume(), 729);
		check(loaded.undo(), "loaded named edit undo");
		loaded.recompute();
		near(loaded.result().volume(), 512);
		loaded.close();
		document.close();

		var plate = new EditableMountingPlate();
		var encoded = DocumentCodec.encode(plate.document);
		var restored = DocumentCodec.decode(encoded);
		check(restored.namedParameters().length == 7, "plate dimensions persisted");
		var restoredOutline:SketchFeature = cast restored.featureAt(0);
		var restoredFinish:FilletFeature = cast restored.outputFeature();
		restored.parameter("plate.width").set(100);
		restored.parameter("plate.depth").set(60);
		restored.parameter("holes.spacingX").set(70);
		restored.parameter("holes.spacingY").set(40);
		restored.recompute();
		near(restoredOutline.width.value, 100);
		check(restoredFinish.selection != null && restoredFinish.selection.expectedCount == 4, "recorded selector persistence");
		near(restored.result().volume(), (100 * 60 - (4 - Math.PI) * 4 - 36 * Math.PI) * 6);
		check(restored.undo(), "last named edit undo");
		restored.recompute();
		near(restored.parameter("holes.spacingY").value, 30);
		restored.close();
		plate.close();

		var rejected = false;
		var builder = new DocumentBuilder();
		try {
			builder.unsupported("variable-radius fillet");
		} catch (error:Dynamic) {
			var failure:ParametricError = cast error;
			rejected = StringTools.contains(failure.message, "cannot be recorded");
		}
		check(rejected, "unsupported operation rejection");
		builder.close();

		rejected = false;
		try {
			DocumentBuilder.build(function(value) {
				value.dimension("unused", 1);
			});
		} catch (error:Dynamic) {
			rejected = true;
		}
		check(rejected, "unbound dimension rejection");

		var failedBuilder:Null<DocumentBuilder> = null;
		rejected = false;
		try {
			DocumentBuilder.build(function(value) {
				failedBuilder = value;
				var amount = value.dimension("amount", 5);
				var radius = value.dimension("radius", 2);
				var circle = value.circle(radius);
				value.extrude(circle, amount, Vector.X());
			});
		} catch (error:Dynamic) {
			rejected = true;
		}
		check(rejected, "unsupported recorded direction rejection");
		var closedRejected = false;
		try {
			var closedBuilder:DocumentBuilder = cast failedBuilder;
			closedBuilder.dimension("late", 1);
		} catch (error:Dynamic) {
			closedRejected = true;
		}
		check(closedRejected, "failed builder cleanup");
	}
}
