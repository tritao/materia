import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.ParametricError;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.CylinderFeature;
import cadkit.parametric.features.HoleFeature;
import cadkit.parametric.features.PolarPatternFeature;
import cadkit.parametric.recording.DocumentBuilder;

/** Resizable flange with a face-attached polar pattern of through countersunk holes. */
class CountersunkFlange {
	public final document:Document;
	public var flange(default, null):CylinderFeature;
	public var hole(default, null):HoleFeature;
	public var holes(default, null):PolarPatternFeature;
	public var finish(default, null):BooleanFeature;

	public function new() {
		document = DocumentBuilder.build(function(builder) {
			var radius = builder.dimension("flange.radius", 30);
			var thickness = builder.dimension("flange.thickness", 8);
			var localX = builder.dimension("holes.local-x", 0);
			var localY = builder.dimension("holes.local-y", 0);
			var boreDiameter = builder.dimension("holes.bore-diameter", 4);
			var mouthDiameter = builder.dimension("holes.mouth-diameter", 10);
			var includedAngle = builder.dimension("holes.included-angle", Math.PI / 2);
			var count = builder.dimension("holes.count", 6);
			var circleRadius = builder.dimension("holes.circle-radius", 20);
			var span = builder.dimension("holes.angular-span", 2 * Math.PI);

			flange = builder.cylinder(radius, thickness);
			hole = builder.throughCountersink(flange,
				new SelectionRecipe("face", "plane", Vector.Z(), "max", Vector.Z(), 1), Vector.X(),
				localX, localY, boreDiameter, mouthDiameter, includedAngle);
			holes = builder.polarPattern(hole, count, circleRadius, span,
				new Vector(), Vector.Z(), Vector.X(), false);
			finish = builder.subtract(flange, holes);
			builder.output(finish);
		});
	}

	public function resize(flangeRadius:Float, thickness:Float, holeCount:Int, circleRadius:Float):Void {
		if (flangeRadius <= 0 || thickness <= 3 || holeCount < 1 || circleRadius <= 0 || circleRadius + 5 >= flangeRadius)
			throw new ParametricError("countersunk holes must remain inside a sufficiently thick flange");
		var transaction = document.beginTransaction();
		try {
			document.parameter("flange.radius").set(flangeRadius);
			document.parameter("flange.thickness").set(thickness);
			document.parameter("holes.count").set(holeCount);
			document.parameter("holes.circle-radius").set(circleRadius);
			document.recompute();
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
		transaction.commit();
	}

	public function close():Void {
		document.close();
	}
}
