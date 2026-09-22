import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.ParametricError;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.CylinderFeature;
import cadkit.parametric.features.PolarPatternFeature;
import cadkit.parametric.recording.DocumentBuilder;

/** Circular flange with an editable full-circle bolt-hole pattern. */
class ParametricFlange {
	public final document:Document;
	public var body(default, null):CylinderFeature;
	public var holes(default, null):PolarPatternFeature;
	public var finish(default, null):BooleanFeature;

	public function new() {
		document = DocumentBuilder.build(function(builder) {
			var radius = builder.dimension("flange.radius", 30);
			var thickness = builder.dimension("flange.thickness", 6);
			var holeRadius = builder.dimension("bolts.hole-radius", 2);
			var count = builder.dimension("bolts.count", 6);
			var circleRadius = builder.dimension("bolts.circle-radius", 20);
			var span = builder.dimension("bolts.angular-span", 2 * Math.PI);

			body = builder.cylinder(radius, thickness);
			var hole = builder.cylinder(holeRadius, thickness);
			holes = builder.polarPattern(hole, count, circleRadius, span,
				new Vector(), Vector.Z(), Vector.X());
			finish = builder.subtract(body, holes);
			builder.output(finish);
		});
	}

	public function resize(flangeRadius:Float, boltCount:Int, boltCircleRadius:Float):Void {
		if (flangeRadius <= 0 || boltCount < 1 || boltCircleRadius <= 0
			|| boltCircleRadius + document.parameter("bolts.hole-radius").value >= flangeRadius)
			throw new ParametricError("bolt holes must remain inside the flange");
		var transaction = document.beginTransaction();
		try {
			document.parameter("flange.radius").set(flangeRadius);
			document.parameter("bolts.count").set(boltCount);
			document.parameter("bolts.circle-radius").set(boltCircleRadius);
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
