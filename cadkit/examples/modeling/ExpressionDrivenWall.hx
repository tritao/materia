import cadkit.parametric.Document;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.ParametricError;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.recording.DocumentBuilder;

/** Wall height derived from upper and base level elevations in explicit units. */
class ExpressionDrivenWall {
	public final document:Document;
	public var wall(default, null):BoxFeature;

	public function new() {
		document = DocumentBuilder.build(function(builder) {
			builder.length("baseLevel.elevation", 0, "m");
			builder.length("upperLevel.elevation", 3, "m");
			var width = builder.length("wall.width", 4, "m");
			var depth = builder.length("wall.depth", 20, "cm");
			var height = builder.expression("wall.height", ParameterKind.Length, "m",
				"upperLevel.elevation - baseLevel.elevation");
			wall = builder.box(width, depth, height);
			builder.output(wall);
		});
	}

	public function setLevels(baseMetres:Float, upperMetres:Float):Void {
		if (!Math.isFinite(baseMetres) || !Math.isFinite(upperMetres) || upperMetres <= baseMetres)
			throw new ParametricError("upper level must be above the base level");
		var transaction = document.beginTransaction();
		try {
			document.parameter("baseLevel.elevation").set(baseMetres, "m");
			document.parameter("upperLevel.elevation").set(upperMetres, "m");
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
