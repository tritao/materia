import cadkit.modeling.Axis;
import cadkit.modeling.BuildPart;
import cadkit.modeling.BuildSketch;
import cadkit.modeling.Locations;
import cadkit.modeling.Part;
import cadkit.modeling.Scope;
import cadkit.modeling.Selection;

/** An 80 x 50 x 6 plate, four diameter-6 holes, and radius-2 outer corners. */
class MountingPlate {
	public static function build():Part {
		return Scope.run(function(scope:Scope) {
			var sketch = scope.own(BuildSketch.build(function(b) {
				b.rectangle(80, 50);
				b.circles(3, Locations.grid(2, 2, 60, 30), Subtract);
			}));
			return BuildPart.build(function(b) {
				b.extrude(sketch, 6);
				var corners = b.edges().parallel(Axis.Z()).filter(function(edge) {
					return Math.abs(Selection.center(edge).x) > 39;
				});
				try {
					b.fillet(corners, 2);
					corners.close();
				} catch (error:Dynamic) {
					corners.close();
					throw error;
				}
			});
		});
	}

	static function main():Int {
		var plate = build();
		try {
			plate.exportStep("mounting-plate.step");
			plate.close();
		} catch (error:Dynamic) {
			plate.close();
			throw error;
		}
		return 0;
	}
}
