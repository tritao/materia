import bimkit.BimDocument;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Element;
import cadkit.parametric.ElementReference;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.Placement;

/** Four instances of one definition hosted on two level-driven walls. */
class RepeatedHostedWindows {
	public final model:BimDocument;
	public final firstWall:Element;
	public final secondWall:Element;
	public final windows:Array<InstanceElement>;
	public final upper:cadkit.parametric.LevelElement;

	public function new() {
		model = new BimDocument();
		var base = model.cad.createLevel("Ground", 0);
		upper = model.cad.createLevel("Upper", 3000);
		var baseRef = new ElementReference(model.cad.id, base.id);
		var upperRef = new ElementReference(model.cad.id, upper.id);
		firstWall = model.createLevelWall("North wall", 6000, 200, baseRef, upperRef);
		secondWall = model.createLevelWall("South wall", 6000, 200, baseRef, upperRef);
		secondWall.setPlacement(new Placement(new Plane(new Vector(0, 5000, 0), Vector.X(), Vector.Z())));
		var definition = model.cad.createWindowDefinition("Shared window", 1000, 1200, 80, 200);
		windows = [];
		for (index in 0...4) {
			var instance = model.cad.createInstance("Window " + (index + 1), definition);
			model.hostOpening(instance, index < 2 ? firstWall.id : secondWall.id, index % 2 == 0 ? 900 : 3500, 900);
			windows.push(instance);
		}
	}

	public function close():Void
		model.close();
}
