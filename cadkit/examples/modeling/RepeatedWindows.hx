import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.*;
import cadkit.parametric.features.BoxFeature;

class RepeatedWindows {
	public final document:Document;
	public final definition:Definition;
	public final first:InstanceElement;
	public final second:InstanceElement;

	public function new() {
		document = new Document();
		definition = document.createWindowDefinition("Window", 1200, 1500, 80, 100);
		var parentA = document.createElement("Facade A", document.add(new BoxFeature(10, 10, 10)));
		var parentB = document.createElement("Facade B", document.add(new BoxFeature(10, 10, 10)));
		parentB.setPlacement(new Placement(new Plane(new Vector(5000, 0, 0), Vector.X(), Vector.Z())));
		first = document.createInstance("Window A", definition);
		second = document.createInstance("Window B", definition);
		first.reparent(new ElementReference(document.id, parentA.id), false);
		second.reparent(new ElementReference(document.id, parentB.id), false);
		document.setOutput(cast parentA.output);
		document.recompute();
	}

	public function close():Void
		document.close();
}
