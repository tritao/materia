import bimkit.BimDocument;

/** Minimal straight-wall hosting example. */
class HostedWall {
	public static function build():BimDocument {
		var model = new BimDocument();
		var wall = model.createWall("Exterior wall", 6000, 200, 3000);
		var window = model.createWindowDefinition("Window", 1200, 1500, 80, 200);
		var instance = model.cad.createInstance("Window 1", window);
		model.hostOpening(instance, wall.id, 1800, 900);
		return model;
	}
}
