package app;

import bimkit.BimDocument;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.ElementReference;
import cadkit.parametric.Placement;

/** Small editable BIM sample shown by the app's spatial editor panel. */
class BimEditorDemo {
	public static function create():BimDocument {
		var model = new BimDocument();
		var project = model.createProject("Studio Project");
		var site = model.createSite("Demo Site", project.id);
		var building = model.createBuilding("Two Storey Studio", site.id);
		var groundLevel = model.cad.createLevel("Ground Level", 0);
		var upperLevel = model.cad.createLevel("Upper Level", 3200);
		var roofLevel = model.cad.createLevel("Roof Level", 6200);
		var groundRef = new ElementReference(model.cad.id, groundLevel.id);
		var upperRef = new ElementReference(model.cad.id, upperLevel.id);
		var roofRef = new ElementReference(model.cad.id, roofLevel.id);
		var ground = model.createStorey("Ground Storey", building.id, groundRef, upperRef);
		var upper = model.createStorey("Upper Storey", building.id, upperRef, roofRef);
		model.createSpace("Lobby", ground.id);
		model.createSpace("Office", upper.id);
		var north = model.createLevelWall("North Exterior Wall", 6000, 200, groundRef, upperRef);
		north.setPlacement(new Placement(new Plane(new Vector(0, 4800, 0), Vector.X(), Vector.Z())));
		model.addToStorey(ground.id, north.id);
		var partition = model.createLevelWall("Interior Partition", 3000, 100, groundRef, upperRef);
		partition.setPlacement(new Placement(new Plane(new Vector(1500, 2400, 0), Vector.X(), Vector.Z())));
		model.addToStorey(ground.id, partition.id);
		var upperWall = model.createLevelWall("Upper South Wall", 6000, 200, upperRef, roofRef);
		upperWall.setPlacement(new Placement(new Plane(new Vector(0, 0, 0), Vector.X(), Vector.Z())));
		model.addToStorey(upper.id, upperWall.id);
		var groundSlab = model.createSlab("Ground Slab", 6000, 5000, 200);
		groundSlab.setPlacement(new Placement(new Plane(new Vector(0, 0, -200), Vector.X(), Vector.Z())));
		model.addToStorey(ground.id, groundSlab.id);
		var upperSlab = model.createSlab("Upper Slab", 6000, 5000, 200);
		upperSlab.setPlacement(new Placement(new Plane(new Vector(0, 0, 3000), Vector.X(), Vector.Z())));
		model.addToStorey(upper.id, upperSlab.id);
		var windowType = model.createWindowDefinition("Standard Window", 1200, 1400, 80, 200);
		var firstWindow = model.createWindow("North Window 01", windowType);
		var secondWindow = model.createWindow("North Window 02", windowType);
		model.addToStorey(ground.id, firstWindow.id);
		model.addToStorey(ground.id, secondWindow.id);
		model.hostOpening(firstWindow, north.id, 900, 900);
		model.hostOpening(secondWindow, north.id, 3600, 900);
		var doorType = model.createDoorDefinition("Standard Door", 900, 2100, 200);
		var door = model.createDoor("Lobby Door", doorType);
		model.addToStorey(ground.id, door.id);
		model.hostOpening(door, partition.id, 900, 0);
		return model;
	}
}
