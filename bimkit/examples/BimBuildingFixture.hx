import bimkit.BimDocument;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Definition;
import cadkit.parametric.Element;
import cadkit.parametric.ElementReference;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.Placement;

/** Small persistent two-storey building used by BIM integration examples and tests. */
class BimBuildingFixture {
	public final model:BimDocument;
	public final project:Element;
	public final site:Element;
	public final building:Element;
	public final groundStorey:Element;
	public final upperStorey:Element;
	public final groundLevel:cadkit.parametric.LevelElement;
	public final upperLevel:cadkit.parametric.LevelElement;
	public final roofLevel:cadkit.parametric.LevelElement;
	public final groundWalls:Array<Element>;
	public final upperWalls:Array<Element>;
	public final windows:Array<InstanceElement>;
	public final doors:Array<InstanceElement>;
	public final slabs:Array<Element>;
	public final spaces:Array<Element>;
	public final windowType:Definition;
	public final doorType:Definition;

	public function new() {
		model = new BimDocument();
		project = model.createProject("Demo Project");
		site = model.createSite("Riverside Site", project.id);
		building = model.createBuilding("Studio Building", site.id);
		groundLevel = model.cad.createLevel("Ground Level", 0);
		upperLevel = model.cad.createLevel("Upper Level", 3200);
		roofLevel = model.cad.createLevel("Roof Level", 6200);
		var groundRef = new ElementReference(model.cad.id, groundLevel.id);
		var upperRef = new ElementReference(model.cad.id, upperLevel.id);
		var roofRef = new ElementReference(model.cad.id, roofLevel.id);
		groundStorey = model.createStorey("Ground Storey", building.id, groundRef, upperRef);
		upperStorey = model.createStorey("Upper Storey", building.id, upperRef, roofRef);

		spaces = [
			model.createSpace("Lobby", groundStorey.id),
			model.createSpace("Workshop", groundStorey.id),
			model.createSpace("Office", upperStorey.id)
		];
		groundWalls = [
			placedLevelWall("North Exterior Wall", 6000, 200, groundRef, upperRef, 0, 4800),
			placedLevelWall("South Exterior Wall", 6000, 200, groundRef, upperRef, 0, 0),
			placedLevelWall("Ground Interior Wall", 3000, 100, groundRef, upperRef, 1500, 2400)
		];
		upperWalls = [
			placedLevelWall("Upper North Wall", 6000, 200, upperRef, roofRef, 0, 4800),
			placedLevelWall("Upper South Wall", 6000, 200, upperRef, roofRef, 0, 0)
		];
		slabs = [
			placedSlab("Ground Slab", groundStorey.id, 6000, 5000, 200, -200),
			placedSlab("Upper Slab", upperStorey.id, 6000, 5000, 200, 3000)
		];

		windowType = model.createWindowDefinition("Standard Window", 1200, 1400, 80, 200);
		windows = [
			model.createWindow("North Window 01", windowType),
			model.createWindow("North Window 02", windowType)
		];
		for (index in 0...windows.length) {
			model.addToStorey(groundStorey.id, windows[index].id);
			model.hostOpening(windows[index], groundWalls[0].id, index == 0 ? 900 : 3600, 900);
		}

		doorType = model.createDoorDefinition("Standard Door", 900, 2100, 200);
		doors = [
			model.createDoor("Lobby Door", doorType),
			model.createDoor("Upper Door", doorType)
		];
		model.addToStorey(groundStorey.id, doors[0].id);
		model.addToStorey(upperStorey.id, doors[1].id);
		model.hostOpening(doors[0], groundWalls[2].id, 900, 0);
		model.hostOpening(doors[1], upperWalls[1].id, 3000, 0);
	}

	private function placedLevelWall(name:String, length:Float, thickness:Float, base:ElementReference, top:ElementReference,
		x:Float, y:Float):Element {
		var result = model.createLevelWall(name, length, thickness, base, top);
		result.setPlacement(new Placement(new Plane(new Vector(x, y, 0), Vector.X(), Vector.Z())));
		var storey = base.elementId.value == groundLevel.id.value ? groundStorey : upperStorey;
		model.addToStorey(storey.id, result.id);
		return result;
	}

	private function placedSlab(name:String, storeyId:cadkit.parametric.ElementId, width:Float, depth:Float, thickness:Float,
		z:Float):Element {
		var result = model.createSlab(name, width, depth, thickness);
		result.setPlacement(new Placement(new Plane(new Vector(0, 0, z), Vector.X(), Vector.Z())));
		model.addToStorey(storeyId, result.id);
		return result;
	}

	public function close():Void
		model.close();
}
