import cadkit.parametric.Document;
import cadkit.parametric.Element;
import cadkit.parametric.Feature;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.TransformFeature;

/** Small multi-output model demonstrating identity independently of regenerated geometry. */
class PersistentBuildingModel {
	public final document:Document;
	public final walls:Array<Element>;
	public final slab:Element;
	private final wallBoxes:Array<BoxFeature>;

	public function new() {
		document = new Document();
		walls = [];
		wallBoxes = [];
		addWall("North wall", 8000, 200, 3000, 0, 5800);
		addWall("South wall", 8000, 200, 3000, 0, 0);
		addWall("East wall", 200, 5600, 3000, 7800, 200);
		addWall("West wall", 200, 5600, 3000, 0, 200);
		var slabBox = document.add(new BoxFeature(8000, 6000, 200));
		slab = document.createElement("Ground slab", slabBox);
		document.setOutput(slabBox);
		document.recompute();
	}

	private function addWall(name:String, width:Float, depth:Float, height:Float, x:Float, y:Float):Void {
		var box = document.add(new BoxFeature(width, depth, height));
		var positioned:Feature = document.add(new TransformFeature(box, x, y, 200));
		wallBoxes.push(box);
		walls.push(document.createElement(name, positioned));
	}

	public function resizeWall(index:Int, width:Float):Void {
		wallBoxes[index].width.set(width);
		document.recompute();
	}

	public function replaceWallOutput(index:Int, output:Feature):Void {
		walls[index].setOutput(output);
		document.recompute();
	}

	public function duplicateWall(index:Int, name:String):Element {
		return document.duplicateElement(walls[index], name);
	}

	public function close():Void document.close();
}
