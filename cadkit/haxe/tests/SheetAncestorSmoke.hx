import CadKit;
import cadkit.Shape;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetSolid;

/** Native adjacency retains source identity and never duplicates a periodic seam's face. */
class SheetAncestorSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	public static function run():Int {
		var graph = SheetDemoParts.p1();
		var source = SheetSolid.build(graph, graph.folded());
		var edge = source.subshape(CadKit.ShapeKind.Edge, 0);
		var parents = source.ancestors(edge, CadKit.ShapeKind.Face);
		var names = source.elementNames(CadKit.ShapeKind.Face);
		try {
			check(parents.length == 2, "a sheet solid edge has two containing faces");
			check(!parents[0].sameAs(parents[1]), "ancestor faces are distinct");
			edge.close(); source.close();
			for (face in parents) {
				check(face.kind() == CadKit.ShapeKind.Face, "ancestor owns a face after its source closes");
				var inherited = face.elementNames(CadKit.ShapeKind.Face);
				check(inherited.length == 1 && names.indexOf(inherited[0]) >= 0, "ancestor retains its source's opaque name");
				face.close();
			}
		} catch (error:Dynamic) {
			for (face in parents) face.close(); edge.close(); source.close(); throw error;
		}
		var cylinder = Shape.cylinder(2, 5);
		var seams = 0;
		try {
			for (index in 0...cylinder.subshapeCount(CadKit.ShapeKind.Edge)) {
				var member = cylinder.subshape(CadKit.ShapeKind.Edge, index);
				try {
					var faces = cylinder.ancestors(member, CadKit.ShapeKind.Face);
					try {
						if (member.curveKind() == CadKit.CurveKind.Line) {
							seams++;
							check(faces.length == 1, "a twice-used periodic seam has one unique ancestor");
							check(faces[0].surfaceKind() == CadKit.SurfaceKind.Cylinder, "periodic seam belongs to the cylinder skin");
						} else check(faces.length == 2, "cylinder cap edge joins two faces");
					} catch (error:Dynamic) { for (face in faces) face.close(); throw error; }
					for (face in faces) face.close(); member.close();
				} catch (error:Dynamic) { member.close(); throw error; }
			}
			check(seams == 1, "cylinder exposes one periodic seam"); cylinder.close();
		} catch (error:Dynamic) { cylinder.close(); throw error; }
		return checks;
	}
	static function main():Void { trace("Sheet ancestry smoke passed (" + run() + " assertions)"); }
}
