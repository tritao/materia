import CadKit;
import cadkit.sheetmetal.FlangePosition;
import cadkit.sheetmetal.LengthSpec;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetFlange;
import cadkit.sheetmetal.SheetSolid;
import cadkit.sheetmetal.WallEdge;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.WallOutlineEditor;

/** Exact flange alignment, source isolation and bend attachment identity under contour edits. */
class SheetFlangeGeometrySmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(actual:Float, expected:Float, label:String):Void {
		check(Math.isFinite(actual) && Math.abs(actual - expected) < 1e-7, label + ": " + actual + " != " + expected);
	}
	static function base(width:Float, height:Float):Sheet {
		var result = new Sheet(SheetDemoParts.materialA());
		var wall = result.addWall(WallOutline.rectangle(width, height));
		return result;
	}
	public static function run():Int {
		var source = base(80, 100);
		var positions = [FlangePosition.MaterialInside, FlangePosition.MaterialOutside, FlangePosition.BendOutside, FlangePosition.Offset];
		var outside = [80.0, 81.5, 82.5, 85.5];
		for (i in 0...positions.length) {
			var graph = SheetFlange.add(source, 1, new WallEdge(1), 40, Math.PI / 2, 1, LengthSpec.OuterSharp, positions[i], 3);
			near(graph.evaluate(graph.folded()).bounds().maxX, outside[i], "outside face follows the chosen alignment");
			near(graph.wall(2).outline.segment(0, 0).length(), 37.5, "drawing dimension derives the straight leg");
			check(graph.wall(2).id == 2 && graph.bend(1).id == 1, "fixed flange outputs have permanent monotonic identities");
			var solid = SheetSolid.build(graph, graph.folded());
			check(CadKit.shapeValidChecked(solid.borrowHandle()) != 0, "aligned flange builds a valid solid");
			solid.close();
		}
		near(source.wall(1).outline.area(), 8000, "flange construction leaves the source graph untouched");
		check(source.walls().length == 1 && source.bends().length == 0, "flange output allocation is isolated");
		for (angle in [Math.PI / 6, 3 * Math.PI / 4, -Math.PI / 2, -3 * Math.PI / 4]) {
			for (position in [FlangePosition.MaterialInside, FlangePosition.MaterialOutside]) {
				var graph = SheetFlange.add(source, 1, new WallEdge(1), 100, angle, 1, LengthSpec.Leg, position);
				var child = graph.evaluate(graph.folded()).wall(2).plane;
				var normal = child.normal;
				var concaveA = angle > 0;
				var sideB = position == FlangePosition.MaterialInside ? concaveA : !concaveA;
				var depth = sideB ? -graph.thickness() : 0.0;
				var point = child.origin.add(normal.scale(depth));
				var x = point.x - normal.z * (depth - point.z) / normal.x;
				near(x, 80, "virtual sharp faces intersect at the authored edge for either bend direction");
			}
		}
		var channel = SheetFlange.add(base(60, 120), 1, new WallEdge(1), 40, Math.PI / 2, 1,
			LengthSpec.OuterSharp, FlangePosition.MaterialInside);
		channel = SheetFlange.add(channel, 1, new WallEdge(3), 40, Math.PI / 2, 1,
			LengthSpec.OuterSharp, FlangePosition.MaterialInside);
		var bounds = channel.evaluate(channel.folded()).bounds();
		var reference = SheetDemoParts.p2().evaluate(SheetDemoParts.p2().folded()).bounds();
		near(bounds.maxX - bounds.minX, reference.maxX - reference.minX, "positioned P2 outside width");
		near(bounds.maxY - bounds.minY, reference.maxY - reference.minY, "positioned P2 length");
		near(bounds.maxZ - bounds.minZ, reference.maxZ - reference.minZ, "positioned P2 height");
		var previous = channel.wall(1).outline;
		var loop = previous.loops()[0];
		var ids = previous.identities()[0];
		var changed = channel.updateWall(1, new WallOutline([loop[2], loop[3], loop[0], loop[1]], [],
			[[ids[2], ids[3], ids[0], ids[1]]]));
		check(channel.bend(1).edgeA.identity == ids[1] && channel.bend(2).edgeA.identity == ids[3],
			"existing bend attachments follow authored identities after contour rotation");
		near(channel.evaluate(channel.folded()).bounds().maxZ, bounds.maxZ, "remapped attachments preserve folded geometry");
		var failed = false;
		var before = channel.wall(1).outline;
		var first = channel.bend(1);
		try {
			var removed = channel.updateWall(1, new WallOutline(before.loops()[0], [], [["new0", "new1", "new2", "new3"]]));
		} catch (error:Dynamic) {
			if (Std.isOfType(error, SheetDiagnosis)) { var diagnosis = Std.downcast(error, SheetDiagnosis); if (diagnosis == null) throw "unexpected error type"; failed = diagnosis.code == "sheet.bend-edge-removed"; }
		}
		check(failed, "removing a consumed edge names the failed bend attachment");
		check(channel.wall(1).outline == before && channel.bend(1) == first, "failed wall edits restore walls and bend attachments");
		var arc = Segment.arc(new Point2(0, 0), 10, 0, Math.PI);
		var diameter = Segment.line(new Point2(-10, 0), new Point2(10, 0));
		var curved = new WallOutline([arc, diameter]);
		var trimmed = WallOutlineEditor.offset(curved, new WallEdge(1), -3);
		var theta = Math.PI - 2 * Math.asin(0.3);
		near(trimmed.area(), 50 * (theta - Math.sin(theta)), "offset intersects and trims a neighbouring circle exactly");
		near(trimmed.segment(0, 0).sweep, theta, "circular neighbour retains its exact sweep");
		check(trimmed.identity(0, 0) == curved.identity(0, 0), "circular trimming preserves authored identity");
		var extended = WallOutlineEditor.offset(curved, new WallEdge(1), 3);
		near(extended.segment(0, 0).sweep, Math.PI + 2 * Math.asin(0.3), "outward offset extends exact circular neighbours");
		return checks;
	}
}
