import machinekit.pressbrake.ToolProfilePoint;
import machinekit.pressbrake.ToolProfilePolygon;

/** Sections retain the empty relief when decomposed for collision queries. */
class ToolProfileTests {
	public static function main():Void run();

	public static function run():Void {
		var points = [new ToolProfilePoint(0, 0), new ToolProfilePoint(4, 0),
			new ToolProfilePoint(4, 1), new ToolProfilePoint(1, 1),
			new ToolProfilePoint(1, 4), new ToolProfilePoint(0, 4)];
		for (reverse in [false, true]) {
			var input = points.copy();
			if (reverse) input.reverse();
			var section = new ToolProfilePolygon(input);
			if (Math.abs(section.area - 7) > 1e-9) throw "Relieved section area";
			input.pop();
			var borrowed = section.points();
			borrowed.pop();
			if (section.points().length != 6) throw "Section owns its outline";
			var sum = 0.0;
			var pieces = section.convexPieces();
			if (pieces.length != 4) throw "Relieved section triangle count";
			for (piece in pieces) {
				var a = piece[0], b = piece[1], c = piece[2];
				var area = ((b.y - a.y) * (c.z - a.z) - (b.z - a.z) * (c.y - a.y)) / 2;
				if (!(area > 0)) throw "Convex piece has positive orientation";
				sum += area;
				var y = (a.y + b.y + c.y) / 3, z = (a.z + b.z + c.z) / 3;
				if (y > 1 && z > 1) throw "Convex piece fills the relief";
			}
			if (Math.abs(sum - 7) > 1e-9) throw "Convex pieces conserve section area";
		}
		reject([new ToolProfilePoint(0, 0), new ToolProfilePoint(4, 4), new ToolProfilePoint(0, 4), new ToolProfilePoint(3, 0)], "crossed outline");
		reject([new ToolProfilePoint(0, 0), new ToolProfilePoint(1, 0), new ToolProfilePoint(2, 0)], "zero area");
		reject([new ToolProfilePoint(0, 0), new ToolProfilePoint(1, 0), new ToolProfilePoint(1, 0), new ToolProfilePoint(0, 1)], "duplicate point");
		var collinear = new ToolProfilePolygon([new ToolProfilePoint(0, 0), new ToolProfilePoint(2, 0),
			new ToolProfilePoint(4, 0), new ToolProfilePoint(4, 4), new ToolProfilePoint(0, 4)]);
		var covered = 0.0;
		for (piece in collinear.convexPieces()) {
			var triangle = new ToolProfilePolygon(piece);
			covered += triangle.area;
		}
		if (Math.abs(covered - 16) > 1e-9) throw "Collinear boundary keeps exact coverage";
		reject([new ToolProfilePoint(0, 0), new ToolProfilePoint(4, 0), new ToolProfilePoint(4, 4),
			new ToolProfilePoint(2, 0), new ToolProfilePoint(0, 4)], "boundary touches another edge");
		trace("Tool profile tests passed");
	}

	static function reject(points:Array<ToolProfilePoint>, label:String):Void {
		var rejected = false;
		try {
			var section = new ToolProfilePolygon(points);
		} catch (_:Dynamic) rejected = true;
		if (!rejected) throw 'Tool profile accepted $label';
	}
}
