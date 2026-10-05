import machinekit.pressbrake.DieProfile;
import machinekit.pressbrake.DieGroove;
import machinekit.pressbrake.ToolingCatalogue;
import machinekit.pressbrake.PunchProfile;
import machinekit.pressbrake.PunchKind;
import machinekit.pressbrake.ToolProfilePrism;
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
		var prism = new ToolProfilePrism(new ToolProfilePolygon(points), 17, 117);
		var solid = prism.geometry();
		if (CadKit.shapeValidChecked(solid.shape.borrowHandle()) == 0) throw "Tool prism is invalid";
		if (Math.abs(solid.volume() - 700) > 1e-6) throw "Tool prism volume";
		var box = solid.shape.bounds();
		if (Math.abs(box.get_min().get_x() - 17) > 1e-6 || Math.abs(box.get_max().get_x() - 117) > 1e-6)
			throw "Tool prism bed placement";
		if (Math.abs(box.get_min().get_y()) > 1e-6 || Math.abs(box.get_max().get_z() - 4) > 1e-6)
			throw "Tool prism section axes";
		solid.close();
		var totalVolume = 0.0;
		for (piece in prism.convexGeometry()) {
			if (CadKit.shapeValidChecked(piece.shape.borrowHandle()) == 0) throw "Convex tool prism is invalid";
			totalVolume += piece.volume();
			piece.close();
		}
		if (Math.abs(totalVolume - 700) > 1e-6) throw "Convex tool prisms conserve volume";
		// A 90-degree, R1 nose gives a 90-degree circular segment below the tangent chord.
		var tangentZ = 1 - Math.sqrt(0.5), tangentY = Math.sqrt(0.5);
		var upperY = tangentY + 10 - tangentZ;
		var upper = new ToolProfilePolygon([new ToolProfilePoint(-tangentY, tangentZ),
			new ToolProfilePoint(tangentY, tangentZ), new ToolProfilePoint(upperY, 10), new ToolProfilePoint(-upperY, 10)]);
		var punch = new PunchProfile("test-r1-90", Straight, 1, Math.PI / 2, 10, upper);
		if (Math.abs(punch.tangentHeight() - tangentZ) > 1e-9 || Math.abs(punch.tangentWidth() - tangentY) > 1e-9)
			throw "Punch circular contact tangents";
		var noseArea = (Math.PI / 2 - 1) / 2;
		var punchSolid = punch.geometry(5, 105);
		if (CadKit.shapeValidChecked(punchSolid.shape.borrowHandle()) == 0) throw "Rounded punch is invalid";
		if (Math.abs(punchSolid.volume() - 100 * (upper.area + noseArea)) > 1e-5)
			throw "Rounded punch exact circular segment volume";
		var punchBox = punchSolid.shape.bounds();
		if (Math.abs(punchBox.get_min().get_z()) > 1e-6 || Math.abs(punchBox.get_max().get_z() - 10) > 1e-6)
			throw "Punch tip and mounting datums";
		punchSolid.close();
		var badTangents = false;
		try {
			var invalid = new PunchProfile("bad-tangent", Straight, 1, Math.PI / 2, 10,
				new ToolProfilePolygon([new ToolProfilePoint(-tangentY, tangentZ), new ToolProfilePoint(tangentY, tangentZ),
					new ToolProfilePoint(1, 10), new ToolProfilePoint(-1, 10)]));
		} catch (_:Dynamic) badTangents = true;
		if (!badTangents) throw "Punch accepted non-tangent sides";
		for (angle in [Math.PI / 6, 88 * Math.PI / 180, Math.PI / 2]) {
			var z = 1 - Math.sin(angle / 2), y = Math.cos(angle / 2);
			var top = y + (10 - z) * Math.tan(angle / 2);
			var profile = new PunchProfile("cover-check", Straight, 1, angle, 10,
				new ToolProfilePolygon([new ToolProfilePoint(-y, z), new ToolProfilePoint(y, z),
					new ToolProfilePoint(top, 10), new ToolProfilePoint(-top, 10)]));
			for (error in [0.1, 0.01, 0.001]) {
				var cover = profile.collisionSection(error);
				var outline = cover.points();
				for (point in outline) if (point.z < z) {
					var distance = Math.sqrt(point.y * point.y + (point.z - 1) * (point.z - 1));
					if (distance < 1 - 1e-9 || distance > 1 + error + 1e-9)
						throw "Contact cover exceeds its outward error bound";
				}
				for (i in 1...100) {
					var theta = -Math.PI + angle / 2 + (Math.PI - angle) * i / 100;
					if (!inside(outline, new ToolProfilePoint(Math.cos(theta), 1 + Math.sin(theta))))
						throw "Contact cover omits a point of the exact nose";
				}
				var area = 0.0;
				for (piece in cover.convexPieces()) area += new ToolProfilePolygon(piece).area;
				if (Math.abs(area - cover.area) > 1e-8) throw "Contact cover decomposition area";
			}
		}
		var catalogue = ToolingCatalogue.punches();
		if (catalogue.length != 3) throw "Reference punch catalogue rows";
		for (profile in catalogue) {
			if (Math.abs(profile.tipRadius - 0.6) > 1e-9) throw "Reference punch radius";
			var part = profile.geometry(0, 100);
			if (CadKit.shapeValidChecked(part.shape.borrowHandle()) == 0) throw "Catalogue punch geometry";
			part.close();
			var cover = profile.collisionSection(0.01);
			var sum = 0.0;
			for (piece in cover.convexPieces()) sum += new ToolProfilePolygon(piece).area;
			if (Math.abs(sum - cover.area) > 1e-7) throw "Catalogue punch convex coverage";
			if (profile.kind == Gooseneck && inside(cover.points(), new ToolProfilePoint(0, 50)))
				throw "Gooseneck collision cover fills its relief";
		}
		var sharpDie = new DieGroove(12, Math.PI / 2, 0);
		if (Math.abs(sharpDie.depth() - 6) > 1e-9) throw "Sharp V12 depth";
		var roundedDie = new DieGroove(12, Math.PI / 2, 1);
		if (Math.abs(roundedDie.tangentY() - (6 - Math.sqrt(0.5))) > 1e-9
			|| Math.abs(roundedDie.tangentZ() - (Math.sqrt(0.5) - 1)) > 1e-9
			|| Math.abs(roundedDie.depth() - (7 - Math.sqrt(2))) > 1e-9)
			throw "Rounded die tangents and depth";
		var consumed = false;
		try { var invalid = new DieGroove(1, Math.PI / 2, 1); }
		catch (_:Dynamic) consumed = true;
		if (!consumed) throw "Die accepted overlapping shoulders";
		for (radius in [0.0, 1.0]) {
			var die = new DieProfile("test-v12", new DieGroove(12, Math.PI / 2, radius), 30, 20);
			var dieSolid = die.geometry(13, 113);
			if (CadKit.shapeValidChecked(dieSolid.shape.borrowHandle()) == 0) throw "Exact die shoulders produce a valid solid";
			if (radius == 0 && Math.abs(dieSolid.volume() - 56400) > 1e-5) throw "Sharp die independent volume";
			if (radius > 0 && !(dieSolid.volume() > 56400)) throw "Rounded die removes less material than sharp V";
			var dieBox = dieSolid.shape.bounds();
			if (Math.abs(dieBox.get_min().get_z() + 20) > 1e-6 || Math.abs(dieBox.get_max().get_z()) > 1e-6)
				throw "Die mounting and top datums";
			var exactArea = dieSolid.volume() / 100;
			for (error in [0.1, 0.01, 0.001]) {
				var cover = die.collisionSection(error);
				if (cover.area < exactArea - 1e-8) throw "Die collision cover loses material";
				var sum = 0.0;
				for (piece in cover.convexPieces()) sum += new ToolProfilePolygon(piece).area;
				if (Math.abs(sum - cover.area) > 1e-7) throw "Die collision decomposition area";
				if (radius > 0) for (i in 0...101) {
					var theta = Math.PI / 2 + Math.PI / 4 * i / 100;
					if (!inside(cover.points(), new ToolProfilePoint(6 + Math.cos(theta), -1 + Math.sin(theta))))
						throw "Die collision cover omits shoulder contact";
				}
			}
			dieSolid.close();
		}
		var dies = ToolingCatalogue.dies();
		if (dies.length != 4) throw "Reference single-V catalogue rows";
		var openings = [8, 12, 16, 22];
		for (i in 0...dies.length) {
			if (dies[i].groove.vWidth != openings[i]) throw "Reference die openings";
			var part = dies[i].geometry(0, 100);
			if (CadKit.shapeValidChecked(part.shape.borrowHandle()) == 0) throw "Reference die native geometry";
			part.close();
		}
		var fourV = ToolingCatalogue.fourV();
		var expectedVolume = -3 * 60 * 60 * 100.0;
		for (face in 0...4) {
			var single = new DieProfile("independent-face", fourV.groove(face), 60, 60).geometry(0, 100);
			expectedVolume += single.volume();
			single.close();
		}
		for (face in 0...4) {
			if (fourV.groove(face).vWidth != openings[face]) throw "Four-V selected contact opening";
			var part = fourV.geometry(0, 100, face);
			if (CadKit.shapeValidChecked(part.shape.borrowHandle()) == 0) throw "Four-V block native validity";
			if (Math.abs(part.volume() - expectedVolume) > 1e-4) throw "Four-V block preserves every groove after rotation";
			var cover = fourV.collisionSection(0.01, face);
			if (cover.area < part.volume() / 100 - 1e-7) throw "Four-V collision cover loses material";
			if (inside(cover.points(), new ToolProfilePoint(0, -0.1))) throw "Four-V active groove filled by collision cover";
			var sum = 0.0;
			for (piece in cover.convexPieces()) sum += new ToolProfilePolygon(piece).area;
			if (Math.abs(sum - cover.area) > 1e-6) throw "Four-V convex pieces conserve area";
			part.close();
		}
		trace("Tool profile tests passed");
	}

	static function inside(points:Array<ToolProfilePoint>, p:ToolProfilePoint):Bool {
		var contained = false;
		for (i in 0...points.length) {
			var a = points[i], b = points[(i + 1) % points.length];
			var cross = (b.y - a.y) * (p.z - a.z) - (b.z - a.z) * (p.y - a.y);
			if (Math.abs(cross) < 1e-10 && p.y >= Math.min(a.y, b.y) - 1e-10
				&& p.y <= Math.max(a.y, b.y) + 1e-10 && p.z >= Math.min(a.z, b.z) - 1e-10
				&& p.z <= Math.max(a.z, b.z) + 1e-10) return true;
			if ((a.z > p.z) != (b.z > p.z)) {
				var crossing = a.y + (p.z - a.z) * (b.y - a.y) / (b.z - a.z);
				if (p.y < crossing) contained = !contained;
			}
		}
		return contained;
	}

	static function reject(points:Array<ToolProfilePoint>, label:String):Void {
		var rejected = false;
		try {
			var section = new ToolProfilePolygon(points);
		} catch (_:Dynamic) rejected = true;
		if (!rejected) throw 'Tool profile accepted $label';
	}
}
