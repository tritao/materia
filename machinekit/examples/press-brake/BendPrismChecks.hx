import cadkit.modeling.Vector;
import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.BendZone;
import cadkit.sheetmetal.KFactorTable;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.process.AirBending;
import cadkit.sheetmetal.process.ProcessTooling;
import machinekit.pressbrake.planning.BendPrismProfile;
import machinekit.pressbrake.planning.ConvexPrism;
import machinekit.pressbrake.planning.CoalPrismQuery;

/** Bend pieces follow exact FK datums and preserve developed holes through bounded curved mapping. */
class BendPrismChecks {
	static var checks:Int = 0;
	public static function main():Void run();
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function sheet(angle:Float, radius:Float):Sheet {
		var stock = new SheetMaterial("bend-collision-steel", 1.5, BendRule.KFactor(new KFactorTable([0], [0.5])),
			{uts: 400, yield: 250, youngs: 205000});
		var result = new Sheet(stock), root = result.addWall(WallOutline.rectangle(50, 100));
		var child = SheetDemoParts.flange(result, root, 1, 40, angle, radius);
		return result;
	}
	static function volume(profile:BendPrismProfile):Float {
		var result = 0.0;
		for (piece in profile.pieces()) {
			var vertices = piece.vertices(), count = Std.int(vertices.length / 2), area = new Vector();
			for (i in 0...count) area = area.add(vertices[i].subtract(vertices[0]).cross(vertices[(i + 1) % count].subtract(vertices[0])));
			result += Math.abs(area.dot(piece.extrusion)) / 2;
		}
		return result;
	}
	static function probe(point:Vector):ConvexPrism {
		var x = point.x, y = point.y, z = point.z;
		return new ConvexPrism("probe", [new Vector(x - 0.01, y - 0.01, z - 0.01),
			new Vector(x + 0.01, y - 0.01, z - 0.01), new Vector(x + 0.01, y + 0.01, z - 0.01),
			new Vector(x - 0.01, y + 0.01, z - 0.01)], new Vector(0, 0, 0.02));
	}
	static function overlaps(profile:BendPrismProfile, point:Vector):Bool {
		var query = new CoalPrismQuery(), target = probe(point);
		for (piece in profile.pieces()) if (query.overlap(piece, target)) return true;
		return false;
	}
	static function uncut(profile:BendPrismProfile, zone:BendZone):Void {
		check(profile.error <= 0.01 + 1e-10, "Bend curvature error stays within its requested bound");
		var expected = zone.allowance * zone.thickness * zone.width;
		if (zone.angle != 0) {
			var delta = Math.abs(zone.angle) / profile.pieces().length;
			expected *= Math.sin(delta) / delta;
		}
		check(Math.abs(volume(profile) - expected) < 1e-5, "Independent polygonal annular-sector volume");
		check(overlaps(profile, zone.point(0.3, 50, zone.thickness / 2)), "Material probe follows the bend FK");
	}
	public static function run():Void {
		checks = 0;
		for (sign in [1.0, -1.0]) {
			var graph = sheet(sign * Math.PI / 2, 2), bend = graph.bends()[0];
			for (fraction in [0.0, 1e-8, 0.4, 1.0]) {
				var zone = graph.evaluate(graph.flat().set(bend.id, bend.angle * fraction)).bend(bend.id);
				uncut(BendPrismProfile.of(bend, zone), zone);
			}
			var tooling = new ProcessTooling("acute", "v12", 0.5, Math.PI / 6, 12, Math.PI / 6, 0.5);
			var prediction = new AirBending().predict(graph.material, tooling, bend.angle, 100);
			var loadedBend = graph.updateBend(bend.id, bend.angle, prediction.innerRadius);
			var loadedZone = graph.evaluateStroke(graph.flat().set(bend.id, prediction.loadedAngle), bend.id, prediction).bend(bend.id);
			uncut(BendPrismProfile.of(loadedBend, loadedZone), loadedZone);
			var allowance = graph.bendAllowance(loadedBend);
			var hole = WallOutline.polygon([new Point2(allowance * 0.4, 40), new Point2(allowance * 0.6, 40),
				new Point2(allowance * 0.6, 60), new Point2(allowance * 0.4, 60)]);
			var cutBend = graph.cutBend(bend.id, hole, "collision-hole");
			for (fraction in [0.0, 0.4, 1.0]) {
				var zone = graph.evaluate(graph.flat().set(bend.id, bend.angle * fraction)).bend(bend.id);
				var profile = BendPrismProfile.of(cutBend, zone);
				check(profile.error <= 0.01 + 1e-8, "Cut mapping error includes curvature and varying thickness normal");
				check(!overlaps(profile, zone.point(0.5, 50, zone.thickness / 2)), "Developed hole stays empty after folding");
				check(overlaps(profile, zone.point(0.2, 50, zone.thickness / 2)), "Uncut bend material remains present");
			}
			var hem = sheet(sign * Math.PI, 0), hemBend = hem.bends()[0];
			var hemZone = hem.evaluate(hem.folded()).bend(hemBend.id);
			uncut(BendPrismProfile.of(hemBend, hemZone), hemZone);
			var hemAllowance = hem.bendAllowance(hemBend);
			var cutHem = hem.cutBend(hemBend.id, WallOutline.polygon([
				new Point2(hemAllowance * 0.4, 40), new Point2(hemAllowance * 0.6, 40),
				new Point2(hemAllowance * 0.6, 60), new Point2(hemAllowance * 0.4, 60)]), "hem-collision-hole");
			var cutHemProfile = BendPrismProfile.of(cutHem, hemZone);
			check(cutHemProfile.error <= 0.01 + 1e-8, "Zero-radius hem cut mapping stays bounded");
			check(!overlaps(cutHemProfile, hemZone.point(0.5, 50, hemZone.thickness / 2)), "Cut hem hole remains empty");
			check(overlaps(cutHemProfile, hemZone.point(0.2, 50, hemZone.thickness / 2)), "Cut hem material remains present");
			var rounded = sheet(sign * Math.PI / 2, 2), roundedBend = rounded.bends()[0];
			var roundedAllowance = rounded.bendAllowance(roundedBend);
			var roundedCut = rounded.cutBend(roundedBend.id, new WallOutline([
				Segment.arc(new Point2(roundedAllowance / 2, 50), roundedAllowance / 10, 0, 2 * Math.PI)]), "round-collision-hole");
			var roundedZone = rounded.evaluate(rounded.folded()).bend(roundedBend.id);
			var roundedProfile = BendPrismProfile.of(roundedCut, roundedZone);
			check(roundedProfile.error <= 0.01 + 1e-8, "Circular cut includes developed and mapped error");
			check(!overlaps(roundedProfile, roundedZone.point(0.5, 50, roundedZone.thickness / 2)), "Circular developed cut stays empty");
			check(overlaps(roundedProfile, roundedZone.point(0.2, 50, roundedZone.thickness / 2)), "Material beside circular cut remains present");
		}
		trace("Bend prism checks passed (" + checks + " assertions)");
	}
}
