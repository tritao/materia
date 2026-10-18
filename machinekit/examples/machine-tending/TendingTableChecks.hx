import machinekit.structural.TSlotTable;
import machinekit.component.ComponentDetail;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;

/** Geometry and collision coverage for the two-bank table integration. */
class TendingTableChecks {
	static function check(ok:Bool, message:String):Void { if (!ok) throw message; }
	public static function main():Void {
		var table = new TSlotTable(540, 330, 1020, "HFS5-4040", 10, 100);
		check(table.memberCuts().length == 12, "Four posts and two four-sided rail levels");
		check(Math.abs(table.cutList()[0].totalLength - 6360) < 1e-8, "Butt joints produce correct stock lengths");
		check(table.cutList().length == 1 && table.cutList()[0].quantity == 12, "Extrusion cut-list aggregation");
		check(table.components().length == 33, "Members, sixteen brackets, feet and top");
		check(table.collisionPieces().length == 49, "Every member, bracket arm, foot and top has a hull");
		for (piece in table.collisionPieces()) {
			check(piece.vertices.length == 24, "Box hull vertices");
			for (i in 0...8) {
				var x = piece.frame.x + piece.vertices[i*3];
				var y = piece.frame.y + piece.vertices[i*3+1];
				var z = piece.frame.z + piece.vertices[i*3+2];
				check(Math.abs(x) <= 270 && Math.abs(y) <= 165 && z >= 0 && z <= 1020, "All supports within table envelope");
			}
		}
		for (component in table.components()) {
			var part = component.component.geometry(Preview);
			check(part.volume() > 0, 'Positive CAD volume: ${component.id}');
			part.close();
		}
		var banks = new TendingCellTable(540,330,1020,570);
		check(banks.definition().occurrences.length == 66, "Both banks flattened with all members");
		check(banks.collisionPieces().length == 98, "Both banks collision coverage");
		for (p in banks.collisionPieces()) {
			check(p.id.indexOf("bank-") == 0, "Collision ids match assembly bank paths");
			check(Math.abs(p.frame.x) >= 300, "Bank hulls translated out of robot corridor");
		}
		var rejected = false;
		try new TSlotTable(100, 100, 200) catch (_:Dynamic) rejected = true;
		check(rejected, "Reject dimensions that cannot fit framing");
		var root = Sys.getCwd();
		while (!sys.FileSystem.exists(root + "/machinekit/examples/machine-tending")) {
			var parent = haxe.io.Path.directory(root);
			if (parent == root || parent.length == 0) throw "Could not locate repository for table preview";
			root = parent;
		}
		var output = root + "/machinekit/examples/machine-tending/build/tslot-tables.mtrg";
		sys.io.File.saveBytes(output,
			SceneArtifact.encode(AssemblyPreview.scene(banks, "tslot-tray-tables")));
		trace("T-slot table CAD, layout, cut-list and collision checks passed");
	}
}
