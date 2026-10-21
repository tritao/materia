package machinekit.structural;

import cadkit.modeling.Part;
import materia.assembly.AssemblyFrames as F;
import materia.casting.EmbeddedAnchors.EmbeddedInsertAnchors;
import machinekit.assembly.PosedParts;

/** Exact occupied volumes of real standard hardware, in casting coordinates. */
class EmbeddedAnchorCad {
	public static function parts(rows:Array<EmbeddedInsertAnchors>):Array<Part> {
		var result:Array<Part> = [],cache=new PosedParts();
		try {
			for (row in rows) {
				var components:Array<InsertAnchorHardware.InsertAnchorComponent>;
				try components=InsertAnchorHardware.parts(row.spec)
				catch (error:Dynamic) throw "Insert "+row.owner+": "+Std.string(error);
				for (hardware in components) {
					var occupancy=cache.posed(hardware.part,F.compose(row.pose,hardware.pose));
					result.push(occupancy);
					// This owned copy represents excluded resin volume, not an external
					// screw machining interface on the resulting casting. Keep the exact
					// B-rep; semantic threads remain on the real component in the BOM.
					occupancy.shape.setThreadedFaces([]);
			}
			}
		} catch (error:Dynamic) {for (part in result) part.close();cache.close();throw error;}
		cache.close();return result;
	}
}
