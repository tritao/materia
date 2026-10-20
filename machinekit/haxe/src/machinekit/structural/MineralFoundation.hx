package machinekit.structural;

import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

typedef CastInsertPlacement = {
	final id:String;
	final part:CastInsert;
	final pose:AssemblyFrame;
}

/** A casting and its separately itemised steel inserts, with matching recesses.
 * Layouts belong to the machine using this assembly, not to the component core.
 */
class MineralFoundation extends MachineAssembly {
	public final body:MineralCasting;

	final inserts:Array<CastInsertPlacement>;
	final recessLinks:Array<{recessId:String, insertId:String}>;

	public function new(bodyValues:ComponentValues, placements:Array<CastInsertPlacement>, ?rootInsert:String) {
		super();
		if (placements == null)
			throw "Foundation needs an insert list";
		inserts = [];
		recessLinks = [];
		var v = machinekit.component.ComponentRecipeSupport.resolve(MineralCasting.parameters(), bodyValues);
		var features = CastingFeatures.decode(v.token("voids"));
		var ids:Map<String, Bool> = ["casting" => true];
		for (row in placements) {
			if (row == null || row.id == null || row.id.length == 0 || ids.exists(row.id) || row.part == null)
				throw "Foundation inserts need unique ids and steel parts";
			ids.set(row.id, true);
			var recess:machinekit.structural.CastingFeatures.CastingBox = {
				id: 'insert-${row.id}',
				width: row.part.parameterNumber("width"),
				depth: row.part.parameterNumber("depth"),
				height: row.part.parameterNumber("height"),
				pose: row.pose
			};
			// Codec validation also takes ownership of a copy of the caller's frame.
			var copy = CastingFeatures.decode(CastingFeatures.encode([recess]))[0];
			features.push(copy);
			recessLinks.push({recessId: copy.id, insertId: row.id});
			inserts.push({id: row.id, part: row.part, pose: copy.pose});
		}
		if (rootInsert != null && (rootInsert == "casting" || !ids.exists(rootInsert)))
			throw "Unknown foundation root insert";
		v.setToken("voids", CastingFeatures.encode(features));
		body = new MineralCasting(v);
		addComponent("casting", body);
		for (row in inserts) {
			addComponent(row.id, row.part, row.pose);
			if (row.id == rootInsert) {
				addMemberConnector(row.id, "castingSeat", F.compose(F.inverse(row.pose), body.connector("mount").frame));
				addMate("casting-insert", "fixed", row.id, "castingSeat", "casting", "mount");
			} else {
				addMemberConnector("casting", row.id + "Seat", F.compose(row.pose, row.part.connector("mount").frame));
				addMate(row.id + "-cast-in", "fixed", "casting", row.id + "Seat", row.id, "mount");
			}
			exposeConnector(row.id, row.id, "top");
		}
		if (rootInsert != null)
			exposeConnector("mount", rootInsert, "mount");
		else
			exposeConnector("mount", "casting", "mount");
	}

	public function insertLocations():Array<CastInsertPlacement>
		return [
			for (row in inserts)
				{id: row.id, part: row.part, pose: F.compose(F.identity(), row.pose)}
		];

	public function insertRecessLinks():Array<{recessId:String, insertId:String}>
		return [for (link in recessLinks) {recessId: link.recessId, insertId: link.insertId}];

	public function castingByWeight():{material:String, kg:Float}
		return body.castingByWeight();
}
