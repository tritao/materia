package machinekit.structural;

import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import materia.casting.CastingPatterns;
import materia.casting.CastingFeature;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

typedef CastInsertPlacement = {
	final id:String;
	final part:CastInsert;
	final pose:AssemblyFrame;
	final openings:Array<CastingOpening>;
}

/** A casting and its separately itemised steel inserts, with matching recesses.
 * Layouts belong to the machine using this assembly, not to the component core.
 */
class MineralFoundation extends MachineAssembly {
	public final body:MineralCasting;

	final inserts:Array<CastInsertPlacement>;

	public function new(bodyValues:ComponentValues, placements:Array<CastInsertPlacement>, ?rootInsert:String) {
		super();
		if (placements == null)
			throw "Foundation needs an insert list";
		inserts = [];
		var v = machinekit.component.ComponentRecipeSupport.resolve(MineralCasting.parameters(), bodyValues);
		var features = CastingPatterns.decode(v.token("features"));
		var ids:Map<String, Bool> = ["casting" => true];
		for (row in placements) {
			if (row == null || row.id == null || row.id.length == 0 || ids.exists(row.id) || row.part == null)
				throw "Foundation inserts need unique ids and steel parts";
			ids.set(row.id, true);
			if (row.openings == null)
				throw "Foundation insert requires declared recess openings: " + row.id;
			var recess:CastingFeature = {
				id: 'insert-${row.id}',
				kind: InsertRecess,
				geometry: {
					solid: materia.casting.CastingFeature.CastingGeometrySolid.Insert(row.part.bodySpec()),
					pose: F.compose(F.identity(), row.pose)
				},
				openings: [for (opening in row.openings) {kind: opening.kind, target: opening.target}],
				draft: {degrees: 0, axis: {x: 0, y: 0, z: 1}},
				cornerRadius: 0,
				owner: row.id,
				adjustable: [],
				derivedFrom: null
			};
			// Validate references with the complete feature set, including caller features.
			var copy = recess;
			features.push(copy);
			inserts.push({id: row.id, part: row.part, pose: copy.geometry.pose, openings: copy.openings});
		}
		if (rootInsert != null && (rootInsert == "casting" || !ids.exists(rootInsert)))
			throw "Unknown foundation root insert";
		var embedded:Array<materia.casting.EmbeddedAnchors.EmbeddedInsertAnchors> = [];
		for (row in inserts) {
			var spec=row.part.insertSpec();
			switch spec.anchorage {case Bolted(_,_):embedded.push({owner:row.id,pose:row.pose,spec:spec});default:}
		}
		if (materia.casting.EmbeddedAnchors.snapshot(haxe.Json.parse(v.token("embeddedAnchors"))).length>0)
			throw "Foundation anchor occupancy is derived from its source inserts, not authored twice";
		v.setToken("embeddedAnchors",haxe.Json.stringify(materia.casting.EmbeddedAnchors.records(embedded)));
		v.setToken("features", CastingPatterns.encode(features));
		body = new MineralCasting(v);
		validateDatums();
		validateHoleExits();
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
			var anchors:Array<InsertAnchorHardware.InsertAnchorComponent>;
			try anchors=InsertAnchorHardware.parts(row.part.insertSpec())
			catch (error:Dynamic) throw "Insert "+row.id+": "+Std.string(error);
			for (hardware in anchors) {
				var id="anchor-"+haxe.crypto.Sha256.encode(row.id+"/"+hardware.anchorId).substr(0,20)+"-"+hardware.kind;
				if (ids.exists(id)) throw "Duplicate foundation anchor member: "+id;ids.set(id,true);
				addComponent(id,hardware.part,F.compose(row.pose,hardware.pose));
				addMemberConnector(row.id,id+"Seat",hardware.pose);
				addMemberConnector(id,"castAnchorSeat",F.identity());
				addMate(id+"-cast-in","fixed",row.id,id+"Seat",id,"castAnchorSeat");
			}
			exposeConnector(row.id, row.id, "top");
		}
		if (rootInsert != null)
			exposeConnector("mount", rootInsert, "mount");
		else
			exposeConnector("mount", "casting", "mount");
	}

	function validateDatums():Void {
		var scheme=body.foundationDatumScheme(),interfaces=body.precisionRequirements();
		if (scheme==null) {
			if (interfaces.length>0 || inserts.filter(insert -> insert.part.insertSpec().interfaces.length>0).length>0)
				diagnostics.warning("casting.datum.missing","casting","Precision interfaces have no source datum scheme.");
			return;
		}
		var unresolved=materia.casting.FoundationDatumScheme.FoundationDatums.validateSource(scheme,body.castingEnvelope(),
			body.castingFeatures(),[for (insert in inserts) {
				var specification=insert.part.insertSpec();
				{id:insert.id,pose:insert.pose,holes:specification.holes,interfaces:specification.interfaces};
			}],interfaces);
		for (datum in unresolved)
			diagnostics.warning("casting.datum.unsupported",datum.datumId,datum.reason+": "+datum.featureId);
	}

	/** A through thread needs an accessible exit, never a closure buried in resin.
	 * The actual finished cavities are authoritative; the insert's own recess is
	 * excluded because a point on its boundary cannot establish access behind it.
	 */
	function validateHoleExits():Void {
		var cavities = body.finishedCavities();
		for (row in inserts) {
			var specification=row.part.insertSpec();
			var before=materia.casting.InsertStages.holes(specification,materia.casting.InsertStages.InsertStage.BeforeCasting);
			for (source in specification.holes) {
				var physical=before.filter(hole -> hole.id==source.id);
				var hole=physical.length==1 ? physical[0] : source;
				if (hole.extent != Through || hole.thread == null)
					continue;
				var frame = F.compose(row.pose, hole.frame);
				var exit = F.compose(frame, F.translation(0, 0, hole.depth));
				var thread:materia.casting.InsertHole.InsertThread = cast hole.thread;
				if (materia.casting.CastingHoleExits.intersectsMaterial(body.castingEnvelope(), cavities,
					body.castingFeatures(), exit, thread.majorDiameter / 2, row.id))
					throw "Through insert thread exits into casting material: " + row.id + "/" + hole.id;
			}
		}
	}

	public function insertLocations():Array<CastInsertPlacement>
		return [
			for (row in inserts)
				{id: row.id, part: row.part, pose: F.compose(F.identity(), row.pose),
					openings: [for (opening in row.openings) {kind: opening.kind, target: opening.target}]}
		];

	public function insertRecessLinks():Array<{recessId:String, insertId:String}>
		return [for (feature in body.castingFeatures()) if (feature.kind == InsertRecess)
			{recessId: feature.id, insertId: feature.owner}];

	public function castingByWeight():{material:String, kg:Float}
		return body.castingByWeight();
}
