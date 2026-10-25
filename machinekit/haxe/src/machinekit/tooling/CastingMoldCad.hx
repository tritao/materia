package machinekit.tooling;

import materia.tooling.casting.CastingMoldPlan.MoldFixtures;
import materia.tooling.casting.CastingMoldPlan.MoldOpenings;
import materia.tooling.casting.MoldCores;
import machinekit.tooling.InsertAnchorGeometry;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.structural.CastInsert;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.FlatWasher;
import machinekit.standard.HexNut;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldPlan.MoldBox;
import materia.tooling.casting.CoreSupport.SupportHardwareKind;
import machinekit.tooling.SweptRemoval.RemovalMember;
import machinekit.tooling.SweptRemoval.RemovalReport;

enum abstract DemoldingStepKind(String) to String {
	var ReleaseHardware = "release-hardware";
	var Withdraw = "withdraw";
	var FlipToService = "flip-to-service";
	var BreakoutEPS = "breakout-eps";
	var ReleaseEPSBonds = "release-eps-bonds";
}

typedef MoldDemoldingStep = {var id:String; var kind:DemoldingStepKind; var memberIds:Array<String>; var dependsOn:Array<String>; var status:materia.tooling.ToolingDiagnostic.ToolingCheckStatus;}
typedef CastingMoldAssessment = {var diagnostics:machinekit.assembly.Diagnostics; var geometry:Null<CastingMoldReview>;}

typedef CastingMoldReview = {
	var fingerprint:String;
	var handling:HandlingPlan;
	var demoldingSteps:Array<MoldDemoldingStep>;
	var symmetricDifferenceMm3:Float;
	var sourceVolumeMm3:Float;
	var reconstructedVolumeMm3:Float;
	var assembly:RemovalReport;
	var demolding:RemovalReport;
	var permanentGroups:Array<{
		id:String,
		members:Array<String>
	}>;
	var assemblyGroups:Array<{
		id:String,
		members:Array<String>
	}>;
	var validation:Array<materia.tooling.ToolingValidation>;
	var fabrication:materia.tooling.ToolingValidation.FabricationReadiness;
	var assemblyMotions:Array<{
		id:String,
		x:Float,
		y:Float,
		z:Float
	}>;
	var demoldingMotions:Array<{
		id:String,
		x:Float,
		y:Float,
		z:Float
	}>;
}

/** CAD bridge for neutral multi-core box plans. Frames and exact operations remain authoritative. */
class CastingMoldCad {
	/** Native planning entry point; pure ManufacturingKit callers can retain
	 * the exact box algebra or supply a different exact kernel explicitly.
	 */
	public static function plan(document:materia.tooling.ToolingDocument,options:CastingMoldOptions):CastingMoldPlan {
		enableGeometry();
		return CastingMoldPlanner.plan(document,options);
	}

	static function enableGeometry():Void {
		if (CastingMoldPlanner.geometry==null) CastingMoldPlanner.setGeometry(new NativeCastingMoldKernel());
	}

	static function box(b:MoldBox):Part
		return ToolingBlock.placedBox(b);

	/** Real source resin, including the exact standard anchor occupancy. */
	public static function source(p:CastingMoldPlan):Part {
		var local=ToolingSourceCasting.fromInput(p.document.input).geometry();
		try {
			var placed=PosedParts.moved(local,p.document.setup.sourceToMold);
			local.close();return placed;
		} catch (error:Dynamic) {local.close();throw error;}
	}

	/** Precast compound occupies real fill space, but is already cured. */
	public static function strutRegion(p:CastingMoldPlan,strut:materia.casting.PrecastStrut):materia.casting.CastingRegion
		return materia.tooling.casting.MoldRegions.moved(materia.tooling.casting.MoldRegions.box(strut.box),p.document.setup.sourceToMold);

	public static function pouredSource(p:CastingMoldPlan):Part {
		var source=source(p),pieces:Array<Part>=[source];
		return Solids.building(pieces,owned->{
			for(strut in p.document.input.precastStruts)owned.push(machinekit.structural.CastingRegionCad.part(strutRegion(p,strut)));
			return owned.length==1 ? source : Solids.cut(source,owned.slice(1));
		});
	}

	public static function anchorComponents(p:CastingMoldPlan,?owner:String):Array<{
		id:String,insertId:String,anchorId:String,part:machinekit.component.MachineComponent,
		pose:materia.assembly.AssemblyRecord.AssemblyFrame
	}> {
		var result=[];
		for (insert in p.document.input.inserts) {
			if (owner!=null && owner!=insert.id) continue;
			var spec:materia.casting.InsertSpec={body:insert.body,holes:insert.holes,origin:insert.origin,
				interfaces:insert.interfaces,anchorage:insert.anchorage};
			for (hardware in machinekit.tooling.InsertAnchorGeometry.components(spec)) {
				var prefix="anchor-"+haxe.crypto.Sha256.encode(insert.id+"/"+hardware.anchorId).substr(0,20);
				result.push({id:prefix+"-"+hardware.kind,insertId:insert.id,anchorId:hardware.anchorId,
					part:hardware.part,pose:F.compose(p.document.setup.sourceToMold,F.compose(insert.pose,hardware.pose))});
			}
		}
		return result;
	}

	static function anchorParts(p:CastingMoldPlan,?owner:String):Array<Part> {
		var result:Array<Part> = [];
		try for (hardware in anchorComponents(p,owner)) {
			var local=hardware.part.geometry();
			try {
				var placed=PosedParts.moved(local,hardware.pose);
				// Owned motion/occupancy geometry, not a screw machining interface.
				placed.shape.setThreadedFaces([]);result.push(placed);
			} catch (error:Dynamic) {local.close();throw error;}
			local.close();
		} catch (error:Dynamic) {for (part in result) part.close();throw error;}
		return result;
	}

	/** Merge coplanar patches left by access tooling touching the casting boundary.
	 * Finalize before STEP, mesh and review derive their surface identities.
	 */
	public static function reconstructed(p:CastingMoldPlan):Part {
		var raw = unrefinedReconstruction(p);
		try {
			var finished = raw.refined();
			raw.close();
			return finished;
		} catch (error:Dynamic) {
			raw.close();
			throw error;
		}
	}

	static function unrefinedReconstruction(p:CastingMoldPlan):Part {
		if (p.construction==SinglePocket) return BoxMoldCad.reconstructed(p);
		var pieces:Array<Part> = [box(p.cavity)];
		return Solids.building(pieces, parts -> {
			for (c in p.cores)
				parts.push(machinekit.structural.CastingRegionCad.part(MoldCores.occupied(c)));
			for (f in p.formers)
				parts.push(ToolingVolume.geometryOf(f.volume));
			// Fixture receivers are enclosed by the bed and sealed insert faces.
			// They remain real clearance holes in the tooling, but are not pour
			// space. The planner validates these full occupied volumes against resin.
			for (volume in MoldFixtures.allVolumes(p))
				for (piece in volume.pieces)
					parts.push(box(piece));
			// Full envelopes include bores protected during pouring. Their actual
			// masks are separate members in the assembly and demolding graphs.
			for (i in p.document.input.inserts)
				parts.push(machinekit.structural.CastingRegionCad.part(
					materia.tooling.casting.MoldRegions.insert(p.document,i.id)));
			for (part in anchorParts(p)) parts.push(part);
			return Solids.cut(parts[0], parts.slice(1));
		});
	}

	public static function steel(p:CastingMoldPlan, id:String):Part {
		var i = p.document.input.inserts.filter(i -> i.id == id)[0];
		var component = ToolingInsertCad.component(i);
		var local = component.geometry();
		try {
			var out = PosedParts.moved(local, F.compose(p.document.setup.sourceToMold, i.pose));
			local.close();
			var anchors=anchorParts(p,id);
			if (anchors.length==0) return out;
			// Keep semantic threads on the separate real source components. Their
			// fused motion solid has intentionally consumed the engaged interfaces.
			out.shape.setThreadedFaces([]);
			return Solids.union([out].concat(anchors));
		} catch (e:Dynamic) {
			if (!local.shape.isClosed()) local.close();
			throw e;
		}
	}

	public static function assess(p:CastingMoldPlan):CastingMoldAssessment {
		CastingMoldPlanner.validate(p);
		var diagnostics = ToolingDiagnostics.fromValidation(p.validation);
		if (p.features.filter(f -> f.kind == UnsupportedFeature).length > 0
			|| p.validation.filter(v -> StringTools.startsWith(v.rule, "tooling.fixture.hole-protection.") && v.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length > 0)
			return {diagnostics: diagnostics, geometry: null};
		return {diagnostics: diagnostics, geometry: review(p)};
	}

	public static function threadedMask(p:CastingMoldPlan, h:materia.tooling.casting.CoreSupport.SupportHardware):Bool
		return h.kind == SupportHardwareKind.BoreMask && boreMaskRecord(p, h.id).hole.thread != null;

	public static function looseHardware(p:CastingMoldPlan, h:materia.tooling.casting.CoreSupport.SupportHardware):Bool
		return h.kind != SupportHardwareKind.FixturePin && (h.kind != SupportHardwareKind.BoreMask || threadedMask(p, h));

	public static function steps(p:CastingMoldPlan, order:Array<String>):Array<MoldDemoldingStep> {
		var result:Array<MoldDemoldingStep> = [
			{
				id: "release-loose-tooling",
				kind: ReleaseHardware,
				memberIds: [
					for (h in p.hardware)
						if (looseHardware(p, h) && h.kind != SupportHardwareKind.BorePlug && !threadedMask(p, h)) h.id
				],
				dependsOn: [],
				status: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending
			}
		];
		var previous = "release-loose-tooling";
		var entryMasks = [for (h in p.hardware) if (threadedMask(p, h)) h.id];
		if (entryMasks.length > 0) {
			result.push({id: "release-entry-thread-masks", kind: ReleaseHardware, memberIds: entryMasks,
				dependsOn: [previous], status: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending});
			previous = "release-entry-thread-masks";
		}
		var farPlugs = [for (h in p.hardware) if (h.kind == SupportHardwareKind.BorePlug) h.id];
		if (farPlugs.length > 0) {
			result.push({id: "release-far-end-thread-plugs", kind: ReleaseHardware, memberIds: farPlugs,
				dependsOn: [previous], status: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending});
			previous = "release-far-end-thread-plugs";
		}
		var bonds:Array<String>=[];
		for(core in p.cores)if(core.support.breakoutAfterLift){
			var id=core.support.foot!=null ? materia.tooling.ToolingIds.derive("",core.id,"-foot")
				: core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge ? MoldCores.carrierId(core) : null;
			if(id!=null && bonds.indexOf(id)<0)bonds.push(id);
		}
		if(bonds.length>0){
			result.push({id:"release-eps-bonds",kind:ReleaseEPSBonds,memberIds:bonds,dependsOn:[previous],
				status:materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending});
			previous="release-eps-bonds";
		}
		for (id in order) {
			var stepId = materia.tooling.ToolingIds.derive("withdraw-", id);
			result.push({
				id: stepId,
				kind: Withdraw,
				memberIds: [id],
				dependsOn: [previous],
				status: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass
			});
			previous = stepId;
		}
		if (p.handling.requiresFlip)
			result.push({
				id: "flip-to-service",
				kind: FlipToService,
				memberIds: ["cured-casting"],
				dependsOn: [previous],
				status: materia.tooling.casting.CastHandling.qualified(p.document, p.options,
					p.handling) ? materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass : materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending
			});
		var dependency=result[result.length-1].id;
		for(core in p.cores)if(core.support.breakoutAfterLift){
			var id=materia.tooling.ToolingIds.derive("breakout-",core.id);
			result.push({id:id,kind:BreakoutEPS,memberIds:MoldCores.assemblyMembers(core),
				dependsOn:[dependency],status:materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending});
			dependency=id;
		}
		return result;
	}

	static function boreMaskRecord(p:CastingMoldPlan, id:String):{
		protection:BoreMaskProtection, hole:materia.casting.InsertHole
	} {
		for (fixture in p.fixtures)
			for (protection in fixture.boreMasks)
				if (protection.hardwareId == id) {
					var insert = p.document.input.inserts.filter(source -> source.id == fixture.insertId)[0];
					var holes = materia.tooling.casting.InsertFixturePlanner.holes(insert).filter(hole -> hole.id == protection.sourceHoleId);
					if (holes.length != 1)
						throw "Bore mask has no unique source hole: " + id;
					return {protection: protection, hole: holes[0]};
				}
		throw "Unknown bore mask: " + id;
	}

	static function panelWorld(m:materia.tooling.panel.PanelAssemblyPlan.PanelPlacement):Part {
		var local = ToolingPanel.fromPlan(m.panel).geometry();
		try {
			var out = PosedParts.moved(local, m.pose);
			local.close();
			return out;
		} catch (e:Dynamic) {
			local.close();
			throw e;
		}
	}

	public static function presentation(p:CastingMoldPlan,definition:materia.assembly.AssemblyDefinition):materia.presentation.AssemblyPresentation {
		if (p.construction==SinglePocket) return BoxMoldCad.presentation(p,definition);
		var b=p.cavity,offsets=new Map<String,materia.assembly.AssemblyDefinition.AssemblyVector>();
		for(m in p.panels.members)offsets.set(m.panel.id,switch m.panel.id {
			case "bed": {x:0.0,y:0.0,z:-b.height*0.7};
			case "east": {x:b.width,y:0.0,z:0.0};
			case "west": {x:-b.width,y:0.0,z:0.0};
			case "north": {x:0.0,y:b.depth*0.4,z:0.0};
			case "south": {x:0.0,y:-b.depth*0.4,z:0.0};
			case _: {x:0.0,y:0.0,z:0.0};
		});
		for(i in 0...p.supportPanels.length)offsets.set(p.supportPanels[i].panel.id,{x:0,y:0,z:b.height*(0.5+i*0.15)});
		for(i in 0...p.cores.length)if(p.cores[i].support.kind!=materia.tooling.casting.CoreSupport.CoreSupportKind.PrecastStruts)
			for(id in MoldCores.assemblyMembers(p.cores[i]))offsets.set(id,{x:0,y:0,z:b.height*(0.8+i*0.12)});
		for(f in p.formers)offsets.set(f.volume.id,{x:f.withdrawal.x*b.width*0.7,y:f.withdrawal.y*b.depth*0.4,z:f.withdrawal.z*b.height*0.7});
		for(f in p.fixtures) for(volume in MoldFixtures.volumes(f))
			offsets.set(volume.id,{x:MoldFixtures.withdrawal(f).x*b.width,y:0,z:MoldFixtures.withdrawal(f).z*b.height*1.4});
		for(i in p.document.input.inserts)offsets.set(materia.tooling.ToolingIds.derive("insert-",i.id),{x:b.width*1.8,y:0,z:0});
		for(hardware in anchorComponents(p))
			offsets.set(hardware.id,offsets.get(materia.tooling.ToolingIds.derive("insert-",hardware.insertId)));
		return ToolingExplode.presentation(definition,"casting-mold",offsets);
	}

	public static function assembly(p:CastingMoldPlan):MachineAssembly {
		if (p.construction==SinglePocket) return BoxMoldCad.assembly(p);
		enableGeometry();
		CastingMoldPlanner.validate(p);
		var a = PanelKit.assembly(p.panels, "coated particleboard"), b = p.cavity;
		for (i in 0...p.supportPanels.length) {
			var m = p.supportPanels[i], pose = F.compose(F.identity(), m.pose);
			a.addComponent(m.panel.id, ToolingPanel.fromPlan(m.panel), pose);
		}
		for (i in 0...p.cores.length) {
			var c = p.cores[i], pose = F.identity();
			if(c.support.assemblyByLayer){
				for(index in 0...c.layers.length)a.addComponent(MoldCores.assemblyLayerId(c,index),
					ToolingVolume.fromCoreCutting(MoldCores.layerId(c,index),c.layers[index]),c.layers[index].stockToMold);
				var foot=c.support.foot;
				if(foot!=null){
					var id=materia.tooling.ToolingIds.derive("",c.id,"-foot");
					a.addComponent(id,ToolingVolume.fromCoreCutting(id,foot.cutting),foot.cutting.stockToMold);
				}
				continue;
			}
			a.addComponent(c.id, ToolingVolume.fromVolume({
				id: c.id,
				material: EPSStock,
				cuts: [],
				regions: [MoldCores.physical(c)],
				profiles: [],
				pieces: []
			}), pose);
		}
		for(strut in p.document.input.precastStruts){
			var b=strut.box;
			a.addComponent(strut.id,ToolingBlock.fromBox(b,"mineral cast"),
				F.compose(p.document.setup.sourceToMold,F.translation(b.x,b.y,b.z)));
		}
		for (f in p.formers) {
			var pose = F.identity();
			a.addComponent(f.volume.id, ToolingVolume.fromVolume(f.volume), pose);
		}
		for (volume in MoldFixtures.allVolumes(p)) {
			var pose = F.identity();
			a.addComponent(volume.id, ToolingVolume.fromVolume(volume), pose);
		}
		for (i in p.document.input.inserts) {
			var pose = F.compose(p.document.setup.sourceToMold, i.pose);
			var part = ToolingInsertCad.component(i);
			a.addComponent(materia.tooling.ToolingIds.derive("insert-", i.id), part, pose);
		}
		for (hardware in anchorComponents(p)) a.addComponent(hardware.id,hardware.part,hardware.pose);
		for (h in p.hardware) {
			if (h.kind == SupportHardwareKind.BoreMask) {
				var mask = ToolingPin.boreMask(boreMaskRecord(p, h.id).hole, boreMaskRecord(p, h.id).protection.plugLength);
				a.addComponent(h.id, mask, F.compose(h.pose, ToolingPin.boreSeat(mask)));
			}
			else if (h.kind == SupportHardwareKind.BorePlug) {
				var protections = [for (fixture in p.fixtures) for (pen in MoldOpenings.records(p,fixture.insertId))
					if (pen.sourceHoleId != null && materia.tooling.ToolingIds.derive("resin-plug-", fixture.insertId + "-" + pen.sourceHoleId) == h.id)
					{insertId: fixture.insertId, holeId: pen.sourceHoleId}];
				for (fixture in p.fixtures) for (mask in fixture.boreMasks)
					if (mask.farEndId == h.id)
						protections.push({insertId: fixture.insertId, holeId: mask.sourceHoleId});
				if (protections.length != 1)
					throw "Resin-side plug has no unique source hole: " + h.id;
				var source = p.document.input.inserts.filter(i -> i.id == protections[0].insertId)[0];
				var hole = materia.tooling.casting.InsertFixturePlanner.holes(source).filter(hole -> hole.id == protections[0].holeId)[0];
				a.addComponent(h.id, machinekit.standard.SocketSetScrew.forThread(
					machinekit.structural.InsertHoles.threadSpec(hole.thread).mating(), h.length),
					F.compose(h.pose, machinekit.standard.SocketSetScrew.boreSeat()));
			}
			else if (h.kind == SupportHardwareKind.LocatingPin
				|| h.kind == SupportHardwareKind.FixturePin)
				a.addComponent(h.id, new ToolingPin(new ComponentValues().setNumber("diameter", h.diameter).setNumber("length", h.length)), h.pose);
			else if (h.kind == SupportHardwareKind.FixtureScrew) {
				var fixings = [for (fixture in p.fixtures) for (fixing in fixture.fixings)
					if (fixing.hardwareId == h.id) {sourceId:fixture.insertId,fixing:fixing}];
				if (fixings.length != 1)
					throw "Fixture screw has no unique source thread: " + h.id;
				var source=p.document.input.inserts.filter(insert -> insert.id==fixings[0].sourceId)[0];
				var fixing=fixings[0].fixing;
				var hole=materia.tooling.casting.InsertFixturePlanner.holes(source).filter(hole -> hole.id==fixing.sourceHoleId)[0];
				ToolingHardwareCad.fixtureScrew(a,h.id,hole,h.length,h.pose,fixing.entry);
			}
			else {
				ToolingHardwareCad.throughBolt(a, h.id, h.diameter, h.length, h.pose);
			}
		}
		var poses = a.solvedPoses(), inverse = F.inverse(poses.get("bed"));
		a.addMemberConnector("bed", "tooling-origin", F.identity());
		for (row in a.components())
			if (p.panels.members.filter(m -> m.panel.id == row.id).length == 0) {
				a.addMemberConnector(row.id, "tooling-datum", F.inverse(F.compose(inverse, poses.get(row.id))));
				a.addMate(materia.tooling.ToolingIds.derive("", row.id, "-positioned"), "fixed", "bed", "tooling-origin", row.id, "tooling-datum");
			}
		return a;
	}

	static function coverage(p:CastingMoldPlan, tooling:Array<Part>, casting:Part):Void {
		var b = p.cavity, s = 0.1, faces:Array<MoldBox> = [
			{
				x: b.x - s,
				y: b.y,
				z: b.z,
				width: s,
				depth: b.depth,
				height: b.height
			},
			{
				x: b.x + b.width,
				y: b.y,
				z: b.z,
				width: s,
				depth: b.depth,
				height: b.height
			},
			{
				x: b.x,
				y: b.y - s,
				z: b.z,
				width: b.width,
				depth: s,
				height: b.height
			},
			{
				x: b.x,
				y: b.y + b.depth,
				z: b.z,
				width: b.width,
				depth: s,
				height: b.height
			},
			{
				x: b.x,
				y: b.y,
				z: b.z - s,
				width: b.width,
				depth: b.depth,
				height: s
			}
		];
		var directions = [
			new Vector(-s, 0, 0),
			new Vector(s, 0, 0),
			new Vector(0, -s, 0),
			new Vector(0, s, 0),
			new Vector(0, 0, -s)
		];
		for (index in 0...faces.length) {
			var slab = box(faces[index]);
			var owned:Array<Part> = [slab];
			try {
				// Check actual resin contact on the closed outer face. A former's
				// shoulder can seal on the face without filling a positive-depth
				// slab behind it. Voids are not resin contact surfaces.
				var translated = casting.translated(directions[index]);
				owned.push(translated);
				var contact = slab.intersect(translated);
				owned.push(contact);
				var uncovered = contact.subtractAll(tooling);
				owned.push(uncovered);
				var volume = uncovered.volume();
				if (volume > 1e-3)
					throw "Uncovered mold boundary " + index + ": " + volume + " mm3";
				for (part in owned)
					part.close();
			} catch (e:Dynamic) {
				for (part in owned)
					if (!part.shape.isClosed())
						part.close();
				throw e;
			}
		}
	}

	/** Check the delivered foam, including its neck and external flange, against
	 * the actual machined receiver. Bounds and contour endpoints cannot prove a
	 * fit: a router-relief arc can extend beyond every stored endpoint.
	 */
	public static function corePanelFit(p:CastingMoldPlan):Array<materia.tooling.ToolingValidation> {
		var checks:Array<materia.tooling.ToolingValidation> = [];
		for (core in p.cores) {
			if (core.support.kind != materia.tooling.casting.CoreSupport.CoreSupportKind.RegisteredFlange)
				continue;
			var panelId = MoldCores.receiverId(core);
			var receivers = p.panels.members.filter(member -> member.panel.id == panelId);
			if (receivers.length != 1)
				throw "Unknown EPS access receiver: " + core.id + "/" + panelId;
			var scope = new cadkit.modeling.Scope();
			try {
				var panel = scope.own(panelWorld(receivers[0]));
				var foam = scope.own(machinekit.structural.CastingRegionCad.part(MoldCores.physical(core)));
				var sealed = scope.own(machinekit.structural.CastingRegionCad.part(MoldCores.occupied(core)));
				for (solid in [foam, sealed]) {
					var overlap = scope.own(solid.intersect(panel));
					if (overlap.volume() > 1e-4)
						throw "EPS access intersects receiver: " + core.id + "/" + panelId;
				}
				checks.push(materia.tooling.export.ToolingLoads.check(
					"tooling.cad.core-panel-fit." + core.id, [core.id, core.featureId, panelId],
					materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass,
					"Finished EPS and its sealed neck clear the machined receiver; the external flange remains outside.",
					[CastingMoldPlanner.fingerprint(p)]));
				scope.close();
			} catch (error:Dynamic) {
				scope.close();
				throw error;
			}
		}
		return checks;
	}

	public static function review(p:CastingMoldPlan):CastingMoldReview {
		if (p.construction==SinglePocket) return BoxMoldCad.commonReview(p);
		enableGeometry();
		CastingMoldPlanner.validate(p);
		var coreFits = corePanelFit(p);
		var references:Array<Part> = [], difference = 0.0, ov = 0.0, rv = 0.0;
		try {
			var original = source(p);
			references.push(original);
			for(strut in p.document.input.precastStruts){
				var declared=machinekit.structural.CastingRegionCad.part(strutRegion(p,strut));
				references.push(declared);
				var outside=declared.subtract(original);
				references.push(outside);
				if(outside.volume()>1e-4)
					throw "Precast compound strut leaves final casting material: "+strut.id;
			}
			var rebuilt = reconstructed(p);
			references.push(rebuilt);
			var missing = original.subtractAll([rebuilt]);
			references.push(missing);
			var excess = rebuilt.subtractAll([original]);
			references.push(excess);
			difference = missing.volume() + excess.volume();
			ov = original.volume();
			var physical = original.massProperties(),
				mass = physical.volume * p.document.recipe.mixDensityKgM3 * 1e-9;
			var mx = mass * physical.centerOfMass.x,
				my = mass * physical.centerOfMass.y,
				mz = mass * physical.centerOfMass.z;
			for (i in p.document.input.inserts) {
				var shape = steel(p, i.id);
				references.push(shape);
				var properties = shape.massProperties(),
					kg = properties.volume * p.document.recipe.steelDensityKgM3 * 1e-9;
				mass += kg;
				mx += kg * properties.centerOfMass.x;
				my += kg * properties.centerOfMass.y;
				mz += kg * properties.centerOfMass.z;
			}
			var center = p.handling.centerOfMassMoldMm;
			if (Math.abs(mass - p.handling.castingMassKg) > 1e-5
				|| Math.abs(mx / mass - center.x) + Math.abs(my / mass - center.y) + Math.abs(mz / mass - center.z) > 1e-5)
				throw "Casting mass/CoM differs from exact CAD with steel bores";
			rv = rebuilt.volume();
			if (difference > 1e-3)
				throw "Casting reconstruction differs by " + difference + " mm3";
			for (part in references)
				part.close();
			references = [];
		} catch (e:Dynamic) {
			for (part in references)
				if (!part.shape.isClosed())
					part.close();
			throw e;
		}
		var parts:Array<Part> = [], members:Array<RemovalMember> = [], permanent:Array<{
			id:String,
			members:Array<String>
		}> = [], groups:Array<{
			id:String,
			members:Array<String>
		}> = [];
		try {
			for (m in p.panels.members) {
				var shape = panelWorld(m);
				parts.push(shape);
				members.push({
					id: m.panel.id,
					part: shape,
					direction: new Vector(m.withdrawal.x, m.withdrawal.y, m.withdrawal.z)
				});
			}
			var rowIds:Array<String> = [];
			var bridgeCores=p.cores.filter(core -> core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge
				|| core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.PrecastStruts);
			for (c in bridgeCores)
				if (rowIds.indexOf(MoldCores.bridgeId(c)) < 0)
					rowIds.push(MoldCores.bridgeId(c));
			for (id in rowIds) {
				var panelIds=[id],shapes:Array<Part>=[];
				for(core in bridgeCores)if(MoldCores.bridgeId(core)==id)
					for(member in core.support.bondedPanelIds)if(panelIds.indexOf(member)<0)panelIds.push(member);
				var shape = Solids.building(shapes, owned -> {
					for (m in p.supportPanels)
						if (panelIds.indexOf(m.panel.id) >= 0)
							owned.push(panelWorld(m));
					return Solids.union(owned);
				});
				parts.push(shape);
				members.push({
					id: materia.tooling.ToolingIds.derive("", id, "-fixture"),
					part: shape,
					direction: new Vector(0, 0, 1)
				});
				permanent.push({id: materia.tooling.ToolingIds.derive("", id, "-fixture"), members: panelIds});
			}
			for (m in p.supportPanels)
				if (rowIds.indexOf(m.panel.id) < 0 && bridgeCores.filter(c -> c.support.bondedPanelIds.indexOf(m.panel.id)>=0).length == 0) {
					var shape = panelWorld(m);
					parts.push(shape);
					members.push({id: m.panel.id, part: shape, direction: new Vector(0, 0, 1)});
				}
			for (f in p.formers) {
				var shape = ToolingVolume.geometryOf(f.volume);
				parts.push(shape);
				members.push({
					id: f.volume.id,
					part: shape,
					direction: new Vector(f.withdrawal.x, f.withdrawal.y, f.withdrawal.z)
				});
			}
			for (f in p.fixtures) for (volume in MoldFixtures.volumes(f)) {
				var shape = ToolingVolume.geometryOf(volume);
				parts.push(shape);
				members.push({
					id: volume.id,
					part: shape,
					direction: new Vector(MoldFixtures.withdrawal(f).x,MoldFixtures.withdrawal(f).y,MoldFixtures.withdrawal(f).z)
				});
			}
			var coreShapes=new Map<String,Part>();
			for(c in p.cores){
				var shape=machinekit.structural.CastingRegionCad.part(MoldCores.occupied(c));
				parts.push(shape);coreShapes.set(c.id,shape);
				if(c.support.kind!=materia.tooling.casting.CoreSupport.CoreSupportKind.PrecastStruts && !c.support.breakoutAfterLift){
					var direction=MoldCores.withdrawal(c,p.panels.members.concat(p.supportPanels));
					members.push({id:c.id,part:shape,direction:new Vector(direction.x,direction.y,direction.z)});
				}
			}
			for (h in p.hardware)
				if (h.kind == SupportHardwareKind.FixturePin || (h.kind == SupportHardwareKind.BoreMask && !threadedMask(p, h))) {
					var component = h.kind == SupportHardwareKind.BoreMask ? ToolingPin.boreMask(boreMaskRecord(p, h.id).hole, boreMaskRecord(p, h.id).protection.plugLength)
						: new ToolingPin(new ComponentValues().setNumber("diameter", h.diameter).setNumber("length", h.length));
					var local = component.geometry(machinekit.component.ComponentDetail.Envelope);
					parts.push(local);
					var shape = PosedParts.moved(local, h.pose);
					parts.push(shape);
					parts.remove(local);
					local.close();
					var direction = new Vector(0, 0, -1);
					if (h.kind == SupportHardwareKind.BoreMask) {
						var v = boreMaskRecord(p, h.id).protection.withdrawal;
						direction = new Vector(v.x, v.y, v.z);
					}
					members.push({id: h.id, part: shape, direction: direction});
				}
			var castingSource=source(p);
			parts.push(castingSource);
			coverage(p,parts.slice(0,parts.length-1),castingSource);
			var retained=p.cores.filter(core->core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.PrecastStruts);
			var liftedEPS=p.cores.filter(core->core.support.breakoutAfterLift || retained.indexOf(core)>=0);
			var castingPieces=[castingSource];
			for(core in liftedEPS){
				var shape=coreShapes.get(core.id);
				if(shape==null)throw "Missing lifted EPS solid: "+core.id;
				castingPieces.push(new Part(shape.shape.cloneShape()));
			}
			if(liftedEPS.length>0)permanent.push({id:"cured-casting",members:
				[for(core in liftedEPS)for(id in MoldCores.assemblyMembers(core))id]
				.concat([for(strut in p.document.input.precastStruts)strut.id])});
			for (i in p.document.input.inserts)
				castingPieces.push(steel(p, i.id));
			var cured = Solids.union(castingPieces);
			parts.push(cured);
			members.push({id: "cured-casting", part: cured, direction: new Vector(0, 0, 1)});
			Sys.println("Checking full demolding sweeps for " + members.length + " members");
			var demoldingMotions = [
				for (m in members)
					{
						id: m.id,
						x: m.direction.x,
						y: m.direction.y,
						z: m.direction.z
					}
			];
			var demolding = SweptRemoval.analyze(members, 1e-4);
			members.pop();
			// These are actual wire-cut stock pieces before bonding. Check their
			// individual installation, including placing inserts between EPS bands.
			for(core in p.cores)if(core.support.breakoutAfterLift){
				if(core.support.assemblyByLayer){
					for(index in 0...core.layers.length){
						var shape=machinekit.structural.CastingRegionCad.part(MoldCores.layerRegion(core.layers[index]));
						parts.push(shape);members.push({id:MoldCores.assemblyLayerId(core,index),part:shape,direction:new Vector(0,0,1)});
					}
					var foot=core.support.foot;
					if(foot!=null){
						var id=materia.tooling.ToolingIds.derive("",core.id,"-foot");
						var shape=machinekit.structural.CastingRegionCad.part(MoldCores.layerRegion(foot.cutting));
						parts.push(shape);members.push({id:id,part:shape,direction:new Vector(0,0,1)});
						var ids=MoldCores.baseAssemblyMembers(core).concat([id]),selected=members.filter(member->ids.indexOf(member.id)>=0);
						var bonded=Solids.union([for(member in selected)new Part(member.part.shape.cloneShape())]);
						for(member in selected)members.remove(member);
						var loaded=materia.tooling.ToolingIds.derive("",core.id,"-base-loaded");
						parts.push(bonded);members.push({id:loaded,part:bonded,direction:new Vector(0,0,1)});
						groups.push({id:loaded,members:ids});
					}
				}else{
					var shape=coreShapes.get(core.id);
					if(shape==null)throw "Missing EPS assembly solid: "+core.id;
					members.push({id:core.id,part:shape,direction:new Vector(0,0,1)});
				}
			}
			for (id in rowIds) {
				var ids = [materia.tooling.ToolingIds.derive("", id, "-fixture")].concat([for (c in bridgeCores)
					if (c.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge && MoldCores.bridgeId(c)==id)
						for(member in MoldCores.bridgeAssemblyMembers(c))member]),
					pieces:Array<Part> = [];
				var n = members.length - 1;
				while (n >= 0) {
					if (ids.indexOf(members[n].id) >= 0) {
						pieces.push(new Part(members[n].part.shape.cloneShape()));
						members.splice(n, 1);
					}
					n--;
				}
				var shape = Solids.union(pieces);
				parts.push(shape);
				members.push({
					id: materia.tooling.ToolingIds.derive("", id, "-loaded"),
					part: shape,
					direction: new Vector(0, 0, 1)
				});
				groups.push({id: materia.tooling.ToolingIds.derive("", id, "-loaded"), members: ids});
			}
			// A pressure board stays dry and comes off after cure. Before the pour,
			// install each connected bridge/core/strut unit as a supported assembly.
			function pressurePanels(core:CastingMoldCore):Array<String>
				return core.support.panelIds.filter(id->id!="bridge-left-rail" && id!="bridge-right-rail");
			var retainedRows:Array<Array<CastingMoldCore>>=[];
			for(core in retained){
				var row=[core],index=retainedRows.length-1;
				while(index>=0){
					var joins=retainedRows[index].filter(other->pressurePanels(other)
						.filter(id->pressurePanels(core).indexOf(id)>=0).length>0).length>0;
					if(joins){row=row.concat(retainedRows[index]);retainedRows.splice(index,1);}
					index--;
				}
				retainedRows.push(row);
			}
			function contains(groupId:String,leaf:String,path:Array<String>):Bool {
				if(groupId==leaf)return true;
				if(path.indexOf(groupId)>=0)throw "Cyclic retained-core assembly group: "+groupId;
				for(group in groups.concat(permanent))if(group.id==groupId)
					for(id in group.members)if(contains(id,leaf,path.concat([groupId])))return true;
				return false;
			}
			for(row in retainedRows){
				row.sort((a,b)->Reflect.compare(a.id,b.id));
				var assemblyIds:Array<String>=[],selected:Array<RemovalMember>=[];
				for(core in row)for(panel in pressurePanels(core)){
					var matches=members.filter(member->contains(member.id,panel,[]));
					if(matches.length!=1)throw "Missing retained core bridge member: "+panel;
					if(selected.indexOf(matches[0])<0)selected.push(matches[0]);
				}
				var shape=Solids.building([],owned->{
					for(core in row){
						owned.push(machinekit.structural.CastingRegionCad.part(MoldCores.occupied(core)));
						assemblyIds.push(core.id);
						for(contact in core.support.precastContacts){
							var strut=p.document.input.precastStruts.filter(strut->strut.id==contact.strutId)[0];
							owned.push(machinekit.structural.CastingRegionCad.part(strutRegion(p,strut)));
							assemblyIds.push(strut.id);
						}
					}
					for(member in selected){owned.push(new Part(member.part.shape.cloneShape()));assemblyIds.push(member.id);}
					return Solids.union(owned);
				});
				for(member in selected)members.remove(member);
				var id=materia.tooling.ToolingIds.derive("",row[0].id,"-loaded");
				parts.push(shape);members.push({id:id,part:shape,direction:new Vector(0,0,1)});
				groups.push({id:id,members:assemblyIds});
			}
			for (i in p.document.input.inserts) {
				var shape = steel(p, i.id);
				parts.push(shape);
				var insertId=materia.tooling.ToolingIds.derive("insert-",i.id);
				var attached=[for (hardware in anchorComponents(p,i.id)) hardware.id];
				var motionId=attached.length==0 ? insertId : materia.tooling.ToolingIds.derive("",insertId,"-anchored");
				if (attached.length>0) groups.push({id:motionId,members:[insertId].concat(attached)});
				members.push({
					id: motionId,
					part: shape,
					direction: new Vector(0, 0, 1)
				});
			}
			members[0].direction = new Vector(0, 0, 1);
			for (member in members) {
				var fixtures = p.fixtures.filter(f -> MoldFixtures.volumes(f).filter(volume -> volume.id==member.id).length==1);
				if (fixtures.length == 1) {
					var v = fixtures[0].assemblyClearance;
					member.direction = new Vector(v.x, v.y, v.z);
				}
				if (p.hardware.filter(h -> h.id==member.id && h.kind==SupportHardwareKind.FixturePin).length==1)
					member.direction=new Vector(0,0,-1);
				if (p.hardware.filter(h -> h.id==member.id && h.kind==SupportHardwareKind.BoreMask && !threadedMask(p,h)).length==1) {
					var direction=boreMaskRecord(p,member.id).protection.withdrawal;
					member.direction=new Vector(direction.x,direction.y,direction.z);
				}
			}
			// Before casting, bed-facing tooling can approach downward from the open
			// fill side. After cure those same pieces leave below the released bed.
			for (m in members)
				if (m.direction.z < -0.9 && p.hardware.filter(h -> h.id == m.id
					&& (h.kind == SupportHardwareKind.FixturePin || h.kind==SupportHardwareKind.BoreMask)).length == 0)
					m.direction = new Vector(0, 0, 1);
			var assemblyMotions = [
				for (m in members)
					{
						id: m.id,
						x: m.direction.x,
						y: m.direction.y,
						z: m.direction.z
					}
			];
			Sys.println("Checking full assembly sweeps for " + members.length + " members");
			var assembly = SweptRemoval.analyze(members, 1e-4);
			for (part in parts)
				if (!part.shape.isClosed())
					part.close();
			var validation = p.validation.concat(coreFits).concat([
				for (name in ["reconstruction", "coverage", "assembly-graph", "demolding-graph"])
					materia.tooling.export.ToolingLoads.check("tooling.cad."
						+ name, [p.document.input.id],
						materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass, "Exact CAD "
						+ name
						+ " check passed.",
						[materia.tooling.casting.CastingMoldPlanner.fingerprint(p)])
			]).concat(ReplicationContactCad.review(p.document.input,p.options.replicationJigs)).concat(FoundationJointCad.review(p.document.input))
				.concat(InsertAnchorGeometry.review(p.document.input)).concat(InsertAnchorGeometry.reviewMold(p));
			return {
				fingerprint: materia.tooling.casting.CastingMoldPlanner.fingerprint(p),
				handling: p.handling,
				demoldingSteps: steps(p, demolding.removalOrder),
				symmetricDifferenceMm3: difference,
				sourceVolumeMm3: ov,
				reconstructedVolumeMm3: rv,
				assembly: assembly,
				demolding: demolding,
				permanentGroups: permanent,
				assemblyGroups: groups,
				validation: validation,
				fabrication: materia.tooling.ToolingValidation.ToolingReadiness.evaluate(validation),
				assemblyMotions: assemblyMotions,
				demoldingMotions: demoldingMotions
			};
		} catch (e:Dynamic) {
			for (part in parts)
				if (!part.shape.isClosed())
					part.close();
			throw e;
		}
	}
}
