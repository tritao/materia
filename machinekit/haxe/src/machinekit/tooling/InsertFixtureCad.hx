package machinekit.tooling;

import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingIds;
import materia.tooling.casting.InsertFixturePlanner.InsertPlatePlan;

typedef InsertFixtureReview = {
	var removal:SweptRemoval.RemovalReport;
	var releaseBeforeTranslation:Array<String>;
}

/** Shared CAD realization of a source-hole carrier: all steel comes from the source input. */
class InsertFixtureCad {
	static function add(assembly:MachineAssembly,root:String,id:String,part:MachineComponent,pose:AssemblyFrame):Void {
		var rootPose = assembly.solvedPoses().get(root);
		assembly.addComponent(id,part,pose);
		assembly.addMemberConnector(root,ToolingIds.derive("",id,"-seat"),F.compose(F.inverse(rootPose),pose));
		assembly.addMemberConnector(id,"fixture-seat",F.identity());
		assembly.addMate(ToolingIds.derive("",id,"-position"),"fixed",root,ToolingIds.derive("",id,"-seat"),id,"fixture-seat");
	}

	/** Exact removal graph after the explicitly required unscrewing operations.
	 * Insert/casting solids are stationary obstacles; only tooling is swept.
	 */
	public static function check(plan:InsertPlatePlan):InsertFixtureReview
		return checkAssembly(plan,assembly(plan,true),[]);

	public static function checkAssembly(plan:InsertPlatePlan,assembly:MachineAssembly,
			additional:Array<{id:String,direction:cadkit.modeling.Vector}>):InsertFixtureReview {
		materia.tooling.casting.InsertFixturePlanner.validatePlate(plan);
		var poses = assembly.solvedPoses();
		var fixedIds = [for (insert in plan.document.input.inserts) ToolingIds.derive("insert-",insert.id)];
		fixedIds.push("cured-casting");
		var directions = new Map<String,cadkit.modeling.Vector>(), release:Array<String> = [];
		var threadMates = new Map<String,String>();
		for (site in plan.sites) {
			threadMates.set(site.id,ToolingIds.derive("insert-",site.insertId));
			if (site.plug != null) threadMates.set(ToolingIds.derive("",site.id,"-far-end"),ToolingIds.derive("insert-",site.insertId));
		}
		for (mask in plan.masks) {
			var insert = plan.document.input.inserts.filter(row -> row.id == mask.insertId)[0];
			var hole = insert.holes.filter(row -> row.id == mask.protection.sourceHoleId)[0];
			if (hole.thread != null) {
				threadMates.set(mask.protection.hardwareId,ToolingIds.derive("insert-",insert.id));
				if (mask.protection.farEndId != null) threadMates.set(mask.protection.farEndId,ToolingIds.derive("insert-",insert.id));
			}
		}
		for (member in plan.assembly.members)
			directions.set(member.panel.id,new cadkit.modeling.Vector(member.withdrawal.x,member.withdrawal.y,member.withdrawal.z));
		for (i in 0...2) directions.set("fixture-locator-"+i,new cadkit.modeling.Vector(0,0,-1));
		for (i in 0...plan.clampPoints.length) {
			var id = "fixture-clamp-"+i;
			release.push(id); release.push(id+"-nut");
			directions.set(id,new cadkit.modeling.Vector(0,0,1));
			directions.set(id+"-top-washer",new cadkit.modeling.Vector(0,0,1));
			directions.set(id+"-bottom-washer",new cadkit.modeling.Vector(0,0,-1));
			directions.set(id+"-nut",new cadkit.modeling.Vector(0,0,-1));
		}
		for (site in plan.sites) {
			var axis = F.transformVector(site.entry,0,0,-1);
			var direction = new cadkit.modeling.Vector(axis.x,axis.y,axis.z);
			directions.set(site.id,direction); directions.set(ToolingIds.derive("",site.id,"-washer"),direction);
			release.push(site.id);
			if (site.plug != null) {
				var id = ToolingIds.derive("",site.id,"-far-end");
				directions.set(id,direction.scale(-1)); release.push(id);
			}
		}
		for (mask in plan.masks) {
			var axis = mask.protection.withdrawal;
			var direction = new cadkit.modeling.Vector(axis.x,axis.y,axis.z);
			directions.set(mask.protection.hardwareId,direction);
			release.push(mask.protection.hardwareId);
			if (mask.protection.farEndId != null) {
				directions.set(mask.protection.farEndId,direction.scale(-1)); release.push(mask.protection.farEndId);
			}
		}
		for (row in additional) {
			if (directions.exists(row.id) || fixedIds.indexOf(row.id) >= 0)
				throw "Duplicate fixture removal direction: " + row.id;
			directions.set(row.id,row.direction);
		}
		var cache = new machinekit.assembly.PosedParts(), owned:Array<cadkit.modeling.Part> = [];
		var moving:Array<SweptRemoval.RemovalMember> = [], fixed:Array<{id:String,part:cadkit.modeling.Part}> = [];
		var report:SweptRemoval.RemovalReport;
		try {
			var min = new cadkit.modeling.Vector(1e30,1e30,1e30), max = new cadkit.modeling.Vector(-1e30,-1e30,-1e30);
			for (row in assembly.components()) {
				var part = cache.posed(row.component,poses.get(row.id)); owned.push(part);
				// Derived removal volumes carry geometry, not threaded-face manufacturing annotations.
				// This is an owned posed copy; source shapes and their attributes are unchanged.
				part.shape.setThreadedFaces([]);
				var bounds = part.shape.bounds(), lo = cadkit.modeling.Vector.fromNative(bounds.get_min()), hi = cadkit.modeling.Vector.fromNative(bounds.get_max());
				min = new cadkit.modeling.Vector(Math.min(min.x,lo.x),Math.min(min.y,lo.y),Math.min(min.z,lo.z));
				max = new cadkit.modeling.Vector(Math.max(max.x,hi.x),Math.max(max.y,hi.y),Math.max(max.z,hi.z));
				var direction = directions.get(row.id);
				if (direction == null) {
					if (fixedIds.indexOf(row.id) < 0) throw "Unknown fixture removal member: " + row.id;
					fixed.push({id:row.id,part:part});
				} else moving.push({id:row.id,part:part,direction:direction});
			}
			report = SweptRemoval.analyze(moving);
			var travel = max.subtract(min).length()+1;
			for (member in moving) {
				var swept = SweptRemoval.sweep(member.part,member.direction.scale(travel));
				try for (obstacle in fixed)
					// Thread engagement is released by the declared unscrewing operation.
					// Its source thread/coaxial entry and depth budget were validated above;
					// the tap-drill B-rep is not a helical thread-clearance model.
					if (threadMates.get(member.id) != obstacle.id && SweptRemoval.intersectionVolume(swept,obstacle.part) > report.contactVolumeToleranceMm3)
						throw "Fixture tooling trapped by stationary source: " + member.id + " / " + obstacle.id;
				catch (error:Dynamic) {swept.close(); throw error;}
				swept.close();
			}
			report.travelMm = Math.max(report.travelMm,travel);
		} catch (error:Dynamic) {
			for (part in owned) part.close(); cache.close(); throw error;
		}
		for (part in owned) part.close(); cache.close();
		release.sort(Reflect.compare);
		return {removal:report,releaseBeforeTranslation:release};
	}

	/** Explicit fastener release precedes the translation checks; the insert stays in the casting. */
	public static function assembly(plan:InsertPlatePlan,cured:Bool = false):MachineAssembly {
		materia.tooling.casting.InsertFixturePlanner.validatePlate(plan);
		var assembly = PanelKit.assembly(plan.assembly);
		var lower = plan.assembly.members[0];
		var root = lower.panel.id, hardware = plan.document.recipe.hardware;
		var thickness = plan.document.recipe.panel.measuredThickness;
		var pinIndex = 0;
		for (at in [plan.primary,plan.secondary]) {
			var pin = new ToolingPin(new ComponentValues().setNumber("diameter",hardware.fixturePinDiameter)
				.setNumber("length",2*thickness+hardware.fixturePinEmbed));
			add(assembly,root,"fixture-locator-"+pinIndex++,pin,F.compose(lower.pose,F.translation(at.x,at.y,0)));
		}
		for (i in 0...plan.clampPoints.length) {
			var at = plan.clampPoints[i];
			ToolingHardwareCad.throughBolt(assembly,"fixture-clamp-"+i,hardware.supportScrewDiameter,2*thickness,
				F.compose(lower.pose,F.translation(at.x,at.y,2*thickness)));
		}
		for (insert in plan.document.input.inserts) {
			var pose = F.compose(plan.document.setup.sourceToMold,insert.pose);
			add(assembly,root,ToolingIds.derive("insert-",insert.id),ToolingInsertCad.component(insert),pose);
		}
		for (site in plan.sites) {
			var washer = machinekit.standard.FlatWasher.metric(site.thread.designation);
			if (Math.abs(washer.thickness-site.washerThickness) > 1e-7)
				throw "Fixture washer differs from selected source-thread hardware";
			var head = F.compose(site.entry,F.fromRotationMatrix(0,0,-thickness,[1,0,0,0,-1,0,0,0,-1]));
			ToolingHardwareCad.fixtureScrew(assembly,site.id,site.thread.designation,site.screwLength-site.washerThickness,head);
			if (site.plug != null) {
				var plug = site.plug;
				var source = plan.document.input.inserts.filter(row -> row.id == site.insertId)[0];
				var hole = source.holes.filter(row -> row.id == site.sourceHoleId)[0];
				add(assembly,root,ToolingIds.derive("",site.id,"-far-end"),
					machinekit.standard.SocketSetScrew.forThread(machinekit.structural.InsertHoles.threadSpec(site.thread).mating(),plug.length),
					F.compose(site.entry,F.translation(0,0,hole.depth-plug.length)));
			}
		}
		for (mask in plan.masks) {
			var insert = plan.document.input.inserts.filter(row -> row.id == mask.insertId)[0];
			var hole = insert.holes.filter(row -> row.id == mask.protection.sourceHoleId)[0];
			var entry = F.compose(plan.document.setup.sourceToMold,F.compose(insert.pose,hole.frame));
			add(assembly,root,mask.protection.hardwareId,ToolingPin.boreMask(hole,mask.protection.plugLength),entry);
			if (mask.protection.farEndId != null)
				add(assembly,root,mask.protection.farEndId,ToolingPin.boreMask(hole,mask.protection.plugLength),
					F.compose(entry,F.translation(0,0,hole.depth-(cast mask.protection.plugLength:Float))));
		}
		if (cured)
			add(assembly,root,"cured-casting",ToolingSourceCasting.fromInput(plan.document.input),plan.document.setup.sourceToMold);
		// Standard hardware helpers place world occurrences; bind them in the same reference frame.
		var poses = assembly.solvedPoses(), rootPose = poses.get(root);
		for (row in assembly.components()) {
			if (!StringTools.startsWith(row.id,"fixture-clamp-")
				&& plan.sites.filter(site -> row.id == site.id || row.id == ToolingIds.derive("",site.id,"-washer")).length == 0)
				continue;
			assembly.addMemberConnector(root,ToolingIds.derive("",row.id,"-seat"),F.compose(F.inverse(rootPose),poses.get(row.id)));
			assembly.addMemberConnector(row.id,"fixture-seat",F.identity());
			assembly.addMate(ToolingIds.derive("",row.id,"-position"),"fixed",root,ToolingIds.derive("",row.id,"-seat"),row.id,"fixture-seat");
		}
		return assembly;
	}
}
