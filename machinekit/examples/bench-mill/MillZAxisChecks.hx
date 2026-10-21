import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.component.MachineKitComponents;
import machinekit.milling.*;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

/** Exact CAD of conservative swept envelopes proves clearance over the entire
 * translation interval; sampled kinematics additionally checks the actual joint.
 */
class MillZAxisChecks {
	static var assertions = 0;
	static function check(ok:Bool, label:String):Void { assertions++; if (!ok) throw label; }
	static function near(a:Float, b:Float, label:String):Void check(Math.abs(a-b) < 1e-6, '$label ($a vs $b)');
	static function pose(poses:Map<String, AssemblyFrame>, id:String):AssemblyFrame {
		var value:AssemblyFrame = poses.get(id); if (value == null) throw 'Missing pose $id'; return value;
	}
	static function rejects(action:Void->Void, label:String):Void {
		var rejected = false; try action() catch (_:Dynamic) rejected = true; check(rejected, label);
	}
	static function noOverlap(a:Part, b:Part, label:String):Void {
		var common = a.intersect(b), volume = common.volume(); common.close();
		check(volume < 1e-5, '$label: overlap $volume mm3');
	}
	public static function main():Void {
		rejects(()->machinekit.structural.CastInsert.recipeType().create(),"Metadata instance does not default a required cast-insert role");
		AxisBuilderChecks.run();
		Sys.println("Shared axis builder regressions passed");
		var reference = new BenchMill(false, false), f = MillFramePreview.layout(reference);
		var l = MillFramePreview.zLayout(reference, f), frame = new MillFrame(f, l, new MillXYStageLayout(f, l.spindleY, BenchMill.RAIL)), axis = new MillZAxis(l);
		axis.validate(); frame.validate();
		rejects(() -> new MillZAxisLayout(f,l.headBottom,l.headHeight,l.spindleY,l.lower,l.profile,null,l.crossDriveZ,
			{carriageWidth:1}), "reject carriage narrower than guide footprint");
		rejects(() -> new MillZAxisLayout(f,l.headBottom,l.headHeight,l.spindleY,l.lower,l.profile,null,l.crossDriveZ,
			{headBoltLength:12}), "reject insufficient head bolt engagement");
		rejects(() -> new MillZAxisLayout(f,l.headBottom,l.headHeight,l.spindleY,l.lower,l.profile,null,l.crossDriveZ,
			{overtravelLimit:Math.NaN}), "reject nonfinite Z overtravel");
		near(axis.axisOvertravel("z"), l.overtravel, "all four blocks determine guide overtravel");
		var poses:Map<String, AssemblyFrame> = axis.solvedPoses();
		near(pose(poses, "screw").x, 0, "screw centred between rails");
		near(pose(poses, "motor").x, 0, "motor centred on screw");
		near(pose(poses, "motor").y, l.screwY, "coaxial motor");
		check(l.motor.designation=="ELM2H-0750LA80E","Z installs the braked Leadshine catalog motor");
		var mountCache=new PosedParts();
		for(member in axis.components())if(member.id=="motorMount"||StringTools.startsWith(member.id,"motorBolt")||StringTools.startsWith(member.id,"motorNut")) {
			near(mountCache.overlap(l.motor,poses.get("motor"),member.component,poses.get(member.id)),0,"Catalog Z pilot, flange and fasteners clear: "+member.id);
		}
		for(member in axis.components())if(member.id=="coupling"||member.id=="screw"||StringTools.startsWith(member.id,"brake-"))
			near(mountCache.overlap(member.component,poses.get(member.id),l.fixed,poses.get("motorMount")),0,"Z drive internals clear purchased housing: "+member.id);
		near(mountCache.overlap(l.screw,poses.get("screw"),l.floating,poses.get("floatingBearing")),0,"BF15 bearing and installed retaining ring clear machined shaft");
		mountCache.close();
		check(2 * l.blockSpan >= 140, "Z guide blocks have a useful vertical moment arm");
		check(l.carriageThickness >= 20, "broad carriage retains head attachment depth");
		check(axis.head.parameterNumber("rootExtension") > 0, "head has a deep braced root");
		var headLocal = axis.head.geometry(), headWorld = PosedParts.moved(headLocal, pose(poses, "head")); headLocal.close();
		for (member in axis.components()) if (StringTools.startsWith(member.id, "headBolt")) {
			var boltPose = pose(poses, member.id), local = member.component.geometry(), bolt = PosedParts.moved(local, boltPose); local.close();
			noOverlap(headWorld, bolt, "head bolt clears flange bore and counterbore"); bolt.close();
			var access = Part.cylinderAlongY(5.5, l.headBack - axis.head.parameterNumber("reach") - 20, l.headBack - 12, boltPose.x, boltPose.z);
			noOverlap(headWorld, access, "outboard head bolts admit a straight tool approach"); access.close();
		}
		headWorld.close();
		for (member in axis.components()) {
			var shape = member.component.geometry();
			check(shape.valid() && shape.solidCount() >= 1, 'valid connected ${member.id}'); shape.close();
		}
		var crossShaft = Part.cylinderAlongY(8, f.columnFront-30, l.screwY+40, 0, f.railPlane+l.block.spec.blockHeight);
		for (id in ["floatingMount","floatingBearing","screw"]) {
			var local=axis.component(id).geometry(), fixed=PosedParts.moved(local,pose(poses,id)); local.close();
			noOverlap(crossShaft,fixed,"raised Y shaft clears lower Z hardware: "+id); fixed.close();
		}
		var unbored=new MillZBearingPlate(axis.component("floatingMount").values().setNumber("crossShaftZ",0)).geometry();
		var blockedCrossing=PosedParts.moved(unbored,pose(poses,"floatingMount")); unbored.close();
		var interference=crossShaft.intersect(blockedCrossing);
		check(interference.volume()>1,"cross-axis negative control detects missing support-plate bore");
		interference.close(); blockedCrossing.close(); crossShaft.close();
		var registry = MachineKitComponents.standard();
		var restored:MachineAssembly = MachineAssembly.decode(axis.encode(), registry);
		restored.validate();
		check(restored.components().length == axis.components().length, "axis recipe roundtrip");
		var restoredModel = new AssemblyModel("mm"); restored.addTo(restoredModel, "");
		var restoredState = new AssemblyState(restoredModel.definition("restored-z"));
		restoredState.setJoint("z", l.lower); restoredState.forwardKinematics();
		near(restoredState.worldPose("head").z, l.headBottom + l.lower, "restored axis retains real Z motion");
		rejects(() -> new MillZAxisLayout(f, l.headBottom, l.headHeight, l.spindleY, -800), "reject excessive Z travel");
		rejects(() -> new MillZMotorStand(new machinekit.component.ComponentValues().setNumber("height", 8)), "reject absent motor stand legs");
		rejects(() -> new MillZCarriage(new machinekit.component.ComponentValues().setNumber("width", 100)), "reject unsupported rail span");
		var model = new AssemblyModel("mm"); axis.addTo(model, "");
		var state = new AssemblyState(model.definition("central-z"));
		for (step in 0...51) {
			var q = l.lower * step / 50;
			state.setJoint("z", q); state.forwardKinematics();
			var moving = state.worldPose("head"), stationary = state.worldPose("motor");
			near(moving.z, l.headBottom + q, "head follows Z");
			near(stationary.z, l.motorFace, "Z motor remains stationary");
			var nut = state.worldPose("nut");
			near(nut.x, 0, "nut remains centred"); near(nut.y, l.screwY, "nut remains coaxial");
			near(nut.z, l.headCentre + l.nut.bodyLength / 2 + q, "nut follows carriage");
			near(state.joint("screw-turn"), l.nut.rotationFor(-q), "signed screw rotation follows handedness and axis direction");
		}
		var fullAudit=MillZAxisIntegrity.check(axis,frame);fullAudit.throwIfErrors();
		check(!fullAudit.hasErrors(),"continuous CAD full-stroke audit");
		var blocked = MillZAxisIntegrity.check(axis, new MillFrame(f));
		check(blocked.hasErrors(), "audit rejects a column without the central channel");
		check(blocked.items.filter(item -> item.code == "mill-z.full-stroke-collision").length > 0,
			"negative control detects an actual swept-volume collision");
		for(id in ["motorMount","floatingBearing"]) {
			var foot=F.compose(poses.get(id),axis.component(id).connector("base").frame);
			near(foot.y,f.columnFront,"supplier base meets backing plate");
		}
		near(pose(poses,"motorMount").z,pose(poses,"motor").z,"motor seats on integrated housing");
		check(l.screw.thread.screwDiameter==20,"Z has a matched SFU2005 race");
		check(!axis.components().filter(m->m.id=="motorStand"||m.id=="couplingGuard"||m.id=="fixedBearing").iterator().hasNext(),"integrated housing replaces custom stack");
		for (pair in [["carriage", "blockLeftLower"], ["carriage", "blockLeftUpper"], ["carriage", "blockRightLower"], ["carriage", "blockRightUpper"], ["motor", "motorMount"], ["motorMount", "fixedMount"],
			["floatingBearing", "floatingMount"], ["nut", "carriage"], ["head", "carriage"]]) {
			var first = axis.components().filter(member -> member.id == pair[0])[0].component.geometry();
			var second = axis.components().filter(member -> member.id == pair[1])[0].component.geometry();
			var a = PosedParts.moved(first, poses.get(pair[0])), b = PosedParts.moved(second, poses.get(pair[1]));
			first.close(); second.close();
			near(a.distance(b), 0, 'physical support contact ${pair[0]}/${pair[1]}');
			noOverlap(a, b, 'no interference at support ${pair[0]}/${pair[1]}'); a.close(); b.close();
		}
		var study = MillFramePreview.concept(reference, frame), studyModel = new AssemblyModel("mm");
		study.addTo(studyModel, "");
		var studyState = new AssemblyState(studyModel.definition("mill-z-integrated"));
		var integratedPoses:Map<String, AssemblyFrame> = study.solvedPoses();
		for (pair in [["zAxis/head", "spindle/motorCarrier"], ["spindle/motorCarrier", "spindle/motor"], ["spindle/spindle","spindle/reactionFrame"], ["spindle/reactionFrame","spindle/cylinder"]]) {
			var first = study.components().filter(member -> member.id == pair[0])[0].component.geometry();
			var second = study.components().filter(member -> member.id == pair[1])[0].component.geometry();
			var a = PosedParts.moved(first, integratedPoses.get(pair[0])), b = PosedParts.moved(second, integratedPoses.get(pair[1]));
			first.close(); second.close(); near(a.distance(b), 0, 'spindle motor support ${pair[0]}/${pair[1]}');
			noOverlap(a, b, "spindle motor support interference"); a.close(); b.close();
		}
		var payloadAudit=MillSpindleIntegrity.checkPayload(study,l); payloadAudit.throwIfErrors();
		check(!payloadAudit.hasErrors(), "spindle package clears itself and fixed structure");
		var obstruction = new MachineAssembly(), obstructed = new MachineAssembly();
		obstruction.addComponent("releaseObstruction", new MillPanel(20, 20, 20), F.translation(0, l.spindleY, (l.headBottom+l.headHeight+80) + 20));
		obstruction.addComponent("releaseCylinder", new MillPanel(20,20,20),F.translation(0,l.spindleY,(l.headBottom+l.headHeight+80)+20));
		obstructed.include("spindle", obstruction);
		check(MillSpindleIntegrity.checkPayload(obstructed, l).hasErrors(), "release-space check rejects an obstructed actuator allocation");
		var upperTool = studyState.worldPose("spindle/tool").z;
		studyState.setJoint("zAxis/z", l.lower); studyState.forwardKinematics();
		near(studyState.worldPose("spindle/tool").z - upperTool, l.lower, "spindle payload follows real Z joint");
		near(studyState.worldPose("xyStage/table").z, pose(study.solvedPoses(), "xyStage/table").z, "Z motion leaves XY table stationary");
		// DT1: drive and spindle selections are layout inputs; defaults reproduce the installed parts.
		var compact = new MillCapacityLayout(), compactZ = compact.zLayout();
		check(compactZ.nut.spec.model == "SFU2005" && compactZ.drive.endProfile == "BK15", "default Z drive is the SFU2005 / BK15 set");
		check(Std.isOfType(compactZ.spindle, MillBeltSpindle), "default spindle package is the belt-driven BT30");
		near(compact.headBottom - compact.gaugeZ, compact.spindle.seat(), "head sits the package seat above the gauge line");
		var explicit = new MillCapacityLayout(400, 300, 380, {spindle: new MillBeltSpindle(), fixtureHeight: 100, noseMin: 150});
		near(explicit.zLayout().screwTop, compactZ.screwTop, "explicit default inputs give the same Z datums");
		var turret = new MillCapacityLayout(400, 300, 380, {toolChangeHeight: 120}), turretZ = turret.zLayout();
		near(turretZ.lower, -500, "tool-change height extends the Z stroke above the working range");
		near(turret.gaugeZ, compact.gaugeZ, "tool-change height leaves the working gauge range unchanged");
		check(turret.frame.zRailLength >= compact.frame.zRailLength + 120, "Z rails extend over the tool-change height");
		var turretAxis = new MillZAxis(turretZ);
		check(turretAxis.components().length > 0, "Z axis builds with the tool-change reserve");
		Sys.println('Central Z axis: $assertions assertions passed');
	}
}
