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
		AxisBuilderChecks.run();
		Sys.println("Shared axis builder regressions passed");
		var reference = new BenchMill(false, false), f = MillFramePreview.layout(reference);
		var l = MillFramePreview.zLayout(reference, f), frame = new MillFrame(f, l, new MillXYStageLayout(f, l.spindleY, BenchMill.RAIL)), axis = new MillZAxis(l);
		axis.validate(); frame.validate();
		near(axis.axisOvertravel("z"), l.overtravel, "all four blocks determine guide overtravel");
		var poses:Map<String, AssemblyFrame> = axis.solvedPoses();
		near(pose(poses, "screw").x, 0, "screw centred between rails");
		near(pose(poses, "motor").x, 0, "motor centred on screw");
		near(pose(poses, "motor").y, l.screwY, "coaxial motor");
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
			check(shape.valid() && shape.solidCount() == 1, 'valid connected ${member.id}'); shape.close();
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
		check(!MillZAxisIntegrity.check(axis, frame).hasErrors(), "continuous CAD full-stroke audit");
		var blocked = MillZAxisIntegrity.check(axis, new MillFrame(f));
		check(blocked.hasErrors(), "audit rejects a column without the central channel");
		check(blocked.items.filter(item -> item.code == "mill-z.full-stroke-collision").length > 0,
			"negative control detects an actual swept-volume collision");
		// Bearing feet contact their plates; motor face and stand top coincide.
		for (id in ["fixedBearing", "floatingBearing"]) {
			var part = axis.components().filter(m -> m.id == id)[0].component;
			var foot = F.compose(poses.get(id), part.connector("mount").frame);
			near(foot.y, f.columnFront, "bearing foot meets backing plate");
		}
		near(pose(poses, "motorStand").z + l.motorStandHeight, pose(poses, "motor").z, "motor supported on stand");
		for (pair in [["carriage", "blockLeftLower"], ["carriage", "blockLeftUpper"], ["carriage", "blockRightLower"], ["carriage", "blockRightUpper"], ["motor", "motorStand"], ["motorStand", "fixedMount"], ["fixedBearing", "fixedMount"],
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
		for (pair in [["zAxis/head", "spindle/spindleMotorRiser"], ["spindle/spindleMotorRiser", "spindle/spindleMotorShelf"],
			["spindle/spindleMotorShelf", "spindle/spindleMotor"]]) {
			var first = study.components().filter(member -> member.id == pair[0])[0].component.geometry();
			var second = study.components().filter(member -> member.id == pair[1])[0].component.geometry();
			var a = PosedParts.moved(first, integratedPoses.get(pair[0])), b = PosedParts.moved(second, integratedPoses.get(pair[1]));
			first.close(); second.close(); near(a.distance(b), 0, 'spindle motor support ${pair[0]}/${pair[1]}');
			noOverlap(a, b, "spindle motor support interference"); a.close(); b.close();
		}
		check(!MillSpindleServiceSpace.checkPayload(study, l).hasErrors(), "spindle drive clears reserved release space");
		var obstruction = new MachineAssembly(), obstructed = new MachineAssembly();
		obstruction.addComponent("releaseObstruction", new MillPanel(20, 20, 20), F.translation(0, l.spindleY, MillSpindleServiceSpace.bottom(l) + 20));
		obstructed.include("spindle", obstruction);
		check(MillSpindleServiceSpace.checkPayload(obstructed, l).hasErrors(), "release-space check rejects an obstructed actuator allocation");
		var upperTool = studyState.worldPose("spindle/tool").z;
		studyState.setJoint("zAxis/z", l.lower); studyState.forwardKinematics();
		near(studyState.worldPose("spindle/tool").z - upperTool, l.lower, "spindle payload follows real Z joint");
		near(studyState.worldPose("xyStage/table").z, pose(study.solvedPoses(), "xyStage/table").z, "Z motion leaves XY table stationary");
		Sys.println('Central Z axis: $assertions assertions passed');
	}
}
