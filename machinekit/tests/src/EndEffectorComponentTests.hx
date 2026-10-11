import machinekit.component.MachineComponent;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.MassProperties.MassSource;
import machinekit.pneumatic.PneumaticManifold;
import machinekit.pneumatic.SuctionCup;
import machinekit.pneumatic.VacuumGenerator;
import machinekit.pneumatic.schmalz.SchmalzPushInFitting;
import machinekit.pneumatic.schmalz.SchmalzSuctionCup;
import machinekit.pneumatic.schmalz.SchmalzVacuumGenerator;
import machinekit.pneumatic.schmalz.SchmalzVacuumHose;
import machinekit.robotics.schmalz.SchmalzSxtMaster;
import machinekit.robotics.schmalz.SchmalzSxtTool;
import cadkit.modeling.Vector;
import cadkit.modeling.Part;
import cadkit.parametric.GeometricConnectors;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import eoat.SchmalzEndEffectorExample;
import machinekit.component.PortInterfaces;
import machinekit.component.PortInterface;
import machinekit.robotics.FrameBar;
import machinekit.robotics.ParallelGripper;
import machinekit.robotics.ParallelGripperBody;
import machinekit.robotics.ParallelGripperSpec;
import machinekit.robotics.ParallelGripperJaw;
import machinekit.robotics.ParallelGripperFinger;
import machinekit.component.CollisionHullFacet;
import machinekit.robotics.ArmGripperTool;
import machinekit.robotics.DualArmGripperTool;
import machinekit.robot.EndEffectorControls;
import machinekit.robotics.RobotFlange;
import machinekit.robot.RobotScene;
import robotkit.execution.ProcessEventValue;
import robotkit.tool.GripperChannels;
import robotkit.manipulation.Grasp;
import robotkit.manipulation.Grasp.GraspPlane;
import robotkit.spatial.Vec3;
import machinekit.robotics.ToolChangerMaster;
import machinekit.robotics.ToolChangerTool;
import machinekit.robotics.CobotArm;
import machinekit.robotics.CobotClass;
import machinekit.assembly.AssemblyPreview;
import machinekit.pneumatic.AirSupply;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

class EndEffectorComponentTests {
	public static function run():Void {
		var vendorCup = new SchmalzSuctionCup("10.01.01.11401");
		var vendorEjector = new SchmalzVacuumGenerator("10.02.01.00563");
		var vendorFitting = new SchmalzPushInFitting("10.08.02.00203");
		var cup30 = new SchmalzSuctionCup("10.01.01.11400");
		var cup50 = new SchmalzSuctionCup("10.01.01.11441");
		if (cup30.massProperties().mass >= vendorCup.massProperties().mass ||
			cup50.massProperties().mass <= vendorCup.massProperties().mass ||
			(cast cup30.effectiveAreaMm2:Float) >= (cast vendorCup.effectiveAreaMm2:Float) ||
			(cast cup50.effectiveAreaMm2:Float) <= (cast vendorCup.effectiveAreaMm2:Float))
			throw "Catalog cup sizes must have distinct mass and rated area";
		var sxtMaster = new SchmalzSxtMaster("10.07.13.00013");
		var sxtTool = new SchmalzSxtTool("10.07.13.00018");
		var masterHalf = sxtMaster.coupling(), toolHalf = sxtTool.coupling();
		if (masterHalf == null || toolHalf == null || masterHalf.key != toolHalf.key ||
			!PortInterfaces.compatible(sxtMaster.port("airOut2").iface,
				sxtTool.port("airIn2").iface) ||
			PortInterfaces.compatible(sxtMaster.port("airOut2").iface,
				sxtTool.port("airIn3").iface) ||
			Math.abs(sxtMaster.massProperties().mass - 1.7) > 1e-9 ||
			Math.abs(sxtTool.massProperties().mass - 0.55) > 1e-9)
			throw "Schmalz SXT pair has incorrect fit, channels or mass";
		var hose = new SchmalzVacuumHose("10.07.09.00001", [new Vector(0, 0, 0),
			new Vector(0, 0, 100), new Vector(100, 0, 100)]);
		if (Math.abs(hose.lengthMm - 200) > 1e-9 ||
			Math.abs(hose.massProperties().mass - 0.0022) > 1e-9 ||
			hose.bom.partNumber.indexOf("10.07.09.00001-L200") != 0 ||
			!PortInterfaces.compatible(hose.port("input").iface,
				vendorFitting.port("hose").iface))
			throw "Cut hose must retain routed length, mass and tube fit";
		var mismatch = new MachineAssembly();
		mismatch.addComponent("fitting", vendorFitting);
		mismatch.addComponent("hose", hose);
		mismatch.exposePort("inlet", "fitting", "hose");
		mismatch.connectPorts("bad-fit", "fitting", "thread", "hose", "input");
		mismatch.validateStructure();
		var rejectedFit = false;
		try mismatch.validate() catch (error:Dynamic)
			rejectedFit = Std.string(error).indexOf("mismatched interfaces") >= 0;
		if (!rejectedFit) throw "Mismatched physical service interfaces must fail validation";
		if (!PortInterfaces.compatible(vendorCup.port("vacuum").iface,
				vendorFitting.port("thread").iface) ||
			PortInterfaces.compatible(Thread("G1/4-F"), Thread("G1/4-F")) ||
			PortInterfaces.compatible(Thread("G1/4-F"), Thread("G1/8-M")))
			throw "Threaded vacuum connections need matching size and opposite sex";
		if (vendorCup.codeOnly || vendorEjector.codeOnly || vendorFitting.codeOnly ||
			vendorCup.bom.description.indexOf("SAF 40") < 0 ||
			vendorEjector.bom.description.indexOf("SBP 05") < 0 ||
			vendorCup.bom.material != "nitrile rubber NBR" ||
			vendorEjector.bom.material != "plastic" ||
			vendorFitting.bom.material != "brass" ||
			vendorCup.bom.partNumber != "10.01.01.11401" ||
			vendorEjector.bom.partNumber != "10.02.01.00563" ||
			vendorFitting.bom.partNumber != "10.08.02.00203")
			throw "Schmalz parts must retain catalog identity in the BOM";
		if (Math.abs((cast vendorCup.effectiveAreaMm2:Float) - 1150) > 1e-9 ||
			Math.abs(vendorCup.massProperties().mass - 0.0136) > 1e-9 ||
			Math.abs(vendorEjector.massProperties().mass - 0.0075) > 1e-9 ||
			Math.abs(vendorFitting.massProperties().mass - 0.015) > 1e-9)
			throw "Schmalz force-derived area or declared masses are incorrect";
		var example = SchmalzEndEffectorExample.build();
		if (example.validate().length != 0 ||
			example.upstreamChain("cup", "vacuum").join(" ← ").indexOf("fitting/thread") < 0 ||
			example.billOfMaterials().lines().length != 6 ||
			example.massPropertiesAtMount().unaccounted.length != 0)
			throw "Schmalz EOAT must validate with a complete service chain, BOM and mass";
		var master = new ToolChangerMaster(2);
		var tool = new ToolChangerTool(2);
		if (master.port("airOut2").name != "airOut2" || tool.port("airIn2").name != "airIn2" ||
			master.port("lock").required != true || tool.port("signalIn").required != true)
			throw "Changer channels or required ports are missing";
		var gripper = new ParallelGripper(new ParallelGripperSpec(32, 50, 60, 12));
		var gripperHousing = [for (entry in gripper.components()) if (entry.id == "body") entry.component][0];
		var gripperFinger = [for (entry in gripper.components()) if (entry.id == "fingerLeft") entry.component][0];
		var fingerRecipe = gripperFinger.componentType();
		if (fingerRecipe == null) throw "A replaceable finger must have a saved component recipe";
		var rebuiltFinger = fingerRecipe.create(gripperFinger.values());
		if (!Std.isOfType(rebuiltFinger, ParallelGripperFinger))
			throw "Saved gripper fingers must rebuild as physical finger parts";
		var rebuiltFingerSpec = (cast rebuiltFinger:ParallelGripperFinger).spec;
		if (rebuiltFingerSpec.expectedPartWidth != 50 || rebuiltFingerSpec.openingTravel != 12 ||
			rebuiltFingerSpec.fingerDepth != gripper.spec.fingerDepth)
			throw "A saved finger recipe must retain its gripper width and opening-derived envelope";
		var toolJawAxis = AssemblyFrames.transformVector(gripperHousing.connector("tcp").frame, 1, 0, 0);
		var toolFingerAxis = AssemblyFrames.transformVector(gripperHousing.connector("tcp").frame, 0, 0, 1);
		var gripperState = new cadkit.modeling.AssemblyState(gripper.definition());
		var fingerPose = gripperState.worldPose("fingerLeft");
		var fingerTip = AssemblyFrames.transformPoint(fingerPose, 0, 0, gripper.spec.fingerLength);
		var tcpFrame = gripperState.worldConnector("body", "tcp");
		var gripperIssues:Array<String> = [];
		if (gripper.spec.totalJawTravel != 24) gripperIssues.push("jaw stroke");
		if (gripper.spec.fingerDepth != gripper.spec.bodyDepth) gripperIssues.push("pad depth");
		if (gripper.connector("tcp").connectorName != "tcp") gripperIssues.push("TCP exposure");
		if (Math.abs(gripperFinger.connector("mount").frame.z) > 1e-9) gripperIssues.push("finger mount frame");
		if (Math.abs(fingerTip.z - tcpFrame.z) > 1e-9) gripperIssues.push('finger/TCP gap ${fingerTip.z - tcpFrame.z}');
		if (Math.abs(toolJawAxis.x - 1) > 1e-9 || Math.abs(toolJawAxis.y) > 1e-9 || Math.abs(toolJawAxis.z) > 1e-9)
			gripperIssues.push('jaw axis ${toolJawAxis.x},${toolJawAxis.y},${toolJawAxis.z}');
		if (Math.abs(toolFingerAxis.x) > 1e-9 || Math.abs(toolFingerAxis.y) > 1e-9 || Math.abs(toolFingerAxis.z - 1) > 1e-9)
			gripperIssues.push('finger axis ${toolFingerAxis.x},${toolFingerAxis.y},${toolFingerAxis.z}');
		if (Math.abs((gripper.spec.expectedPartWidth + gripper.spec.openingTravel - gripper.spec.bodyWidth) / 2 -
			ParallelGripperSpec.HOUSING_SIDE_CLEARANCE) > 1e-9) gripperIssues.push("housing clearance");
		if (gripperHousing.port("open").kind != machinekit.component.PortKind.Signal ||
			gripperHousing.port("close").kind != machinekit.component.PortKind.Signal) gripperIssues.push("signal ports");
		if (gripper.components().filter(entry -> entry.id == "jawLeft" || entry.id == "jawRight").length != 2)
			gripperIssues.push("paired jaws");
		if (gripperIssues.length > 0) throw 'Pneumatic gripper assembly interface is incomplete: ${gripperIssues.join(", ")}';
		var tendingSpec = new ParallelGripperSpec(32, 60, 60, 16, false, 40, 6);
		var tendingFinger = new ParallelGripperFinger(tendingSpec, -1);
		var tendingFingerRecipe = tendingFinger.componentType();
		var tendingJaw = new ParallelGripperJaw(tendingSpec, -1);
		var tendingJawRecipe = tendingJaw.componentType();
		if (tendingSpec.fingerDepth != 40 || tendingSpec.fingerDepth >= tendingSpec.bodyDepth ||
			tendingSpec.fingerWidth != 6 ||
			tendingFingerRecipe == null ||
			tendingJawRecipe == null ||
			(cast tendingFingerRecipe.create(tendingFinger.values()):ParallelGripperFinger).spec.fingerDepth != 40 ||
			(cast tendingFingerRecipe.create(tendingFinger.values()):ParallelGripperFinger).spec.fingerWidth != 6 ||
			Math.abs((cast tendingJawRecipe.create(tendingJaw.values()):ParallelGripperJaw)
				.connector("fingerSeat").frame.x - tendingJaw.connector("fingerSeat").frame.x) > 1e-9)
			throw "Tending contact finger dimensions must survive saved part recipes";
		var supportedGripper = new ParallelGripper(new ParallelGripperSpec(32, 60, 60, 16, true));
		var leftFinger = [for (entry in supportedGripper.components()) if (entry.id == "fingerLeft") entry.component][0];
		var rightFinger = [for (entry in supportedGripper.components()) if (entry.id == "fingerRight") entry.component][0];
		var leftStep = CollisionHullFacet.of(leftFinger), rightStep = CollisionHullFacet.of(rightFinger);
		if (leftStep == null || rightStep == null || leftStep.hulls.length != 2 || rightStep.hulls.length != 2 ||
			!(leftFinger.massProperties().mass > gripper.components().filter(entry -> entry.id == "fingerLeft")[0].component.massProperties().mass))
			throw "A supported gripper needs separate convex pad and load-bearing step shapes";
		var leftToeMinX = Math.POSITIVE_INFINITY, rightToeMaxX = Math.NEGATIVE_INFINITY;
		var leftToeMinZ = Math.POSITIVE_INFINITY, leftToeMaxZ = Math.NEGATIVE_INFINITY;
		for (index in 0...leftStep.hulls[1].length) {
			var coordinate = index % 3;
			var value = leftStep.hulls[1][index];
			if (coordinate == 0) leftToeMinX = Math.min(leftToeMinX, value);
			if (coordinate == 2) {
				leftToeMinZ = Math.min(leftToeMinZ, value);
				leftToeMaxZ = Math.max(leftToeMaxZ, value);
			}
		}
		for (valueIndex in 0...rightStep.hulls[1].length) if (valueIndex % 3 == 0)
			rightToeMaxX = Math.max(rightToeMaxX, rightStep.hulls[1][valueIndex]);
		if (leftToeMinX <= 0 || rightToeMaxX >= 0 ||
			Math.abs(leftToeMinZ - 60) > 1e-9 ||
			Math.abs(leftToeMaxZ - (60 + ParallelGripperSpec.FINGER_SUPPORT_STEP_PROJECTION)) > 1e-9)
			throw "The mirrored support steps must extend inward beneath the fingertip plane";
		var gripperBody = new ParallelGripperBody(new ParallelGripperSpec(32, 50, 60, 12));
		var gripperTool = new ArmGripperTool(32, 50, 60, 12).build(new RobotFlange(50));
		var gripperControls = EndEffectorControls.derive(gripperTool, "tool");
		if (gripperControls.grippers.length != 1 || gripperControls.grippers[0].sensor != "tool/gripper/gripped" ||
			gripperControls.gripperChannel(gripperControls.grippers[0].member) != "tool/gripper/body.close" ||
			gripperControls.grippers[0].minimumPartWidthMm != 38 ||
			gripperControls.grippers[0].maximumPartWidthMm != 62 ||
			gripperControls.grippers[0].fingerPadFrictionCoefficient != ParallelGripperSpec.FINGER_PAD_FRICTION_COEFFICIENT)
			throw "Arm gripper tool must publish its logical command and physical jaw switch";
		var gripperSceneTools = RobotScene.robotTools(gripperTool, "tool");
		if (gripperSceneTools.length != 1 || gripperSceneTools[0].kind != "gripper" ||
			gripperSceneTools[0].contact.occurrence != "tool/gripper/body" ||
			gripperSceneTools[0].sensor != "tool/gripper/gripped")
			throw "Robot scene must bind the physical TCP, close channel and jaw switch";
		var logicalGripper = new GripperChannels(gripperControls.gripperChannel(gripperControls.grippers[0].member));
		var logicalDeclaration = logicalGripper.declarations()[0];
		var safeOpen = switch logicalDeclaration.safeValue {case Digital(value): !value; case _: false;};
		if (logicalDeclaration.id != "tool/gripper/body.close" || !safeOpen || !logicalDeclaration.keepOnStop)
			throw "Gripper command must default open on a fault and retain a grasp through an ordinary stop";
		var faces:Array<GraspPlane> = [
			{origin: new Vec3(-30, 0, 15), normal: new Vec3(-1, 0, 0)},
			{origin: new Vec3(30, 0, 15), normal: new Vec3(1, 0, 0)},
			{origin: new Vec3(0, -25, 15), normal: new Vec3(0, -1, 0)},
			{origin: new Vec3(0, 25, 15), normal: new Vec3(0, 1, 0)},
			{origin: new Vec3(0, 0, 0), normal: new Vec3(0, 0, -1)},
			{origin: new Vec3(0, 0, 30), normal: new Vec3(0, 0, 1)}
		];
		var hull:Array<Vec3> = [];
		for (x in [-30.0, 30.0]) for (y in [-25.0, 25.0]) for (z in [0.0, 30.0])
			hull.push(new Vec3(x, y, z));
		var grasp = Grasp.find(faces, hull, new Vec3(0, 0, 1), 44, 60, 76, 60);
		var jawAxis = grasp.toolRotation.rotate(new Vec3(1, 0, 0));
		var fingerAxis = grasp.toolRotation.rotate(new Vec3(0, 0, 1));
		if (Math.abs(grasp.widthMm - 60) > 1e-6 || Math.abs(grasp.contact.z) > 1e-6 ||
			Math.abs(jawAxis.x - 1) > 1e-6 || Math.abs(fingerAxis.z + 1) > 1e-6)
			throw "Grasp must choose nominal opposing faces and put the inserted finger TCP at the far-side plane, pointing into the blank";
		var sideGrasp = Grasp.find(faces, hull, new Vec3(0, 1, 0), 44, 60, 76, 60,
			new Vec3(1, 0, 0));
		var sideJawAxis = sideGrasp.toolRotation.rotate(new Vec3(1, 0, 0));
		var sideFingerAxis = sideGrasp.toolRotation.rotate(new Vec3(0, 0, 1));
		if (Math.abs(sideGrasp.widthMm - 60) > 1e-6 || Math.abs(sideGrasp.contact.y + 25) > 1e-6 ||
			Math.abs(sideJawAxis.x - 1) > 1e-6 || Math.abs(sideFingerAxis.y + 1) > 1e-6)
			throw "Side-entry grasp must preserve the jaw axis and point the fingers into the blank";
		var noFit = false;
		try Grasp.find(faces, hull, new Vec3(0, 0, 1), 61, 65, 70, 60) catch (_:Dynamic) noFit = true;
		if (!noFit) throw "Grasp planner accepted a blank outside the jaw opening range";
		var squareFaces:Array<GraspPlane> = [
			{origin: new Vec3(-30, 0, 15), normal: new Vec3(-1, 0, 0)},
			{origin: new Vec3(30, 0, 15), normal: new Vec3(1, 0, 0)},
			{origin: new Vec3(0, -30, 15), normal: new Vec3(0, -1, 0)},
			{origin: new Vec3(0, 30, 15), normal: new Vec3(0, 1, 0)},
			{origin: new Vec3(0, 0, 0), normal: new Vec3(0, 0, -1)},
			{origin: new Vec3(0, 0, 30), normal: new Vec3(0, 0, 1)}
		];
		var squareHull:Array<Vec3> = [];
		for (x in [-30.0, 30.0]) for (y in [-30.0, 30.0]) for (z in [0.0, 30.0])
			squareHull.push(new Vec3(x, y, z));
		var preferred = Grasp.find(squareFaces, squareHull, new Vec3(0, 0, 1), 44, 60, 76, 60,
			new Vec3(1, 0, 0));
		if (Math.abs(preferred.widthAxis.x - 1.0) > 1e-9)
			throw "An equal-width grasp must use the cell's preferred jaw axis";
		var measuredBox = Part.box(60, 50, 30);
		var measuredFaces = GeometricConnectors.planarFaces(GeometricConnectors.describeFaces(measuredBox.shape));
		measuredBox.close();
		if (measuredFaces.length != 6) throw "Saved CAD face descriptors must decode as typed planar faces";
		var dualTool = new DualArmGripperTool(32, 60, 60, 16);
		var dualEffector = dualTool.build(new RobotFlange(50));
		var dualControls = EndEffectorControls.derive(dualEffector, "tool");
		var dualPoses = dualEffector.solvedPoses();
		var dualA = dualPoses.get("gripperA/body"), dualB = dualPoses.get("gripperB/body");
		if (dualA == null || dualB == null) throw "Dual gripper adapter lost one of its two mounted bodies";
		if (dualControls.grippers.length != 2 ||
			dualControls.gripperChannel("gripperA/body") != "tool/gripperA/body.close" ||
			dualControls.gripperChannel("gripperB/body") != "tool/gripperB/body.close")
			throw "Dual gripper controls must derive two independently supplied close channels";
		var dualSceneTools = RobotScene.robotTools(dualEffector, "tool");
		if (dualSceneTools.length != 2 || dualSceneTools[0].channel != "tool/gripperA/body.close" ||
			dualSceneTools[1].channel != "tool/gripperB/body.close")
			throw "Robot scene must expose both supplied gripper controls";
		var stationX = dualA.x - dualB.x, stationY = dualA.y - dualB.y;
		var stationDistance = Math.sqrt(stationX * stationX + stationY * stationY);
		if (Math.abs(dualA.z - dualB.z) > 1e-6 ||
			Math.abs(stationDistance - 2 * dualTool.mountOffsetMm) > 1e-6)
			throw "Dual grippers must sit at opposing stations in the adapter plate plane";
		var mountedArm = new CobotArm(Reach850, new ArmGripperTool(32, 60, 60, 16, true));
		var armCell = new MachineAssembly();
		armCell.include("arm", mountedArm);
		armCell.addComponent("airSupply", new AirSupply(), AssemblyFrames.translation(1200, -500, 0));
		armCell.connectPorts("gripper-air", "airSupply", "air", "arm", "compressedAir");
		var mountedTool = mountedArm.tool;
		if (mountedTool == null) throw "Cobot gripper assembly has no tool";
		var gripperArtifact = AssemblyPreview.scene(armCell, "physical-gripper-tool");
		gripperArtifact.robotTools = RobotScene.robotTools(mountedTool, "arm/tool");
		var restoredGripper = SceneArtifact.decode(SceneArtifact.encode(gripperArtifact));
		var restoredTools = restoredGripper.robotTools;
		if (restoredTools == null || restoredTools.length != 1)
			throw "Physical gripper tool capability must survive scene artifact validation and round trip";
		var restoredTool = restoredTools[0], restoredCapability = restoredTool.gripper;
		if (restoredTool.kind != "gripper" || restoredCapability == null ||
			restoredTool.sensor != "arm/tool/gripper/gripped" ||
			restoredCapability.fingerPadFrictionCoefficient !=
				ParallelGripperSpec.FINGER_PAD_FRICTION_COEFFICIENT)
			throw "Physical gripper tool capability must survive scene artifact validation and round trip";
		var savedAssemblyDefinition = restoredGripper.assemblyDefinition;
		if (savedAssemblyDefinition == null) throw "Physical gripper round trip lost its assembly definition";
		var savedLeftFinger = [for (entry in savedAssemblyDefinition.definitions)
			if (entry.id == "arm/tool/gripper/fingerLeft") entry][0];
		var savedRightFinger = [for (entry in savedAssemblyDefinition.definitions)
			if (entry.id == "arm/tool/gripper/fingerRight") entry][0];
		if (savedLeftFinger.collisionHulls == null || savedRightFinger.collisionHulls == null ||
			savedLeftFinger.collisionHulls.length != 2 || savedRightFinger.collisionHulls.length != 2)
			throw "Scene artifact round trips must retain both convex support-step collision pieces per finger";
		var convertedTool = AssemblySimulationBridge.toRobotModel(restoredGripper.assemblyDefinition,
			AssemblyPhysicalPartView.fromSceneArtifact(restoredGripper), restoredGripper.assemblyState);
		if (convertedTool.jointInterfaces.filter(pair ->
			(pair.first == "arm/tool/gripper/body" && pair.second == "arm/tool/gripper/rod") ||
			(pair.first == "arm/tool/gripper/rod" && pair.second == "arm/tool/gripper/body")).length != 1)
			throw "A gripper's piston rod and housing are recognized as its prismatic mechanical interface";
		if (convertedTool.jointInterfaces.filter(pair ->
			(pair.first == "arm/tool/valve" && pair.second == "arm/tool/gripper/rod") ||
			(pair.first == "arm/tool/gripper/rod" && pair.second == "arm/tool/valve")).length != 1)
			throw "A valve mounted on the cylinder body is recognized against the moving piston rod";
		var suppliedGripper = new MachineAssembly();
		suppliedGripper.include("tool", gripperTool);
		suppliedGripper.addComponent("airSupply", new machinekit.pneumatic.AirSupply());
		suppliedGripper.addMate("air-source-mount", "fixed", "tool/plate", "robot", "airSupply", "mount");
		suppliedGripper.connectPorts("tool-air", "airSupply", "air", "tool", "compressedAir");
		if (suppliedGripper.validate().length != 0) throw "Pneumatic gripper and routed hoses do not validate";
		var gripperModel = new cadkit.modeling.AssemblyModel("mm");
		suppliedGripper.addTo(gripperModel, "");
		var gripperDefinition = gripperModel.definition("physical-gripper");
		var compiledActuators = gripperDefinition.actuators == null ? [] : gripperDefinition.actuators;
		if (compiledActuators.length != 1)
			throw "Arm gripper must compile its actual cylinder force through the routed valve circuit";
		var physicalDrive = compiledActuators[0];
		var pneumatic = physicalDrive.pneumatic;
		var force = physicalDrive.maxEffort;
		if (pneumatic == null || force == null || Math.abs(force - gripper.spec.ratedGripForce) > 1e-6)
			throw "Arm gripper must compile its actual cylinder force through the routed valve circuit";
		if (pneumatic.channelA != "tool/valve/coilA" || pneumatic.channelB != "tool/valve/coilB" ||
			pneumatic.normallyToA || pneumatic.extendSign != 1 || pneumatic.stroke != gripper.spec.jawTravel() ||
			pneumatic.pressurePa != 600000)
			throw "Gripper close must extend the cylinder, with the double valve resting at its open outlet";
		var switches = gripperDefinition.sensors == null ? [] : gripperDefinition.sensors;
		var gripSwitch = [for (sensor in switches) if (sensor.id == "tool/gripper/gripped") sensor];
		var emptySwitch = [for (sensor in switches) if (sensor.id == "tool/gripper/jaw-closed-empty") sensor];
		var openSwitch = [for (sensor in switches) if (sensor.id == "tool/gripper/jaw-open") sensor];
		if (gripSwitch.length != 1 || emptySwitch.length != 1 || openSwitch.length != 1 ||
			gripSwitch[0].joint != physicalDrive.joint || openSwitch[0].joint != physicalDrive.joint)
			throw "Jaw feedback must distinguish the expected blank width from an empty full-stroke close";
		var gripLower = gripSwitch[0].windowLower, gripUpper = gripSwitch[0].windowUpper;
		var emptyLower = emptySwitch[0].windowLower, emptyUpper = emptySwitch[0].windowUpper;
		var openUpper = openSwitch[0].windowUpper;
		var expectedGripWindow = gripper.spec.grippedTravelWindow();
		if (gripLower == null || gripUpper == null || emptyLower == null || emptyUpper == null || openUpper == null ||
			Math.abs(gripLower - expectedGripWindow.lower) > 1e-9 ||
			Math.abs(gripUpper - expectedGripWindow.upper) > 1e-9 ||
			gripLower <= 0.0 || gripUpper >= gripper.spec.jawTravel() ||
			gripLower - openUpper < ParallelGripperSpec.SWITCH_EDGE_CLEARANCE ||
			emptyLower - gripUpper < ParallelGripperSpec.SWITCH_EDGE_CLEARANCE ||
			gripUpper >= emptyUpper)
			throw "Jaw feedback must span supported part widths while excluding open and empty end-switch bands";
		var components:Array<MachineComponent> = [master, tool, gripperBody, new FrameBar(30, 20, 90),
			new PneumaticManifold(2), new VacuumGenerator(), new SuctionCup(25, 18)];
		var ratedCup = new SuctionCup(40, 18, 1000, 5);
		var ratedGenerator = new VacuumGenerator(60);
		if ((cast ratedCup.effectiveAreaMm2:Float) != 1000 ||
			(cast ratedCup.ratedMomentNm:Float) != 5 ||
			(cast ratedGenerator.ratedVacuumKpa:Float) != 60 ||
			ratedCup.designation == new SuctionCup(40, 18).designation ||
			ratedGenerator.designation == new VacuumGenerator().designation)
			throw "Rated suction components must retain distinct design data";
		var rejected = false;
		try new SuctionCup(40, 18, 2000) catch (_:Dynamic) rejected = true;
		if (!rejected) throw "Suction cup accepted an effective area larger than its face";
		for (component in components) {
			var properties = component.massProperties();
			if (properties.mass <= 0 || properties.inertia == null)
				throw 'Missing geometry-derived mass for ${component.designation}';
			switch properties.source {
				case Computed(Preview):
				case _: throw 'Mass is not computed from geometry for ${component.designation}';
			}
		}
	}
}
