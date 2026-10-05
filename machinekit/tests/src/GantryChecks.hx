import machinekit.gantry.Gantry;
import machinekit.gantry.GantrySpec;
import machinekit.gantry.GantrySpec.GantryDrive;
import machinekit.assembly.AssemblyPreview;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import machinekit.transmission.TimingBeltProfile;
import machinekit.transmission.TimingPulley;
import machinekit.transmission.ValueBasis;
import machinekit.transmission.SpurGear;
import machinekit.component.ComponentValues;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;
import machinekit.motion.NemaStepper;
import cadkit.modeling.AssemblyState;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;

/** Physical FK, transmission and coupled-drive checks for each supported gantry drive. */
class GantryChecks {
	static function check(value:Bool, message:String):Void {
		if (!value) throw "Gantry: " + message;
	}
	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-8):Void {
		if (!Math.isFinite(actual) || Math.abs(actual - expected) > tolerance)
			throw 'Gantry $message: expected $expected, got $actual';
	}
	public static function run():Void {
		checkGearBore();
		var thread = new LeadScrewThread(MetricTrapezoidal, 12, 3);
		var drives:Array<GantryDrive> = [Screw(thread), Belt(TimingBeltProfile.GT2, 20, 9), Rack(1, 20, 8)];
		var failures:Array<String> = [];
		for (drive in drives) try runDrive(drive)
		catch (error:Dynamic) failures.push(Std.string(drive) + ": " + Std.string(error));
		if (failures.length > 0) throw failures.join("\n");
		var table = new Gantry(new GantrySpec(200, 150, 100, drives[1], drives[1], drives[0], false,
			"MGN12C", 17, "HFS5-4040", "HFS5-4040", true));
		for (member in table.components()) check(!StringTools.startsWith(member.id, "post"), "table-mounted variant omits posts");
		check(table.check().hasErrors() == false, "single-Y table variant is mechanically and electrically valid");
	}
	static function checkGearBore():Void {
		var legacy = new ComponentValues().setNumber("moduleSize", 1).setInteger("teeth", 20).setNumber("faceWidth", 8)
			.setNumber("pressureAngle", SpurGear.STANDARD_PRESSURE_ANGLE).setNumber("profileShift", 0).setNumber("backlash", 0);
		var blank:SpurGear = cast SpurGear.recipeType().create(legacy);
		check(blank.boreDiameter == 0 && blank.designation == "SPUR-M1-20T", "old gear recipe retains its solid blank and designation");
		var bored = new SpurGear(1, 20, 8, SpurGear.STANDARD_PRESSURE_ANGLE, 0, 0, 6.35);
		var rebuilt:SpurGear = cast SpurGear.recipeType().create(bored.values());
		near(rebuilt.boreDiameter, 6.35, "shaft bore survives recipe reconstruction");
		check(bored.designation != blank.designation, "bored and blank gears do not share geometry or BOM identity");
		var before = blank.geometry(), after = bored.geometry();
		try {
			near(before.volume() - after.volume(), Math.PI * 6.35 * 6.35 / 4 * 8, "OCCT removes the shaft bore volume", 1e-3);
			before.close(); after.close();
		} catch (error:Dynamic) { before.close(); after.close(); throw error; }
		var rejected = false;
		try { var impossible = new SpurGear(1, 20, 8, SpurGear.STANDARD_PRESSURE_ANGLE, 0, 0, 20); }
		catch (_:Dynamic) rejected = true;
		check(rejected, "a shaft bore cannot remove the tooth roots");
	}

	static function checkShaftTips(gantry:Gantry, definition:AssemblyDefinition, state:AssemblyState):Void {
		for (joint in definition.joints) if (joint.type == materia.assembly.AssemblyDefinition.AssemblyJointType.Continuous) {
			var parent = gantry.component(joint.parent);
			if (!Std.isOfType(parent, NemaStepper)) continue;
			var child = gantry.component(joint.child);
			var width:Float;
			if (Std.isOfType(child, TimingPulley)) { var pulley:TimingPulley = cast child; width = pulley.thickness; }
			else if (Std.isOfType(child, SpurGear)) { var gear:SpurGear = cast child; width = gear.faceWidth; }
			else continue;
			var tip = state.worldConnector(joint.parent, "shaftTip");
			var farFace = AssemblyFrames.transformPoint(state.worldPose(joint.child), 0, 0, width);
			near(tip.x, farFace.x, "shaft engages the pulley/pinion width X");
			near(tip.y, farFace.y, "shaft engages the pulley/pinion width Y");
			near(tip.z, farFace.z, "shaft engages the pulley/pinion width Z");
		}
	}

	static function runDrive(drive:GantryDrive):Void {
		var gantry = new Gantry(new GantrySpec(200, 150, 100, drive, drive, drive));
		check(!gantry.check().hasErrors(), "the assembly validates");
		check(gantry.spec.racking == 0.5 && gantry.spec.rackingBasis == ValueBasis.Assumed,
			"the default racking tolerance has assumption provenance");
		var exported = gantry.connector("toolFlange");
		check(exported.instanceId == "flange" && exported.connectorName == "face", "the standard tool flange is exposed");
		var scene = materia.project.SceneArtifact.decode(materia.project.SceneArtifact.encode(
			AssemblyPreview.scene(gantry, "gantry-check")));
		var definition = scene.assemblyDefinition;
		if (definition == null) throw "Gantry scene has no assembly definition";
		var state = new AssemblyState(definition);
		var clearance = new GantryClearanceChecks(gantry, definition);
		var expectedFace = AssemblyFrames.compose(gantry.flangeZero, gantry.component("flange").connector("face").frame);
		var expectedRotation = AssemblyFrames.toRotationMatrix(expectedFace);
		try {
		for (includeOvertravel in [false, true]) {
		if (includeOvertravel) {
			// CAD state enforces soft bounds; consume only the declared guide allowance
			// in this private geometry fixture. The exported machine is unchanged.
			var extended = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(definition));
			for (joint in extended.joints) if (joint.limits.overtravel != null) {
				if (joint.limits.lower != null) joint.limits.lower -= joint.limits.overtravel;
				if (joint.limits.upper != null) joint.limits.upper += joint.limits.overtravel;
			}
			state = new AssemblyState(extended);
		}
		var roomX = includeOvertravel ? gantry.axisOvertravel("x") : 0.0;
		var roomY = includeOvertravel ? gantry.axisOvertravel("y") : 0.0;
		var roomZ = includeOvertravel ? gantry.axisOvertravel("z") : 0.0;
		for (x in [-roomX, gantry.spec.travelX + roomX]) for (y in [-roomY, gantry.spec.travelY + roomY]) for (z in [-roomZ, gantry.spec.travelZ + roomZ]) {
			state.setJoint("x", x); state.setJoint("y", y); state.setJoint("z", z);
			state.forwardKinematics();
			var flange = state.worldConnector("flange", "face");
			near(flange.x, gantry.flangeZero.x + x, "corner flange X");
			near(flange.y, gantry.flangeZero.y + y, "corner flange Y");
			near(flange.z, gantry.flangeZero.z - z, "corner flange Z");
			var rotation = AssemblyFrames.toRotationMatrix(flange);
			for (entry in 0...9) near(rotation[entry], expectedRotation[entry], "corner flange orientation");
			var outward = AssemblyFrames.transformVector(flange, 0, 1, 0);
			near(outward.x, 0, "flange normal X");
			near(outward.y, 0, "flange normal Y");
			near(outward.z, -1, "flange points down toward the work");
			near(state.worldPose("blockYRight").y, y, "right guide follows the single Y leader");
			clearance.check(state);
			checkShaftTips(gantry, definition, state);
		}
		}
		} catch (error:Dynamic) { clearance.close(); throw error; }
		clearance.close();
		for (axis in gantry.axes) check(gantry.axisOvertravel(axis.id) > 0, "each guide leaves room beyond both travel ends");
		var couplings = definition.couplings;
		if (couplings == null) throw "Gantry has no physical drive couplings";
		var expectedMagnitude:Float = switch drive {
			case Screw(thread): 2 * Math.PI / thread.lead;
			case Belt(profile, teeth, _): 2 * Math.PI / (TimingPulley.profileDimensions(profile).pitch * teeth);
			case Rack(moduleSize, teeth, _): 2 / (moduleSize * teeth);
		};
		for (coupling in couplings) near(Math.abs(coupling.ratio), expectedMagnitude, "part-derived transmission ratio");
		var actuators = definition.actuators;
		if (actuators == null) throw "Gantry has no compiled motors";
		check(actuators.length == 4, "dual Y has two motors alongside X and Z");
		var yMotors = 0;
		for (actuator in actuators) {
			for (coupling in couplings) if (coupling.target == actuator.joint && coupling.source == "y") yMotors++;
		}
		check(yMotors == 2, "both Y motors follow the same leader joint");
		var yJoints = 0;
		for (joint in definition.joints) if (joint.id == "y") yJoints++;
		check(yJoints == 1, "the rigid gantry has exactly one Y leader");
		var converted = AssemblySimulationBridge.toRobotModel(definition, AssemblyPhysicalPartView.fromSceneArtifact(scene), scene.assemblyState).model;
		check(converted.validate().length == 0, "the physical robot model validates");
		var flangeFrames = 0;
		for (frame in converted.frames) if (frame.id == "flange robot flange") flangeFrames++;
		check(flangeFrames == 1, "preview export preserves the physical flange ownership frame");
		var summaries:Array<String> = [];
		for (axis in gantry.axes) {
			var limits = converted.coupledLimits(axis.id);
			check(limits.requireVelocity() > 0 && limits.requireAcceleration() > 0, "coupled limits are derived from actual motors and moving mass");
			summaries.push(axis.id + " " + limits.requireVelocity() * 1000 + " mm/s, " + limits.requireAcceleration() + " m/s²");
		}
		check(gantry.billOfMaterials().quantity(gantry.component("flange").bom.partNumber) == 1, "BOM includes the ISO-style tool flange");
		check(gantry.billOfMaterials().quantity(gantry.component("motorYLeft").bom.partNumber) == 4, "BOM includes all four motor occurrences");
		Sys.println('gantry $drive: ${definition.definitions.length} definitions, ${definition.occurrences.length} occurrences; ' + summaries.join("; "));
	}
}
