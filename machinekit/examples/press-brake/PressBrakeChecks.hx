import cadkit.modeling.AssemblyModel;
import machinekit.assembly.MachineAssembly;
import machinekit.pressbrake.GaugeFinger;
import cadkit.modeling.AssemblyState;
import machinekit.pressbrake.planning.PressGeometry;
import machinekit.pressbrake.planning.PressEnvelope;
import machinekit.pressbrake.ToolLayout;
import machinekit.pressbrake.ToolStation;
import machinekit.pressbrake.ToolRow;
import machinekit.pressbrake.ToolSegment;
import machinekit.pressbrake.ToolProfiles;
import machinekit.pressbrake.ToolingCatalogue;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.process.AirBending;

/** Check the installed press's joint tree, coupled ram and physical tool datums. */
class PressBrakeChecks {
	public static function main():Void run();
	public static function run():Void {
		BendSetupChecks.run();
		BendRegistrationChecks.run();
		SingleBendSetupChecks.run();
		PrismCollisionChecks.run();
		SheetPrismChecks.run();
		BendPrismChecks.run();
		PressCollisionGeometryChecks.run();
		var recipeFixture = new MachineAssembly();
		recipeFixture.addComponent("finger", new GaugeFinger());
		recipeFixture.addComponent("base", new GaugeFinger());
		recipeFixture.addMateOnAxis("contact-hinge", "revolute", "base", "mount", "finger", "mount",
			{x: -1, y: 0, z: 0}, 0,
			{lower: -GaugeFinger.DROP_ANGLE, upper: 0, velocity: null, effort: null,
				overtravel: GaugeFinger.OVERTRAVEL});
		recipeFixture.setPassive("contact-hinge");
		recipeFixture.setJointStop("contact-hinge", GaugeFinger.stopResponse());
		recipeFixture.addSwitch("contact-observed", "contact-hinge",
			{lower: -GaugeFinger.SWITCH_WINDOW, upper: GaugeFinger.OVERTRAVEL, velocity: null, effort: null}, 0);
		var rebuilt = MachineAssembly.decode(recipeFixture.encode());
		rebuilt.addTo(new AssemblyModel("mm"), "");
		var invalidObserved = MachineAssembly.decode(recipeFixture.encode());
		invalidObserved.addSwitch("outside-observed", "contact-hinge",
			{lower: GaugeFinger.OVERTRAVEL * 2, upper: GaugeFinger.OVERTRAVEL * 3, velocity: null, effort: null}, 0);
		var rejectedObservation = false;
		try invalidObserved.addTo(new AssemblyModel("mm"), "") catch (_:Dynamic) rejectedObservation = true;
		if (!rejectedObservation) throw "Switch outside declared observed travel was accepted";
		var stopped = rebuilt.describe().mechanical.joints[0].stop;
		if (stopped == null) throw "Saved contact hinge lost its mechanical stop";
		near(stopped.timeConstantSec, GaugeFinger.stopResponse().timeConstantSec, "Saved contact hinge stop response");
		var contact = rebuilt.components()[0].component.simulationContact();
		if (contact == null || contact.filter != materia.project.SceneContactFilter.PairsAndEnvironment)
			throw "Saved gauge finger lost its authored contact surface";
		near(contact.contactTimeConstant, GaugeFinger.stopResponse().timeConstantSec, "Saved gauge finger contact compliance");
		var tallest = new ToolLayout(1600, [new ToolStation("four-V", new ToolRow([
			new ToolSegment(Punch(ToolingCatalogue.gooseneck()), 200)], []), new ToolRow([
			new ToolSegment(FourV(ToolingCatalogue.fourV(), 0), 200)], []), 700)]);
		for (variant in 0...4) {
			var withR = variant != 0;
			var press = new PressBrake(withR, variant == 2 ? tallest : null, null, variant == 3 ? 30 : 300);
			press.validate();
			var geometry = PressGeometry.of(press), reference = PressGeometry.reference(press.toolLayout, withR, press.fingerHalfSpacing);
			near(geometry.bedLength, reference.bedLength, "Derived bed length");
			near(geometry.bedTopHeight, reference.bedTopHeight, "Derived bed top");
			near(geometry.openHeight, reference.openHeight, "Derived open mount separation");
			near(geometry.throatY, reference.throatY, "Derived throat plane");
			near(geometry.capacityN, reference.capacityN, "Declared frame capacity");
			near(geometry.ramTravel.upper, reference.ramTravel.upper, "Derived ram stroke");
			near(geometry.backgaugeX.upper, reference.backgaugeX.upper, "Derived X travel");
			near(geometry.backgaugeR.upper, reference.backgaugeR.upper, "Derived R travel");
			checkEnvelope(geometry.ramEnvelope, reference.ramEnvelope, "Derived ram bounds");
			for (i in 0...geometry.fingers().length) checkEnvelope(geometry.fingers()[i], reference.fingers()[i], "Derived finger bounds");
			for (i in 0...geometry.fingerContacts().length) {
				var actual = geometry.fingerContacts()[i], expected = reference.fingerContacts()[i];
				if (actual.id != expected.id) throw "Derived finger contact identity";
				near(actual.x, expected.x, "Derived finger contact lateral setup");
				near(actual.y, expected.y, "Derived finger contact zero X");
				near(actual.z, expected.z, "Derived finger contact zero R");
			}
			for (station in geometry.stations()) {
				var expected = reference.station(station.id);
				near(station.dieTopHeight, expected.dieTopHeight, "Derived die top");
				near(station.punchLineY, expected.punchLineY, "Derived punch line");
				near(station.openTipHeight, expected.openTipHeight, "Derived open tool gap");
			}
			checkCapacityAndFrame(press, geometry);
			for (entry in press.components()) {
				var part = entry.component.geometry();
				var valid = CadKit.shapeValidChecked(part.shape.borrowHandle()) != 0 && part.volume() > 0;
				part.close();
				if (!valid) throw 'Press component "${entry.id}" has invalid or empty native geometry';
			}
			var bomQuantity = 0;
			for (line in press.billOfMaterials().lines()) bomQuantity += line.quantity;
			if (bomQuantity < press.components().length) throw "Press BOM loses components";
			var model = new AssemblyModel("mm");
			press.addTo(model, "");
			var definition = model.definition("press-brake");
			for (name in ["fingerLeft", "fingerRight"]) {
				var joints = [for (joint in definition.joints) if (joint.id == '$name-hinge') joint];
				if (joints.length != 1 || joints[0].passive != true || joints[0].driven == true)
					throw 'Press finger "$name" lost physical joint ownership';
				var lower = joints[0].limits.lower, overtravel = joints[0].limits.overtravel;
				if (lower == null || overtravel == null) throw 'Press finger "$name" lost its mechanical bounds';
				near(lower, -GaugeFinger.DROP_ANGLE, "Declared finger drop travel");
				near(overtravel, GaugeFinger.OVERTRAVEL, "Declared finger observation margin");
			}
			var switches = definition.sensors;
			if (switches == null) throw "Press lost its measured switches";
			for (id in ["ram-top", "ram-speed-change", "ram-bottom", "fingerLeft-contact", "fingerRight-contact"]) {
				var found = [for (sensor in switches) if (sensor.id == id && sensor.kind == "joint_switch") sensor];
				if (found.length != 1) throw 'Press lost measured switch "$id"';
			}
			var hydraulicCount = 0, installedForce = 0.0;
			if (definition.actuators != null) for (actuator in definition.actuators) {
				var drive = actuator.hydraulic;
				if (drive != null) {
					hydraulicCount++; installedForce += actuator.maxEffort;
					near(drive.pressurePa, press.supplyPressurePa, "Installed hydraulic pressure");
					near(drive.changePoint, press.phaseSpeeds.changePoint, "Installed change travel");
					if (drive.channelA != "ramValve/coilA" || drive.channelB != "ramValve/coilB" || drive.extendSign != 1)
						throw "Press cylinder lost its common directional valve or downward guide alignment";
				}
			}
			if (hydraulicCount != 2) throw "Press needs both installed hydraulic cylinder drives";
			near(installedForce, press.capacityN, "Regulated force respects the assumed frame capacity");
			var state = new AssemblyState(definition);
			for (travel in [0.0, 37, 100]) {
				state.setJoint("Y1", travel);
				state.setJoint("X", travel * 5);
				state.setJoint("depth", travel);
				if (withR) state.setJoint("R", travel * 1.2);
				state.forwardKinematics();
				near(state.worldPose("guideLeft").z, press.bedHeight + press.openHeight - travel, "Left ram guide");
				near(state.worldPose("guideRight").z, state.worldPose("guideLeft").z, "Torsion-bar equality");
				near(state.worldConnector("ram", "tool-mount").z, press.bedHeight + press.openHeight - travel, "Ram mount");
				near(state.worldConnector("fingerLeft", "contact").y, travel * 5, "Backgauge distance");
				near(state.worldConnector("fingerRight", "contact").z, press.gaugeBaseHeight + (withR ? travel * 1.2 : 0), "Backgauge lift");
				near(state.worldPose("depthStop").z, press.bedHeight + press.openHeight - travel, "Independent depth stop");
			}
		}
		trace("Press brake assembly checks passed");
	}
	static function checkCapacityAndFrame(press:PressBrake, geometry:PressGeometry):Void {
		var process = new AirBending();
		for (station in press.toolLayout.stations()) {
			var prediction = process.predict(SheetDemoParts.materialA(), station.processTooling(), Math.PI / 2, geometry.bedLength);
			if (prediction.force * 1000 > geometry.capacityN) throw "Full-bed material-A bend exceeds frame capacity";
			var contact = geometry.station(station.id);
			if (prediction.depth > geometry.ramTravel.upper - (contact.openTipHeight - contact.dieTopHeight))
				throw "Installed tool stack cannot reach the predicted bending depth";
		}
		var tallestPunch = 0.0, tallestDie = 0.0;
		for (profile in ToolProfiles.catalogueProfiles()) {
			if (ToolProfiles.isPunch(profile)) tallestPunch = Math.max(tallestPunch, ToolProfiles.height(profile));
			else tallestDie = Math.max(tallestDie, ToolProfiles.height(profile));
		}
		if (tallestPunch + tallestDie >= geometry.openHeight - geometry.ramTravel.upper)
			throw "Tallest catalogue tooling consumes the structural mount opening";
		for (sheet in [SheetDemoParts.p1(), SheetDemoParts.p2(), SheetDemoParts.p3(), SheetDemoParts.p4(), SheetDemoParts.p5()]) {
			// These demo descendants are rectangular and all nested hinges are parallel. The
			// straight-leg plus outer bend-arc length bounds vertical/rearward reach at every angle.
			var reach = sheet.thickness();
			for (wall in sheet.walls()) reach = Math.max(reach, sheet.thickness() + branchReach(sheet, wall.id));
			var root = sheet.walls()[0], rear = 0.0;
			for (segment in root.outline.loops()[0]) {
				if (!segment.isLine()) throw "Demo frame bound needs straight root edges";
				rear = Math.max(rear, Math.max(segment.start.x, segment.end.x));
			}
			if (rear + reach >= geometry.throatY) throw "Demo swing envelope reaches the rear throat plane";
			for (station in geometry.stations())
				if (station.dieTopHeight + reach >= geometry.ramEnvelope.minZ - geometry.ramTravel.upper)
					throw "Demo swing envelope reaches the ram beam within its stroke";
			if (rear + reach > geometry.backgaugeX.upper - geometry.backgaugeX.lower)
				throw "Demo rearward envelope exceeds the backgauge range";
		}
	}
	static function branchReach(sheet:Sheet, wall:Int):Float {
		for (bend in sheet.bends()) if (bend.wallB == wall) {
			var outline = sheet.wall(wall).outline;
			if (outline.loops().length != 1 || outline.loops()[0].length != 4
				|| bend.edgeB.span.start.x != 0 || bend.edgeB.span.end.x != 0)
				throw "Demo frame bound needs rectangular descendants hinged at x=0";
			for (segment in outline.loops()[0]) if (!segment.isLine()) throw "Demo frame bound needs straight child edges";
			for (incoming in sheet.bends()) if (incoming.wallB == bend.wallA) {
				var a = incoming.edgeB.span, b = bend.edgeA.span;
				var cross = (a.end.x - a.start.x) * (b.end.y - b.start.y) - (a.end.y - a.start.y) * (b.end.x - b.start.x);
				if (Math.abs(cross) > 1e-9 * a.length() * b.length()) throw "Demo frame bound needs parallel nested hinges";
			}
			return branchReach(sheet, bend.wallA) + outline.segment(0, 0).length()
				+ sheet.bendAllowance(bend) + Math.abs(bend.angle) * sheet.thickness();
		}
		return 0;
	}
	static function checkEnvelope(actual:PressEnvelope, expected:PressEnvelope, label:String):Void {
		near(actual.minX, expected.minX, label); near(actual.minY, expected.minY, label); near(actual.minZ, expected.minZ, label);
		near(actual.maxX, expected.maxX, label); near(actual.maxY, expected.maxY, label); near(actual.maxZ, expected.maxZ, label);
	}
	public static function near(actual:Float, expected:Float, label:String):Void {
		if (!Math.isFinite(actual) || Math.abs(actual - expected) > 1e-6) throw '$label: $actual instead of $expected';
	}
}
