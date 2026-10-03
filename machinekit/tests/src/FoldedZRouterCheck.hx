/** Focused folded router fixture without the rest of MachineKit smoke. */
import CncRouterPreview.CncRouterChecks;
import machinekit.assembly.Transmission;
import machinekit.transmission.TimingBelt;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import robotkit.model.DriveLoads;
import cadkit.modeling.AssemblyState;
import materia.project.SceneArtifact;

class FoldedZRouterCheck {
	public static function main():Void {
		runFoldedZ();
		var combined = new CncRouter(true, true);
		var geometry = combined.describe();
		if (geometry.mechanical.elasticNetworks == null || geometry.mechanical.elasticNetworks.length != 1 ||
			combined.check().hasErrors()) throw "The folded Z stage must combine with the router's X/Y carriage belts";
	}
	/** Compare the new shaft-belt Z stage with the existing direct Z on the same router. */
	public static function runFoldedZ():Void {
		var directScene = SceneArtifact.decode(CncRouterPreview.router());
		var folded = new CncRouter(false, true);
		var foldedScene = SceneArtifact.decode(CncRouterPreview.foldedZRouter());
		var physical = AssemblyPhysicalPartView.fromSceneArtifact(foldedScene);
		var converted = AssemblySimulationBridge.toRobotModel(foldedScene.assemblyDefinition, physical,
			foldedScene.assemblyState).model;
		var direct = AssemblySimulationBridge.toRobotModel(directScene.assemblyDefinition,
			AssemblyPhysicalPartView.fromSceneArtifact(directScene), directScene.assemblyState).model;
		var errors = converted.validate();
		if (errors.length > 0) throw 'Folded Z robot model is invalid: $errors';
		if (converted.elasticNetworks.length != 1 || converted.elasticNetworks[0].spans.length != 2)
			throw "Folded Z needs one two-span belt network";
		var belt:Null<TimingBelt> = null;
		for (entry in folded.components()) if (entry.id == "beltZ") belt = cast entry.component;
		if (belt == null || Math.abs(belt.slack()) > 1e-8)
			throw "Folded Z belt must have whole teeth at its attached centres";
		var stage = folded.transmissionFor("screwZ-belt");
		if (stage == null || !switch stage.source { case BeltReduction("beltZ", "pulleyScrewZ", "pulleyMotorZ"): true; case _: false; })
			throw "Folded Z must compile from its actual two pulleys";
		var directZ = direct.coupledLimits("z"), foldedZ = converted.coupledLimits("z");
		CncRouterChecks.near(foldedZ.requireVelocity() * 2, directZ.requireVelocity(), "the two-to-one belt uses twice the motor rate", 1e-9);
		if (!(foldedZ.requireAcceleration() > 0 && foldedZ.requireAcceleration() < directZ.requireAcceleration()))
			throw "The folded motor's inertia must tighten the Z acceleration bound";
		var directLoad = DriveLoads.forAxis(direct, "z"), foldedLoad = DriveLoads.forAxis(converted, "z");
		if (directLoad == null || foldedLoad == null || foldedLoad.stiffness <= 0 || directLoad.stiffness != 0)
			throw "Folded Z must add the belt's elastic compliance to the direct screw";
		var state = new AssemblyState(foldedScene.assemblyDefinition);
		for (position in [[150.0, 150, 0], [0.0, 0, -80], [300.0, 300, -80]]) {
			state.setJoint("x", position[0]); state.setJoint("y", position[1]); state.setJoint("z", position[2]);
			state.forwardKinematics();
			var nose = state.worldConnector("spindle", "nose");
			var expected = CncRouter.noseAt(position[0], position[1], position[2]);
			CncRouterChecks.near(nose.x, expected.x, "folded Z nose x", 1e-6);
			CncRouterChecks.near(nose.y, expected.y, "folded Z nose y", 1e-6);
			CncRouterChecks.near(nose.z, expected.z, "folded Z nose z", 1e-6);
			CncRouterChecks.near(state.joint("screwZ-turn"), Math.PI * position[2], "folded Z screw follows its lead", 1e-6);
			CncRouterChecks.near(state.joint("motorZ-turn"), -2 * Math.PI * position[2], "folded Z motor keeps the belt's world direction", 1e-6);
		}
		for (position in [[150.0, 150, 0], [0.0, 0, -80], [300.0, 300, -80]]) {
			CncRouterChecks.checkClear(folded, state, position, ["motorZ"], ["xPlate", "zPlate", "spindle", "uprightRight", "beamUpper", "beamLower"]);
			CncRouterChecks.checkClear(folded, state, position, ["beltZ"], ["zPlate", "spindle", "uprightRight", "beamUpper"]);
			CncRouterChecks.checkClear(folded, state, position, ["foldedMotorPlateZ"], ["screwZ", "spindle", "uprightRight", "beamUpper"]);
		}
		Sys.println('folded Z: ${belt.teeth} teeth; direct/folded speed ${directZ.requireVelocity() * 1000}/${foldedZ.requireVelocity() * 1000} mm/s, acceleration ${directZ.requireAcceleration() * 1000}/${foldedZ.requireAcceleration() * 1000} mm/s², stiffness rigid/${foldedLoad.stiffness} N/m, backlash ${directLoad.backlash * 1000}/${foldedLoad.backlash * 1000} mm, mass ${Math.round(new CncRouter().massProperties().mass * 10) / 10}/${Math.round(folded.massProperties().mass * 10) / 10} kg');
	}

}
