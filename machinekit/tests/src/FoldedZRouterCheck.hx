/** Focused folded router fixture without the rest of MachineKit smoke. */
import CncRouterPreview.CncRouterChecks;
import machinekit.assembly.PosedParts;
import machinekit.assembly.Transmission;
import machinekit.transmission.TimingBelt;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import robotkit.model.DriveLoads;
import cadkit.modeling.AssemblyState;
import materia.project.SceneArtifact;

class FoldedZRouterCheck {
	public static function main():Void {
		CncRouterChecks.withGeometry(runFoldedZ);
		var combined = new CncRouter(true, true);
		var networks = combined.definition().elasticNetworks;
		if (networks == null || networks.length != 1 ||
			combined.check().hasErrors()) throw "The folded Z stage must combine with the router's X/Y carriage belts";
		var combinedScene = SceneArtifact.decode(CncRouterPreview.router(true, true));
		var combinedModel = AssemblySimulationBridge.toRobotModel(combinedScene.assemblyDefinition,
			AssemblyPhysicalPartView.fromSceneArtifact(combinedScene), combinedScene.assemblyState).model;
		var combinedLoads = DriveLoads.of(combinedModel);
		if (combinedLoads.length != 3) throw "Mixed carriage and shaft belts need three driven axes";
		for (axis in combinedLoads) if (axis.axis == "x" || axis.axis == "y") {
			for (assumption in axis.assumptions) if (assumption.label == "belt stiffness with pretension")
				throw 'Folded Z belt assumption leaked onto ${axis.axis}';
		}
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
		Sys.println('folded Z acceleration: direct ${directZ.requireAcceleration() * 1000}, folded ${foldedZ.requireAcceleration() * 1000} mm/s²');
		CncRouterChecks.near(directZ.requireVelocity() * 1000, 43.6539272481, "direct Z speed baseline", 1e-6);
		CncRouterChecks.near(foldedZ.requireVelocity() * 1000, 21.8269636240, "folded Z speed baseline", 1e-6);
		CncRouterChecks.near(directZ.requireAcceleration() * 1000, 6116.92589072, "direct Z acceleration baseline", 1e-3);
		CncRouterChecks.near(foldedZ.requireAcceleration() * 1000, 3263.33221329, "folded Z acceleration baseline", 1e-3);
		CncRouterChecks.near(foldedZ.requireVelocity() * 2, directZ.requireVelocity(), "the two-to-one belt uses twice the motor rate", 1e-9);
		if (!(foldedZ.requireAcceleration() > 0 && foldedZ.requireAcceleration() < directZ.requireAcceleration()))
			throw "The folded motor's inertia must tighten the Z acceleration bound";
		var directLoad = DriveLoads.forAxis(direct, "z"), foldedLoad = DriveLoads.forAxis(converted, "z");
		if (directLoad == null || foldedLoad == null || foldedLoad.stiffness <= 0 || directLoad.stiffness != 0)
			throw "Folded Z must add the belt's elastic compliance to the direct screw";
		CncRouterChecks.near(foldedLoad.stiffness / 1e6, 486.86732785, "folded Z stiffness baseline, MN/m", 0.01);
		CncRouterChecks.near(directLoad.backlash * 1000, 0.05, "direct Z backlash baseline", 1e-6);
		CncRouterChecks.near(foldedLoad.backlash * 1000, 0.051, "folded Z backlash baseline", 1e-6);
		Sys.println('router mass: direct ${new CncRouter().massProperties().mass}, folded ${folded.massProperties().mass}');
		CncRouterChecks.near(new CncRouter().massProperties().mass, 37.013596686290356, "direct router mass baseline", 1e-6);
		CncRouterChecks.near(folded.massProperties().mass, 37.15604020969244, "folded router mass baseline", 1e-6);
		if (CncRouter.FoldedZMotorPlate.TENSION_TRAVEL < 4.0)
			throw "Folded Z motor plate needs at least 4 mm of slot adjustment";
		var centre = belt.wraps()[1].x - belt.wraps()[0].x;
		var outward = CncRouter.FoldedZMotorPlate.TENSION_TRAVEL / 2 * 70 / centre;
		var reachablePretension = TimingBelt.cordStiffnessPerMm(belt.beltProfile) * belt.width *
			2 * outward / belt.length;
		if (reachablePretension <= belt.assumedPretension())
			throw "Folded Z slot cannot reach the stated installed pretension";
		belt.checkTension(1.26 / (20 * 2 / (2 * Math.PI) / 1000), belt.assumedPretension());
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
		PosedParts.scope(parts -> {
			for (position in [[150.0, 150, 0], [0.0, 0, -80], [300.0, 300, -80]]) {
				CncRouterChecks.checkClearWith(parts, folded, state, position, ["motorZ"], ["xPlate", "zPlate", "spindle", "uprightRight", "beamUpper", "beamLower"]);
				CncRouterChecks.checkClearWith(parts, folded, state, position, ["beltZ"], ["zPlate", "spindle", "uprightRight", "beamUpper"]);
				CncRouterChecks.checkClearWith(parts, folded, state, position, ["foldedMotorPlateZ"], ["screwZ", "spindle", "uprightRight", "beamUpper"]);
			}
		});
		Sys.println('folded Z: ${belt.teeth} teeth; direct/folded speed ${directZ.requireVelocity() * 1000}/${foldedZ.requireVelocity() * 1000} mm/s, acceleration ${directZ.requireAcceleration() * 1000}/${foldedZ.requireAcceleration() * 1000} mm/s², stiffness rigid/${foldedLoad.stiffness} N/m, backlash ${directLoad.backlash * 1000}/${foldedLoad.backlash * 1000} mm, mass ${Math.round(new CncRouter().massProperties().mass * 10) / 10}/${Math.round(folded.massProperties().mass * 10) / 10} kg');
	}

}
