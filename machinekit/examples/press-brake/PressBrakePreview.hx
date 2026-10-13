import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import machinekit.pressbrake.planning.PressGeometry;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactDigitalProgram;

/** The press alone, running an empty hydraulic stroke with installed-tool clearance. */
class PressBrakePreview {
	public static function pressBrake():Bytes {
		var press = new PressBrake();
		var scene = AssemblyPreview.scene(press, "press-brake");
		var motion = scene.machineMotion;
		if (motion == null) throw "Press preview lost its machine metadata";
		motion.digitalProgram = emptyStroke(press);
		return SceneArtifact.encode(scene);
	}

	/** Controller fixture; registration stock is supplied by the script before requesting a stroke. */
	public static function controlledPressBrake():Bytes {
		var press = new PressBrake(), geometry = PressGeometry.of(press);
		var scene = AssemblyPreview.scene(press, "press-brake");
		var motion = scene.machineMotion;
		if (motion == null) throw "Press preview lost its machine metadata";
		var station = press.toolLayout.stations()[0];
		var stock = cadkit.sheetmetal.SheetDemoParts.materialA();
		var prediction = new cadkit.sheetmetal.process.AirBending().predict(stock,
			station.processTooling(), Math.PI / 2, 100);
		var installed = geometry.stations()[0];
		var gap = installed.openTipHeight - installed.dieTopHeight;
		motion.pressBrake = {
			ramJoint: "Y1", depthAxis: "depth", gaugeX: "X", gaugeR: "R",
			topSwitch: "ram-top", fingerSwitches: ["fingerLeft-contact", "fingerRight-contact"],
			// Registration fixture stays behind the tool line; deforming stock belongs to SM12.
			steps: [{gaugeXmm: 150, gaugeRmm: 0, clampTravelMm: gap - stock.thickness,
				depthTravelMm: gap + prediction.depth, withdrawalMm: 20}],
			positionToleranceMm: 0.05, requestTimeoutSeconds: 20,
			// Assumed simulation unloading limits, independent of real hydraulic safety feedback.
			releaseTravelMm: 0.5, releaseClosingForceN: 5000
		};
		return SceneArtifact.encode(scene);
	}

	/** Ideal timing leaves 10 mm tool clearance; a full-travel return seats home before the neutral dwell. */
	public static function emptyStroke(press:PressBrake):SceneArtifactDigitalProgram {
		var geometry = PressGeometry.of(press);
		var gap = Math.POSITIVE_INFINITY;
		for (station in geometry.stations()) gap = Math.min(gap, station.openTipHeight - station.dieTopHeight);
		var travel = Math.min(press.stroke, gap - 10);
		if (!Math.isFinite(travel) || travel <= 0) throw "Installed press tooling leaves no empty stroke clearance";
		var start = 0.2;
		var extended = start + press.phaseSpeeds.extensionTime(0, travel);
		var retract = extended + 0.2;
		var returned = retract + press.phaseSpeeds.returnTime(press.stroke, 0);
		return {durationSec: returned + 0.2, loop: true, events: [
			{timeSec: 0, channel: "ramValve/coilA", value: false},
			{timeSec: 0, channel: "ramValve/coilB", value: false},
			{timeSec: start, channel: "ramValve/coilA", value: true},
			{timeSec: extended, channel: "ramValve/coilA", value: false},
			{timeSec: retract, channel: "ramValve/coilB", value: true},
			{timeSec: returned, channel: "ramValve/coilB", value: false}
		]};
	}
}

/** Standalone builds exercise the same physical assembly checks as the kit smoke suite. */
function main():Void PressBrakeChecks.run();
