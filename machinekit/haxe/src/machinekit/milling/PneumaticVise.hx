package machinekit.milling;

import machinekit.assembly.MachineAssembly;
import machinekit.pneumatic.PneumaticCylinder;
import machinekit.component.MachineComponent;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A preset fixed jaw, guided moving jaw, parallels and end stop.
 * Jaw travel closes the opening; the 6 mm air stroke is separate from the screw preset.
 * The datum is the fixed jaw face, parallels' top and end stop face.
 */
class PneumaticVise extends MachineAssembly {
	public final jawWidth:Float = 100;
	public final stroke:Float = 6;
	public final blankWidth:Float;
	public final blankLength:Float;
	public final clearance:Float;
	public final body:MillPanel;
	public final fixedJaw:MillPanel;
	public final movingJaw:MillPanel;
	public final parallels:MillPanel;
	public final endStop:MillPanel;
	public final cylinder:PneumaticCylinder;
	public final datum:AssemblyFrame;
	public final openGap:Float;

	public function new(blankWidth:Float = 40, blankLength:Float = 60, clearance:Float = 1.5) {
		super();
		if (!(blankWidth > stroke) || !(blankLength > 0 && blankLength < jawWidth) ||
			!(clearance > 0 && 2 * clearance < stroke)) throw "Vise preset needs room for the blank and air stroke";
		this.blankWidth = blankWidth; this.blankLength = blankLength; this.clearance = clearance;
		openGap = blankWidth + 2 * clearance;
		cylinder = new PneumaticCylinder("ISO15552", 32, stroke, 30);
		body = new MillPanel(jawWidth, Math.max(100, openGap + 45), 25);
		fixedJaw = new MillPanel(jawWidth, 15, 30);
		movingJaw = new MillPanel(jawWidth, 15, 30);
		parallels = new MillPanel(blankLength, 5, 10);
		endStop = new MillPanel(6, blankWidth, 15);
		addComponent("body", body);
		var fixedPose = AssemblyFrames.translation(0, -blankWidth / 2 - fixedJaw.depth / 2, body.height);
		fixed("fixedJaw", fixedJaw, fixedPose);
		var jawPose = AssemblyFrames.translation(0, -blankWidth / 2 + openGap + movingJaw.depth / 2, body.height);
		addComponent("movingJaw", movingJaw, jawPose);
		addMemberConnector("body", "jawGuide", jawPose);
		addMemberConnector("movingJaw", "guide", AssemblyFrames.identity());
		var jawDefault = 2 * clearance;
		addMateOnAxis("jaw", "prismatic", "body", "jawGuide", "movingJaw", "guide", {x: 0, y: -1, z: 0},
			jawDefault, {lower: 0, upper: stroke, velocity: null, effort: null,
				overtravel: cylinder.endStopCompliance});
		// The piston rod is fixed to the moving jaw and reaches back into the cylinder body.
		var rodFrame:AssemblyFrame = {x: 0, y: 0, z: 0, qx: Math.sin(Math.PI / 4), qy: 0,
			qz: 0, qw: Math.cos(Math.PI / 4)};
		addComponent("clampRod", cylinder.movingRod(), AssemblyFrames.compose(jawPose, rodFrame));
		addMemberConnector("movingJaw", "clampRodMount", rodFrame);
		addMate("clamp-rod-mount", "fixed", "movingJaw", "clampRodMount", "clampRod", "mount");
		// The cylinder body is part of the vise; only its air ports cross this assembly boundary.
		var cylinderPose:AssemblyFrame = {x: 0, y: jawPose.y + cylinder.bodyLength + cylinder.stemLength - jawDefault,
			z: jawPose.z, qx: rodFrame.qx, qy: rodFrame.qy, qz: rodFrame.qz, qw: rodFrame.qw};
		fixed("cylinder", cylinder, cylinderPose);
		exposePort("airA", "cylinder", "A");
		exposePort("airB", "cylinder", "B");
		var parallelPose = AssemblyFrames.translation(0, -(blankWidth / 2 - 1.5 * parallels.depth), body.height);
		fixed("parallel0", parallels, parallelPose);
		fixed("parallel1", parallels, AssemblyFrames.translation(0, -parallelPose.y, parallelPose.z));
		var stopPose = AssemblyFrames.translation(-blankLength / 2 - endStop.width / 2, 0, body.height);
		fixed("endStop", endStop, stopPose);
		datum = AssemblyFrames.translation(stopPose.x + endStop.width / 2,
			fixedPose.y + fixedJaw.depth / 2, parallelPose.z + parallels.height);
		addMemberConnector("body", "datum", datum);
		exposeConnector("datum", "body", "datum");
		exposeConnector("mount", "body", "mount");
	}

	function fixed(id:String, part:MachineComponent, pose:AssemblyFrame):Void {
		addComponent(id, part, pose);
		addMemberConnector("body", '$id-seat', pose);
		addMemberConnector(id, "seat", AssemblyFrames.identity());
		addMate('$id-mount', "fixed", "body", '$id-seat', id, "seat");
	}
}
