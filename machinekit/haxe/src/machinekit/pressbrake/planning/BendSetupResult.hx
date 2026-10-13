package machinekit.pressbrake.planning;

import cadkit.sheetmetal.process.BendProcessResult;

/** Rebuilt single-bend data. Stroke poses are sampled geometry, not a collision clearance certificate. */
class BendSetupResult {
	public final bend:Int;
	public final station:String;
	public final orientation:BendOrientation;
	public final gaugeEdge:GaugeEdge;
	public final registration:BendRegistration;
	public final process:BendProcessResult;
	public final ramTravel:Float;
	public final sampledSwing:PressEnvelope;
	final strokePoses:Array<PressSheetPose>;
	public function new(bend:Int, station:String, orientation:BendOrientation, gaugeEdge:GaugeEdge,
			registration:BendRegistration, process:BendProcessResult, ramTravel:Float,
			sampledSwing:PressEnvelope, stroke:Array<PressSheetPose>) {
		this.bend = bend; this.station = station; this.orientation = orientation; this.gaugeEdge = gaugeEdge;
		this.registration = registration; this.process = process; this.ramTravel = ramTravel;
		this.sampledSwing = sampledSwing; strokePoses = stroke.copy();
	}
	public function stroke():Array<PressSheetPose> return strokePoses.copy();
}
