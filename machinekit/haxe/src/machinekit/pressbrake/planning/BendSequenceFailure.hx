package machinekit.pressbrake.planning;

import cadkit.sheetmetal.SheetDiagnosis;

/** A failed action retains its completed state and authored bend/station identities. */
class BendSequenceFailure {
	final done:Array<Int>;
	public final bend:Int;
	public final station:String;
	public final diagnosis:SheetDiagnosis;
	public final orientation:Null<BendOrientation>;
	public final gaugeEdge:Null<GaugeEdge>;
	public final grip:Null<SheetGrip>;
	public function new(completed:Array<Int>,bend:Int,station:String,diagnosis:SheetDiagnosis,
			?orientation:BendOrientation,?gaugeEdge:GaugeEdge,?grip:SheetGrip) {
		done = completed.copy(); this.bend = bend; this.station = station; this.diagnosis = diagnosis;
		this.orientation = orientation; this.gaugeEdge = gaugeEdge; this.grip = grip;
	}
	public function completed():Array<Int> return done.copy();
	/** Bounded reports retain different failed actions and completed states before repeated candidates. */
	public function sameReason(other:BendSequenceFailure):Bool {
		if (bend != other.bend || station != other.station || diagnosis.code != other.diagnosis.code || done.length != other.done.length) return false;
		for (i in 0...done.length) if (done[i] != other.done[i]) return false;
		if ((diagnosis.code == "press.collision" || diagnosis.code == "press.grip-collision")
				&& diagnosis.elements.length >= 2 && other.diagnosis.elements.length >= 2)
			return diagnosis.elements[1] == other.diagnosis.elements[1];
		return true;
	}
	public function same(other:BendSequenceFailure):Bool {
		if (bend != other.bend || station != other.station || diagnosis.code != other.diagnosis.code
			|| diagnosis.message != other.diagnosis.message || done.length != other.done.length
			|| diagnosis.elements.length != other.diagnosis.elements.length) return false;
		for (i in 0...done.length) if (done[i] != other.done[i]) return false;
		for (i in 0...diagnosis.elements.length) if (diagnosis.elements[i] != other.diagnosis.elements[i]) return false;
		var a = orientation, b = other.orientation;
		if ((a == null) != (b == null)) return false;
		if (a != null && b != null && (a.faceAUp != b.faceAUp || a.reverse != b.reverse || a.bedCenter != b.bedCenter)) return false;
		var ga = gaugeEdge, gb = other.gaugeEdge;
		if ((ga == null) != (gb == null)) return false;
		if (ga != null && gb != null && (ga.wall != gb.wall || ga.edge.loop != gb.edge.loop || ga.edge.segment != gb.edge.segment)) return false;
		var first = grip, second = other.grip;
		if ((first == null) != (second == null)) return false;
		return first == null || second == null || BendTimeModel.sameGrip(first,second);
	}
}
