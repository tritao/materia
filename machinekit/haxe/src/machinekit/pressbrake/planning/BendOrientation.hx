package machinekit.pressbrake.planning;

/** Authored insertion: upward face, end toward the gauge and lateral bend centre, in mm. */
class BendOrientation {
	public final faceAUp:Bool;
	public final reverse:Bool;
	public final bedCenter:Float;
	public function new(faceAUp:Bool, reverse:Bool, bedCenter:Float) {
		if (!Math.isFinite(bedCenter)) throw "Bend centre must be finite press-frame mm";
		this.faceAUp = faceAUp; this.reverse = reverse; this.bedCenter = bedCenter;
	}
}
