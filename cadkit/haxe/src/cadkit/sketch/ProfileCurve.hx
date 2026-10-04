package cadkit.sketch;

import cadkit.modeling.Vector;

/** Exact solved profile curve in canonical wall-local millimetres. */
class ProfileCurve {
	public final entityId:String;
	public final start:Vector;
	public final end:Vector;
	public final center:Null<Vector>;
	public final radius:Float;
	public final startAngle:Float;
	public final sweep:Float;
	public function new(entityId:String, start:Vector, end:Vector, ?center:Vector, radius:Float = 0,
		startAngle:Float = 0, sweep:Float = 0) {
		this.entityId = entityId;
		this.start = start;
		this.end = end;
		this.center = center;
		this.radius = radius;
		this.startAngle = startAngle;
		this.sweep = sweep;
	}
}
