package machinekit.pressbrake.planning;

/** One physical finger contact connector at zero backgauge travel, in press-frame mm. */
class PressFingerDatum {
	public final id:String;
	public final x:Float;
	public final y:Float;
	public final z:Float;
	public function new(id:String, x:Float, y:Float, z:Float) {
		if (id == null || id.length == 0 || !Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z))
			throw "Finger contact datum needs an identity and finite coordinates";
		this.id = id; this.x = x; this.y = y; this.z = z;
	}
}
