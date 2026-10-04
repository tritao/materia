package cadkit.sketch;

/** One validated outer boundary and its directly nested holes, retaining authored entity identity. */
class ProfileRegion {
	public final id:String;
	final outerCurves:Array<ProfileCurve>;
	final holeCurves:Array<Array<ProfileCurve>>;
	public function new(id:String, outer:Array<ProfileCurve>, holes:Array<Array<ProfileCurve>>) {
		this.id = id;
		outerCurves = outer.copy();
		holeCurves = [for (hole in holes) hole.copy()];
	}
	public function outer():Array<ProfileCurve> { return outerCurves.copy(); }
	public function holes():Array<Array<ProfileCurve>> { return [for (hole in holeCurves) hole.copy()]; }
}
