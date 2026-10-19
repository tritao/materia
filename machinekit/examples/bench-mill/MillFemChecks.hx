import machinekit.milling.MillZAxisLayout;

/** Tests physical datum motion before any FE discretization is introduced. */
class MillFemChecks {
	static var assertions = 0;
	static function check(value:Bool, label:String):Void {
		assertions++;
		if (!value) throw label;
	}
	static function near(a:Float,b:Float,label:String):Void
		check(Math.abs(a-b)<1e-6,'$label: $a vs $b');
	static function rejected(action:Void->Void,label:String):Void {
		var failed = false;
		try action() catch (_:Dynamic) failed = true;
		check(failed,label);
	}
	public static function main():Void {
		near(MillFemExport.coordinate("1.25e2"),125,"numeric coordinate in millimetres");
		rejected(() -> {var unused = MillFemExport.coordinate("1mm");},"reject ambiguous unit suffix");
		rejected(() -> {var unused = MillFemExport.coordinate("1e999");},"reject numeric overflow");
		var mill = MillFemExport.build(), layout:MillZAxisLayout = mill.frame.zAxis;
		var state = MillFemExport.state(mill.assembly,0,0,layout.lower/2);
		var origin = MillFemExport.datums(state), fixed = state.worldPose("frame/base/casting");
		state.setJoint("xyStage/x",10);
		var x = MillFemExport.datums(state);
		near(x.workpieceReferenceMm.x-origin.workpieceReferenceMm.x,-10,"positive X joint moves table in world -X");
		near(x.toolTipMm.x,origin.toolTipMm.x,"tool independent of X table motion");
		state.setJoint("xyStage/y",20);
		var y = MillFemExport.datums(state);
		near(y.workpieceReferenceMm.y-x.workpieceReferenceMm.y,-20,"positive Y joint moves table in world -Y");
		near(y.toolTipMm.y,origin.toolTipMm.y,"tool independent of Y table motion");
		state.setJoint("zAxis/z",layout.lower/2+30);
		var z = MillFemExport.datums(state);
		near(z.toolTipMm.z-origin.toolTipMm.z,30,"tool follows Z");
		near(z.workpieceReferenceMm.z,origin.workpieceReferenceMm.z,"workpiece independent of Z");
		near(z.workpieceReferenceMm.z-z.tableTopMm.z,50,"payload height above table");
		var base = state.worldPose("frame/base/casting");
		near(base.x,fixed.x,"base stationary X"); near(base.y,fixed.y,"base stationary Y"); near(base.z,fixed.z,"base stationary Z");
		rejected(() -> {var unused = MillFemExport.state(mill.assembly,Math.NaN,0,layout.lower/2);},"reject non-finite pose");
		rejected(() -> {var unused = MillFemExport.state(mill.assembly,10000,0,layout.lower/2);},"reject travel outside limits");
		for (height in [layout.lower,-0.01,0.0]) {
			var edge = MillFemExport.state(mill.assembly,125,80,height);
			var directions = MillFemExport.jointDirections(edge);
			near(directions[0].positiveDirectionWorld.x,-1,"X world direction at travel limit");
			near(directions[1].positiveDirectionWorld.y,-1,"Y world direction at travel limit");
			near(directions[2].positiveDirectionWorld.z,1,"Z world direction near travel limit");
			near(edge.joint("xyStage/x"),125,"direction sampling restores X");
			near(edge.joint("xyStage/y"),80,"direction sampling restores Y");
			near(edge.joint("zAxis/z"),height,"direction sampling restores Z");
		}
		Sys.println('Mill FEM datum checks: $assertions assertions');
	}
}
