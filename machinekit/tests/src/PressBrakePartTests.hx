import machinekit.pressbrake.HydraulicCylinder;

/** Cylinder forces and volumes follow explicit dimensions rather than a guessed tonnage. */
class PressBrakePartTests {
	public static function main():Void run();
	public static function run():Void {
		var cylinder = new HydraulicCylinder(160, 100, 100);
		near(cylinder.extendForce(10e6), Math.PI * 160 * 160 * 10 / 4, "Full-bore force");
		near(cylinder.retractForce(10e6), Math.PI * (160 * 160 - 100 * 100) * 10 / 4, "Annular force");
		near(cylinder.extendForce(0), 0, "Unpressurised force");
		var body = cylinder.geometry(), rod = cylinder.movingRod().geometry();
		if (CadKit.shapeValidChecked(body.shape.borrowHandle()) == 0 || CadKit.shapeValidChecked(rod.shape.borrowHandle()) == 0)
			throw "Hydraulic cylinder native validity";
		near(body.volume(), Math.PI / 4 * (200 * 200 * 180 - 160 * 160 * 100 - 100 * 100 * 40), "Barrel material volume");
		near(rod.volume(), Math.PI / 4 * 100 * 100 * 190, "Moving rod volume");
		var rodLength = cylinder.movingRod().length;
		near(cylinder.bodyLength + cylinder.stemLength - rodLength, cylinder.capLength, "Retracted rod end in bore");
		near(cylinder.bodyLength + cylinder.stemLength + cylinder.stroke - rodLength,
			cylinder.bodyLength - cylinder.capLength, "Extended rod end in bore");
		body.close(); rod.close();
		near(cylinder.connector("extension").frame.z, 230, "Retracted rod-tip datum");
		reject(function() { var invalid = new HydraulicCylinder(100, 100, 100); });
		reject(function() { var invalid = new HydraulicCylinder(160, 100, Math.POSITIVE_INFINITY); });
		reject(function() { var invalid = cylinder.extendForce(-1); });
		trace("Press brake part tests passed");
	}
	static function near(actual:Float, expected:Float, label:String):Void {
		if (Math.abs(actual - expected) > 1e-6 * Math.max(1, Math.abs(expected))) throw '$label: $actual instead of $expected';
	}
	static function reject(fn:Void->Void):Void {
		var failed = false;
		try fn() catch (_:Dynamic) failed = true;
		if (!failed) throw "Invalid hydraulic dimensions or pressure accepted";
	}
}
