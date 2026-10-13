import machinekit.pressbrake.HydraulicCylinder;
import machinekit.fluid.CylinderPhaseSpeeds;

/** Cylinder forces and volumes follow explicit dimensions rather than a guessed tonnage. */
class PressBrakePartTests {
	public static function main():Void run();
	public static function run():Void {
		checkPhaseSpeeds();
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
	static function checkPhaseSpeeds():Void {
		// An 80 mm open tip gap above a 1.5 mm sheet changes speed 10 mm above it.
		var phases = new CylinderPhaseSpeeds(100, 80, 8, 70, 80 - 1.5 - 10);
		near(phases.extensionSpeed(68.499), 80, "Approach before change point");
		near(phases.extensionSpeed(68.5), 8, "Working at change point");
		near(phases.extensionSpeed(100.1), 8, "Working during end-stop overtravel");
		near(phases.maximumSpeed(), 80, "Maximum phase speed");
		near(phases.extensionTime(0, 100), 4.79375, "Full no-load extension cycle");
		near(phases.extensionTime(60, 76.5), 1.10625, "Interval crossing speed change");
		near(phases.returnTime(100, 0), 10.0 / 7, "Full return cycle");
		near(phases.extensionTime(0, 68.5) + phases.extensionTime(68.5, 100),
			phases.extensionTime(0, 100), "Phase times compose");
		near(new CylinderPhaseSpeeds(100, 80, 8, 70, 0).extensionTime(0, 100), 12.5, "Entire stroke working");
		near(new CylinderPhaseSpeeds(100, 80, 8, 70, 100).extensionTime(0, 100), 1.25, "Entire stroke approaching");
		reject(function() { var invalid = new CylinderPhaseSpeeds(100, 80, 0, 70, 68.5); });
		reject(function() { var invalid = new CylinderPhaseSpeeds(100, 80, 8, 70, 101); });
		reject(function() { var invalid = new CylinderPhaseSpeeds(100, 80, 8, 70, -1); });
		reject(function() { var invalid = new CylinderPhaseSpeeds(100, Math.POSITIVE_INFINITY, 8, 70, 68.5); });
		reject(function() { var invalid = phases.extensionTime(10, 0); });
		reject(function() { var invalid = phases.returnTime(0, 10); });
		reject(function() { var invalid = phases.extensionTime(0, 101); });
		reject(function() { var invalid = phases.extensionSpeed(Math.NaN); });
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
