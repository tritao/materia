import machinekit.motion.MotorDriver;
import machinekit.component.ComponentValues;
import machinekit.catalog.CatalogMetadata.Conformance;

/** Driver settings survive recipes and reject settings outside their assumed ratings. */
class MotorDriverTests {
	static function check(condition:Bool, message:String):Void {
		if (!condition) throw message;
	}
	static function rejects(build:Void->MotorDriver, message:String):Void {
		var failed = false;
		try { var driver = build(); } catch (error:Dynamic) { failed = true; }
		check(failed, message);
	}
	public static function run():Void {
		for (name in MotorDriver.catalog().designations()) {
			check(MotorDriver.catalog().metadata(name).conformance == GenericApproximation,
				"Generic drivers must declare assumed catalog provenance");
			var driver = new MotorDriver(name, new machinekit.power.PhaseRmsCurrent(1.68), 1);
			var rebuilt:MotorDriver = cast driver.componentType().create(driver.values());
			check(rebuilt.current.amps() == driver.current.amps() && rebuilt.microsteps == driver.microsteps &&
				rebuilt.statedVoltage == driver.statedVoltage && rebuilt.port("power").required, "Driver settings and wired source requirement must survive recipes");
			check(rebuilt.bom.partNumber == driver.bom.partNumber && rebuilt.ports().length == 3,
				"Driver recipe must preserve its BOM and service ports");
		}
		var stepper = new MotorDriver("GENERIC-TMC2209", new machinekit.power.PhaseRmsCurrent(1.68), 16, new machinekit.power.DcVoltage(24));
		check(!stepper.port("power").required && stepper.port("step-dir") != null,
			"An explicit driver voltage permits an unmodelled supply");
		var wired = new MotorDriver("GENERIC-DM542", new machinekit.power.PhaseRmsCurrent(2.8), 32);
		check(wired.port("power").required && wired.statedVoltage == null,
			"A driver without a stated voltage requires a wired supply");
		var rebuilt:MotorDriver = cast wired.componentType().create(wired.values());
		check(rebuilt.statedVoltage == null && rebuilt.port("power").required && rebuilt.microsteps == 32,
			"A wired driver remains wired after recipe rebuild");
		check(new MotorDriver("GENERIC-SERVO-AMP", new machinekit.power.PhaseRmsCurrent(5), 1, new machinekit.power.DcVoltage(24)).port("bus") != null,
			"A servo amplifier takes bus commands");
		rejects(() -> new MotorDriver("GENERIC-TMC2209", new machinekit.power.PhaseRmsCurrent(2.1), 16, new machinekit.power.DcVoltage(24)), "Reject excessive driver current");
		rejects(() -> new MotorDriver("GENERIC-DM542", new machinekit.power.PhaseRmsCurrent(0), 16, new machinekit.power.DcVoltage(24)), "Reject zero driver current");
		rejects(() -> new MotorDriver("GENERIC-TMC2209", new machinekit.power.PhaseRmsCurrent(1), 3, new machinekit.power.DcVoltage(24)), "Reject unsupported microsteps");
		rejects(() -> new MotorDriver("GENERIC-DM542", new machinekit.power.PhaseRmsCurrent(1), 256, new machinekit.power.DcVoltage(24)), "Reject microsteps above the driver rating");
		rejects(() -> new MotorDriver("GENERIC-SERVO-AMP", new machinekit.power.PhaseRmsCurrent(1), 16, new machinekit.power.DcVoltage(24)), "Reject microsteps on a servo driver");
		rejects(() -> new MotorDriver("GENERIC-TMC2209", new machinekit.power.PhaseRmsCurrent(1), 16, new machinekit.power.DcVoltage(48)), "Reject excessive supply voltage");
		rejects(() -> new MotorDriver("GENERIC-DM542", new machinekit.power.PhaseRmsCurrent(1), 16, new machinekit.power.DcVoltage(12)), "Reject insufficient supply voltage");
	}
}
