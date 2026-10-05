package machinekit.motion;

import materia.assembly.AssemblyDefinition.AssemblyActuator;

/** Conservative simultaneous peak demand, without assuming that axes take turns
 * accelerating. The source and amplifier support this indefinitely; motor rated
 * torque remains the planner's thermal limit. Currents here are phase RMS for
 * the amplifier and DC for the source, never interchanged.
 */
class ServoElectricalLoad {
	public final ratedPower:Float;
	public final peakPower:Float;
	public final regenerativePower:Float;
	public final phaseCurrent:Float;
	public final minimumBusVoltage:Float;

	public function new(actuator:AssemblyActuator, winding:ServoElectricalRating, driver:MotorDriver) {
		var rated = actuator.ratedTorque, speed = actuator.ratedSpeed, curve = actuator.torqueSpeed;
		if (rated == null || speed == null || curve == null) throw "Electrical sizing needs a servo's complete torque-speed curve";
		var ratedCurrent = rated / winding.torqueConstant;
		ratedPower = (rated * speed / winding.mechanicalEfficiency + 3 * winding.phaseResistance * ratedCurrent * ratedCurrent) / driver.rating.efficiency;
		var peak = ratedPower, regeneration = 0.0, amps = 0.0, volts = 0.0;
		var index = 0;
		while (index < curve.length) {
			var rate = curve[index], torque = curve[index + 1], current = torque / winding.torqueConstant;
			var mechanical = rate * torque;
			peak = Math.max(peak, (mechanical / winding.mechanicalEfficiency + 3 * winding.phaseResistance * current * current) / driver.rating.efficiency);
			regeneration = Math.max(regeneration, mechanical);
			amps = Math.max(amps, current);
			volts = Math.max(volts, Math.sqrt(2.0) * (winding.backEmfConstant * rate + Math.sqrt(3.0) * winding.phaseResistance * current));
			index += 2;
		}
		peakPower = peak;
		regenerativePower = regeneration;
		phaseCurrent = amps;
		minimumBusVoltage = volts;
	}

	public function check(driver:MotorDriver, voltage:machinekit.power.DcVoltage, id:String):Void {
		if (driver.current.amps() + 1e-9 < phaseCurrent)
			throw 'Servo "$id" needs $phaseCurrent A phase RMS, but its driver is limited to ${driver.current.amps()} A';
		if (voltage.volts() + 1e-9 < minimumBusVoltage)
			throw 'Servo "$id" needs at least $minimumBusVoltage V DC for its curve, but its bus supplies ${voltage.volts()} V';
	}
}
