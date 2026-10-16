package machinekit.printing;

/** Authored choices for a first-order thermal block's PI controller and fault monitoring.
 * Every field is required. This policy describes the assumed plant; a controller with
 * transport delay or different thermal dynamics needs its own tuning.
 */
typedef HeaterControlPolicySpec = {
	final responseTimeFraction:Float;
	final integralTimeFraction:Float;
	final maxDuty:Float;
	final bandCelsius:Float;
	final pidRangeCelsius:Float;
	final gainWindowFraction:Float;
	final gainWindowMinSeconds:Float;
	final gainMinCelsius:Float;
	final hysteresisCelsius:Float;
	final hysteresisWindowSeconds:Float;
}

typedef HeaterControlSettings = {
	final kp:Float;
	final ki:Float;
	final kd:Float;
	final maxDuty:Float;
	final bandCelsius:Float;
	final pidRangeCelsius:Float;
	final gainWindowSeconds:Float;
	final gainMinCelsius:Float;
	final hysteresisCelsius:Float;
	final hysteresisWindowSeconds:Float;
}

/** One immutable policy, applied to each heater's own heat capacity, power and loss. */
class HeaterControlPolicy {
	final spec:HeaterControlPolicySpec;

	public function new(value:HeaterControlPolicySpec) {
		if (value == null) throw "A heater control policy is required";
		for (number in [value.responseTimeFraction, value.integralTimeFraction, value.maxDuty,
			value.bandCelsius, value.pidRangeCelsius, value.gainWindowFraction, value.gainWindowMinSeconds,
			value.gainMinCelsius, value.hysteresisCelsius, value.hysteresisWindowSeconds])
			if (!Math.isFinite(number) || number <= 0) throw "Heater policy values must be finite and positive";
		if (value.maxDuty > 1 || value.pidRangeCelsius < value.bandCelsius)
			throw "Heater policy needs duty at most one and a PID range covering its settling band";
		// Snapshot the authored record: callers cannot mutate a running policy through an alias.
		spec = {responseTimeFraction: value.responseTimeFraction, integralTimeFraction: value.integralTimeFraction,
			maxDuty: value.maxDuty, bandCelsius: value.bandCelsius, pidRangeCelsius: value.pidRangeCelsius,
			gainWindowFraction: value.gainWindowFraction, gainWindowMinSeconds: value.gainWindowMinSeconds,
			gainMinCelsius: value.gainMinCelsius, hysteresisCelsius: value.hysteresisCelsius,
			hysteresisWindowSeconds: value.hysteresisWindowSeconds};
	}

	/** C dT/dt = P*duty - loss*(T-room); tau=C/loss, kp=C/(P*lambda), ki=kp/Ti.
	 * There is no derivative term: this assumed block has no transport delay to lead.
	 */
	public function settings(capacity:Float, power:Float, loss:Float):HeaterControlSettings {
		for (number in [capacity, power, loss])
			if (!Math.isFinite(number) || number <= 0) throw "First-order heater tuning needs positive finite C, P and loss";
		var tau = capacity / loss;
		var kp = capacity / (power * (tau * spec.responseTimeFraction));
		var ki = kp / (tau * spec.integralTimeFraction);
		var window = Math.max(spec.gainWindowMinSeconds, tau * spec.gainWindowFraction);
		for (number in [tau, kp, ki, window])
			if (!Math.isFinite(number) || number <= 0) throw "Heater tuning exceeds numeric range";
		return {kp: kp, ki: ki, kd: 0.0, maxDuty: spec.maxDuty, bandCelsius: spec.bandCelsius,
			pidRangeCelsius: spec.pidRangeCelsius, gainWindowSeconds: window, gainMinCelsius: spec.gainMinCelsius,
			hysteresisCelsius: spec.hysteresisCelsius, hysteresisWindowSeconds: spec.hysteresisWindowSeconds};
	}
}
