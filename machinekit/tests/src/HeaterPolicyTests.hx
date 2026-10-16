import machinekit.printing.HeaterControlPolicy;
import machinekit.printing.HeaterControlPolicy.HeaterControlPolicySpec;

/** Plant response checks independent of the runtime heater implementation. */
class HeaterPolicyTests {
	static function check(value:Bool, message:String):Void { if (!value) throw message; }
	static function choices():HeaterControlPolicySpec return {
		responseTimeFraction: 0.05, integralTimeFraction: 0.125, maxDuty: 1.0,
		bandCelsius: 3.0, pidRangeCelsius: 10.0, gainWindowFraction: 0.25,
		gainWindowMinSeconds: 20.0, gainMinCelsius: 2.0,
		hysteresisCelsius: 15.0, hysteresisWindowSeconds: 10.0};
	public static function main():Void run();
	public static function run():Void {
		var authored = choices(), policy = new HeaterControlPolicy(authored);
		Reflect.setField(authored,"bandCelsius",9);
		check(policy.settings(10,40,0.05).bandCelsius == 3, "Policy retained a mutable input alias");
		for (capacity in [10.0,100.0]) for (power in [40.0,250.0]) for (loss in [0.05,0.7]) {
			var control = policy.settings(capacity,power,loss), tau = capacity/loss;
			var dt = tau/10000, temperature = 26.0, target = 27.0, ambient = 22.0;
			var integral = loss*(target-ambient)/power;
			for (_ in 0...20000) {
				var error = target-temperature;
				integral += control.ki*error*dt;
				var duty = Math.max(0,Math.min(control.maxDuty,control.kp*error+integral));
				temperature += (power*duty-loss*(temperature-ambient))*dt/capacity;
				check(Math.isFinite(temperature) && temperature>ambient && temperature<target+1,
					"PI response escaped the first-order plant's neighbourhood");
			}
			check(Math.abs(temperature-target)<0.001,"PI policy did not reject a temperature disturbance");
			check(control.gainWindowSeconds>=20 && control.gainWindowSeconds>=tau/4,
				"Gain watchdog window does not cover the authored plant fraction");
		}
		for (bad in [0.0,-1.0,Math.NaN,Math.POSITIVE_INFINITY]) {
			var input = choices(); Reflect.setField(input,"bandCelsius",bad);
			var rejected = false; try new HeaterControlPolicy(input) catch (_:Dynamic) rejected = true;
			check(rejected,"Invalid temperature policy accepted");
			rejected = false; try policy.settings(bad,40,0.05) catch (_:Dynamic) rejected = true;
			check(rejected,"Invalid thermal plant accepted");
		}
		var invalid = choices(); Reflect.setField(invalid,"maxDuty",1.1);
		var rejected = false; try new HeaterControlPolicy(invalid) catch (_:Dynamic) rejected = true;
		check(rejected,"Duty outside the physical range accepted");
		Sys.println("Heater control policy response checks passed");
	}
}
