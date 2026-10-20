package machinekit.sizing;

import machinekit.motion.ServoMotor.ServoRating;

typedef LinearDriveInput = {
    var massKg:Float;
    var motors:Int;
    /** Metres of travel per rotor radian, including reduction. */
    var metresPerRadian:Float;
    var efficiency:Float;
    /** Each motor's external rotating inertia, referred to its rotor. Excludes rotor itself. */
    var rotaryInertiaKgM2:Float;
    var acceleration:Float;
    var velocity:Float;
    var strokeM:Float;
    var cuttingForceN:Float;
    var dragForceN:Float;
    var gravityForceN:Float;
    var rotaryDragNm:Float;
    var reserve:Float;
}

typedef LinearDriveDemand = {
    var peakNm:Float;
    var rmsNm:Float;
    var sustainedNm:Float;
    var motorRpm:Float;
    var reflectedLinearInertiaKgM2:Float;
    var externalInertiaKgM2:Float;
    var inertiaRatio:Float;
    var peakForcePerMotorN:Float;
    var cycleSeconds:Float;
    var phases:Array<{name:String,seconds:Float,torqueNm:Float}>;
    var failures:Array<String>;
}

/** Preliminary conservative envelope, not a supplier thermal/torque-speed qualification.
 * Duty: full-stroke loaded out-and-back, acceleration/cruise/deceleration and one second dwell.
 * Force magnitudes add cutting, gravity, drag and inertia in either direction, never cancel.
 * External rotary inertia receives a conservative efficiency penalty, even where
 * some of it sits at the reducer input. The motor rotor itself receives none.
 * Continuous screening also checks indefinite cutting at the requested rapid speed.
 */
class LinearDriveSizing {
    public static function demand(input:LinearDriveInput,motor:ServoRating):LinearDriveDemand {
        for(value in [input.massKg,input.metresPerRadian,input.efficiency,input.acceleration,input.velocity,input.strokeM,input.reserve,
                motor.ratedTorque,motor.peakTorque,motor.ratedSpeed,motor.maxSpeed,motor.rotorInertia])
            if(!Math.isFinite(value)||value<=0)throw "Sizing requires finite positive mass, motion and drive ratings";
        if(input.motors<1||input.efficiency>1||input.reserve<1)throw "Invalid drive sharing, efficiency or reserve";
        for(value in [input.rotaryInertiaKgM2,input.cuttingForceN,input.dragForceN,input.gravityForceN,input.rotaryDragNm])
            if(!Math.isFinite(value)||value<0)throw "Sizing inertia and force allowances must be finite and nonnegative";
        var q=input.metresPerRadian,share=1.0/input.motors,rotor=motor.rotorInertia;
        var velocity=Math.min(input.velocity,Math.sqrt(input.strokeM*input.acceleration));
        var ramp=velocity/input.acceleration,cruise=Math.max(0,input.strokeM/velocity-ramp);
        var phases:Array<{name:String,seconds:Float,torqueNm:Float}> = [];
        var force=(input.cuttingForceN+input.dragForceN+input.gravityForceN)*share;
        var sustained=force*q/input.efficiency+input.rotaryDragNm;
        var accelerationTorque=input.massKg*share*input.acceleration*q/input.efficiency+
            (input.rotaryInertiaKgM2/input.efficiency+rotor)*input.acceleration/q;
        for(leg in ["outbound","return"]) {
            phases.push({name:leg+" acceleration",seconds:ramp,torqueNm:sustained+accelerationTorque});
            if(cruise>0)phases.push({name:leg+" cruise",seconds:cruise,torqueNm:sustained});
            phases.push({name:leg+" deceleration upper bound",seconds:ramp,torqueNm:sustained+accelerationTorque});
        }
        phases.push({name:"powered dwell",seconds:1.0,torqueNm:input.gravityForceN*share*q/input.efficiency});
        var squares=0.0,time=0.0,peak=0.0;
        for(phase in phases) {squares+=phase.torqueNm*phase.torqueNm*phase.seconds;time+=phase.seconds;peak=Math.max(peak,phase.torqueNm);}
        var reflected=input.massKg*share*q*q,external=reflected+input.rotaryInertiaKgM2;
        var rms=Math.sqrt(squares/time),rpm=input.velocity/q*60/(2*Math.PI),failures:Array<String> = [];
        // Use only the nameplate constant-torque region; no invented high-speed curve.
        if(rpm>motor.ratedSpeed*60/(2*Math.PI))failures.push(motor.designation+": required speed exceeds qualified rated-speed region");
        if(peak*input.reserve>motor.peakTorque)failures.push(motor.designation+": peak torque with reserve");
        if(rms*input.reserve>motor.ratedTorque)failures.push(motor.designation+": duty RMS torque with reserve");
        if(sustained*input.reserve>motor.ratedTorque)failures.push(motor.designation+": sustained cutting torque with reserve");
        return {peakNm:peak,rmsNm:rms,sustainedNm:sustained,motorRpm:rpm,reflectedLinearInertiaKgM2:reflected,
            externalInertiaKgM2:external,inertiaRatio:external/rotor,peakForcePerMotorN:(input.massKg*input.acceleration+
            input.gravityForceN+input.cuttingForceN+input.dragForceN)*share,cycleSeconds:time,phases:phases,failures:failures};
    }
}
