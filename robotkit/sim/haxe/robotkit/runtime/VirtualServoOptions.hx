package robotkit.runtime;

import robotkit.model.RobotModel;
import robotkit.model.ActuatorDrive.ServoDrive;

/** Servo emulation on the virtual board's pulse grid; physical stepper wiring stays in DeviceBinding. */
class VirtualServoOptions {
  /** Reserve 2% of acceleration for clock fitting (up to 0.5% rate error) and float polynomial conversion. */
  public static inline var ACCELERATION_SCALE:Float = 0.98;
  public static function fromModel(model:RobotModel, tickHz:Int = 40000, baud:Int = 3000000):VirtualDeviceOptions {
    if (model == null || tickHz <= 0 || baud <= 0 || model.actuators.length == 0) throw "Virtual servos need a model and tick rate";
    var options = new VirtualDeviceOptions();
    options.stepTickHz = tickHz;
    // Full servo polynomials carry more coefficients than the stepper defaults;
    // the endpoint still qualifies every submitted plan against this link rate.
    options.baud = baud;
    options.targetError = 0.0;
    for (actuator in model.actuators) {
      var drive = actuator.drive;
      if (drive == null || !Std.isOfType(drive, ServoDrive)) throw 'Virtual servo "${actuator.id}" needs servo drive data';
      var servo = cast(drive, ServoDrive);
      var rate = actuator.planningRate();
      if (rate == null || rate <= 0.0) throw 'Virtual servo "${actuator.id}" needs a speed limit';
      // The simulated pulse board cannot represent more than one count per tick at rated peak speed.
      // Expose that coarser command grid explicitly; never change the CAD motor or its physical encoder.
      switch actuator.transmission {
        case SimpleTransmission(joint, ratio, offset):
          var index = -1;
          for (i in 0...model.joints.length) if (model.joints[i].id == joint) index = i;
          if (index < 0) throw "Virtual servo joint is missing";
          var encoderCountsPerUnit = servo.encoderCounts / (2.0 * Math.PI);
          if (actuator.encoder != "") {
            encoderCountsPerUnit = 0.0;
            for (encoder in model.encoders) if (encoder.id == actuator.encoder && encoder.joint == joint)
              encoderCountsPerUnit = encoder.countsPerUnit / Math.abs(ratio);
          }
          if (encoderCountsPerUnit <= 0.0) throw 'Virtual servo "${actuator.id}" needs its authored encoder';
          // At most one count per tick at peak speed: either the grid is coarsened to exactly that (one
          // tick), or the encoder is coarser and its counts at peak speed set the ticks.
          var coarsened = tickHz / rate < encoderCountsPerUnit;
          var countsPerUnit = coarsened ? tickHz / rate : encoderCountsPerUnit;
          var ticks = coarsened ? 1 : StepTicks.forLimit(rate * encoderCountsPerUnit, tickHz);
          options.actuators.push(new VirtualActuatorOptions(actuator.id, index, ratio, offset, countsPerUnit, ticks));
          options.targetError = Math.max(options.targetError, 1.0 / (countsPerUnit * Math.abs(ratio)));
      }
    }
    return options;
  }
}
