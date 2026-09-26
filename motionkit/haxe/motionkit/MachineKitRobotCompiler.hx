package motionkit;

import machinekit.assembly.LinearAxis;
import motionkit.axis.MotionSystemBlueprint;
import robotkit.model.RobotModel;

/** Root-package facade for the MachineKit bridge. */
class MachineKitRobotCompiler {
  public static inline final MILLIMETRES_TO_METRES:Float =
    motionkit.bridge.MachineKitRobotCompiler.MILLIMETRES_TO_METRES;
  public static inline final DEFAULT_MAX_VELOCITY:Float =
    motionkit.bridge.MachineKitRobotCompiler.DEFAULT_MAX_VELOCITY;
  public static inline final DEFAULT_MAX_ACCELERATION:Float =
    motionkit.bridge.MachineKitRobotCompiler.DEFAULT_MAX_ACCELERATION;

  public static function compileLinearAxis(axis:LinearAxis, axisId:String,
      ?maxVelocity:Float = DEFAULT_MAX_VELOCITY,
      ?maxAcceleration:Float = DEFAULT_MAX_ACCELERATION):MotionSystemBlueprint {
    return motionkit.bridge.MachineKitRobotCompiler.compileLinearAxis(axis, axisId,
      maxVelocity, maxAcceleration);
  }

  public static function compileLinearAxisModel(axis:LinearAxis, axisId:String,
      ?maxVelocity:Float = DEFAULT_MAX_VELOCITY):RobotModel {
    return motionkit.bridge.MachineKitRobotCompiler.compileLinearAxisModel(axis, axisId,
      maxVelocity);
  }

  public static function compileXYZGantry(xAxis:LinearAxis, yAxis:LinearAxis, zAxis:LinearAxis,
      ?maxVelocity:Float = DEFAULT_MAX_VELOCITY,
      ?maxAcceleration:Float = DEFAULT_MAX_ACCELERATION):MotionSystemBlueprint {
    return motionkit.bridge.MachineKitRobotCompiler.compileXYZGantry(xAxis, yAxis, zAxis,
      maxVelocity, maxAcceleration);
  }
}
