package materia.assembly;

/** Effective cord rigidity inferred from a measured single mode, not a belt manufacturer's material test.
 * Pose values are physical RobotKit joint coordinates in SI, at acquisition. Sources include content hashes.
 */
@:wire typedef BeltCalibration = {
  @:id(1) var rigidityPerWidth:Float;
  @:id(2) var dampingRatio:Float;
  @:id(3) var axis:String;
  @:id(4) var pose:Array<BeltCalibrationPosition>;
  @:id(5) var frequencyHz:Float;
  @:id(6) var massKg:Float;
  @:id(7) var spectrumSource:String;
  @:id(8) var decaySource:String;
  @:id(9) var fitError:Float;
  @:id(10) var method:String;
  @:id(11) var decayStart:Float;
  @:id(12) var decayEnd:Float;
}
typedef BeltCalibrationPosition = materia.assembly.AssemblyDefinition.AssemblyJointCoordinate;

class BeltCalibrations {
  public static inline var METHOD = "single-mode-free-decay/common-belt-v1";
  public static function validate(value:BeltCalibration):Void {
    if (!(value.rigidityPerWidth>0) || !Math.isFinite(value.rigidityPerWidth) ||
        !(value.frequencyHz>0) || !Math.isFinite(value.frequencyHz) || !(value.massKg>0) || !Math.isFinite(value.massKg) ||
        !(value.dampingRatio>0 && value.dampingRatio<0.5) || !(value.fitError>=0 && value.fitError<=0.05) ||
        !Math.isFinite(value.decayStart) || !Math.isFinite(value.decayEnd) || value.decayEnd<=value.decayStart ||
        value.axis == null || value.axis.length == 0 || value.method != METHOD ||
        value.spectrumSource == null || value.spectrumSource.length == 0 || value.decaySource == null || value.decaySource.length == 0)
      throw "Invalid measured belt calibration";
    var seen = new Map<String,Bool>();
    for (position in value.pose) {
      if (position.joint == null || position.joint.length == 0 || seen.exists(position.joint) || !Math.isFinite(position.value))
        throw "Calibration needs unique finite physical joint coordinates";
      seen.set(position.joint,true);
    }
    if (!seen.exists(value.axis)) throw "Calibration pose omits its measured axis";
  }
  public static function copy(value:BeltCalibration,map:String->String):BeltCalibration {
    return {rigidityPerWidth:value.rigidityPerWidth,dampingRatio:value.dampingRatio,axis:map(value.axis),
      pose:[for (p in value.pose) {joint:map(p.joint),value:p.value}],frequencyHz:value.frequencyHz,massKg:value.massKg,
      spectrumSource:value.spectrumSource,decaySource:value.decaySource,fitError:value.fitError,method:value.method,decayStart:value.decayStart,decayEnd:value.decayEnd};
  }
}

/** Results of a validated capture, before model-dependent mass/geometry inference. */
typedef ResonanceMeasurement = {
  var axis:String;
  var frequencyHz:Float;
  var dampingRatio:Float;
  var fitError:Float;
  var spectrumSource:String;
  var decaySource:String;
  var decayStart:Float;
  var decayEnd:Float;
}

/** Portable measurement overlay for a source-generated assembly whose parts need not have rebuild recipes. */
@:wire typedef BeltCalibrationSet = {
  @:id(1) var schemaVersion:Int;
  @:id(2) var assemblyHash:String;
  @:id(3) var modelHash:String;
  @:id(4) var belts:Array<CalibratedBelt>;
}
@:wire typedef CalibratedBelt = {
  @:id(1) var belt:String;
  @:id(2) var calibration:BeltCalibration;
}
