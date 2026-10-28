package toolpathkit.motion;

/** Extra encoder delay excludes delay already carried in the observation timestamp. */
typedef GearEncoderTiming = {
  var latencyNs:Int;
  var quantum:Float;
  var loopHz:Float;
}
