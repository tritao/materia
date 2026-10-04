package motionkit.path;

/** Add a lateral weave without changing authored length, feed or event distances. */
class WeavePath {
  public static function apply(base:PosePath, profile:WeaveProfile, frame:WeaveFrame,
      ?phaseDistance:Float = 0.0):PosePath {
    if (base == null || profile == null || frame == null || !Math.isFinite(phaseDistance) || phaseDistance < 0.0)
      throw "Weave path requires a base, profile, material frame and nonnegative phase distance";
    if (profile.amplitude == 0.0) return base;
    var primitives:Array<PosePrimitive> = [];
    var offset = 0.0;
    var taper = Math.min(profile.period / 4.0, base.length() / 2.0);
    for (primitive in base.primitives) {
      var cuts = [0.0];
      for (at in profile.breaks(offset + phaseDistance, offset + primitive.length() + phaseDistance))
        cuts.push(at - offset - phaseDistance);
      cuts.push(primitive.length());
      for (i in 0...(cuts.length - 1)) primitives.push(new PoseWeave(primitive, cuts[i], cuts[i + 1], offset,
        profile, frame, base.length(), taper, phaseDistance));
      offset += primitive.length();
    }
    return new PosePath(base.frameId, primitives, base.events);
  }
}
