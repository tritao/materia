package gearkit;

/** The closed one-pitch polygon shared by the tooth profiles. `side` is the right-hand boundary in
    order: root circle point, root blend to the flank join, flank, tip blend to the tip circle. */
class ToothOutline {
  /** Region of a point at `radius` on the tooth: flank strictly between the two join radii, root blend up to
      the root join, tip (blend and land) from the tip join on, whatever the gear kind. The join points
      themselves belong to the blends (to 1e-9 mm, so outline points built at a join classify consistently). */
  public static function regionAt(profile:ToothProfile, radius:Float):ToothRegion {
    var sign = profile.gearSpec().sign(), rootJoin = profile.rootJoinRadius();
    var along = sign * (radius - rootJoin), span = sign * (profile.tipJoinRadius() - rootJoin);
    return along <= 1e-9 ? Root : along >= span - 1e-9 ? Tip : Flank;
  }

  public static function close(spec:GearSpec, side:Array<Point2>, samples:Int):Array<Point2> {
    var result:Array<Point2> = [];
    var pitchHalf = Math.PI / spec.teeth;
    result.push(new Point2(-spec.rootRadius() * Math.sin(pitchHalf), spec.rootRadius() * Math.cos(pitchHalf)));
    var rootAngle = Math.atan2(side[0].x, side[0].y);
    for (i in 1...samples + 1) {
      var angle = -pitchHalf + (pitchHalf - rootAngle) * i / samples;
      result.push(new Point2(spec.rootRadius() * Math.sin(angle), spec.rootRadius() * Math.cos(angle)));
    }
    var j = 1;
    while (j < side.length) {
      result.push(new Point2(-side[j].x, side[j].y));
      j++;
    }
    var tipAngle = Math.atan2(side[side.length - 1].x, side[side.length - 1].y);
    for (i in 1...samples + 1) {
      var angle = -tipAngle + 2 * tipAngle * i / samples;
      result.push(new Point2(spec.tipRadius() * Math.sin(angle), spec.tipRadius() * Math.cos(angle)));
    }
    j = side.length - 2;
    while (j >= 0) result.push(side[j--]);
    for (i in 1...samples + 1) {
      var angle = rootAngle + (pitchHalf - rootAngle) * i / samples;
      result.push(new Point2(spec.rootRadius() * Math.sin(angle), spec.rootRadius() * Math.cos(angle)));
    }
    // The last-to-first edge follows the root chord across a pitch; repeat/rotate for a full gear.
    result.push(result[0]);
    return result;
  }
}
