package toolpathkit.tool;

/**
  One piece of a tool half-profile. `r` is the distance from the tool axis and
  `z` the height above the tip, both in metres.
**/
enum CutterSegment {
  Line(r0:Float, z0:Float, r1:Float, z1:Float, zone:CutterZone);
  /** The minor circular arc from (r0, z0) to (r1, z1) about (centerR, centerZ). */
  Arc(centerR:Float, centerZ:Float, r0:Float, z0:Float, r1:Float, z1:Float,
    zone:CutterZone);
}
