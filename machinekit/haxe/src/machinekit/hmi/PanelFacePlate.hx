package machinekit.hmi;

import cadkit.modeling.RoundedRectangle;
import cadkit.modeling.Footprint;
import cadkit.modeling.FootprintGeometry;
import cadkit.modeling.Part;
import machinekit.component.PanelBoss;

/**
 * The panel plate as a solid: a rounded plate with the cutouts of every control and the extra holes of
 * the wheel. Printed on its own it is the layout mock-up: drop the real parts into it. With screw
 * positions (see PendantCase) it also carries M3 counterbored holes for the posts of the shell.
 */
class PanelFacePlate {
  public static inline var CORNER_RADIUS:Float = 8.0;
  public static inline var SCREW_HOLE:Float = 3.4;
  public static inline var COUNTERBORE:Float = 6.2;
  public static inline var COUNTERBORE_DEPTH:Float = 1.8;

  public static function part(face:PanelLayout,
    ?fasteners:Array<{x:Float, y:Float}>, ?extraBosses:Array<PanelBoss>):Part {
    var plate = RoundedRectangle.box(face.width, face.height, CORNER_RADIUS, 0.0, face.thickness);
    var tools:Array<Part> = [];
    try {
      for (c in face.controls) {
        tools.push(FootprintGeometry.solid(c.cutout(), c.x, c.y, -1.0, face.thickness + 1.0));
        for (hole in c.holes()) tools.push(FootprintGeometry.solid(
          Circle(hole.diameter),
          c.x + hole.x,
          c.y + hole.y,
          -1.0,
          face.thickness + 1.0
        ));
      }
      if (fasteners != null) for (screw in fasteners) {
        tools.push(FootprintGeometry.solid(Circle(SCREW_HOLE), screw.x, screw.y, -1.0, face.thickness + 1.0));
        tools.push(Part.cylinderSpan(
          COUNTERBORE / 2,
          face.thickness - COUNTERBORE_DEPTH,
          face.thickness + 1.0,
          screw.x,
          screw.y
        ));
      }
      // Bosses stand on the plate's back (z below 0) and are fused to it before the holes are cut.
      var raised = plate;
      var bosses:Array<PanelBoss> = extraBosses == null ? [] : extraBosses.copy();
      for (c in face.controls) for (b in c.bosses()) bosses.push({
        x: c.x + b.x,
        y: c.y + b.y,
        diameter: b.diameter,
        height: b.height,
        pilotDiameter: b.pilotDiameter,
        pilotDepth: b.pilotDepth
      });
      for (b in bosses) {
        var boss = Part.cylinderSpan(b.diameter / 2, -b.height, 0.5, b.x, b.y);
        var fused = raised.combine(boss);
        if (raised != plate) raised.close();
        boss.close();
        raised = fused;
        tools.push(Part.cylinderSpan(b.pilotDiameter / 2, -b.height - 1.0, -b.height + b.pilotDepth, b.x, b.y));
      }
      var result = raised.subtractAll(tools);
      if (raised != plate) raised.close();
      plate.close();
      for (t in tools) t.close();
      return result;
    } catch (error:Dynamic) {
      plate.close();
      for (t in tools) t.close();
      throw error;
    }
  }
}
