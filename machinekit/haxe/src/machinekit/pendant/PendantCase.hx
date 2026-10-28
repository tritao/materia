package machinekit.pendant;

import cadkit.modeling.RoundedRectangle;
import cadkit.modeling.FootprintGeometry;
import machinekit.hmi.*;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/**
 * The shell behind the face plate. The face plate lies on the shell's rim (z = 0 up, the plate from 0 to its
 * thickness) and M3 screws pass down through it into posts that rise from the back wall. Posts are
 * placed where nothing in front or behind needs the room: clear of every keep-out (so a screw head
 * does not land on a control) and of every part's rear body. The cable leaves through a round hole in the
 * bottom wall at mid depth. No gasket, strap or hang point yet.
 */
class PendantCase {
  public static inline var BACK_WALL:Float = 3.0;
  public static inline var POST_DIAMETER:Float = 8.0;
  public static inline var PILOT_DIAMETER:Float = 2.6;
  public static inline var PILOT_DEPTH:Float = 14.0;
  public static inline var CABLE_HOLE:Float = 9.0;
  /** Room kept around a screw head and around a post. */
  public static inline var HEAD_CLEARANCE:Float = 3.0;
  public static inline var BODY_CLEARANCE:Float = 1.5;

  public final face:PanelLayout;
  public final posts:Array<{x:Float, y:Float}>;
  public final cableX:Float;
  public final cableZ:Float;

  function new(face:PanelLayout, posts:Array<{x:Float, y:Float}>, cableX:Float, cableZ:Float) {
    this.face = face; this.posts = posts; this.cableX = cableX; this.cableZ = cableZ;
  }

  /** Places six posts (corners and mid top and bottom) and the cable hole for a face, or throws if there is no room. */
  public static function plan(face:PanelLayout):PendantCase {
    var hw = face.width / 2 - face.wall - POST_DIAMETER / 2 - 1.0, hh = face.height / 2 - face.wall - POST_DIAMETER / 2 - 1.0;
    var wanted = [[-hw, hh], [0.0, hh], [hw, hh], [-hw, -hh], [0.0, -hh], [hw, -hh]];
    var posts:Array<{x:Float, y:Float}> = [];
    for (target in wanted) {
      var best:Null<{x:Float, y:Float}> = null, bestDistance = 1e9;
      var x = -hw;
      while (x <= hw + 1e-9) {
        var y = -hh;
        while (y <= hh + 1e-9) {
          var d = Math.sqrt((x - target[0]) * (x - target[0]) + (y - target[1]) * (y - target[1]));
          if (d < bestDistance && free(face, x, y, posts)) { best = {x: x, y: y}; bestDistance = d; }
          y += 1.5;
        }
        x += 1.5;
      }
      if (best == null) throw "No room for a screw post near (" + target[0] + ", " + target[1] + ")";
      posts.push(best);
    }
    // The cable leaves at the bottom, at the free x nearest the centre, at a depth below every part's rear.
    var cableZ = -(Math.min(face.requiredInnerDepth() + 4.0, face.caseInnerDepth - CABLE_HOLE / 2 - 1.0));
    var cableX = 0.0, found = false;
    for (step in 0...120) {
      var candidate = (step % 2 == 0 ? 1 : -1) * Math.floor((step + 1) / 2) * 1.5;
      if (Math.abs(candidate) > face.width / 2 - face.wall - 15) continue;
      if (cableClear(face, candidate, cableZ)) { cableX = candidate; found = true; break; }
    }
    if (!found) throw "No clear place for the cable hole";
    return new PendantCase(face, posts, cableX, cableZ);
  }

  static function free(face:PanelLayout, x:Float, y:Float, taken:Array<{x:Float, y:Float}>):Bool {
    for (c in face.controls) {
      if (FootprintGeometry.distanceTo(c.keepOut(), c.x, c.y, x, y) < COUNTERBORE_HEAD() + HEAD_CLEARANCE - 0.0) return false;
      if (FootprintGeometry.distanceTo(c.body(), c.x, c.y, x, y) < POST_DIAMETER / 2 + BODY_CLEARANCE) return false;
      for (h in c.holes()) if (Math.sqrt((c.x + h.x - x) * (c.x + h.x - x) + (c.y + h.y - y) * (c.y + h.y - y)) < POST_DIAMETER / 2 + 2.0) return false;
    }
    for (t in taken) if (Math.sqrt((t.x - x) * (t.x - x) + (t.y - y) * (t.y - y)) < POST_DIAMETER + 4.0) return false;
    return true;
  }

  static function COUNTERBORE_HEAD():Float return PanelFacePlate.COUNTERBORE / 2;

  /** The cable runs horizontally at depth z, so only parts that reach that deep and sit near the bottom wall can block it. */
  static function cableClear(face:PanelLayout, x:Float, z:Float):Bool {
    for (c in face.controls) {
      if (c.behind() < -z - CABLE_HOLE / 2) continue;
      if (FootprintGeometry.distanceTo(c.body(), c.x, c.y, x, -face.height / 2 + face.wall) < CABLE_HOLE / 2 + BODY_CLEARANCE) return false;
    }
    return true;
  }

  /** The posts and the cable hole as marks for PanelLayoutSvg: a dashed post and its counterbore, and the hole in the bottom wall. */
  public function marks():Array<SvgMark> {
    var result:Array<SvgMark> = [];
    for (post in posts) {
      result.push({x: post.x, y: post.y, size: POST_DIAMETER, height: 0.0, filled: false});
      result.push({x: post.x, y: post.y, size: PanelFacePlate.COUNTERBORE, height: 0.0, filled: false});
    }
    result.push({x: cableX, y: -face.height / 2 + face.wall / 2, size: CABLE_HOLE, height: face.wall, filled: true});
    return result;
  }

  public function outerDepth():Float return face.caseInnerDepth + BACK_WALL;

  /** The shell: a rounded box open at the top, with the posts and the cable hole. */
  public function shell():Part {
    var depth = outerDepth();
    var outer = RoundedRectangle.box(face.width, face.height, PanelFacePlate.CORNER_RADIUS, -depth, 0.0);
    var cavity = RoundedRectangle.box(face.width - 2 * face.wall, face.height - 2 * face.wall,
      PanelFacePlate.CORNER_RADIUS - face.wall, -face.caseInnerDepth, 1.0);
    var tools:Array<Part> = [cavity];
    var hole = Part.cylinderAlongY(CABLE_HOLE / 2, -face.height / 2 - 1.0, -face.height / 2 + face.wall + 1.0, cableX, cableZ);
    tools.push(hole);
    var body = outer.subtractAll(tools);
    outer.close();
    for (t in tools) t.close();
    var current = body;
    for (p in posts) {
      var post = Part.cylinderSpan(POST_DIAMETER / 2, -face.caseInnerDepth - 0.5, 0.0, p.x, p.y);
      var pilot = Part.cylinderSpan(PILOT_DIAMETER / 2, -PILOT_DEPTH, 1.0, p.x, p.y);
      var withPost = current.combine(post);
      var drilled = withPost.subtract(pilot);
      current.close(); post.close(); pilot.close(); withPost.close();
      current = drilled;
    }
    return current;
  }

  /** The face plate with the screw holes that match the posts. */
  public function plate():Part return PanelFacePlate.part(face, posts);
}
