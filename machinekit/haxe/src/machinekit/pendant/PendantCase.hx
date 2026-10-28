package machinekit.pendant;

import cadkit.modeling.Footprint;
import cadkit.modeling.FootprintGeometry;
import cadkit.modeling.Part;
import cadkit.modeling.RoundedRectangle;
import cadkit.modeling.Vector;
import machinekit.hmi.*;

/**
 * The shell behind the face plate, and what the plan puts in it. The plate lies on the shell's rim (z = 0 up, the
 * plate from 0 to its thickness) and M3 screws pass down through it into six posts that rise from the back wall.
 * Plan, in this order, each placed where nothing in front or behind needs the room:
 *
 * - posts: clear of every keep-out (a screw head must not land on a control) and every rear body;
 * - the M12 receptacle hole in the bottom wall, with room for its body;
 * - a board envelope behind the controls, with four standoffs. Parts that pass through the board's plane (the deep
 *   E-stop) get notches, as do the posts, the receptacle and the enabling switch;
 * - the enabling switch's blanked pocket in the back wall, and two strap slots;
 * - a perimeter tongue on the plate that fits inside the walls, interrupted wherever a rear body or a post is near the wall.
 *
 * `notes` lists what did not fit. The board is an envelope for fitting, not the board design.
 */
class PendantCase {
  public static inline var BACK_WALL:Float = 3.0;
  public static inline var POST_DIAMETER:Float = 8.0;
  public static inline var PILOT_DIAMETER:Float = 2.6;
  public static inline var PILOT_DEPTH:Float = 14.0;
  /** Room kept around a screw head and around a post. */
  public static inline var HEAD_CLEARANCE:Float = 3.0;
  public static inline var BODY_CLEARANCE:Float = 1.5;
  /** M12 panel receptacle: hole for an M16 rear-mount thread (assumed, check the part), body and length behind the wall. */
  public static inline var RECEPTACLE_HOLE:Float = 16.2;
  public static inline var RECEPTACLE_BODY:Float = 22.0;
  public static inline var RECEPTACLE_LENGTH:Float = 25.0;
  public static inline var TONGUE_HEIGHT:Float = 3.0;
  public static inline var TONGUE_WIDTH:Float = 1.6;
  public static inline var TONGUE_CLEARANCE:Float = 0.25;
  public static inline var BOARD_THICKNESS:Float = 1.6;
  /** Height of the board's components on the side away from the face (assumed). */
  public static inline var BOARD_COMPONENTS:Float = 10.0;
  public static inline var BOARD_MARGIN:Float = 2.0;
  /** Controls deeper than this pass through the board's plane instead of ending above it. */
  public static inline var PASS_THROUGH:Float = 30.0;
  public static inline var STANDOFF_DIAMETER:Float = 6.0;
  public static inline var STANDOFF_PILOT:Float = 2.2;
  public static inline var BOARD_HOLE:Float = 2.7;
  /** A 22.3 mm opening for a 3-position enabling switch whose body is about 26 mm across and 30 deep (assumed). */
  public static inline var ENABLING_HOLE:Float = 22.3;
  public static inline var ENABLING_BODY:Float = 26.0;
  public static inline var ENABLING_DEPTH:Float = 30.0;
  public static inline var ENABLING_SKIN:Float = 0.8;
  public static inline var STRAP_SLOT_WIDTH:Float = 24.0;
  public static inline var STRAP_SLOT_HEIGHT:Float = 4.0;

  public final face:PanelLayout;
  public final posts:Array<{x:Float, y:Float}>;
  public final connectorX:Float;
  public final connectorZ:Float;
  /** Top face of the board envelope, behind the shallow controls. */
  public final boardTop:Float;
  public final standoffs:Array<{x:Float, y:Float}>;
  public final strapSlots:Array<{x:Float, y:Float}>;
  /** The enabling switch's pocket, or null when no free column exists. */
  public final enabling:Null<{x:Float, y:Float}>;
  public final notes:Array<String>;

  function new(face:PanelLayout, posts:Array<{x:Float, y:Float}>, connectorX:Float, connectorZ:Float, boardTop:Float,
      standoffs:Array<{x:Float, y:Float}>, strapSlots:Array<{x:Float, y:Float}>, enabling:Null<{x:Float, y:Float}>, notes:Array<String>) {
    this.face = face; this.posts = posts; this.connectorX = connectorX; this.connectorZ = connectorZ; this.boardTop = boardTop;
    this.standoffs = standoffs; this.strapSlots = strapSlots; this.enabling = enabling; this.notes = notes;
  }

  public static function plan(face:PanelLayout):PendantCase {
    var notes:Array<String> = [];
    var posts = placePosts(face);
    // The receptacle leaves at the bottom, at the free x nearest the centre, low enough to clear the shallow parts.
    var connectorZ = -(face.caseInnerDepth - RECEPTACLE_BODY / 2 - 1.0);
    var wallY = -face.height / 2 + face.wall;
    var connectorX = 0.0, found = false;
    for (step in 0...160) {
      var candidate = (step % 2 == 0 ? 1 : -1) * Math.floor((step + 1) / 2) * 1.5;
      if (Math.abs(candidate) > face.width / 2 - face.wall - 15) continue;
      if (receptacleClear(face, candidate, connectorZ, wallY)) { connectorX = candidate; found = true; break; }
    }
    if (!found) throw "No clear place for the M12 receptacle";
    // The board sits just behind the deepest control that ends above it.
    var shallow = 0.0;
    for (c in face.controls) if (c.behind() <= PASS_THROUGH) shallow = Math.max(shallow, c.behind());
    var boardTop = -(shallow + 1.0);
    if (boardTop - BOARD_THICKNESS - BOARD_COMPONENTS < -(face.caseInnerDepth - 1.0))
      throw "The case is too shallow for a board behind the controls";
    var enabling = placeEnabling(face, posts, connectorX, wallY, notes);
    var standoffs = placeStandoffs(face, posts, connectorX, wallY, enabling);
    var strapSlots = placeStrapSlots(face, posts, standoffs, enabling);
    return new PendantCase(face, posts, connectorX, connectorZ, boardTop, standoffs, strapSlots, enabling, notes);
  }

  static function placePosts(face:PanelLayout):Array<{x:Float, y:Float}> {
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
          if (d < bestDistance && postFree(face, x, y, posts)) { best = {x: x, y: y}; bestDistance = d; }
          y += 1.5;
        }
        x += 1.5;
      }
      if (best == null) throw "No room for a screw post near (" + target[0] + ", " + target[1] + ")";
      posts.push(best);
    }
    return posts;
  }

  static function postFree(face:PanelLayout, x:Float, y:Float, taken:Array<{x:Float, y:Float}>):Bool {
    for (c in face.controls) {
      if (FootprintGeometry.distanceTo(c.keepOut(), c.x, c.y, x, y) < PanelFacePlate.COUNTERBORE / 2 + HEAD_CLEARANCE) return false;
      if (FootprintGeometry.distanceTo(c.body(), c.x, c.y, x, y) < POST_DIAMETER / 2 + BODY_CLEARANCE) return false;
      for (h in c.holes()) if (dist(c.x + h.x, c.y + h.y, x, y) < POST_DIAMETER / 2 + 2.0) return false;
    }
    for (t in taken) if (dist(t.x, t.y, x, y) < POST_DIAMETER + 4.0) return false;
    return true;
  }

  static function receptacleFootprint():Footprint return Rect(RECEPTACLE_BODY + 4.0, RECEPTACLE_LENGTH + 2.0);
  static function receptacleCentreY(face:PanelLayout, wallY:Float):Float return wallY + RECEPTACLE_LENGTH / 2;

  /** The receptacle runs horizontally at depth z: only parts reaching below its top, near the bottom wall, can block it. */
  static function receptacleClear(face:PanelLayout, x:Float, z:Float, wallY:Float):Bool {
    for (c in face.controls) {
      if (c.behind() <= -(z + RECEPTACLE_BODY / 2) - 2.0) continue;
      if (FootprintGeometry.gap(c.body(), c.x, c.y, receptacleFootprint(), x, receptacleCentreY(face, wallY)) < BODY_CLEARANCE) return false;
    }
    return true;
  }

  /** The enabling switch's body needs a free column from the back wall up ENABLING_DEPTH: left half first, then right. */
  static function placeEnabling(face:PanelLayout, posts:Array<{x:Float, y:Float}>, connectorX:Float, wallY:Float,
      notes:Array<String>):Null<{x:Float, y:Float}> {
    var limit = face.caseInnerDepth - ENABLING_DEPTH - 1.0;
    var hw = face.width / 2 - face.wall - ENABLING_BODY / 2 - 2.0, hh = face.height / 2 - face.wall - ENABLING_BODY / 2 - 2.0;
    for (side in [-1.0, 1.0]) {
      var best:Null<{x:Float, y:Float}> = null, bestDistance = 1e9;
      var x = side * 10.0;
      while (Math.abs(x) <= hw) {
        var y = -hh;
        while (y <= hh + 1e-9) {
          if (enablingFree(face, x, y, limit, posts, connectorX, wallY) && Math.abs(x) < bestDistance) { best = {x: x, y: y}; bestDistance = Math.abs(x); }
          y += 3.0;
        }
        x += side * 3.0;
      }
      if (best != null) {
        if (side > 0) notes.push("The enabling switch has no free 26 mm column on the left; it fits at x=" + Math.round(best.x) + " mm, behind the wheel, under the wheel hand.");
        return best;
      }
    }
    notes.push("No free column for the enabling switch anywhere behind the face.");
    return null;
  }

  static function enablingFree(face:PanelLayout, x:Float, y:Float, limit:Float, posts:Array<{x:Float, y:Float}>, connectorX:Float, wallY:Float):Bool {
    for (c in face.controls) {
      if (c.behind() <= limit) continue;
      if (FootprintGeometry.gap(c.body(), c.x, c.y, Circle(ENABLING_BODY), x, y) < BODY_CLEARANCE) return false;
    }
    for (p in posts) if (dist(p.x, p.y, x, y) < ENABLING_BODY / 2 + POST_DIAMETER / 2 + 2.0) return false;
    if (FootprintGeometry.gap(receptacleFootprint(), connectorX, receptacleCentreY(face, wallY), Circle(ENABLING_BODY), x, y) < 2.0) return false;
    return true;
  }

  /** The board outline's half extents. */
  static function boardHalf(face:PanelLayout):{x:Float, y:Float} return {x: face.width / 2 - face.wall - BOARD_MARGIN, y: face.height / 2 - face.wall - BOARD_MARGIN};

  /** Everything the board must leave room for, as footprints at positions (plan view). */
  function notches():Array<{f:Footprint, x:Float, y:Float}> {
    var result:Array<{f:Footprint, x:Float, y:Float}> = [];
    for (c in face.controls) if (c.behind() > PASS_THROUGH) result.push({f: FootprintGeometry.grow(c.body(), 2.0), x: c.x, y: c.y});
    for (p in posts) result.push({f: Circle(POST_DIAMETER + 2.0), x: p.x, y: p.y});
    result.push({f: receptacleFootprint(), x: connectorX, y: receptacleCentreY(face, -face.height / 2 + face.wall)});
    if (enabling != null) result.push({f: Circle(ENABLING_BODY + 2.0), x: enabling.x, y: enabling.y});
    return result;
  }

  static function placeStandoffs(face:PanelLayout, posts:Array<{x:Float, y:Float}>, connectorX:Float, wallY:Float,
      enabling:Null<{x:Float, y:Float}>):Array<{x:Float, y:Float}> {
    var half = boardHalf(face);
    var wanted = [[-half.x + 6, half.y - 6], [half.x - 6, half.y - 6], [-half.x + 6, -half.y + 6], [half.x - 6, -half.y + 6]];
    var result:Array<{x:Float, y:Float}> = [];
    for (target in wanted) {
      var best:Null<{x:Float, y:Float}> = null, bestDistance = 1e9;
      var x = -half.x + 5.0;
      while (x <= half.x - 5.0 + 1e-9) {
        var y = -half.y + 5.0;
        while (y <= half.y - 5.0 + 1e-9) {
          var d = dist(x, y, target[0], target[1]);
          if (d < bestDistance && standoffFree(face, x, y, posts, connectorX, wallY, enabling, result)) { best = {x: x, y: y}; bestDistance = d; }
          y += 1.5;
        }
        x += 1.5;
      }
      if (best == null) throw "No room for a board standoff near (" + target[0] + ", " + target[1] + ")";
      result.push(best);
    }
    return result;
  }

  static function standoffFree(face:PanelLayout, x:Float, y:Float, posts:Array<{x:Float, y:Float}>, connectorX:Float, wallY:Float,
      enabling:Null<{x:Float, y:Float}>, taken:Array<{x:Float, y:Float}>):Bool {
    var need = STANDOFF_DIAMETER / 2 + 3.0;
    for (c in face.controls) if (c.behind() > PASS_THROUGH && FootprintGeometry.distanceTo(c.body(), c.x, c.y, x, y) < need) return false;
    for (p in posts) if (dist(p.x, p.y, x, y) < STANDOFF_DIAMETER / 2 + POST_DIAMETER / 2 + 3.0) return false;
    if (FootprintGeometry.distanceTo(receptacleFootprint(), connectorX, receptacleCentreY(face, wallY), x, y) < need) return false;
    if (enabling != null && dist(enabling.x, enabling.y, x, y) < ENABLING_BODY / 2 + need) return false;
    for (t in taken) if (dist(t.x, t.y, x, y) < 20.0) return false;
    return true;
  }

  static function placeStrapSlots(face:PanelLayout, posts:Array<{x:Float, y:Float}>, standoffs:Array<{x:Float, y:Float}>,
      enabling:Null<{x:Float, y:Float}>):Array<{x:Float, y:Float}> {
    var result:Array<{x:Float, y:Float}> = [];
    var halfX = face.width / 2 - face.wall - STRAP_SLOT_WIDTH / 2 - 8.0, halfY = face.height / 2 - face.wall - 10.0;
    for (target in [[-45.0, 30.0], [-45.0, -30.0]]) {
      var best:Null<{x:Float, y:Float}> = null, bestDistance = 1e9;
      var x = -halfX;
      while (x <= halfX + 1e-9) {
        var y = -halfY;
        while (y <= halfY + 1e-9) {
          var d = dist(x, y, target[0], target[1]);
          if (d < bestDistance && slotFree(face, x, y, posts, standoffs, enabling, result)) { best = {x: x, y: y}; bestDistance = d; }
          y += 1.5;
        }
        x += 1.5;
      }
      if (best == null) throw "No room for a strap slot near (" + target[0] + ", " + target[1] + ")";
      result.push(best);
    }
    return result;
  }

  static function slotFree(face:PanelLayout, x:Float, y:Float, posts:Array<{x:Float, y:Float}>, standoffs:Array<{x:Float, y:Float}>,
      enabling:Null<{x:Float, y:Float}>, taken:Array<{x:Float, y:Float}>):Bool {
    var slot = Rect(STRAP_SLOT_WIDTH, STRAP_SLOT_HEIGHT);
    // A strap passes behind the parts: keep clear of any that leave less than 8 mm to the back wall.
    for (c in face.controls) if (c.behind() > face.caseInnerDepth - 8.0 && FootprintGeometry.gap(c.body(), c.x, c.y, slot, x, y) < BODY_CLEARANCE) return false;
    for (p in posts) if (FootprintGeometry.distanceTo(slot, x, y, p.x, p.y) < POST_DIAMETER / 2 + 3.0) return false;
    for (s in standoffs) if (FootprintGeometry.distanceTo(slot, x, y, s.x, s.y) < STANDOFF_DIAMETER / 2 + 3.0) return false;
    if (enabling != null && FootprintGeometry.distanceTo(slot, x, y, enabling.x, enabling.y) < ENABLING_BODY / 2 + 3.0) return false;
    for (t in taken) if (dist(t.x, t.y, x, y) < 20.0) return false;
    return true;
  }

  static function dist(ax:Float, ay:Float, bx:Float, by:Float):Float return Math.sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));

  /** The posts, standoffs, receptacle, enabling pocket and strap slots as marks for PanelLayoutSvg. */
  public function marks():Array<SvgMark> {
    var result:Array<SvgMark> = [];
    for (post in posts) {
      result.push({x: post.x, y: post.y, size: POST_DIAMETER, height: 0.0, filled: false});
      result.push({x: post.x, y: post.y, size: PanelFacePlate.COUNTERBORE, height: 0.0, filled: false});
    }
    for (s in standoffs) result.push({x: s.x, y: s.y, size: STANDOFF_DIAMETER, height: 0.0, filled: true});
    for (s in strapSlots) result.push({x: s.x, y: s.y, size: STRAP_SLOT_WIDTH, height: STRAP_SLOT_HEIGHT, filled: true});
    if (enabling != null) result.push({x: enabling.x, y: enabling.y, size: ENABLING_HOLE, height: 0.0, filled: false});
    result.push({x: connectorX, y: -face.height / 2 + face.wall / 2, size: RECEPTACLE_HOLE, height: face.wall, filled: true});
    return result;
  }

  public function outerDepth():Float return face.caseInnerDepth + BACK_WALL;

  /** The shell: a rounded box open at the top, with posts, standoffs, the receptacle hole, strap slots and the enabling pocket. */
  public function shell():Part {
    var depth = outerDepth();
    var outer = RoundedRectangle.box(face.width, face.height, PanelFacePlate.CORNER_RADIUS, -depth, 0.0);
    var cavity = RoundedRectangle.box(face.width - 2 * face.wall, face.height - 2 * face.wall,
      PanelFacePlate.CORNER_RADIUS - face.wall, -face.caseInnerDepth, 1.0);
    var tools:Array<Part> = [cavity];
    tools.push(Part.cylinderAlongY(RECEPTACLE_HOLE / 2, -face.height / 2 - 1.0, -face.height / 2 + face.wall + 1.0, connectorX, connectorZ));
    for (s in strapSlots) tools.push(FootprintGeometry.solid(Rect(STRAP_SLOT_WIDTH, STRAP_SLOT_HEIGHT), s.x, s.y, -depth - 1.0, -face.caseInnerDepth + 1.0));
    if (enabling != null) tools.push(Part.cylinderSpan(ENABLING_HOLE / 2, -depth + ENABLING_SKIN, -face.caseInnerDepth + 0.5, enabling.x, enabling.y));
    var current = outer.subtractAll(tools);
    outer.close();
    for (t in tools) t.close();
    var underside = boardTop - BOARD_THICKNESS;
    for (p in posts) {
      var post = Part.cylinderSpan(POST_DIAMETER / 2, -face.caseInnerDepth - 0.5, 0.0, p.x, p.y);
      var pilot = Part.cylinderSpan(PILOT_DIAMETER / 2, -PILOT_DEPTH, 1.0, p.x, p.y);
      current = fuseThenDrill(current, post, pilot);
    }
    for (s in standoffs) {
      var standoff = Part.cylinderSpan(STANDOFF_DIAMETER / 2, -face.caseInnerDepth - 0.5, underside, s.x, s.y);
      var pilot = Part.cylinderSpan(STANDOFF_PILOT / 2, underside - 8.0, underside + 1.0, s.x, s.y);
      current = fuseThenDrill(current, standoff, pilot);
    }
    return current;
  }

  static function fuseThenDrill(body:Part, add:Part, hole:Part):Part {
    var joined = body.combine(add);
    var drilled = joined.subtract(hole);
    body.close(); add.close(); hole.close(); joined.close();
    return drilled;
  }

  /** The board envelope: an outline the size of the case's inside less a margin, with every notch and the four mounting holes. */
  public function board():Part {
    var half = boardHalf(face);
    var base = RoundedRectangle.box(half.x * 2, half.y * 2, 3.0, boardTop - BOARD_THICKNESS, boardTop);
    var tools:Array<Part> = [];
    for (n in notches()) tools.push(FootprintGeometry.solid(n.f, n.x, n.y, boardTop - BOARD_THICKNESS - 1.0, boardTop + 1.0));
    for (s in standoffs) tools.push(Part.cylinderSpan(BOARD_HOLE / 2, boardTop - BOARD_THICKNESS - 1.0, boardTop + 1.0, s.x, s.y));
    var result = base.subtractAll(tools);
    base.close();
    for (t in tools) t.close();
    return result;
  }

  /** The face plate with the screw holes that match the posts and a perimeter tongue that fits inside the walls. */
  public function plate():Part {
    var plain = PanelFacePlate.part(face, posts);
    var tongue = perimeterTongue();
    var joined = plain.combine(tongue);
    plain.close(); tongue.close();
    return joined;
  }

  /** A ring just inside the walls, cut away around every rear body and post so it never meets a part. */
  function perimeterTongue():Part {
    var inset = face.wall + TONGUE_CLEARANCE;
    var radius = Math.max(PanelFacePlate.CORNER_RADIUS - inset, 1.0);
    var outer = RoundedRectangle.box(face.width - 2 * inset, face.height - 2 * inset, radius, -TONGUE_HEIGHT, 0.0);
    var inner = RoundedRectangle.box(face.width - 2 * (inset + TONGUE_WIDTH), face.height - 2 * (inset + TONGUE_WIDTH),
      Math.max(radius - TONGUE_WIDTH, 0.5), -TONGUE_HEIGHT - 1.0, 1.0);
    var tools:Array<Part> = [inner];
    for (c in face.controls) tools.push(FootprintGeometry.solid(FootprintGeometry.grow(c.body(), 1.0), c.x, c.y, -TONGUE_HEIGHT - 1.0, 1.0));
    for (p in posts) tools.push(FootprintGeometry.solid(Circle(POST_DIAMETER + 2.0), p.x, p.y, -TONGUE_HEIGHT - 1.0, 1.0));
    var ring = outer.subtractAll(tools);
    outer.close();
    for (t in tools) t.close();
    return ring;
  }
}
