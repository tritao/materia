package machinekit.pendant;

import cadkit.modeling.Footprint;
import cadkit.modeling.FootprintGeometry;
import cadkit.modeling.Part;
import cadkit.modeling.RoundedRectangle;
import cadkit.modeling.Vector;
import machinekit.component.PanelBoss;
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
 * - an opening in a side wall for the enabling switch, which lies against the back wall, and two strap slots;
 * - a perimeter tongue on the plate that fits inside the walls, interrupted wherever a rear body or a post is near the wall.
 *
 * `notes` lists what did not fit. The board is an envelope for fitting, not the board design.
 */
class PendantCase {
  public static inline var BACK_WALL:Float = 3.0;
  public static inline var POST_DIAMETER:Float = 9.0;

  /** Hole for an M3 heat-set insert (about 4.6 mm outer, 5.7 mm long; check the insert's datasheet). */
  public static inline var PILOT_DIAMETER:Float = 4.2;

  public static inline var PILOT_DEPTH:Float = 7.5;

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

  /** A tower that does not touch a wall is tied to the nearest one by a rib this thick, up to just below the board. */
  public static inline var RIB_THICKNESS:Float = 1.6;

  /** How far a post's circle runs into the wall it is placed against, so the two print as one body. */
  static inline var POST_EMBED:Float = 0.75;

  /** The longest rib worth printing; a tower further from every wall is tied to a nearby post instead. */
  static inline var MAX_RIB:Float = 16.0;

  /** Ribs and the elements they join overlap by this much, which makes the union one solid. */
  static inline var RIB_OVERLAP:Float = 0.2;

  /** A strip of board narrower than this beside a notch is not worth keeping. */
  static inline var SLIVER:Float = 8.0;

  /** Controls deeper than this pass through the board's plane instead of ending above it. */
  public static inline var PASS_THROUGH:Float = 30.0;

  public static inline var STANDOFF_DIAMETER:Float = 7.0;

  /** Hole for an M2.5 heat-set insert (about 3.6 mm outer, 4 mm long). */
  public static inline var STANDOFF_PILOT:Float = 3.4;

  public static inline var STANDOFF_PILOT_DEPTH:Float = 4.4;
  /** The bosses on the plate's back that the board's standoffs screw into. */
  public static inline var BOSS_HEIGHT:Float = 4.0;
  public static inline var BOARD_HOLE:Float = 2.7;

  /** A 22.3 mm opening in a side wall for a 3-position enabling switch whose body is about 26 mm across and 30 long, pointing inwards (assumed). */
  public static inline var ENABLING_HOLE:Float = 22.3;

  public static inline var ENABLING_BODY:Float = 26.0;
  public static inline var ENABLING_DEPTH:Float = 30.0;
  public static inline var STRAP_SLOT_WIDTH:Float = 24.0;
  public static inline var STRAP_SLOT_HEIGHT:Float = 4.0;

  public final face:PanelLayout;
  public final posts:Array<{x:Float, y:Float}>;
  public final connectorX:Float;
  public final connectorZ:Float;

  /** Top face of the board envelope, behind the shallow controls. */
  public final boardTop:Float;

  public final standoffs:Array<{x:Float, y:Float}>;

  /** The standard length of the standoffs that hold the board to the plate's bosses. */
  public final standoffLength:Float;
  public final strapSlots:Array<{x:Float, y:Float}>;

  /** The enabling switch's body centre in plan, lying against a side wall at the back, or null when no free place exists. */
  public final enabling:Null<{x:Float, y:Float}>;

  /** Plan rectangles (centre and size) of the ribs that tie free-standing posts and standoffs to a wall. */
  public final ribs:Array<{x:Float, y:Float, w:Float, h:Float}>;

  public final notes:Array<String>;

  function new(
    face:PanelLayout,
    posts:Array<{x:Float, y:Float}>,
    connectorX:Float,
    connectorZ:Float,
    boardTop:Float,
    standoffs:Array<{x:Float, y:Float}>,
    standoffLength:Float,
    strapSlots:Array<{x:Float, y:Float}>,
    enabling:Null<{x:Float, y:Float}>,
    ribs:Array<{x:Float, y:Float, w:Float, h:Float}>,
    notes:Array<String>
  ) {
    this.face = face;
    this.posts = posts;
    this.connectorX = connectorX;
    this.connectorZ = connectorZ;
    this.boardTop = boardTop;
    this.standoffs = standoffs;
    this.standoffLength = standoffLength;
    this.strapSlots = strapSlots;
    this.enabling = enabling;
    this.ribs = ribs;
    this.notes = notes;
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
      if (receptacleClear(face, candidate, connectorZ, wallY)) {
        connectorX = candidate;
        found = true;
        break;
      }
    }
    if (!found) throw "No clear place for the M12 receptacle";
    // The board sits just behind the deepest control that ends above it.
    var shallow = 0.0;
    for (c in face.controls) if (c.behind() <= PASS_THROUGH) shallow = Math.max(shallow, c.behind());
    // The board hangs from the plate on standoffs of a standard length from the bosses on the plate's back, below the shallow parts.
    var standoffLength = HexStandoff.standardLength(shallow + 1.0 - BOSS_HEIGHT);
    var boardTop = -(BOSS_HEIGHT + standoffLength);
    if (boardTop - BOARD_THICKNESS - BOARD_COMPONENTS
      < -(face.caseInnerDepth - 1.0)) throw "The case is too shallow for a board behind the controls";
    var enabling = placeEnabling(face, posts, connectorX, wallY, notes);
    var standoffs = placeStandoffs(face, posts, connectorX, wallY, enabling);
    var ribs = placeRibs(face, posts, [], connectorX, wallY, enabling, boardTop);
    var strapSlots = placeStrapSlots(face, posts, [], enabling, ribs);
    return new PendantCase(
      face,
      posts,
      connectorX,
      connectorZ,
      boardTop,
      standoffs,
      standoffLength,
      strapSlots,
      enabling,
      ribs,
      notes
    );
  }

  static function placePosts(face:PanelLayout):Array<{x:Float, y:Float}> {
    var hw = face.width / 2 - face.wall - POST_DIAMETER / 2 + POST_EMBED,
      hh = face.height / 2 - face.wall - POST_DIAMETER / 2 + POST_EMBED;
    var wanted = [[-hw, hh], [hw, hh], [-hw, 0.0], [hw, 0.0], [-hw, -hh], [hw, -hh]];
    var posts:Array<{x:Float, y:Float}> = [];
    for (target in wanted) {
      var best:Null<{x:Float, y:Float}> = null, bestDistance = 1e9;
      var x = -hw;
      while (x <= hw + 1e-9) {
        var y = -hh;
        while (y <= hh + 1e-9) {
          var d = Math.sqrt((x - target[0]) * (x - target[0]) + (y - target[1]) * (y - target[1]));
          if (d < bestDistance && postFree(face, x, y, posts)) {
            best = {x: x, y: y};
            bestDistance = d;
          }
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
      if (FootprintGeometry.distanceTo(c.keepOut(),
        c.x, c.y, x, y) < PanelFacePlate.COUNTERBORE / 2 + HEAD_CLEARANCE) return false;
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
      if (FootprintGeometry.gap(c.body(),
        c.x, c.y, receptacleFootprint(), x, receptacleCentreY(face, wallY)) < BODY_CLEARANCE) return false;
    }
    return true;
  }

  static function enablingFootprint():Footprint return Rect(ENABLING_DEPTH, ENABLING_BODY);

  /** The switch's axis sits this far behind the face: against the back wall, where only the deepest parts can reach. */
  public static function enablingZ(face:PanelLayout):Float return -(face.caseInnerDepth - ENABLING_BODY / 2 - 1.0);

  /** The enabling switch lies against a side wall at the back, its button through the wall: left wall first, then right. */
  static function placeEnabling(
    face:PanelLayout,
    posts:Array<{x:Float, y:Float}>,
    connectorX:Float,
    wallY:Float,
    notes:Array<String>
  ):Null<{
    x:Float,
    y:Float
  }> {
    var limit = face.caseInnerDepth - ENABLING_BODY - 1.0;
    var hh = face.height / 2 - face.wall - ENABLING_BODY / 2 - 2.0;
    for (side in [-1.0, 1.0]) {
      var x = side * (face.width / 2 - face.wall - ENABLING_DEPTH / 2);
      var best:Null<{x:Float, y:Float}> = null;
      var y = -hh;
      while (y <= hh + 1e-9) {
        if (enablingFree(face,
          x, y, limit, posts, connectorX, wallY) && (best == null || Math.abs(y) < Math.abs(best.y))) best = {
            x: x,
            y: y
          };
        y += 3.0;
      }
      if (best != null) {
        if (side
          > 0) notes.push("The enabling switch has no free place on the left wall; it fits on the right wall at y="
          + Math.round(best.y) + " mm.");
        return best;
      }
    }
    notes.push("No free place for the enabling switch on either side wall.");
    return null;
  }

  static function enablingFree(
    face:PanelLayout,
    x:Float,
    y:Float,
    limit:Float,
    posts:Array<{x:Float, y:Float}>,
    connectorX:Float,
    wallY:Float
  ):Bool {
    for (c in face.controls) {
      if (c.behind() <= limit) continue;
      if (FootprintGeometry.gap(c.body(), c.x, c.y, enablingFootprint(), x, y) < BODY_CLEARANCE) return false;
    }
    for (p in posts) if (FootprintGeometry.distanceTo(enablingFootprint(),
      x, y, p.x, p.y) < POST_DIAMETER / 2 + 2.0) return false;
    if (FootprintGeometry.gap(
      receptacleFootprint(),
      connectorX,
      receptacleCentreY(face, wallY),
      enablingFootprint(),
      x,
      y
    ) < 2.0) return false;
    return true;
  }

  /** The board outline's half extents. */
  public static function boardHalf(face:PanelLayout): {x: Float, y: Float} return {
    x: face.width / 2 - face.wall - BOARD_MARGIN,
    y: face.height / 2 - face.wall - BOARD_MARGIN
  };

  /** Everything the board must leave room for, as footprints at positions (plan view). */
  public function notches():Array<{f:Footprint, x:Float, y:Float}> {
    var raw:Array<{f:Footprint, x:Float, y:Float}> = [];
    for (c in face.controls) if (c.behind() > PASS_THROUGH) raw.push({
      f: FootprintGeometry.grow(c.body(), 2.0),
      x: c.x,
      y: c.y
    });
    for (p in posts) raw.push({f: Circle(POST_DIAMETER + 2.0), x: p.x, y: p.y});
    raw.push({f: receptacleFootprint(), x: connectorX, y: receptacleCentreY(face, -face.height / 2 + face.wall)});
    if (enabling != null) raw.push({f: FootprintGeometry.grow(enablingFootprint(), 2.0), x: enabling.x, y: enabling.y});
    // Strips of board narrower than SLIVER, between a notch and the outline or between two notches, are not worth keeping.
    var result = [for (n in raw) reachEdges(n.f, n.x, n.y)];
    var merged = true;
    while (merged) {
      merged = false;
      for (i in 0...result.length) {
        for (j in i + 1...result.length) {
          if (merged) break;
          if (FootprintGeometry.gap(result[i].f,
            result[i].x, result[i].y, result[j].f, result[j].x, result[j].y) >= SLIVER) continue;
          result[i] = union(result[i], result[j]);
          result.splice(j, 1);
          merged = true;
        }
        if (merged) break;
      }
    }
    return result;
  }

  static function union(a: {f: Footprint, x: Float, y: Float}, b: {f: Footprint, x: Float, y: Float}): {
    f: Footprint,
    x: Float,
    y: Float
  } {
    var ea = FootprintGeometry.extent(a.f, a.x, a.y), eb = FootprintGeometry.extent(b.f, b.x, b.y);
    var x0 = Math.min(ea.x0, eb.x0), x1 = Math.max(ea.x1, eb.x1);
    var y0 = Math.min(ea.y0, eb.y0), y1 = Math.max(ea.y1, eb.y1);
    return {f: Rect(x1 - x0, y1 - y0), x: (x0 + x1) / 2, y: (y0 + y1) / 2};
  }

  /** A notch that would leave a sliver of board between itself and the outline is widened to the outline. */
  function reachEdges(f:Footprint, x:Float, y:Float): {f: Footprint, x: Float, y: Float} {
    var board = boardHalf(face), box = FootprintGeometry.extent(f, x, y);
    var x0 = box.x0, x1 = box.x1, y0 = box.y0, y1 = box.y1;
    var changed = false;
    if (board.x - x1 < SLIVER) {
      x1 = board.x + 1.0;
      changed = true;
    }
    if (x0 + board.x < SLIVER) {
      x0 = -board.x - 1.0;
      changed = true;
    }
    if (board.y - y1 < SLIVER) {
      y1 = board.y + 1.0;
      changed = true;
    }
    if (y0 + board.y < SLIVER) {
      y0 = -board.y - 1.0;
      changed = true;
    }
    return changed ? {
      f:Rect(x1 - x0, y1 - y0), x:(x0 + x1) / 2, y:(y0 + y1) / 2
    }
    : {f: f, x: x, y: y};
  }

  static function placeStandoffs(
    face:PanelLayout,
    posts:Array<{x:Float, y:Float}>,
    connectorX:Float,
    wallY:Float,
    enabling:Null<{x:Float, y:Float}>
  ):Array<{
    x:Float,
    y:Float
  }> {
    var half = boardHalf(face);
    var wanted = [
      [-half.x + 6, half.y - 6],
      [half.x - 6, half.y - 6],
      [-half.x + 6, -half.y + 6],
      [half.x - 6, -half.y + 6]
    ];
    var result:Array<{x:Float, y:Float}> = [];
    for (target in wanted) {
      var best:Null<{x:Float, y:Float}> = null, bestDistance = 1e9;
      var x = -half.x + 5.0;
      while (x <= half.x - 5.0 + 1e-9) {
        var y = -half.y + 5.0;
        while (y <= half.y - 5.0 + 1e-9) {
          var d = dist(x, y, target[0], target[1]);
          if (d < bestDistance && standoffFree(face, x, y, posts, connectorX, wallY, enabling, result)) {
            best = {x: x, y: y};
            bestDistance = d;
          }
          y += 1.5;
        }
        x += 1.5;
      }
      if (best == null) throw "No room for a board standoff near (" + target[0] + ", " + target[1] + ")";
      result.push(best);
    }
    return result;
  }

  static function standoffFree(
    face:PanelLayout,
    x:Float,
    y:Float,
    posts:Array<{x:Float, y:Float}>,
    connectorX:Float,
    wallY:Float,
    enabling:Null<{x:Float, y:Float}>,
    taken:Array<{x:Float, y:Float}>
  ):Bool {
    var need = STANDOFF_DIAMETER / 2 + 3.0;
    // The standoffs run from the plate down past the shallow parts, so they clear every control's body.
    for (c in face.controls) if (FootprintGeometry.distanceTo(c.body(), c.x, c.y, x, y) < need) return false;
    for (p in posts) if (dist(p.x, p.y, x, y) < STANDOFF_DIAMETER / 2 + POST_DIAMETER / 2 + 3.0) return false;
    if (FootprintGeometry.distanceTo(receptacleFootprint(),
      connectorX, receptacleCentreY(face, wallY), x, y) < need) return false;
    if (enabling != null && FootprintGeometry.distanceTo(enablingFootprint(),
      enabling.x, enabling.y, x, y) < need) return false;
    for (t in taken) if (dist(t.x, t.y, x, y) < 20.0) return false;
    return true;
  }

  /** The top of the ribs: just under the board's underside. */
  public static function ribTop(boardTop:Float):Float return boardTop - BOARD_THICKNESS - 1.0;

  /**
	 * Every post and standoff must be part of the surrounding structure, not a tower standing on the back wall alone: one
	 * that touches a wall is already joined to it, any other gets a thin rib to the nearest wall that is clear of the
	 * deep parts, the receptacle, the enabling switch and the other towers.
	 */
  static function placeRibs(
    face:PanelLayout,
    posts:Array<{x:Float, y:Float}>,
    standoffs:Array<{x:Float, y:Float}>,
    connectorX:Float,
    wallY:Float,
    enabling:Null<{x:Float, y:Float}>,
    boardTop:Float
  ):Array<{
    x:Float,
    y:Float,
    w:Float,
    h:Float
  }> {
    var result:Array<{x:Float, y:Float, w:Float, h:Float}> = [];
    var towers:Array<{x:Float, y:Float, r:Float}> = [];
    for (p in posts) towers.push({x: p.x, y: p.y, r: POST_DIAMETER / 2});
    for (s in standoffs) towers.push({x: s.x, y: s.y, r: STANDOFF_DIAMETER / 2});
    var halfX = face.width / 2 - face.wall, halfY = face.height / 2 - face.wall;
    for (tower in towers) {
      // Distance from the tower's edge to each wall's inner face: left, right, bottom, top.
      var gaps = [
        tower.x + halfX - tower.r,
        halfX - tower.x - tower.r,
        tower.y + halfY - tower.r,
        halfY - tower.y - tower.r
      ];
      var smallest = Math.min(Math.min(gaps[0], gaps[1]), Math.min(gaps[2], gaps[3]));
      if (smallest <= 0.0) continue;
      var order = [0, 1, 2, 3];
      order.sort((a, b) -> gaps[a] < gaps[b] ? -1 : (gaps[a] > gaps[b] ? 1 : 0));
      var placed = false;
      for (direction in order) {
        if (gaps[direction] > MAX_RIB) continue;
        var length = gaps[direction] + 2 * RIB_OVERLAP;
        var rib = switch direction {
          case 0:
            {x: -halfX - RIB_OVERLAP + length / 2, y: tower.y, w: length, h: RIB_THICKNESS};
          case 1:
            {x: halfX + RIB_OVERLAP - length / 2, y: tower.y, w: length, h: RIB_THICKNESS};
          case 2:
            {x: tower.x, y: -halfY - RIB_OVERLAP + length / 2, w: RIB_THICKNESS, h: length};
          default:
            {x: tower.x, y: halfY + RIB_OVERLAP - length / 2, w: RIB_THICKNESS, h: length};
        };
        if (ribFree(face, rib, tower, towers, connectorX, wallY, enabling, boardTop)) {
          result.push(rib);
          placed = true;
          break;
        }
      }
      // A standoff far from every wall, or with the way blocked, leans on a nearby post that is in line with it.
      if (!placed) for (p in posts) {
        if (p.x == tower.x && p.y == tower.y) continue;
        var dx = p.x - tower.x, dy = p.y - tower.y;
        var horizontal = Math.abs(dy) <= 2.0 && Math.abs(dx) <= MAX_RIB + POST_DIAMETER;
        var vertical = Math.abs(dx) <= 2.0 && Math.abs(dy) <= MAX_RIB + POST_DIAMETER;
        if (!horizontal && !vertical) continue;
        var span = (horizontal ? Math.abs(dx) : Math.abs(dy)) - tower.r - POST_DIAMETER / 2;
        if (span > MAX_RIB) continue;
        var length = span + 2 * RIB_OVERLAP;
        var sign = horizontal ? (dx > 0 ? 1.0 : -1.0) : (dy > 0 ? 1.0 : -1.0);
        var reach = tower.r - RIB_OVERLAP + length / 2;
        var rib = horizontal ? {
          x:tower.x + sign * reach, y:tower.y, w:length, h:RIB_THICKNESS
        }
        : {x: tower.x, y: tower.y + sign * reach, w: RIB_THICKNESS, h: length};
        var others = [for (t in towers) if (!(t.x == p.x && t.y == p.y)) t];
        if (ribFree(face, rib, tower, others, connectorX, wallY, enabling, boardTop)) {
          result.push(rib);
          placed = true;
          break;
        }
      }
      if (!placed) throw "No clear rib ties the tower at (" + tower.x + ", " + tower.y + ") to a wall";
    }
    return result;
  }

  static function ribFree(
    face:PanelLayout,
    rib: {x: Float, y: Float, w: Float, h: Float},
    own: {x: Float, y: Float, r: Float},
    towers:Array<{x:Float, y:Float, r:Float}>,
    connectorX:Float,
    wallY:Float,
    enabling:Null<{x:Float, y:Float}>,
    boardTop:Float
  ):Bool {
    var footprint = Rect(rib.w, rib.h);
    var top = -ribTop(boardTop);
    for (c in face.controls) if (c.behind() > top && FootprintGeometry.gap(c.body(),
      c.x, c.y, footprint, rib.x, rib.y) < BODY_CLEARANCE) return false;
    if (FootprintGeometry.gap(
      receptacleFootprint(),
      connectorX,
      receptacleCentreY(face, wallY),
      footprint,
      rib.x,
      rib.y
    ) < 1.0) return false;
    if (enabling != null && FootprintGeometry.gap(enablingFootprint(),
      enabling.x, enabling.y, footprint, rib.x, rib.y) < 1.0) return false;
    for (t in towers) if (t != own && FootprintGeometry.distanceTo(footprint,
      rib.x, rib.y, t.x, t.y) < t.r + 0.5) return false;
    return true;
  }

  static function placeStrapSlots(
    face:PanelLayout,
    posts:Array<{x:Float, y:Float}>,
    standoffs:Array<{x:Float, y:Float}>,
    enabling:Null<{x:Float, y:Float}>,
    ribs:Array<{x:Float, y:Float, w:Float, h:Float}>
  ):Array<{
    x:Float,
    y:Float
  }> {
    var result:Array<{x:Float, y:Float}> = [];
    var halfX = face.width / 2 - face.wall - STRAP_SLOT_WIDTH / 2 - 8.0, halfY = face.height / 2 - face.wall - 10.0;
    for (target in [[0.0, 20.0], [0.0, -20.0]]) {
      var best:Null<{x:Float, y:Float}> = null, bestDistance = 1e9;
      var x = -halfX;
      while (x <= halfX + 1e-9) {
        var y = -halfY;
        while (y <= halfY + 1e-9) {
          var d = dist(x, y, target[0], target[1]);
          if (d < bestDistance && slotFree(face, x, y, posts, standoffs, enabling, ribs, result)) {
            best = {x: x, y: y};
            bestDistance = d;
          }
          y += 1.5;
        }
        x += 1.5;
      }
      if (best == null) throw "No room for a strap slot near (" + target[0] + ", " + target[1] + ")";
      result.push(best);
    }
    return result;
  }

  static function slotFree(
    face:PanelLayout,
    x:Float,
    y:Float,
    posts:Array<{x:Float, y:Float}>,
    standoffs:Array<{x:Float, y:Float}>,
    enabling:Null<{x:Float, y:Float}>,
    ribs:Array<{x:Float, y:Float, w:Float, h:Float}>,
    taken:Array<{x:Float, y:Float}>
  ):Bool {
    var slot = Rect(STRAP_SLOT_WIDTH, STRAP_SLOT_HEIGHT);
    // A strap passes behind the parts: keep clear of any that leave less than 8 mm to the back wall.
    for (c in face.controls) if (c.behind() > face.caseInnerDepth - 8.0 && FootprintGeometry.gap(
      c.body(),
      c.x,
      c.y,
      slot,
      x,
      y
    ) < BODY_CLEARANCE) return false;
    for (p in posts) if (FootprintGeometry.distanceTo(slot, x, y, p.x, p.y) < POST_DIAMETER / 2 + 3.0) return false;
    for (s in standoffs) if (FootprintGeometry.distanceTo(slot,
      x, y, s.x, s.y) < STANDOFF_DIAMETER / 2 + 3.0) return false;
    if (enabling != null && FootprintGeometry.gap(slot,
      x, y, enablingFootprint(), enabling.x, enabling.y) < 3.0) return false;
    for (r in ribs) if (FootprintGeometry.gap(slot, x, y, Rect(r.w, r.h), r.x, r.y) < 2.0) return false;
    for (t in taken) if (dist(t.x, t.y, x, y) < 20.0) return false;
    return true;
  }

  static function dist(ax:Float,
    ay:Float, bx:Float, by:Float):Float return Math.sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));

  /**
	 * The inserts and screws the plan implies: an M3 insert in each post with a button head screw through the plate's
	 * counterbore, and the board's standoffs: an insert in each boss on the plate's back, a standoff of standard length screwed
	 * into it, and a screw from under the board into the standoff. Screw lengths are the shortest standard ones that reach 75 %
	 * of what they thread into, and never past it.
	 */
  public function hardware():Array<PanelHardware> {
    var result:Array<PanelHardware> = [];
    var caseInsert = ThreadedInsert.m3(), bossInsert = ThreadedInsert.m2_5Short();
    var bearing = face.thickness - PanelFacePlate.COUNTERBORE_DEPTH;
    var caseLength = standardLength(bearing + 0.75 * caseInsert.length, bearing + caseInsert.length);
    var underside = boardTop - BOARD_THICKNESS;
    var standoff = HexStandoff.m2_5(standoffLength);
    var boardLength = standardLength(BOARD_THICKNESS + 0.75 * standoff.boreDepth, BOARD_THICKNESS + standoff.boreDepth);
    var i = 0;
    for (p in posts) {
      i++;
      result.push(new PanelHardware('insert-$i', caseInsert, p.x, p.y, 0.0));
      result.push(new PanelHardware('screw-$i', ButtonHeadScrew.m3(caseLength), p.x, p.y, bearing));
    }
    i = 0;
    for (s in standoffs) {
      i++;
      // An insert in each boss, pressed in from the boss's end; the standoff's stud into it; the board on the standoff's foot;
      // and a screw from under the board into the standoff.
      result.push(new PanelHardware('board-insert-$i', bossInsert, s.x, s.y, -BOSS_HEIGHT, true));
      result.push(new PanelHardware('standoff-$i', standoff, s.x, s.y, boardTop));
      result.push(new PanelHardware('board-screw-$i', ButtonHeadScrew.m2_5(boardLength), s.x, s.y, underside, true));
    }
    return result;
  }

  static function standardLength(atLeast:Float, atMost:Float):Float {
    for (l in [4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0]) if (l >= atLeast) {
      if (l > atMost) throw 'No standard screw length between $atLeast and $atMost mm';
      return l;
    }
    throw 'No standard screw length reaches $atLeast mm';
  }

  /** The posts, standoffs, receptacle, enabling switch and strap slots as marks for PanelLayoutSvg. */
  public function marks():Array<SvgMark> {
    var result:Array<SvgMark> = [];
    for (post in posts) {
      result.push({x: post.x, y: post.y, size: POST_DIAMETER, height: 0.0, filled: false});
      result.push({x: post.x, y: post.y, size: PanelFacePlate.COUNTERBORE, height: 0.0, filled: false});
    }
    for (s in standoffs) result.push({x: s.x, y: s.y, size: STANDOFF_DIAMETER, height: 0.0, filled: true});
    for (s in strapSlots) result.push({x: s.x,
      y: s.y, size: STRAP_SLOT_WIDTH, height: STRAP_SLOT_HEIGHT, filled: true});
    for (r in ribs) result.push({x: r.x, y: r.y, size: r.w, height: r.h, filled: true});
    if (enabling != null) result.push({
      x: enabling.x,
      y: enabling.y,
      size: ENABLING_DEPTH,
      height: ENABLING_BODY,
      filled: false
    });
    result.push({
      x: connectorX,
      y: -face.height / 2 + face.wall / 2,
      size: RECEPTACLE_HOLE,
      height: face.wall,
      filled: true
    });
    return result;
  }

  public function outerDepth():Float return face.caseInnerDepth + BACK_WALL;

  /** The shell: a rounded box open at the top, with posts, standoffs, the receptacle hole, strap slots and the enabling switch opening. */
  public function shell():Part {
    var depth = outerDepth();
    var outer = RoundedRectangle.box(face.width, face.height, PanelFacePlate.CORNER_RADIUS, -depth, 0.0);
    var cavity = RoundedRectangle.box(
      face.width - 2 * face.wall,
      face.height - 2 * face.wall,
      PanelFacePlate.CORNER_RADIUS - face.wall,
      -face.caseInnerDepth,
      1.0
    );
    var tools:Array<Part> = [cavity];
    tools.push(Part.cylinderAlongY(
      RECEPTACLE_HOLE / 2,
      -face.height / 2 - 1.0,
      -face.height / 2 + face.wall + 1.0,
      connectorX,
      connectorZ
    ));
    for (s in strapSlots) tools.push(FootprintGeometry.solid(
      Rect(STRAP_SLOT_WIDTH, STRAP_SLOT_HEIGHT),
      s.x,
      s.y,
      -depth - 1.0,
      -face.caseInnerDepth + 1.0
    ));
    if (enabling != null) {
      var side = enabling.x < 0 ? -1.0 : 1.0;
      tools.push(Part.cylinderAlong(
        ENABLING_HOLE / 2,
        new Vector(side * (face.width / 2 + 1.0), enabling.y, enablingZ(face)),
        new Vector(-side, 0.0, 0.0),
        face.wall + 2.0
      ));
    }
    var current = outer.subtractAll(tools);
    outer.close();
    for (t in tools) t.close();
    var underside = boardTop - BOARD_THICKNESS;
    for (p in posts) {
      var post = Part.cylinderSpan(POST_DIAMETER / 2, -face.caseInnerDepth - 0.5, 0.0, p.x, p.y);
      var pilot = Part.cylinderSpan(PILOT_DIAMETER / 2, -PILOT_DEPTH, 1.0, p.x, p.y);
      current = fuseThenDrill(current, post, pilot);
    }
    var top = ribTop(boardTop);
    for (r in ribs) {
      var rib = FootprintGeometry.solid(Rect(r.w, r.h), r.x, r.y, -face.caseInnerDepth - 0.5, top);
      var joined = current.combine(rib);
      current.close();
      rib.close();
      current = joined;
    }
    return current;
  }

  static function fuseThenDrill(body:Part, add:Part, hole:Part):Part {
    var joined = body.combine(add);
    var drilled = joined.subtract(hole);
    body.close();
    add.close();
    hole.close();
    joined.close();
    return drilled;
  }

  /** The board envelope: an outline the size of the case's inside less a margin, with every notch and the four mounting holes. */
  public function board():Part {
    var half = boardHalf(face);
    var base = RoundedRectangle.box(half.x * 2, half.y * 2, 3.0, boardTop - BOARD_THICKNESS, boardTop);
    var tools:Array<Part> = [];
    for (n in notches()) tools.push(FootprintGeometry.solid(
      n.f,
      n.x,
      n.y,
      boardTop - BOARD_THICKNESS - 1.0,
      boardTop + 1.0
    ));
    for (s in standoffs) tools.push(Part.cylinderSpan(
      BOARD_HOLE / 2,
      boardTop - BOARD_THICKNESS - 1.0,
      boardTop + 1.0,
      s.x,
      s.y
    ));
    var result = base.subtractAll(tools);
    base.close();
    for (t in tools) t.close();
    return result;
  }

  /** The face plate with the screw holes that match the posts and a perimeter tongue that fits inside the walls. */
  /** The bosses on the plate's back that the board's standoffs screw into, in panel coordinates. */
  public function bosses():Array<PanelBoss> return [for (s in standoffs) {
    x:s.x, y:s.y,
      diameter:STANDOFF_DIAMETER, height:BOSS_HEIGHT, pilotDiameter:STANDOFF_PILOT, pilotDepth:STANDOFF_PILOT_DEPTH
  }
  ];

  public function plate():Part {
    var plain = PanelFacePlate.part(face, posts, bosses());
    var tongue = perimeterTongue();
    var joined = plain.combine(tongue);
    plain.close();
    tongue.close();
    return joined;
  }

  /** A ring just inside the walls, cut away around every rear body and post so it never meets a part. */
  function perimeterTongue():Part {
    var inset = face.wall + TONGUE_CLEARANCE;
    var radius = Math.max(PanelFacePlate.CORNER_RADIUS - inset, 1.0);
    var outer = RoundedRectangle.box(face.width - 2 * inset, face.height - 2 * inset, radius, -TONGUE_HEIGHT, 0.0);
    var inner = RoundedRectangle.box(
      face.width - 2 * (inset + TONGUE_WIDTH),
      face.height - 2 * (inset + TONGUE_WIDTH),
      Math.max(radius - TONGUE_WIDTH, 0.5),
      -TONGUE_HEIGHT - 1.0,
      1.0
    );
    var tools:Array<Part> = [inner];
    for (c in face.controls) tools.push(FootprintGeometry.solid(
      FootprintGeometry.grow(c.body(), 1.0),
      c.x,
      c.y,
      -TONGUE_HEIGHT - 1.0,
      1.0
    ));
    for (p in posts) tools.push(FootprintGeometry.solid(
      Circle(POST_DIAMETER + 2.0),
      p.x,
      p.y,
      -TONGUE_HEIGHT - 1.0,
      1.0
    ));
    var ring = outer.subtractAll(tools);
    outer.close();
    for (t in tools) t.close();
    return ring;
  }
}
