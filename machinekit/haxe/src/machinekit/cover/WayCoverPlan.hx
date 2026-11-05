package machinekit.cover;

import machinekit.cover.WayCoverCatalog.WayCoverRow;

/**
 * The cross-section family of a cover. A ring surrounds its opening on all four sides, centred on the axis it protects. An open
 * cover (a box cover) has a roof and two walls and no floor: it stands on a surface of the machine (the base, a saddle top, a
 * column face) that closes it, its frame origin on that surface and its opening running from there up `height`. The skins of an open
 * cover nest outward on the three walled sides only, so they never grow into the surface they stand on.
 */
typedef CoverShape = {
  var open:Bool;
  /** Open cover: the range of heights above the surface in which the host face of each flange exists, mm; bolt holes go only there. */
  var hostMin:Float;
  var hostMax:Float;
};

/**
 * What a way cover is for a given opening and stroke, derived from its catalogue row: how many folds or sections, its
 * compressed and extended length, and the nested skins that stand in for its envelope. Everything follows from the row,
 * the clear opening and the stroke; nothing is typed in per machine.
 *
 * A cover is drawn as `levels + 1` skins that nest, each a thin rectangular sleeve one `step` larger than the one before,
 * moving `level / levels` of the extension: the envelope of the cover at every state of its joint, which is what a sweep
 * must clear, with the folds themselves not drawn. The fixed flange carries skin 0, the moving flange skin `levels`.
 */
class WayCoverPlan {
  public final row:WayCoverRow;
  /** The clear opening the cover surrounds (the protected profile and its running clearance), mm. */
  public final width:Float;
  public final height:Float;
  public final stroke:Float;
  /** Folds (a bellows) or sections (telescopic); 0 for the other kind. */
  public final folds:Int;
  public final sections:Int;
  /** The length between the mounting planes at the end stop where the cover is shortest, and at the far stop, mm. */
  public final space:Float;
  public final extended:Float;
  /** The shortest the cover can fold to between its mounting planes, mm. */
  public final compressed:Float;
  /** Axial length of the folded body between the flange plates when fully folded, mm. */
  public final body:Float;
  /** Skins that move relative to each other beyond the flange carrying skin 0 and the one carrying `levels`. */
  public final levels:Int;
  public final shape:CoverShape;
  /** The clearance an open cover's walls keep above the surface they stand on, mm. */
  public static inline var FLOOR_GAP:Float = 1.5;
  /** Axial clearance between a skin's end and the flange plate it faces, mm. */
  public static inline var AXIAL_GAP:Float = 1.2;
  /** Length of the slide lip with which a loose skin rides on the next smaller one, mm. */
  public static inline var LIP:Float = 6;

  /**
   * `space` is the length between the two mounting planes at the end stop where they are closest; `stroke` is how far they
   * part. The cover must fold into `space` and reach `space + stroke`, which fixes its folds (or sections): the fewest that
   * extend far enough, and then they must fold short enough.
   */
  public static function ring():CoverShape return {open: false, hostMin: 0, hostMax: 100000};

  public static function boxCover(hostMin:Float = 0, hostMax:Float = 100000):CoverShape return {open: true, hostMin: hostMin, hostMax: hostMax};

  public function new(row:WayCoverRow, width:Float, height:Float, stroke:Float, space:Float, ?shape:CoverShape) {
    if (!(width > 0) || !(height > 0) || !(stroke > 0) || !(space > 0) || !Math.isFinite(width + height + stroke + space)) throw "A way cover needs a positive opening, stroke and space";
    this.row = row;
    this.shape = shape == null ? ring() : shape;
    this.width = width;
    this.height = height;
    this.stroke = stroke;
    this.space = space;
    extended = space + stroke;
    var reach = extended - 2 * row.flangeThickness;
    switch row.kind {
      case Bellows:
        folds = Math.ceil(reach / (row.ratio * row.foldCompressed) - 1e-9);
        sections = 0;
        body = folds * row.foldCompressed;
      case Telescopic:
        var needed = Math.ceil((reach - row.overlap) / (row.sectionLength - row.overlap) - 1e-9);
        sections = needed < 2 ? 2 : needed;
        folds = 0;
        // The sections nest along the cover: each further one adds its end fold to the folded stack.
        body = row.sectionLength + (sections - 1) * row.stackStep;
    }
    compressed = body + 2 * row.flangeThickness;
    if (compressed > space + 1e-9) throw 'Way cover ${row.id} needs ${folds > 0 ? '$folds folds' : '$sections sections'} to reach ${r(extended)} mm and folds to ${r(compressed)} mm, but only ${r(space)} mm is free at the end stop';
    var skin = body - 2 * AXIAL_GAP;
    if (!(skin > LIP + 1)) throw 'Way cover ${row.id} folds to ${body} mm: too short for its skins';
    // Each loose skin keeps its lip on the next smaller skin: neighbours are no farther apart than a skin is long less the lip.
    var skins = Math.ceil((reach - body) / (skin - LIP) - 1e-9);
    levels = skins < 1 ? 1 : skins;
  }

  static function r(v:Float):Float return Math.round(v * 10) / 10;

  /** Slack of the body at the compressed stop: how much longer the free span is than the fully folded body, mm. */
  public function slack():Float return space - 2 * row.flangeThickness - body;

  /** The step between two nested skins, mm. */
  public function step():Float return row.wall + row.gap;

  /** Half the opening of skin `level`, across and up, mm. */
  public function innerHalfWidth(level:Int):Float return width / 2 + row.gap + level * step();

  public function innerHalfHeight(level:Int):Float return height / 2 + row.gap + level * step();

  /**
   * The inner extent of skin `level` up the section frame (y), mm: a ring is centred on the axis; an open cover runs from the
   * surface (below it, so the cut goes through) to its roof.
   */
  public function innerLow(level:Int):Float return shape.open ? -1 : -innerHalfHeight(level);

  public function innerHigh(level:Int):Float return shape.open ? height + row.gap + level * step() : innerHalfHeight(level);

  /** The nominal opening up the frame (below the surface for an open cover, so a cut goes through), mm. */
  public function openingLow():Float return shape.open ? -1 : -height / 2;

  public function openingHigh():Float return shape.open ? height : height / 2;

  /** The outer extent of skin `level` up the frame: its walls and roof added to the inner extent, the open cover's feet `FLOOR_GAP` above the surface. */
  public function outerLow(level:Int):Float return shape.open ? FLOOR_GAP : innerLow(level) - row.wall;

  public function outerHigh(level:Int):Float return innerHigh(level) + row.wall;

  /** The flanges' outer footprint, mm. */
  public function outerWidth():Float return 2 * (innerHalfWidth(levels) + row.wall + row.lip);

  public function outerHeight():Float return plateHigh(levels) - plateLow(levels);

  /** The plate of a flange carrying skin `level` spans this range up the frame: the skin's outer extent and the lip, mm. */
  public function plateLow(level:Int):Float return shape.open ? FLOOR_GAP : innerLow(level) - row.wall - row.lip;

  public function plateHigh(level:Int):Float return outerHigh(level) + row.lip;

  /** Length of the loose skins and of the two attached to the flanges, mm. */
  public function looseSkin():Float return body - 2 * AXIAL_GAP;

  public function flangeSkin():Float return body - AXIAL_GAP;

  /** The speed the cover is rated for, mm/s. */
  public function maxSpeed():Float return row.maxSpeed;
}
