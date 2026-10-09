package collisionkit;

/**
 * The clearance required between bodies, per pair of body groups (CL-D4's
 * pair classes: a robot's links, its tool, held parts, the environment).
 * Groups are numbered from 0; every pair starts at `initial`.
 */
class CollisionMargins {
  public final groups:Int;
  final table:Array<Float>;

  public function new(groups:Int, initial:Float) {
    if (groups < 1) throw "Collision margins need at least one group";
    checkMargin(initial);
    this.groups = groups;
    table = [for (_ in 0...groups * groups) initial];
  }

  /** One margin for every pair. */
  public static function uniform(margin:Float):CollisionMargins return new CollisionMargins(1, margin);

  public function set(g:Int, h:Int, margin:Float):CollisionMargins {
    checkGroup(g);
    checkGroup(h);
    checkMargin(margin);
    table[g * groups + h] = margin;
    table[h * groups + g] = margin;
    return this;
  }

  public function between(g:Int, h:Int):Float {
    checkGroup(g);
    checkGroup(h);
    return table[g * groups + h];
  }

  /** The square table, row-major (as `collisionkit.h` reads it). */
  public function flat():Array<Float> return table.copy();

  function checkGroup(group:Int):Void {
    if (group < 0 || group >= groups) throw 'Collision group $group is outside the margins (0..${groups - 1})';
  }

  static function checkMargin(margin:Float):Void {
    if (!Math.isFinite(margin) || margin < 0) throw "Collision margins must be finite and not negative";
  }
}
