package machinekit.cover;

import machinekit.assembly.AxisBuilder;
import machinekit.assembly.AxisBuilder.AxisSpec;
import machinekit.cover.WayCoverSection.WayCoverRole;
import machinekit.design.DesignRequirement;
import materia.assembly.AssemblyFrames;

/** Where a way cover goes: between a fixed and a moving member along a prismatic joint. */
typedef WayCoverPlacement = {
  /** Names the cover's members (`<id>-fixed`, `<id>-s1` ..., `<id>-moving`) and joints. */
  var id:String;
  /** A `WayCoverCatalog` row. */
  var row:String;
  /** The members the two flanges are fixed to. */
  var fixedParent:String;
  var movingParent:String;
  /** The joint the moving member rides on (made by `AxisBuilder.slide`): the cover follows it. */
  var leader:AxisSpec;
  /** World centre of the fixed flange's mounting plane, and the unit vectors along the cover (from the fixed flange to the moving one) and up. */
  var origin:Array<Float>;
  var along:Array<Float>;
  var up:Array<Float>;
  /** The clear opening the cover surrounds (the protected profile with its clearance), mm. */
  var width:Float;
  var height:Float;
  /** Length between the two mounting planes with the joint at its end stop where they are closest, mm. */
  var space:Float;
};

/**
 * A bought way cover (bellows or telescopic) between a fixed and a moving member of a prismatic joint. `install` derives it
 * from the catalogue row, the opening, and the travel of the joint: its folds or sections, its folded and extended length
 * (it throws, with the numbers, when it will not fold into the space at the end stop or cannot reach the far stop), and the
 * members that make it: two end flanges and the loose skins between them, each on its own prismatic joint coupled to the leader
 * (`WayCoverPlan` explains the nested skins). The sweep of `AssemblyIntegrity` then sees the cover at every sampled state of the joint,
 * its ends move with the members they are fixed to, and the flange bolts are derived from their `BoltedFace`s
 * (`MachineAssembly.deriveFasteners`). One BOM line: the whole cover.
 */
class WayCover {
  public final id:String;
  public final plan:WayCoverPlan;
  public final fixedMember:String;
  public final movingMember:String;
  public final skinMembers:Array<String>;
  /** The joint value at which the cover is folded shortest. */
  public final foldedAt:Float;

  function new(id:String, plan:WayCoverPlan, fixedMember:String, movingMember:String, skins:Array<String>, foldedAt:Float) {
    this.id = id;
    this.plan = plan;
    this.fixedMember = fixedMember;
    this.movingMember = movingMember;
    skinMembers = skins;
    this.foldedAt = foldedAt;
  }

  /** Every member of the cover, fixed flange first. */
  public function members():Array<String> return [fixedMember].concat(skinMembers).concat([movingMember]);

  public static function install(builder:AxisBuilder, p:WayCoverPlacement):WayCover {
    var row = WayCoverCatalog.get(p.row);
    var plan = new WayCoverPlan(row, p.width, p.height, p.leader.upper - p.leader.lower, p.space);
    var axis = builder.axisDirection(p.leader.id);
    var sense = axis[0] * p.along[0] + axis[1] * p.along[1] + axis[2] * p.along[2];
    if (Math.abs(Math.abs(sense) - 1) > 1e-6) throw 'Way cover ${p.id}: the cover must run along the joint ${p.leader.id}, not across it';
    var s:Float = sense > 0 ? 1 : -1;
    // The cover is shortest where the moving member comes nearest the fixed flange.
    var folded = s > 0 ? p.leader.lower : p.leader.upper;
    var extension0 = s * (0 - folded);
    var fixedId = p.id + "-fixed", movingId = p.id + "-moving";
    function section(role:WayCoverRole, level:Int):WayCoverSection return new WayCoverSection(row, role, level, p.width, p.height, p.leader.upper - p.leader.lower, p.space);
    function pose(distance:Float, flip:Bool) return AxisBuilder.orient(
      p.origin[0] + p.along[0] * distance, p.origin[1] + p.along[1] * distance, p.origin[2] + p.along[2] * distance,
      p.up, flip ? [-p.along[0], -p.along[1], -p.along[2]] : p.along
    );
    builder.attach(fixedId, section(Fixed, 0), pose(0, false), p.fixedParent);
    var skins:Array<String> = [];
    var n = plan.levels, tf = row.flangeThickness;
    // Skin k of n sits k/n along the free span; each rides on the one before it, so each joint carries 1/n of the growth.
    for (k in 1...n) {
      var id = p.id + "-s" + k;
      var start = tf + WayCoverPlan.AXIAL_GAP + k / n * (plan.slack() + s * (0 - folded));
      var parent = k == 1 ? fixedId : p.id + "-s" + (k - 1);
      builder.hang(id, section(Sleeve, k), pose(start, false), parent);
      var ratio = s / n;
      var low = Math.min(ratio * p.leader.lower, ratio * p.leader.upper), high = Math.max(ratio * p.leader.lower, ratio * p.leader.upper);
      builder.addMateOnAxis(id + "-joint", "prismatic", parent, 'to-$id', id, 'attach-$id', {x: p.along[0], y: p.along[1], z: p.along[2]},
        ratio * p.leader.initial, {lower: low, upper: high, velocity: null, effort: null});
      builder.addCoupling(id + "-follows", p.leader.id, id + "-joint", ratio);
      skins.push(id);
    }
    builder.attach(movingId, section(Moving, n), pose(p.space + extension0, true), p.movingParent);
    return new WayCover(p.id, plan, fixedId, movingId, skins, folded);
  }
}
