import machinekit.assembly.AssemblyIntegrity;
import machinekit.assembly.AssemblyIntegrity.IntegrityReport;
import machinekit.assembly.AxisBuilder;
import machinekit.component.BoltedFace;
import machinekit.cover.WayCover;
import machinekit.cover.WayCoverCatalog;
import machinekit.cover.WayCoverPlan;
import machinekit.design.DesignRequirement;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import materia.assembly.AssemblyFrames;

typedef Vec = {x:Float, y:Float, z:Float}

/**
 * `WayCover` on a vertical axis: a carriage on a rail rises from a base, a bellows or telescopic cover between base and carriage.
 * Plan numbers, the sweep seeing the cover, flange bolts, the BOM line, the save round trip, the drain rule and the service removal.
 */
class WayCoverTests {
  static function check(condition:Bool, message:String):Void {
    if (!condition) throw 'Way cover: $message';
  }

  static function codes(report:IntegrityReport):Array<String> return [for (item in report.errors()) item.code];

  static function messages(report:IntegrityReport):String return [for (item in report.errors()) item.message].join("; ");

  static var down:Vec = {x: 0.0, y: 0.0, z: -1.0};
  static var up:Vec = {x: 0.0, y: 0.0, z: 1.0};
  static var across:Vec = {x: 1.0, y: 0.0, z: 0.0};

  /** Base (top at z 0) and carriage (bottom at z = space at q = 0) joined by a rail along +Z; the cover opening is 40 x 40 at x = 80. */
  static function axis(rowId:String, space:Float, stroke:Float, ?extra:AxisBuilder -> Void, bore:Bool = false):{builder:AxisBuilder, cover:WayCover} {
    var builder = new AxisBuilder();
    var plan = new WayCoverPlan(WayCoverCatalog.get(rowId), 40, 40, stroke, space);
    var hx = plan.outerWidth() / 2 + 20, hy = plan.outerHeight() / 2 + 20;
    builder.place("base", new TestSlab("BASE", [80 - hx, -hy, -20], [80 + hx, hy, 0],
      [BoltedFace.drilledToSuit("top", BoltedFace.frameOf({x: 80, y: 0, z: 0}, down, across), plan.outerWidth() + 20, plan.outerHeight() + 20, 12, 20, CastIron)],
      bore ? {x: 80, y: 0, diameter: 12} : null), AssemblyFrames.translation(0, 0, 0));
    var up3 = [1.0, 0, 0], along = [0.0, 0, 1];
    var length = stroke + space + 200;
    builder.place("column", new TestSlab("COLUMN", [-48, -30, -10], [-8, 30, 10 + length + 20],
      [BoltedFace.drilledToSuit("seat", BoltedFace.frameOf({x: -8, y: 0, z: 10 + length / 2}, {x: -1.0, y: 0.0, z: 0.0}, {x: 0.0, y: 1.0, z: 0.0}), 40, length + 30, 12, 40, CastIron)]),
      AssemblyFrames.translation(0, 0, 0));
    builder.attach("rail", LinearRail.metric("MGN12C", length), AxisBuilder.orient(0, 0, 10, up3, along), "column");
    builder.slideOn({id: "z", lower: 0, upper: stroke, initial: 0}, "rail", "block", LinearRailBlock.metric("MGN12C"), space + 15 - 10);
    builder.attach("carriage", new TestSlab("CARRIAGE", [5, -hy, space], [80 + hx, hy, space + 30],
      [BoltedFace.drilledToSuit("bottom", BoltedFace.frameOf({x: 80, y: 0, z: space}, up, across), plan.outerWidth() + 20, plan.outerHeight() + 20, 12, 30, CastIron)]),
      AssemblyFrames.translation(0, 0, 0), "block");
    var cover = WayCover.install(builder, {
      id: "cover", row: rowId, fixedParent: "base", movingParent: "carriage", leader: {id: "z", lower: 0, upper: stroke, initial: 0},
      origin: [80.0, 0, 0], along: [0.0, 0, 1], up: [1.0, 0, 0], width: 40, height: 40, space: space
    });
    if (extra != null) extra(builder);
    return {builder: builder, cover: cover};
  }

  public static function main():Void run();

  public static function run():Void {
    plans();
    var sound = axis("BELLOWS-PU-6x8", 120, 300);
    var report = AssemblyIntegrity.check(sound.builder, {runningGap: 1.0});
    check(report.errors().length == 0, "a bellows between base and carriage should be sound, found: " + messages(report));
    check(sound.cover.skinMembers.length == sound.cover.plan.levels - 1, "one member per loose skin");
    // A post inside the cover's envelope is found by the sweep at some state of the joint.
    var blocked = axis("BELLOWS-PU-6x8", 120, 300, b -> b.attach("post", new TestSlab("POST", [112, -5, 5], [118, 5, 200]), AssemblyFrames.translation(0, 0, 0), "base"));
    var found = codes(AssemblyIntegrity.check(blocked.builder));
    check(found.indexOf("integrity.sweep") >= 0 || found.indexOf("integrity.overlap") >= 0, "a post inside the cover is found, found: " + found);
    // A telescopic cover too.
    var tele = axis("TELESCOPIC-SS-250", 300, 600);
    check(tele.cover.plan.sections >= 3, "a telescopic cover of 600 mm stroke has several sections");
    report = AssemblyIntegrity.check(tele.builder, {runningGap: 1.0});
    check(report.errors().length == 0, "a telescopic cover should be sound, found: " + messages(report));
    fasteners();
    drain();
    service();
    Sys.println("way cover: plans, sweep, flange bolts, BOM, drain rule and service removal pass");
  }

  static function plans():Void {
    var plan = new WayCoverPlan(WayCoverCatalog.get("BELLOWS-PU-6x8"), 40, 40, 300, 120);
    check(plan.extended == 420, "the cover must reach space + stroke");
    check(plan.compressed <= 120 + 1e-9 && plan.body == plan.folds * 6, "folds and folded length " + plan.compressed);
    check(plan.body * 8 >= 420 - 2 * 3 - 1e-9, "the folds extend as far as the stroke needs: " + plan.folds);
    var thrown = false;
    try new WayCoverPlan(WayCoverCatalog.get("BELLOWS-PU-6x8"), 40, 40, 3000, 60) catch (e:Dynamic) thrown = Std.string(e).indexOf("only 60") >= 0;
    check(thrown, "a cover that cannot fold into the space at the end stop is refused, with the numbers");
  }

  static function fasteners():Void {
    var parts = axis("BELLOWS-PU-6x8", 120, 300);
    var derived = parts.builder.deriveFasteners();
    var flanges = [for (joint in derived.joints) if (StringTools.startsWith(joint.clampedMember, "cover-")) joint];
    check(flanges.length == 2, "two flange joints, found " + flanges.length);
    for (joint in flanges) check(joint.size == "M5" && joint.count >= 4, "M5 screws around the flange: " + joint.size + " x " + joint.count);
    var report = AssemblyIntegrity.fasteners(parts.builder);
    check(report.errors().length == 0, "the flange screws fit: " + messages(report));
    var lines = [for (line in parts.builder.billOfMaterials().lines()) if (StringTools.startsWith(line.partNumber, "WAYCOVER")) line];
    check(lines.length == 1 && lines[0].quantity == 1, "the whole cover is one purchase line, found " + lines.length);
    // Every member of the cover rebuilds from its recipe values (the saved form), to the same part.
    for (id in parts.cover.members()) {
      var component = parts.builder.component(id), type = component.componentType();
      check(type != null, "cover member " + id + " has a recipe");
      var again = type.create(component.values());
      check(again.designation == component.designation && again.bom.partNumber == component.bom.partNumber, "cover member " + id + " rebuilds from its values");
    }
  }

  static function drain():Void {
    var floorDrain:ChannelDrain = {high: [80.0, -20, -0.0], low: [80.0, 0, -0.1], drain: "base", diameter: 12.0, outletZ: -20.0};
    var none = axis("BELLOWS-PU-6x8", 120, 300, b -> b.addDesignRequirement(GuardedChannel("chan", "base", ["cover-fixed"], [])));
    var found = codes(AssemblyIntegrity.check(none.builder));
    check(found.indexOf("integrity.drain.chan") >= 0, "a guarded channel without a drain is refused, found " + found);
    var flat = axis("BELLOWS-PU-6x8", 120, 300, b -> b.addDesignRequirement(GuardedChannel("chan", "base", ["cover-fixed"], [floorDrain])), true);
    found = codes(AssemblyIntegrity.check(flat.builder));
    check(found.indexOf("integrity.drain.chan") >= 0, "a floor falling 0.1 mm over 20 is too flat, found " + found);
    var good:ChannelDrain = {high: [80.0, -20, 0.0], low: [80.0, 0, -0.4 - 1.6], drain: "base", diameter: 12.0, outletZ: -20.0};
    var ok = axis("BELLOWS-PU-6x8", 120, 300, b -> b.addDesignRequirement(GuardedChannel("chan", "base", ["cover-fixed"], [good])), true);
    var report = AssemblyIntegrity.check(ok.builder);
    check(report.errors().length == 0, "a falling floor with its bore is sound: " + messages(report));
    var missing = axis("BELLOWS-PU-6x8", 120, 300, b -> b.addDesignRequirement(GuardedChannel("chan", "base", ["cover-fixed"], [good])), false);
    found = codes(AssemblyIntegrity.check(missing.builder));
    check(found.indexOf("integrity.drain.chan") >= 0, "a drain that is not cut in the base is refused, found " + found);
  }

  static function service():Void {
    // A block on the base removed upward; a cover plate over it must come off first.
    function build(withPrerequisite:Bool, steps:Int = 1):AxisBuilder {
      var builder = new AxisBuilder();
      builder.place("base", new TestSlab("BASE", [-100, -100, -10], [100, 100, 0]), AssemblyFrames.translation(0, 0, 0));
      builder.attach("unit", new TestSlab("UNIT", [-20, -20, 0], [20, 20, 40]), AssemblyFrames.translation(0, 0, 0), "base");
      builder.attach("post", new TestSlab("POST", [-60, -60, 0], [-45, 60, 60]), AssemblyFrames.translation(0, 0, 0), "base");
      builder.attach("lid", new TestSlab("LID", [-60, -60, 60], [60, 60, 70]), AssemblyFrames.translation(0, 0, 0), "post");
      builder.addDesignRequirement(ServiceRemoval("unit", ["unit"], [{direction: [0.0, 0, 1], distance: 100.0}], withPrerequisite ? ["lid"] : [], []));
      return builder;
    }
    var found = AssemblyIntegrity.check(build(false));
    check(codes(found).indexOf("integrity.service.unit") >= 0 && messages(found).indexOf("blocked by lid") >= 0, "the lid blocks the unit: " + messages(found));
    found = AssemblyIntegrity.check(build(true));
    check(found.errors().length == 0, "with the lid off the unit comes out: " + messages(found));
  }
}
