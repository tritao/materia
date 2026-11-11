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
    mountRecipes();
    fasteners();
    drain();
    service();
    boxes();
    Sys.println("way cover: plans, sweep, flange bolts, BOM, drain rule, service removal and box covers pass");
  }

  /** Save full mount stock/interfaces and derived holes, including a locating dowel. */
  static function mountRecipes():Void {
    for (foot in [false, true]) {
      var frame = materia.assembly.AssemblyFrames.identity();
      var attachment:Null<machinekit.cover.WayCoverMount.MountAttachment> = foot ? {
        stock:[{x0:30.0, x1:50.0, y0:-10.0, y1:0.0, z0:-14.0, z1:0.0}],
        bolted:[BoltedFace.clamped("foot", frame, [{x:40.0, y:-5.0}], 5.5, 14, "M5", null, false)],
        pinned:[new machinekit.component.BoltedFace.PinnedFace("locate", frame,
          [{x:35.0, y:-5.0}], 4, 7, false, false, "H7")]
      } : null;
      var mount = new machinekit.cover.WayCoverMount("TEST-COVER-MOUNT" + (foot ? "-FOOT" : ""), 50, 50, 14,
        [{x:-20.0, y:25.0, d:8.0}], [{x0:10.0, x1:25.0, y0:0.0, y1:18.0}], 5, attachment);
      // Drilled down from the top face (z = 0) through the 14 mm stock: the face frame's +Z points into the material.
      var top = materia.assembly.AssemblyFrames.fromRotationMatrix(0, 0, 0, [1.0, 0, 0, 0, -1, 0, 0, 0, -1]);
      mount.machine([{frame:top, x:0.0, y:-30.0, diameter:5.0, depth:14.0, through:true, kind:"clearance"}]);
      var assembly = new machinekit.assembly.MachineAssembly();
      assembly.addComponent("mount", mount);
      var restored = machinekit.assembly.MachineAssembly.decode(assembly.encode()).components()[0].component;
      check(mount.bom.partNumber == restored.bom.partNumber && mount.bom.fingerprint == restored.bom.fingerprint,
        "a mount retains its drawing identity and drilled revision");
      check(mount.values().token("attachment") == restored.values().token("attachment"),
        "saved feet retain stock, bolt faces and dowel patterns");
      check(haxe.Json.stringify(mount.machinedCuts()) == haxe.Json.stringify(restored.machinedCuts()),
        "saved mounts retain derived drilling");
      var a = mount.geometry(), b = restored.geometry(), added = a.subtract(b), removed = b.subtract(a);
      var difference = added.volume() + removed.volume();
      a.close(); b.close(); added.close(); removed.close();
      check(difference < 0.01, "saved mount stock is solid-equivalent within 0.01 mm3");
    }
  }

  /**
   * A box cover (roof and two walls, no floor) over a rail on a bed: the base top closes it. The carriage is a bridge over the block with its
   * face at `space` along the bed; a wall at the bed end carries the fixed flange. Returns the cover for the checks.
   */
  static function bed(rowId:String, space:Float, stroke:Float, ?extra:AxisBuilder -> Void, reversed:Bool = false):{builder:AxisBuilder, cover:WayCover, plan:WayCoverPlan} {
    var builder = new AxisBuilder();
    var shape = WayCoverPlan.boxCover(13, 100, reversed ? 50 : 100000, reversed);
    var plan = new WayCoverPlan(WayCoverCatalog.get(rowId), 60, 25, stroke, space, shape);
    var half = plan.outerWidth() / 2 + 10, wallTop = plan.outerHeight() + 20;
    var across = {x: 1.0, y: 0.0, z: 0.0};
    var length = stroke + space + 100;
    builder.place("bed", new TestSlab("BED", [-half - 20, -60, -30], [half + 20, space + stroke + 200, 0],
      [BoltedFace.drilledToSuit("seat", BoltedFace.frameOf({x: 0, y: length / 2 + 10, z: 0}, {x: 0.0, y: 0.0, z: -1.0}, {x: 1.0, y: 0.0, z: 0.0}), 40, length + 30, 12, 30, CastIron)]),
      AssemblyFrames.translation(0, 0, 0));
    builder.attach("wall", new TestSlab("WALL", [-half, -24, 0], [half, 0, wallTop],
      [BoltedFace.drilledToSuit("cover", BoltedFace.frameOf({x: 0, y: 0, z: wallTop / 2}, {x: 0.0, y: -1.0, z: 0.0}, across), 2 * half, wallTop, 16, 24, CastIron)]),
      AssemblyFrames.translation(0, 0, 0), "bed");
    builder.attach("rail", LinearRail.metric("MGN12C", length), AxisBuilder.orient(0, 10, LinearRail.metric("MGN12C", length).spec.railHeight, [0.0, 0, 1], [0.0, 1, 0]), "bed");
    builder.slideOn({id: "y", lower: 0, upper: stroke, initial: 0}, "rail", "block", LinearRailBlock.metric("MGN12C"), space + 20);
    builder.attach("carriage", new TestSlab("CARRIAGE", [-half - 40, space, 13], [half + 40, space + 60, 200],
      [BoltedFace.drilledToSuit("cover", BoltedFace.frameOf({x: 0, y: space, z: 106.5}, {x: 0.0, y: 1.0, z: 0.0}, across), 2 * half + 80, 187, 12, 60, CastIron)]),
      AssemblyFrames.translation(0, 0, 0), "block");
    var cover = WayCover.install(builder, {
      id: "box", row: rowId, fixedParent: "wall", movingParent: "carriage", leader: {id: "y", lower: 0, upper: stroke, initial: 0},
      origin: [0.0, 0, 0], along: [0.0, 1, 0], up: [0.0, 0, 1], width: 60, height: 25, space: space, shape: shape
    });
    if (extra != null) extra(builder);
    return {builder: builder, cover: cover, plan: plan};
  }

  static function boxes():Void {
    for (row in ["BELLOWS-LAMELLA-8x8", "TELESCOPIC-SS-150"]) {
      var space = row == "BELLOWS-LAMELLA-8x8" ? 60.0 : 200.0;
      var sound = bed(row, space, 250);
      var plan = sound.plan;
      check(plan.shape.open && plan.outerHigh(0) > plan.height && plan.outerLow(0) == WayCoverPlan.FLOOR_GAP, "a box cover stands on the surface: no bottom wall, feet at the floor gap");
      var report = AssemblyIntegrity.check(sound.builder, {runningGap: 1.0});
      check(report.errors().length == 0, "box cover " + row + " should be sound on its bed, found: " + messages(report));
      var derived = sound.builder.deriveFasteners();
      var flanges = [for (joint in derived.joints) if (StringTools.startsWith(joint.clampedMember, "box-")) joint];
      check(flanges.length == 2, "two box flange joints for " + row + ", found " + flanges.length);
      for (joint in flanges) check(joint.count >= 4, "the box flange has screws round it: " + joint.count);
      // The rail is screwed down before the cover goes over it; without that order its screws are under the cover's skin.
      var open = AssemblyIntegrity.fasteners(sound.builder);
      check(codes(open).indexOf("fasteners.access") >= 0, "the cover stands over the rail screws until they are fitted first: " + codes(open));
      var order:Array<machinekit.assembly.AssemblyIntegrity.FastenerFitOrder> = [for (id in sound.cover.members().concat(["wall"])) {member: id, fastened: ["rail", "bed"]}];
      var fasteners = AssemblyIntegrity.fasteners(sound.builder, {fittedBefore: order});
      check(fasteners.errors().length == 0, "the box flange screws fit on " + row + ": " + messages(fasteners));
      // A post across the bed inside the envelope is found by the sweep.
      var blocked = bed(row, space, 250, b -> b.attach("post", new TestSlab("POST", [-5, space + 100, 0], [5, space + 110, 30]), AssemblyFrames.translation(0, 0, 0), "bed"));
      var found = codes(AssemblyIntegrity.check(blocked.builder));
      check(found.indexOf("integrity.sweep") >= 0 || found.indexOf("integrity.overlap") >= 0, "a post inside the box cover is found, found: " + found);
    }
    // The nesting can grow toward the fixed end instead: the moving flange then carries the smallest skin, no wider than its host face.
    var small = bed("BELLOWS-LAMELLA-8x8", 60, 250, null, true);
    check(small.plan.movingLevel() == 0 && small.plan.fixedLevel() == small.plan.levels, "reversed nesting: the fixed flange carries the largest skin");
    check(small.plan.plateHalf() <= 50 + 1e-9, "a flange plate is no wider than the host face it is bolted to");
    var moving = small.builder.component("box-moving");
    check(cast(moving, machinekit.cover.WayCoverSection).holes().length >= 3, "the narrow flange still has holes on the host");
    var report = AssemblyIntegrity.check(small.builder, {runningGap: 1.0});
    check(report.errors().length == 0, "reversed box cover should be sound on its bed, found: " + messages(report));
    var order:Array<machinekit.assembly.AssemblyIntegrity.FastenerFitOrder> = [for (id in small.cover.members().concat(["wall"])) {member: id, fastened: ["rail", "bed"]}];
    var screws = AssemblyIntegrity.fasteners(small.builder, {fittedBefore: order});
    check(screws.errors().length == 0, "the reversed cover's flange screws fit: " + messages(screws));
    for (id in small.cover.members()) {
      var component = small.builder.component(id), again = component.componentType().create(component.values());
      check(again.designation == component.designation, "reversed cover member " + id + " rebuilds from its values");
    }
    // A box cover is refused where it cannot fold into the space, with the numbers.
    var thrown = false;
    try new WayCoverPlan(WayCoverCatalog.get("TELESCOPIC-SS-150"), 60, 25, 250, 60, WayCoverPlan.boxCover()) catch (e:Dynamic) thrown = Std.string(e).indexOf("only 60") >= 0;
    check(thrown, "a telescopic box cover that cannot fold into 60 mm is refused");
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
    var scoped = build(false);
    scoped.addDesignRequirement(ServiceRemoval("free", ["unit"], [{direction: [0.0, 0, 1], distance: 100.0}], ["lid"], []));
    check(codes(AssemblyIntegrity.service(scoped)).indexOf("integrity.service.unit") >= 0,
      "the default service audit still checks every removal");
    check(AssemblyIntegrity.service(scoped, {serviceProcedures: ["free"]}).errors().length == 0,
      "a named removal audit keeps the complete obstacle context");
    check(codes(AssemblyIntegrity.service(scoped, {serviceProcedures: ["unit"]})).indexOf("integrity.service.unit") >= 0,
      "a selected blocked removal remains blocked");
    check(codes(AssemblyIntegrity.service(scoped, {serviceProcedures: ["missing"]})).indexOf("integrity.service.selection") >= 0,
      "unknown procedure names cannot silently pass");
    scoped.addDesignRequirement(GuardedChannel("flow", "base", ["lid"], []));
    check(codes(AssemblyIntegrity.service(scoped, {serviceProcedures: ["free"]})).indexOf("integrity.drain.flow") >= 0,
      "a named removal audit still checks every guarded channel");
  }
}
