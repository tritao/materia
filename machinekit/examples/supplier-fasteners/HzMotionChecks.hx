import machinekit.assembly.*;
import machinekit.component.*;
import machinekit.motion.*;
import cadkit.modeling.Part;

class HzMotionChecks {
  static function check(ok:Bool, label:String):Void {
    if (!ok) throw label;
  }
  static function near(a:Float, b:Float, label:String):Void {
    check(Math.isFinite(a) && Math.abs(a - b) < 1e-5, '$label: $a != $b');
  }
  static function overlap(a:Part, b:Part):Float {
    var common = a.intersect(b);
    try {
      var volume = common.volume();
      common.close();
      return volume;
    } catch (e:Dynamic) {
      common.close();
      throw e;
    }
  }
  public static function main():Void {
    var registry = MachineKitComponents.standard(), rows:Array<Dynamic> = [];
    check(
      registry.has(MotorMount.recipeType().id)
      && registry.has(ScrewSupportUnit.recipeType().id) && registry.has(MotorAdapterPlate.recipeType().id),
      "HZMotion's product kinds are in the standard registry"
    );
    check(HzMotionCatalog.coverage().length == 135, "Full six-family coverage index retained");
    for (model in HzMotionCatalog.models()) {
      var kind = HzMotionCatalog.get(model).kind, component = HzMotionParts.of(model);
      Sys.println('Checking $model ($kind)');
      var p = component.geometry();
      check(p.valid(), 'Valid drawing solid: $model');
      check(p.solidCount() == 1, 'One connected purchased component: $model');
      if (kind == "mount") {
        var mount:MotorMount = cast component, s = mount.spec;
        var depth = s.pilotDepth == null ? 3.0 : Math.min(3, s.pilotDepth);
        var probe = Part.cylinderSpan(s.pilot / 2 - 0.1, 0.1, depth - 0.1, 0, s.shaftHeight);
        near(overlap(p, probe), 0, 'Pilot passage $model');
        probe.close();
        for (hole in s.baseHoles) {
          var id = 'baseBolt${s.baseHoles.indexOf(hole)+1}', datum = mount.connector(id).frame;
          near(datum.x, hole.x, "Base hole X datum");
          near(datum.z, hole.z, "Base hole Z datum");
          var probe = Part.cylinderAlong(
            hole.diameter / 2 - 0.1,
            new cadkit.modeling.Vector(hole.x, -1, hole.z),
            cadkit.modeling.Vector.Y(),
            s.rearHeight + 2
          );
          near(overlap(p, probe), 0, 'Actual base bore $model/$id');
          probe.close();
        }
        if (s.massKg == null) {
          var rejected = false;
          try mount.massProperties() catch (_:Dynamic) rejected = true;
          check(rejected, "Unknown supplier mass must not become an engineering estimate");
        } else near(mount.massProperties().mass, s.massKg ?? 0, "Published purchased assembly mass");
      } else if (kind == "support") {
        var support:ScrewSupportUnit = cast component;
        // The row's interface is HZMotion's drawing: same bore, shaft height and bolt pattern.
        var drawing = HzMotionCatalog.get(model);
        near(support.boreDiameter, drawing.shaftBore, 'Journal bore $model');
        near(support.spec.shaftHeight, drawing.shaftHeight, 'Shaft height $model');
        var holes = support.footHoles();
        for (i in 0...drawing.baseHoles.length) {
          near(holes[i].x, drawing.baseHoles[i].x, 'Bolt X $model');
          near(holes[i].z, drawing.baseHoles[i].z, 'Bolt Z $model');
        }
        check(support.sourceOf("HZMotion") != null, 'HZMotion publishes $model');
      }
      p.close();
    }
    for (model in HzMotionCatalog.cadModels()) {
      if (model == "M60D-BK12-C" || model == "M80E-BK15-C") continue;
      // The STEP's housing and bearing, in the support frame; its loose ring is left to the assembly.
      var support = ScrewSupportUnit.model(model, "hzmotion"), s = support.spec, p = support.geometry();
      // The STEP's three solids are housing, bearing and the loose ring; the unit keeps the first two.
      check(p.valid() && p.solidCount() <= 2, "Supplier bearing unit leaves its loose ring to the assembly");
      var bounds = p.shape.bounds();
      near(bounds.get_min().get_y(), -s.shaftHeight, "Supplier base plane");
      near(bounds.get_min().get_z(), -s.length / 2, "Supplier front plane");
      near(bounds.get_max().get_z(), s.length / 2, "Supplier support length");
      for (c in support.connectors()) if (StringTools.startsWith(c.name, "mount") && c.name != "mount") {
        var probe = Part.cylinderAlong(
          support.footHoles()[0].diameter / 2 - 0.1,
          new cadkit.modeling.Vector(c.frame.x, -s.shaftHeight - 1, c.frame.z),
          cadkit.modeling.Vector.Y(),
          s.height + 2
        );
        near(overlap(p, probe), 0, "Supplier base hole matches connector");
        probe.close();
      }
      var bore = Part.cylinderSpan(s.bore / 2 - 0.1, -3.5, 3.5);
      near(overlap(p, bore), 0, "Supplier shaft passage matches connector");
      bore.close();
      var rebuilt:ScrewSupportUnit = cast support.componentType().create(support.values());
      check(
        rebuilt.designation == support.designation && rebuilt.drawing == "hzmotion",
        "Saved recipe keeps the HZMotion drawing"
      );
      check(ScrewSupportUnit.model(model).designation == support.designation, "Drawing leaves the part number");
      p.close();
    }
    var blocked = false;
    try MotorMount.of("M60D-EK12-C", null, "hzmotion") catch (_:Dynamic) blocked = true;
    check(blocked, "Discrepant mount STEP is excluded from recipes");
    var oldDrawing = MotorMount.of("M60D-BK12-C", "hzmotion-pdf-2025-04-09-v1");
    near(oldDrawing.spec.baseHoles[2].x, -25, "Saved v1 drawing retains its historical transcription");
    check(
      cast(oldDrawing.componentType().create(oldDrawing.values()),
      MotorMount).spec.revision == oldDrawing.spec.revision,
      "Legacy drawing revision still reconstructs"
    );
    var copy = HzMotionCatalog.get("M60D-BK12-C");
    copy.pilot = 999;
    near(HzMotionCatalog.get("M60D-BK12-C").pilot, 50, "Catalog records cannot mutate the shared source");
    var modular = new MachineAssembly();
    modular.addComponent("bracket", MotorMount.of("M60D-FK12-A"));
    modular.addComponent("bearing", ScrewSupportUnit.model("FK12"));
    modular.addMate("bearing-seat", "fixed", "bracket", "fixedSupport", "bearing", "flangeFront");
    modular.validate();
    var solved:Map<String, materia.assembly.AssemblyRecord.AssemblyFrame> = modular.solvedPoses();
    var cache = new PosedParts(), members = modular.components();
    near(
      cache.overlap(
        members.filter(m -> m.id == "bracket")[0].component,
        solved.get("bracket"),
        members.filter(m -> m.id == "bearing")[0].component,
        solved.get("bearing")
      ),
      0,
      "Modular FK12 pilot fits bracket rear seat"
    );
    cache.close();
    var kk = new MachineAssembly();
    kk.addComponent("body", MotorMount.of("KK86-FK10L"));
    kk.addComponent("adapter", new MotorAdapterPlate("KK86-80EP-adapter"));
    kk.addMate("adapter-seat", "fixed", "body", "motor", "adapter", "body");
    kk.validate();
    var kkPoses:Map<String, materia.assembly.AssemblyRecord.AssemblyFrame> = kk.solvedPoses();
    var kkCache = new PosedParts(), kkMembers = kk.components();
    near(
      kkCache.overlap(
        kkMembers.filter(m -> m.id == "body")[0].component,
        kkPoses.get("body"),
        kkMembers.filter(m -> m.id == "adapter")[0].component,
        kkPoses.get("adapter")
      ),
      0,
      "KK adapter locating spigot seats without interference"
    );
    kkCache.close();
    var zFindings =
      new MotorAdapterPlate("KK86-80EP-adapter").findings(LeadshineServo.model("ELM2H-0750LA80E").mounting());
    check(zFindings.length == 2, "Z locating dimensions match but pilot depth and M5 joint remain unqualified");
    var motor = LeadshineServo.model("ELM2H-0400LA60F");
    var findings = MotorMount.of("M60D-BK12-C").findings(motor.mounting());
    check(
      findings.length == 1 && findings[0].indexOf("thread depth") >= 0,
      "400 W nominal mounting dimensions match; thread depth remains unknown"
    );
    check(MotorMount.of("M80E-BK15-C").findings(motor.mounting()).length > 1, "Wrong frame is rejected");
    rows.push({
      model: "BF12",
      status: "Housing and bearing from the STEP; the DIN 471 ring is its own part, installed in the groove"
    });
    HzMotionAxis.checkAndWrite(rows);
    Sys.println("HZMotion catalog geometry, compatibility and drive-stack checks passed");
  }
}
