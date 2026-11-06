import machinekit.gantry.Gantry;
import machinekit.gantry.GantrySpec;
import machinekit.gantry.GantrySpec.GantryDrive;
import machinekit.gantry.LinearTrack;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.PosedParts;

/** Required-mount factory integration without the unrelated scene catalogue.
 * Each constructor executes the native RackMount contact/load-path contract. */
class RackBuilderIntegrationChecks {
  static var assertions = 0;
  static function check(ok:Bool, label:String):Void {
    assertions++;
    if (!ok) throw "Rack builder integration: " + label;
  }
  static function near(a:Float, b:Float, label:String):Void check(
    Math.isFinite(a) && Math.abs(a - b) < 1e-7,
    label + ": " + a + " / " + b
  );
  static function mountSolids(gantry:Gantry):Void {
    // The legacy travel sweep skips same-motion pairs. Check new brackets
    // against those bodies too, including the flange and their real hosts.
    var conflicts:Array<String> = [];
    PosedParts.scope(cache -> {for (id in [for (m in gantry.components()) if (StringTools.startsWith(
      m.id,
      "rackSupport"
    ) || StringTools.startsWith(
      m.id,
      "switchY"
    ) || StringTools.startsWith(
      m.id,
      "switchTriggerY"
    ) || m.id == "motorZSupport") m.id]) {
      var part = gantry.component(id);
      for (member in gantry.components()) if (member.id != id) {
        var volume = cache.overlap(part, gantry.zeroPose(id), member.component, gantry.zeroPose(member.id));
        assertions++;
        if (!Math.isFinite(volume) || volume > 0.001) conflicts.push('$id / ${member.id}: $volume mm3');
      }
    }
    });
    check(conflicts.length == 0, "new mount/switch static intersections:\n" + conflicts.join("\n"));
  }
  public static function main():Void {
    var rack:GantryDrive = Rack(1, 20, 8);
    for (dual in [true, false]) {
      var gantry = new Gantry(new GantrySpec(200, 150, 100, rack, rack, rack, dual));
      gantry.check().throwIfErrors();
      mountSolids(gantry);
      var definition = gantry.definition(), couplings = definition.couplings;
      check(
        couplings != null && couplings.length == (dual ? 4 : 3),
        "all X/Y/Z rack drives retain their physical rolling couplings"
      );
      if (couplings == null) throw "Missing rack couplings";
      for (coupling in couplings) check(
        Math.isFinite(coupling.ratio) && coupling.ratio != 0,
        "rack ratio remains finite and nonzero"
      );
      var state = new AssemblyState(definition), rackX = state.worldPose("rackX"), flange = state.worldPose("flange");
      state.setJoint("x", 50);
      state.forwardKinematics();
      near(state.worldPose("flange").x - flange.x, 50, "tool follows X travel");
      near(state.worldPose("rackX").x, rackX.x, "X rack stays on its supporting beam");
      Sys.println("Required-mount gantry X/Y/Z integration passes; dual Y=" + dual);
    }
    var track = new LinearTrack(3000, 300);
    track.check().throwIfErrors();
    var definition = track.definition(), couplings = definition.couplings;
    check(couplings != null && couplings.length == 1, "track retains one rack coupling");
    if (couplings == null) throw "Missing track coupling";
    near(couplings[0].ratio, 0.05, "module-2 20-tooth track ratio");
    var state = new AssemblyState(definition);
    for (q in [0.0, 300.0, 3000.0]) {
      state.setJoint("track", q);
      state.forwardKinematics();
      near(state.worldPose("carriage").x, q, "track carriage follows its axis");
      near(state.joint("pinion-turn"), q * 0.05, "track shaft rolls on its supported rack");
    }
    Sys.println("RackBuilderIntegrationChecks: " + assertions + " assertions pass");
  }
}
