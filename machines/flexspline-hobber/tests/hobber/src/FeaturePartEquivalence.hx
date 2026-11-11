import hobber.*;
import cadkit.modeling.Part;
import machinekit.assembly.PosedParts;
import machinekit.component.MachineComponent;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import sys.io.File;

/**
 * The machine-frame snapshot of the parts this machine designs, which restructuring them must preserve.
 *
 * Every member's world pose, and for each part the hobber designs itself (the classes in `machine/hobber`): its BOM identity,
 * mass, solid properties, route holes, fastening faces and pinned faces, all expressed in the machine frame so that moving
 * a part into its own frame changes nothing here. One record per line, `kind <TAB> key <TAB> name=value ...`,
 * sorted, so a failure diffs readably.
 *
 * Usage: `hl main.hl` compares with the checked-in fixture; `hl main.hl --write [path]` rewrites it.
 * Face counts are reported, not failed: re-expressing a solid as features may legitimately split or merge faces.
 */
class FeaturePartEquivalence {
  static final FIXTURE = "machines/flexspline-hobber/tests/hobber/fixtures/feature-parts-baseline.tsv";

  static function fixture():String return machinekit.catalog.RepositoryRoot.path(FIXTURE);

  /** Absolute and relative tolerance by value name; anything unlisted is compared exactly. */
  static function tolerance(name:String):{abs:Float, rel:Float} return switch name {
    case "volume" | "area": {abs: 1e-3, rel: 1e-9};
    case "mass": {abs: 1e-6, rel: 1e-9};
    case "ixx" | "ixy" | "ixz" | "iyy" | "iyz" | "izz": {abs: 1.0, rel: 1e-7};
    case "lo" | "hi": {abs: 0.05, rel: 0};
    case _: {abs: 1e-4, rel: 0};
  };

  /** The parts this machine designs itself. */
  static function designed(component:MachineComponent):Null<String> {
    if (Std.isOfType(component, HobberMadePart)) return "made";
    if (Std.isOfType(component, machinekit.structural.MineralCasting)) return "casting";
    if (Std.isOfType(component, machinekit.structural.CastInsert)) return "cast-insert";
    if (Std.isOfType(component, HobberCarriageRib)) return "rib";
    if (Std.isOfType(component, HobberChipCover)) return "chip-cover";
    if (Std.isOfType(component, HobberYPart)) return "y-part";
    if (Std.isOfType(component, HobberZBellowsPart)) return "z-bellows";
    return null;
  }

  static function n(v:Float):String {
    if (!Math.isFinite(v)) throw "non-finite value in snapshot";
    var r = Math.fround(v * 1e6) / 1e6;
    return Std.string(r == 0 ? 0.0 : r);
  }

  static function frame(prefix:String, f:AssemblyFrame):String {
    var m = F.toRotationMatrix(f);
    return '${prefix}x=${n(f.x)} ${prefix}y=${n(f.y)} ${prefix}z=${n(f.z)} ' + [for (i in 0...9) '${prefix}r$i=${n(m[i])}'].join(" ");
  }

  static function point(name:String, pose:AssemblyFrame, p:Array<Float>):String {
    var q = F.transformPoint(pose, p[0], p[1], p[2]);
    return '$name=${n(q.x)},${n(q.y)},${n(q.z)}';
  }

  static function vector(name:String, pose:AssemblyFrame, d:Array<Float>):String {
    var q = F.transformVector(pose, d[0], d[1], d[2]);
    return '$name=${n(q.x)},${n(q.y)},${n(q.z)}';
  }

  static function token(s:Null<String>):String return s == null ? "-" : StringTools.replace(StringTools.replace(s, "\t", " "), " ", "_");

  public static function snapshot(machine:FlexsplineHobber):Array<String> {
    var lines:Array<String> = [];
    var poses = machine.solvedPoses();
    var seen = 0;
    for (member in machine.components()) {
      var pose = poses.get(member.id);
      if (pose == null) throw 'No pose for ${member.id}';
      lines.push('pose\t${member.id}\t' + frame("", pose));
      var kind = designed(member.component);
      if (kind == null) continue;
      seen++;
      var c = member.component, bom = c.bom, mass = c.massProperties();
      var part = c.geometry(Preview), placed = PosedParts.moved(part, pose);
      part.close();
      var props = placed.massProperties(), box = PosedParts.boxOf(placed), faces = placed.shape.faces().count();
      placed.close();
      var i = props.inertia;
      lines.push('part\t${member.id}\tkind=$kind pn=${token(bom.partNumber)} material=${token(bom.material)} '
        + 'mass=${n(mass.mass)} source=${token(Std.string(mass.source))} volume=${n(props.volume)} area=${n(props.surfaceArea)} '
        + 'cx=${n(props.centerOfMass.x)} cy=${n(props.centerOfMass.y)} cz=${n(props.centerOfMass.z)} '
        + 'ixx=${n(i.xx)} ixy=${n(i.xy)} ixz=${n(i.xz)} iyy=${n(i.yy)} iyz=${n(i.yz)} izz=${n(i.zz)} '
        + 'lo=${n(box.minX)} lo=${n(box.minY)} lo=${n(box.minZ)} hi=${n(box.maxX)} hi=${n(box.maxY)} hi=${n(box.maxZ)} faces=$faces');
      for (h in c.derivedRouteHoles()) lines.push('hole\t${member.id}#${h.id}\tkind=${token(h.kind)} thread=${token(h.thread)} '
        + 'd=${n(h.diameter)} nominal=${n(h.nominal)} fit=${token(h.fit)} depth=${n(h.depth)} '
        + point("at", pose, h.position) + " " + vector("dir", pose, h.direction)
        + (h.slotLength == null ? "" : ' slot=${n(h.slotLength)} ' + vector("slotAxis", pose, h.slotAxis)));
      for (b in c.fasteningFaces()) {
        var holes = b.holes == null ? "holes=-" : [for (h in b.holes) 'hole=${n(h.x)},${n(h.y)}'].join(" ");
        lines.push('face\t${member.id}#${b.id}\trole=${token(Std.string(b.role))} w=${n(b.width)} h=${n(b.height)} '
          + frame("f", F.compose(pose, b.frame)) + " " + holes);
      }
      for (p in c.pinnedFaces()) lines.push('pin\t${member.id}#${p.id}\td=${n(p.diameter)} depth=${n(p.depth)} through=${p.through} '
        + 'fit=${token(p.fit)} ' + frame("f", F.compose(pose, p.frame)) + " " + [for (h in p.holes) 'hole=${n(h.x)},${n(h.y)}'].join(" "));
    }
    if (seen == 0) throw "No designed parts found";
    lines.sort(Reflect.compare);
    return lines;
  }

  static function fields(rest:String):Array<{name:String, value:String}> return [for (t in rest.split(" ")) if (t != "") {
    var eq = t.indexOf("=");
    eq < 0 ? {name: t, value: ""} : {name: t.substr(0, eq), value: t.substr(eq + 1)};
  }];

  static function sameValue(name:String, a:String, b:String):Bool {
    if (a == b) return true;
    var as = a.split(","), bs = b.split(",");
    if (as.length != bs.length) return false;
    var tol = tolerance(name);
    for (k in 0...as.length) {
      var x = Std.parseFloat(as[k]), y = Std.parseFloat(bs[k]);
      if (Math.isNaN(x) || Math.isNaN(y)) return false;
      if (Math.abs(x - y) > tol.abs + tol.rel * Math.max(Math.abs(x), Math.abs(y))) return false;
    }
    return true;
  }

  /** Differences between two snapshots: failures, and topology notes that are reported only. */
  public static function compare(expected:Array<String>, actual:Array<String>):{failures:Array<String>, notes:Array<String>} {
    function index(lines:Array<String>):Map<String, String> {
      var m = new Map<String, String>();
      for (l in lines) {
        var parts = l.split("\t");
        if (parts.length != 3) throw 'Bad snapshot line: $l';
        m.set(parts[0] + "\t" + parts[1], parts[2]);
      }
      return m;
    }
    var e = index(expected), a = index(actual), failures:Array<String> = [], notes:Array<String> = [];
    for (key in e.keys()) {
      var got = a.get(key);
      if (got == null) {
        failures.push('missing $key');
        continue;
      }
      var ef = fields(e.get(key)), af = fields(got);
      if (ef.length != af.length) {
        failures.push('$key: field count ${ef.length} -> ${af.length}');
        continue;
      }
      for (k in 0...ef.length) {
        var x = ef[k], y = af[k];
        if (x.name != y.name) {
          failures.push('$key: field ${x.name} -> ${y.name}');
          break;
        }
        if (x.name == "faces") {
          if (x.value != y.value) notes.push('$key: faces ${x.value} -> ${y.value}');
        } else if (!sameValue(x.name, x.value, y.value)) failures.push('$key: ${x.name} ${x.value} -> ${y.value}');
      }
    }
    for (key in a.keys()) if (!e.exists(key)) failures.push('extra $key');
    failures.sort(Reflect.compare);
    notes.sort(Reflect.compare);
    return {failures: failures, notes: notes};
  }

  /** The parts recorded so far really take their geometry and history from their recordings. */
  static function checkRecorded(machine:FlexsplineHobber):Void {
    var expected = [
      "Xbridge" => ["Stock · Ground plate", "Front window", "Fasteners · Derived holes"],
      "XdriveFixedBar" => ["Stock · Ground bar", "Fasteners · Derived holes"],
      "XdriveFloatingBar" => ["Stock · Ground bar", "Fasteners · Derived holes"],
      "ZdriveFixedBar" => ["Stock · Ground bar", "Fasteners · Derived holes"],
      "ZdriveFloatingBar" => ["Stock · Ground bar", "Z running clearance", "Fasteners · Derived holes"],
      "XscaleSeat" => ["Stock · Ground bar", "Housing holes"],
      "ZscaleSeat" => ["Stock · Ground bar", "Housing holes"],
      "ZscaleArm" => ["Arm"],
      "coolantPort" => ["Block", "Coolant bore"]
    ];
    for (id => steps in expected) {
      var found = [for (member in machine.components()) if (member.id == id) member.component];
      if (found.length != 1) throw 'No member $id';
      var history = found[0].recordedHistory();
      if (history == null) throw '$id does not record its geometry';
      var labels = [for (step in history.steps) step.label];
      for (step in steps) if (labels.indexOf(step) < 0) throw '$id history lacks "$step": ${labels.join(", ")}';
      for (step in history.steps) if (step.faces.length == 0) throw '$id step "${step.label}" names no faces';
    }
  }

  public static function main():Void {
    var args = Sys.args(), write = args.indexOf("--write");
    var started = Sys.time();
    var machine = new FlexsplineHobber();
    var built = Sys.time();
    var actual = snapshot(machine);
    checkRecorded(machine);
    var done = Sys.time();
    var parts = actual.filter(l -> StringTools.startsWith(l, "part\t")).length;
    Sys.println('Snapshot: ${actual.length} records, $parts designed parts; build ${Math.round((built - started) * 10) / 10} s, '
      + 'snapshot ${Math.round((done - built) * 10) / 10} s');
    if (write >= 0) {
      var path = write + 1 < args.length ? args[write + 1] : fixture();
      File.saveContent(path, actual.join("\n") + "\n");
      Sys.println('Wrote $path');
      return;
    }
    var expected = [for (l in File.getContent(fixture()).split("\n")) if (l != "") l];
    var result = compare(expected, actual);
    for (note in result.notes) Sys.println('note: $note');
    for (failure in result.failures) Sys.println('FAIL: $failure');
    if (result.failures.length > 0) throw '${result.failures.length} feature-part equivalence differences';
    Sys.println('Feature-part equivalence: ${expected.length} records match (${result.notes.length} topology notes)');
  }
}
