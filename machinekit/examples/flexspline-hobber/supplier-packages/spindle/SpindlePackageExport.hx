import machinekit.assembly.PosedParts;
import machinekit.hobbing.HobArbor;
import machinekit.hobbing.HobSpindle;
import machinekit.component.ComponentValues;
import machinekit.tooling.CastingSectionContours;

/** Writes the hob spindle package from the model: a STEP file per made part in its own
    frame, a STEP of the posed cartridge, and each made part's exact axial section
    (lines and arcs, section plane through the axis) for spindle_package.py.
    The arbor is the one FlexsplineHobber installs. Usage: hl main.hl <output dir> */
class SpindlePackageExport {
  // Members we have made; everything else in the cartridge is bought.
  static final MADE = ["arbor"];

  static function main() {
    var args = Sys.args();
    if (args.length != 1) throw "usage: hl main.hl <output dir>";
    var dir = args[0];
    sys.FileSystem.createDirectory(dir);
    // FlexsplineHobber's arbor: 175 long, hob centre 60 from the gauge face, Ø15 x 22 support journal.
    var spindle = new HobSpindle(null, new HobArbor(new ComponentValues().setNumber("height", 175).setNumber(
      "hobCentreOffset",
      60
    ).setNumber("supportDiameter", 15).setNumber("supportLength", 22)));
    spindle.validate();
    var poses = spindle.solvedPoses(), cache = new PosedParts();
    var plane = CastingSectionContours.normalFrame({x: 0, y: 1, z: 0}, "spindle-axial");
    var made:Array<Dynamic> = [], bought:Array<Dynamic> = [], posed:Array<cadkit.Shape> = [], members:Array<Dynamic> = [];
    try {
      for (m in spindle.components()) {
        var part = cache.local(m.component);
        var pose = poses.get(m.id);
        if (pose == null) throw 'unposed member ${m.id}';
        posed.push(PosedParts.moved(part, pose).shape);
        var axis = materia.assembly.AssemblyFrames.transformVector(pose, 0, 0, 1);
        members.push({
          id: m.id,
          partNumber: m.component.bom.partNumber,
          description: m.component.bom.description,
          position: [pose.x, pose.y, pose.z],
          axisZ: axis.z
        });
        if (MADE.indexOf(m.id) < 0) continue;
        var file = 'spindle-${m.id}.step';
        part.shape.exportStep(dir + "/" + file);
        var contours = CastingSectionContours.section(part, plane, m.id);
        var b = part.shape.bounds();
        made.push({
          id: m.id,
          partNumber: m.component.bom.partNumber,
          description: m.component.bom.description,
          material: m.component.materialSpec(),
          massKg: m.component.massProperties().mass,
          step: file,
          axialMin: b.get_min().get_z(),
          axialMax: b.get_max().get_z(),
          outsideDiameter: 2 * Math.max(b.get_max().get_x(), -b.get_min().get_x()),
          section: [for (c in contours) {
            depth: c.depth,
            segments: [for (s in c.contour) {
              kind: Std.string(s.kind),
              start: [s.start.x, s.start.y],
              end: [s.end.x, s.end.y],
              center: s.center == null ? null : [s.center.x, s.center.y],
              sweep: s.sweep
            }]
          }]
        });
      }
      cadkit.Shape.compound(posed).exportStep(dir + "/spindle-assembly.step");
      for (line in spindle.billOfMaterials().lines()) bought.push({
        partNumber: line.partNumber,
        description: line.description,
        quantity: line.quantity,
        material: line.material
      });
    } catch (e:Dynamic) {
      cache.close();
      throw e;
    }
    cache.close();
    var motor = spindle.motor.rating;
    var data = {
      source: "materia machinekit/haxe/src/machinekit/hobbing/HobSpindle.hx (HobSpindle, as FlexsplineHobber installs it)",
      units: "mm; section x = radial, y = axial from the housing rear datum (tool face at 180) for cartridge parts, from the gauge face for the arbor",
      spindleInterface: spindle.interfaceName(),
      motor: {
        designation: motor.designation,
        ratedTorque: motor.ratedTorque,
        ratedRpm: motor.ratedSpeed * 60 / (2 * Math.PI),
        maxRpm: motor.maxSpeed * 60 / (2 * Math.PI),
        shaftDiameter: motor.shaftDiameter,
        shaftLength: motor.shaftLength,
        face: motor.bodyDiameter
      },
      layout: {
        shaftRear: HobSpindle.SHAFT_REAR,
        encoderLength: HobSpindle.ENCODER_LENGTH,
        encoderDiameter: HobSpindle.ENCODER_DIAMETER,
        encoderCounts: HobSpindle.ENCODER_COUNTS,
        pulleyFront: HobSpindle.pulleyFront(motor.shaftLength),
        pulleyWidth: HobSpindle.PULLEY_WIDTH,
        pulleyTeeth: HobSpindle.SPINDLE_TEETH,
        motorTeeth: HobSpindle.MOTOR_TEETH,
        centres: HobSpindle.centres(),
        toolFace: 180.0
      },
      arbor: {
        length: spindle.arbor.parameterNumber("height"),
        hobSeatDiameter: spindle.arbor.parameterNumber("outside"),
        hobSeatLength: spindle.arbor.parameterNumber("hobSeatLength"),
        hobCentre: spindle.arbor.parameterNumber("hobCentreOffset"),
        supportDiameter: spindle.arbor.parameterNumber("supportDiameter"),
        supportLength: spindle.arbor.parameterNumber("supportLength")
      },
      made: made,
      members: members,
      bom: bought
    };
    sys.io.File.saveContent(dir + "/spindle-package.json", haxe.Json.stringify(data, null, "  "));
    Sys.println('wrote $dir: ${made.length} made parts, ${bought.length} BOM lines');
  }
}
