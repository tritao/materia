package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.HzMotionCatalog;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/**
 * A purchased adapter plate between a motor flange and a linear module or mount (HZMotion KK86-80EP): a square
 * plate, a locating spigot towards the module, the motor's pilot recess and tapped bolts on its pitch circle. Frame:
 * plate from z = 0 (motor side) to its thickness, centred on the shaft at the origin.
 */
class MotorAdapterPlate extends MachineComponent {
  public static function models():Array<String
    > return [for (model in HzMotionCatalog.models()) if (HzMotionCatalog.get(model).kind == "adapter") model];

  public final model:String;
  public final face:Float;
  public final thickness:Float;
  public final pilot:Float;
  public final pcd:Float;
  public final thread:String;

  public function new(model:String = "KK86-80EP-adapter") {
    if (models().indexOf(model) < 0) throw 'Unknown motor adapter plate $model';
    var s = HzMotionCatalog.get(model);
    super('HZMOTION-$model', 'HZMotion $model motor adapter plate', "aluminium 6061");
    this.model = model;
    face = s.face;
    thickness = s.flange;
    pilot = s.pilot;
    pcd = s.pcd;
    thread = s.thread;
    addConnector("motor", Mount, Solids.axial(0, 0, 0));
    addConnector("shaftAxis", Axis, Solids.axial(0, 0, 0));
    addConnector("body", Mount, Solids.axial(0, 0, s.length));
    var half = pcd / (2 * Math.sqrt(2)), index = 1;
    for (x in [-half, half]) for (y in [-half, half]) addConnector('motorBolt${index++}', Face, Solids.axial(x, y, 0));
  }

  /** Whether `flange` fits the plate's motor face nominally; empty when it does. */
  public function findings(flange:Null<MotorFlange>):Array<String> {
    var half = pcd / (2 * Math.sqrt(2)), spec = HzMotionCatalog.get(model);
    var out = MotorFlanges.findings({pilot: pilot,
      pilotDepth: spec.pilotDepth, bolts: [for (x in [-half, half]) for (y in [
        -half,
        half
      ]) {
      x:x, y:y
    }
    ], thread:thread, threadDepth:spec.threadDepth}, flange);
    // Its drawing's M5 screws pass 6.5 mm servo holes: the joint is not qualified by the interface match.
    out.push("M5 motor fasteners in 6.5 mm motor holes need washer, bearing-stress and joint qualification");
    return out;
  }

  override public function hasGeometry():Bool return true;

  /** Undimensioned recess depth is a visual approximation from the drawing. */
  override public function geometry(detail:ComponentDetail = Preview):Part return Solids.building([],
    tracked -> {function keep(p: Part): Part {
    tracked.push(p);
    return p;
  }
  var plate = keep(Part.prism(
    [
      new Vector(-face / 2, -face / 2),
      new Vector(face / 2, -face / 2),
      new Vector(face / 2, face / 2),
      new Vector(-face / 2, face / 2)
    ],
    0,
    thickness
  ));
  var spigot = keep(Part.cylinderSpan(25, thickness - 0.01, thickness + 3.5));
  if (detail == Envelope) return Solids.union([plate, spigot]);
  var cuts = [keep(Part.cylinderSpan(23, -1, thickness + 5)), keep(Part.cylinderSpan(pilot / 2, -1, 6))];
  var half = pcd / (2 * Math.sqrt(2)), root = Std.parseFloat(thread.substr(1)) == 6 ? 2.5 : 2.1;
  for (x in [-half, half]) for (y in [-half, half]) cuts.push(keep(Part.cylinderSpan(root, -1, thickness + 1, x, y)));
  return Solids.cut(Solids.union([plate, spigot]), cuts);
  });

  static var recipe:Null<ComponentType>;

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.motion.motor-adapter-plate",
      [P.choice("model", models(), "KK86-80EP-adapter")],
      v -> new MotorAdapterPlate(v.token("model")),
      true
    ).numbered(v -> 'HZMOTION-${v.token("model").toUpperCase()}');
    return recipe;
  }

  override public function componentType():Null<ComponentType> return recipeType();

  override public function values():ComponentValues return new ComponentValues().setToken("model", model).setToken(
    "material",
    materialSpec()
  );
}
