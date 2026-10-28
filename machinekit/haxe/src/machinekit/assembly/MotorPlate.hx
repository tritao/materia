package machinekit.assembly;

import cadkit.modeling.Part;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.Solids;
import machinekit.motion.AxisMotor;
import machinekit.motion.LeadshineServo;
import machinekit.motion.MotorFlange;
import machinekit.motion.NemaStepper;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport as P;

typedef ProfilePlateMount = {
  var systemId:String;
  var railSpacing:Float;
  var alongPitch:Float;
  var holeDiameter:Float;
}

/**
 * A flat plate a flanged motor bolts to, for any motor with a declared `MotorFlange`: a pilot hole and clearance
 * holes on the flange's pattern. Screws go through the plate into a tapped flange; a drilled flange takes through
 * bolts and nuts. Optionally drilled for T-slot profile rails on its far face. Its back at z=0 mates to the motor face.
 */
class MotorPlate extends MachineComponent {
  public static inline var MINIMUM_ENGAGEMENT_DIAMETERS:Float = 1.3;
  public static inline var TIP_CLEARANCE:Float = 0.5;
  public final motor:MachineComponent;
  public final flange:MotorFlange;
  final axis:AxisMotor;
  public final thickness:Float;
  public final size:Float;
  public final margin:Float;
  final profileMount:Null<ProfilePlateMount>;
  static var recipe:Null<ComponentType>;

  public function new(motor:MachineComponent, thickness:Float, margin:Float = 10, ?profileMount:ProfilePlateMount) {
    if (!Math.isFinite(thickness) || thickness <= 0
      || !Math.isFinite(margin) || margin < 0) throw "Motor plate needs a positive thickness and nonnegative margin";
    var axis = AxisMotor.of(motor), flange = axis.flange;
    if (flange.thread == null && flange.screw == null && !(flange.holeDiameter
      > 0)) throw 'Motor ${motor.designation} flange declares neither a thread nor clearance holes';
    var plateSize = flange.face + 2 * margin;
    if (!Math.isFinite(plateSize)) throw "Motor plate size must be finite";
    if (profileMount != null) validateProfileMount(flange, boltRadius(flange), plateSize, profileMount);
    super(
      'PLATE-${motor.designation}-${Dimension.format(thickness)}' + (margin
      == 10 ? "" : "-W" + Dimension.format(plateSize)) + (profileMount == null ? ""
      : '-${profileMount.systemId}-${profileMount.railSpacing}x${profileMount.alongPitch}-D${profileMount.holeDiameter}'),
      'Motor plate for ${motor.designation}',
      "aluminium 6061"
    );
    this.motor = motor;
    this.axis = axis;
    this.flange = flange;
    this.thickness = thickness;
    this.margin = margin;
    this.profileMount = profileMount == null ? null : {
      systemId : profileMount.systemId,
      railSpacing : profileMount.railSpacing,
      alongPitch : profileMount.alongPitch,
      holeDiameter : profileMount.holeDiameter
    };
    size = plateSize;
    addConnector("motor", Mount, Solids.axial(0, 0, 0));
    var i = 1;
    for (point in flange.holes) addConnector('bolt${i++}', Face, Solids.axial(point.x, point.y, thickness));
    i = 1;
    for (point in profileHolePositions()) addConnector(
      'profileBolt${i++}',
      Face,
      Solids.axial(
        point.x,
        point.y,
        thickness
      )
    );
  }

  /** Parallel extrusion rails run along local Y, on the plate's +Z face.
	 * Grid spacing is a nominal interface candidate, not supplier fit certification.
	 */
  static function validateProfileMount(flange:MotorFlange, motorRadius:Float, size:Float, m:ProfilePlateMount):Void {
    var pitch = switch m.systemId {
      case "TSLOT-30-SLOT8":
        30.0;
      case "TSLOT-40-SLOT8":
        40.0;
      default:
        throw "Unsupported profile mounting system";
    };
    if (!Math.isFinite(m.railSpacing + m.alongPitch + m.holeDiameter) || m.railSpacing <= 0 || m.alongPitch <= 0
      || m.holeDiameter <= 0 || Math.abs(m.railSpacing / pitch - Math.fround(m.railSpacing / pitch))
      > 0.000001) throw "Profile rail spacing must be a positive multiple of its grid pitch";
    var radius = m.holeDiameter / 2;
    if (Math.max(m.railSpacing, m.alongPitch) / 2 + radius + 2 > size / 2 || m.alongPitch
      < m.holeDiameter + 2) throw "Profile holes need 2 mm edge and inter-hole ligaments; increase plate margin";
    // Reserve a clear passage between the nominal rail bodies for the motor pilot/shaft.
    if (m.railSpacing - pitch < flange.pilotDiameter + 0.2) throw "Profile rails obstruct the motor pilot passage";
    for (x in [-m.railSpacing / 2, m.railSpacing / 2]) for (y in [-m.alongPitch / 2, m.alongPitch / 2]) {
      if (Math.sqrt(x * x + y * y)
        < radius + (flange.pilotDiameter + 0.2) / 2 + 2) throw "Profile hole overlaps motor pilot clearance";
      for (p in flange.holes) if (Math.sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y))
        < radius + motorRadius + 2) throw "Profile hole overlaps a motor mounting hole";
    }
  }
  public function profileHolePositions():Array<{x:Float, y:Float}> {
    if (profileMount == null) return [];
    var m:ProfilePlateMount = cast profileMount;
    return [for (x in [-m.railSpacing / 2, m.railSpacing / 2]) for (y in [-m.alongPitch / 2, m.alongPitch / 2]) {
      x:x, y:y
    }
    ];
  }
  /** Plate-side radius of a motor bolt hole: the clearance of the flange's screw. */
  static function boltRadius(flange:MotorFlange):Float return SocketHeadCapScrew.metric(
    flange.thread != null ? flange.thread : flange.screw != null ? flange.screw : throughSize(flange),
    10
  ).clearanceDiameter(Medium) / 2;

  /** The largest metric screw size a drilled flange's holes clear. */
  static function throughSize(flange:MotorFlange):String {
    var best:Null<String> = null, bestDiameter = 0.0;
    for (size in SocketHeadCapScrew.catalog().designations()) {
      var spec = SocketHeadCapScrew.catalog().get(size);
      if (spec.clearanceMedium <= flange.holeDiameter && spec.diameter > bestDiameter) {
        best = size;
        bestDiameter = spec.diameter;
      }
    }
    if (best == null) throw "Motor flange holes clear no catalogue screw";
    return best;
  }

  /** The motors a saved plate can name: NEMA stepper variants and Leadshine servos. */
  static function motorModels():Array<String
    > return NemaStepper.variantCatalog().designations().concat(LeadshineServo.catalog().designations());

  static function namedMotor(model:String):MachineComponent return LeadshineServo.catalog().exists(model)
    ? LeadshineServo.model(model) : NemaStepper.model(model);

  function motorModel():Null<String> {
    if (Std.isOfType(motor, NemaStepper)) {
      var stepper:NemaStepper = cast motor;
      return stepper.componentType() == NemaStepper.namedRecipeType() ? stepper.variant.designation : null;
    }
    if (Std.isOfType(motor, LeadshineServo)) return cast(motor, LeadshineServo).rating.designation;
    return null;
  }

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.assembly.motor-plate", [
      P.choice(
        "motor",
        motorModels(),
        "17HS19-1684S1"
      ),
      P.length(
        "thickness",
        6
      ),
      P.length(
        "margin",
        10
      ),
      P.choice(
        "profileSystem",
        [
          "None",
          "TSLOT-30-SLOT8",
          "TSLOT-40-SLOT8"
        ],
        "None"
      ),
      P.length(
        "railSpacing",
        0
      ),
      P.length(
        "alongPitch",
        0
      ),
      P.length(
        "profileHoleDiameter",
        0
      )
    ], v -> {var mount: Null<ProfilePlateMount> = null;
    if (v.token("profileSystem") == "None") {
      if (v.number("railSpacing") != 0 || v.number("alongPitch") != 0
        || v.number("profileHoleDiameter") != 0) throw "Profile hole dimensions require a profile mounting system";
    } else mount = {
      systemId: v.token("profileSystem"),
      railSpacing: v.number("railSpacing"),
      alongPitch: v.number("alongPitch"),
      holeDiameter: v.number("profileHoleDiameter")
    };
    return new MotorPlate(namedMotor(v.token("motor")), v.number("thickness"), v.number("margin"), mount);
    });
    return recipe;
  }
  override public function componentType():Null<ComponentType> return motorModel() != null ? recipeType() : null;
  override public function values():ComponentValues {
    var model = motorModel();
    if (model == null) throw "Custom/generic motor plate requires an explicit interface recipe";
    var m = profileMount;
    return new ComponentValues().setToken("motor",
      model).setNumber("thickness", thickness).setNumber("margin", margin).setToken(
        "profileSystem",
        m == null ? "None" : m.systemId
      ).setNumber(
        "railSpacing",
        m == null ? 0 : m.railSpacing
      ).setNumber(
        "alongPitch",
        m == null ? 0 : m.alongPitch
      ).setNumber(
        "profileHoleDiameter",
        m == null ? 0 : m.holeDiameter
      ).setToken(
        "material",
        materialSpec()
      );
  }

  /** The motor screws into a tapped flange, by the MotorShaftBearings rule: 0.5 mm tip clearance and at least 1.3 diameters of engagement. */
  public function mountScrew():SocketHeadCapScrew {
    if (flange.thread != null) {
      var depth:Float = flange.threadDepth ?? 0;
      var screw = SocketHeadCapScrew.metric(flange.thread, Math.ffloor(thickness + depth - TIP_CLEARANCE));
      if (screw.length - thickness
        < MINIMUM_ENGAGEMENT_DIAMETERS * screw.diameter) throw "Mount screw engagement is too short";
      return screw;
    }
    throw "A drilled motor flange needs through bolts and nuts";
  }

  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var plate = Part.box(size, size, thickness);
    if (detail == Envelope) return plate;
    return Solids.building([plate], owned -> {var tools: Array<Part> = [Part.cylinderSpan(
      (flange.pilotDiameter + 0.2) / 2,
      -0.1,
      thickness + 0.1
    )];
    for (p in flange.holes) tools.push(Part.cylinderSpan(boltRadius(flange), -0.1, thickness + 0.1, p.x, p.y));
    for (tool in tools) owned.push(tool);
    if (profileMount != null) {
      var m:ProfilePlateMount = cast profileMount;
      for (p in profileHolePositions()) {
        var bore = Part.cylinderSpan(m.holeDiameter / 2, -0.1, thickness + 0.1, p.x, p.y);
        owned.push(bore);
        tools.push(bore);
      }
    }
    return Solids.cut(plate, tools);
    });
  }
}
