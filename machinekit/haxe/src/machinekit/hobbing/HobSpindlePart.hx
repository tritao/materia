package machinekit.hobbing;

import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Dedicated cartridge parts. Dimensions and fits are assumed design values in mm,
    not catalogue interfaces. Coordinates share the housing datum. */
class HobSpindlePart extends MachineComponent {
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.hobbing.spindle-part",
      EngineeringMetadata.parameters().concat([P.text("role", "shaft")]),
      v -> new HobSpindlePart(v.token("role"), v)
    ).numbered(v -> 'HOB-SPINDLE-${v.token("role").toUpperCase()}').procured(_ -> Make("turning and machining"))
      .requiring(v -> {
        var role = v.token("role");
        var dims = switch role {
          case "shaft": [25.0, 180.0];
          case "drawbar": [6.0, 172.0];
          case "bearingRetainer": [62.0, 17.0];
          case "bearingLocknut": [32.0, 6.0];
          default: throw "unknown spindle cartridge part";
        };
        return [machinekit.component.Procurement.ProcurementCodec.choice("role", role),
          machinekit.component.Procurement.ProcurementCodec.requirement("bodyDiameter", "mm", null, null, dims[0]),
          machinekit.component.Procurement.ProcurementCodec.requirement("bodyLength", "mm", null, null, dims[1])];
      });
    return recipe;
  }
  public function new(role:String, ?overrides:ComponentValues) {
    var v = P.resolve(EngineeringMetadata.parameters().concat([P.text("role", role)]), overrides);
    super("HOB-SPINDLE-" + v.token("role"), "Assumed spindle cartridge " + v.token("role"), "steel", false, v);
    if (["shaft", "motorAdapter", "bearingRetainer",
      "bearingLocknut"].indexOf(v.token("role")) < 0) throw "unknown spindle cartridge part";
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addFacet(new EngineeringMetadata(v));
    addFacet(machinekit.motion.PrecisionErrors.fromValues(v, []));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return switch (values().token("role")) {
      case "shaft":
        // Rear stub for the coupling, bearing journals, shoulder and nose; solid, no drawbar bore.
        var body = Solids.union([
          Part.cylinderSpan(10, -30, 0),
          Part.cylinderSpan(12.5, 0, 163),
          Part.cylinderSpan(15, 163, 170),
          Part.cylinderSpan(20, 170, 180),
          Part.cylinderSpan(2, 178, 182, 14, 0),
          Part.cylinderSpan(2, 178, 182, -14, 0)
        ]);
        // Four M4 arbor-screw holes on a 32 mm PCD, modelled at the thread major diameter.
        var tools = [
          Part.revolve([
            {r: 0.0, z: 160.0},
            {r: 8.0, z: 160.0},
            {r: 10.0, z: 180.0},
            {r: 10.0, z: 180.1},
            {r: 0.0, z: 180.1}
          ])
        ];
        for (i in 0...4) {
          var angle = Math.PI / 4 + i * Math.PI / 2;
          tools.push(Part.cylinderSpan(2, 170, 180.1, 16 * Math.cos(angle), 16 * Math.sin(angle)));
        }
        Solids.cut(body, tools);
      case "motorAdapter":
        // Front flange bolted to the housing rear face (four M4 on Ø72), Ø64 tube with two
        // coupling access windows, and a 90 mm square rear flange for the 80-frame servo
        // (Ø70 pilot recess, four M6 on Ø90). Threads omitted from CAD.
        var holes = [
          Part.cylinderSpan(25, -70.1, 0.1),
          Part.cylinderSpan(35, -70.1, -66.8),
          Part.prism([new Vector(-50, -10), new Vector(50, -10), new Vector(50, 10), new Vector(-50, 10)], -50, -20)
        ];
        for (i in 0...4) {
          var angle = Math.PI / 4 + i * Math.PI / 2;
          holes.push(Part.cylinderSpan(2.25, -8.1, 0.1, 36 * Math.cos(angle), 36 * Math.sin(angle)));
          holes.push(Part.cylinderSpan(3, -70.1, -59.9, 45 * Math.cos(angle), 45 * Math.sin(angle)));
        }
        Solids.cut(Solids.union([
          Part.cylinderSpan(40, -8, 0),
          Part.cylinderSpan(32, -60, -8),
          Part.prism([new Vector(-45, -45), new Vector(45, -45), new Vector(45, 45), new Vector(-45, 45)], -70, -60)
        ]), holes);
      case "bearingRetainer":
        // Front gland: spigot clamping the pair's outer rings at 163, flange on the housing face
        // held by four M4 on Ø72; Ø42 bore round the Ø40 nose as the labyrinth gap.
        var holes = [Part.cylinderSpan(21, 162.9, 186.1)];
        for (i in 0...4) {
          var angle = Math.PI / 4 + i * Math.PI / 2;
          holes.push(Part.cylinderSpan(2.25, 179.9, 186.1, 36 * Math.cos(angle), 36 * Math.sin(angle)));
        }
        Solids.cut(Solids.union([Part.cylinderSpan(31, 163, 180), Part.cylinderSpan(40, 180, 186)]), holes);
      case "bearingLocknut":
        // Catalogue precision locknut envelope, KM5 size (DIN 981 / ISO 2982-2): M25 x 1.5, Ø38 x 7.
        Solids.cut(Part.cylinderSpan(19, 122, 129), [Part.cylinderSpan(12.5, 121.9, 129.1)]);
      default:
        throw "unknown spindle cartridge part";
    }
  }
}
