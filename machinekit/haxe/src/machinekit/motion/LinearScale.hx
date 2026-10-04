package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyDefinition.AssemblyEncoder;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;

/**
 * A linear scale laid along a rail, read by a head on the carriage: `countsPerMillimetre` counts a
 * millimetre of the carriage's travel, with a reference mark at the joint's zero when it has an index.
 * It is a load-side encoder: it sees where the carriage is, whatever the drive stretches or lost.
 * The strip is 10 mm by 2 mm, an assumption. CAD frame: along +Z from z=0 to `length`, like a linear
 * rail. Connectors: `axis` and `mount` at the start of the strip.
 */
class LinearScale extends MachineComponent implements EncoderPart {
  public final countsPerMillimetre:Float;
  public final length:Float;
  public final absolute:Bool;
  public final hasIndex:Bool;
  static inline var WIDTH = 10.0;
  static inline var THICKNESS = 2.0;

  public function new(length:Float, countsPerMillimetre:Float, absolute:Bool = false, hasIndex:Bool = true) {
    if (!(length > 0) || !Math.isFinite(length)) throw "Linear scale needs a positive length";
    if (!(countsPerMillimetre > 0) || !Math.isFinite(countsPerMillimetre)) throw "Linear scale needs a positive resolution";
    super('SCALE-${Dimension.format(countsPerMillimetre)}${absolute ? "A" : "I"}${hasIndex ? "Z" : ""}-${Dimension.format(length)}',
      'Linear scale, ${Dimension.format(countsPerMillimetre)} counts per mm, ${Dimension.format(length)} mm, ${absolute ? "absolute" : "incremental"}'
      + (hasIndex ? " with reference mark" : ""), "aluminium 6061");
    this.length = length;
    this.countsPerMillimetre = countsPerMillimetre;
    this.absolute = absolute;
    this.hasIndex = hasIndex;
    addConnector("axis", Axis, Solids.axial(0, 0, 0));
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
  }

  static var recipeTypeCache:Null<ComponentType>;

  public static function recipeType():ComponentType {
    if (recipeTypeCache == null) recipeTypeCache = new ComponentType("machinekit.motion.linear-scale", [ComponentRecipeSupport.length("length", 300), ComponentRecipeSupport.scalar("countsPerMillimetre", 200),
      ComponentRecipeSupport.flag("absolute", false), ComponentRecipeSupport.flag("index", true)],
      v -> new LinearScale(v.number("length"), v.number("countsPerMillimetre"), v.boolean("absolute"),
      	v.boolean("index")), true);
    return recipeTypeCache;
  }

  override public function componentType():Null<ComponentType>
    return Std.isExactType(this, LinearScale) ? recipeType() : null;

  override public function values():machinekit.component.ComponentValues
    return new machinekit.component.ComponentValues().setNumber("length", length)
      .setNumber("countsPerMillimetre", countsPerMillimetre).setBoolean("absolute", absolute)
      .setBoolean("index", hasIndex).setToken("material", materialSpec());

  public function encoder(id:String, joint:String):AssemblyEncoder {
    var result:AssemblyEncoder = {id: id, joint: joint, kind: absolute ? "absolute" : "incremental", counts: countsPerMillimetre};
    if (hasIndex) result.index = true;
    return result;
  }

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var half = WIDTH / 2, thick = THICKNESS / 2;
    return Solids.named(Part.prism([new Vector(-half, -thick), new Vector(half, -thick), new Vector(half, thick),
      new Vector(-half, thick)], 0, length), "strip");
  }
}
