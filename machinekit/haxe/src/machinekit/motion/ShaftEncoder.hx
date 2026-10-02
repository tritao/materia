package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyDefinition.AssemblyEncoder;

/**
 * A rotary encoder that rides a shaft end, such as a stepper's back shaft: the housing a standard
 * optical encoder gives, reading `countsPerRevolution` quadrature counts a turn, optionally with an
 * index pulse once a turn. On a motor's own joint it is a motor-side encoder, which sees lost steps.
 * Sizes are those of a typical 38 mm housing, assumptions rather than a catalogue part.
 * CAD frame: mounting face at z=0, housing toward +Z, shaft axis along Z. Connectors: `mount` (the
 * face to put against the motor's back) and `axis`.
 */
class ShaftEncoder extends MachineComponent implements EncoderPart {
  public final countsPerRevolution:Float;
  public final absolute:Bool;
  public final hasIndex:Bool;
  public final bodyDiameter:Float;
  public final bodyLength:Float;

  public function new(countsPerRevolution:Float, absolute:Bool = false, hasIndex:Bool = true, bodyDiameter:Float = 38.0,
      bodyLength:Float = 22.0) {
    if (!(countsPerRevolution > 0) || !Math.isFinite(countsPerRevolution)) throw "Shaft encoder needs a positive count per revolution";
    if (!(bodyDiameter > 0) || !(bodyLength > 0)) throw "Shaft encoder needs a positive body size";
    var size = '${Dimension.format(bodyDiameter)}x${Dimension.format(bodyLength)}';
    super('ENCODER-${Dimension.format(countsPerRevolution)}${absolute ? "A" : "I"}${hasIndex ? "Z" : ""}-$size',
      'Shaft encoder, ${Dimension.format(countsPerRevolution)} counts per revolution, ${absolute ? "absolute" : "incremental"}'
      + (hasIndex ? " with index" : ""), "aluminium 6061");
    this.countsPerRevolution = countsPerRevolution;
    this.absolute = absolute;
    this.hasIndex = hasIndex;
    this.bodyDiameter = bodyDiameter;
    this.bodyLength = bodyLength;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("axis", Axis, Solids.axial(0, 0, 0));
  }

  public static function recipeType():machinekit.component.ComponentType
    return machinekit.component.MachineKitAdditionalRecipes.byId("machinekit.motion.shaft-encoder");

  override public function componentType():Null<machinekit.component.ComponentType>
    return Std.isExactType(this, ShaftEncoder) ? recipeType() : null;

  override public function values():machinekit.component.ComponentValues
    return new machinekit.component.ComponentValues().setNumber("countsPerRevolution", countsPerRevolution)
      .setBoolean("absolute", absolute).setBoolean("index", hasIndex).setNumber("bodyDiameter", bodyDiameter)
      .setNumber("bodyLength", bodyLength).setToken("material", materialSpec());

  public function encoder(id:String, joint:String):AssemblyEncoder {
    var result:AssemblyEncoder = {id: id, joint: joint, kind: absolute ? "absolute" : "incremental", counts: countsPerRevolution};
    if (hasIndex) result.index = true;
    return result;
  }

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part
    return Solids.named(Part.cylinderSpan(bodyDiameter / 2, 0, bodyLength), "body");
}
