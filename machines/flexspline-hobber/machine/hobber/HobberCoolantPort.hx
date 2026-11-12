package hobber;

import machinekit.hobbing.*;

import cadkit.modeling.Part;
import machinekit.motion.ErobActuatorPart;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.component.ManufacturingRoute.MachinedFace;
import machinekit.structural.GroundStock;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** GE-coolant; owns its stock, interfaces and manufacturing route. */
/**
 * The coolant nozzle block on the Y carriage: a small block with a bore through it to the nozzle. Recorded in its own frame,
 * its mounting face where the bore comes out (`frameFor`): +Z into the block along the bore. Its sizes are its own design
 * choices, so they are its recipe inputs.
 */
class HobberCoolantPort extends HobberMadePart {
  static final portRecipe = new machinekit.cache.Once<ComponentType>();

  static function inputs():Array<ComponentParameter> return [P.length("width", 24), P.length("height", 10.3), P.length("depth", 12),
    P.length("bore", 4)];

  public static function portType():ComponentType return portRecipe.get(() -> new ComponentType("machinekit.hobbing.coolant-port", inputs(),
    v -> new HobberCoolantPort(v), false, false, null, false).versioned(1).numbered(_ -> "HOBBER-GE-COOLANT")
    .procured(_ -> Make("steel fabrication and machining")).requiringInputs(["width", "height", "depth", "bore"]));

  /** The port's own frame in its native (Y carriage) frame: the mounting face at x 45, centred on the bore, +Z into the block. */
  public static function frameFor():AssemblyFrame return BoltedFace.frameOf({x: 45, y: 64.35, z: 72}, {x: -1, y: 0, z: 0}, {x: 0, y: 0, z: 1});

  public function new(?values:ComponentValues) {
    super("GE-coolant", P.resolve(inputs(), values));
    finish(false);
  }
  override public function componentType():Null<ComponentType> return portType();
  override public function machinable():Bool return false;

  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    var width = b.length("width", values().number("width")), height = b.length("height", values().number("height"));
    var depth = b.length("depth", values().number("depth")), bore = b.length("bore", values().number("bore"));
    var block = b.label(b.block(width, height, depth), "Block");
    var drill = b.cylinderAlong(new Vector(0, 0, 0), Vector.Z(), bore.value / 2, -0.1, depth.value + 0.1);
    return b.label(b.cutAll(block, [drill], ["coolant bore"]), "Coolant bore");
  }
}
