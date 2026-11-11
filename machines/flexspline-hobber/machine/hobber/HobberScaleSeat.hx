package hobber;

import machinekit.hobbing.*;

import cadkit.modeling.Part;
import machinekit.motion.ErobActuatorPart;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.component.ManufacturingRoute.MachinedFace;
import machinekit.structural.GroundStock;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** GE-X-scale-seat; owns its stock, interfaces and manufacturing route. */
/**
 * The ground bar a sealed linear scale's housing is bolted to (X or Z). Recorded in its own frame (`frameFor`): the centre of
 * the bar's housing-side face, +Z into the bar, +X along the scale. The housing sits off-centre across the bar, at the
 * `housing` datum. Its four tapped housing holes are a recorded step; its hole table and its bolted face read the same holes.
 */
class HobberScaleSeat extends HobberMadePart {
  static final seatRecipe = new machinekit.cache.Once<ComponentType>();

  /** Its own recipe: which axis; the bar follows from the scale and its housing. */
  public static function seatType():ComponentType return seatRecipe.get(() -> new ComponentType("machinekit.hobbing.scale-seat",
    [P.choice("axis", ["X", "Z"], "X")], v -> new HobberScaleSeat(v.token("axis")), false, false, null, false).versioned(1)
    .numbered(v -> 'HOBBER-GE-${v.token("axis")}-SCALE-SEAT').procured(_ -> Make("steel fabrication and machining"))
    .requiring(_ -> route().requirements()));

  /** The housing face in the part datum frame, +Z into the bar. */
  static function housingFrame(axis:HobberScaleAxis):AssemblyFrame return HobberScales.seatFace(axis, [0.0, 0, 0], HobberScales.SEAT_BAR).frame;

  /** The bar's corners' bounds in `frame` (lows, then highs). */
  static function barBounds(axis:HobberScaleAxis, frame:AssemblyFrame):{lo:Array<Float>, hi:Array<Float>} {
    var toFrame = F.inverse(frame), box = barBox(axis), lo = [1e300, 1e300, 1e300], hi = [-1e300, -1e300, -1e300];
    for (x in [box[0], box[3]]) for (y in [box[1], box[4]]) for (z in [box[2], box[5]]) {
      var p = F.transformPoint(toFrame, x, y, z), v = [p.x, p.y, p.z];
      for (i in 0...3) {
        lo[i] = Math.min(lo[i], v[i]);
        hi[i] = Math.max(hi[i], v[i]);
      }
    }
    return {lo: lo, hi: hi};
  }

  /** The seat's own frame in the part datum frame: the housing face moved to the centre of the bar's face. */
  public static function frameFor(axis:HobberScaleAxis):AssemblyFrame {
    var housing = housingFrame(axis), b = barBounds(axis, housing);
    return F.compose(housing, F.translation((b.lo[0] + b.hi[0]) / 2, (b.lo[1] + b.hi[1]) / 2, b.lo[2]));
  }
  /** The seat bar under a scale housing as a box (it is authored in the work frame, so no origin shift). */
  public static function barBox(axis:HobberScaleAxis):Array<Float> {
    var b = axis.seatBar;
    if (b == null) throw "Scale " + axis.id + " has no seat bar";
    return [b[0], b[2], b[4], b[1], b[3], b[5]];
  }
  public function new(axis:String="X", ?values:ComponentValues) {
    if(axis!="X"&&axis!="Z")throw "Unknown scale seat";
    super("GE-"+axis+"-scale-seat",values,new ComponentValues().setToken("axis",axis));
    finish(true);
  }
  override public function componentType():Null<ComponentType> return seatType();

  /** The housing holes in this bar's frame. */
  function housingHoles():Array<cadkit.parametric.HoleSpec> {
    var toOwn = F.inverse(frameFor(scaleAxis()));
    return [for (hole in HobberScales.seatHoles(scaleAxis(), HobberScales.SEAT_BAR)) RouteHoles.spec(hole, toOwn)];
  }

  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    var frame = frameFor(scaleAxis()), bounds = barBounds(scaleAxis(), frame), lo = bounds.lo, hi = bounds.hi;
    var bar:Null<cadkit.parametric.Feature> = null;
    b.group("Stock", () -> {
      var length = b.length("bar.length", hi[0] - lo[0]).derivedBy("scale housing length plus its end lands");
      var width = b.length("bar.width", hi[1] - lo[1]).fixedBy("ground flat bar width");
      var thickness = b.length("bar.thickness", hi[2] - lo[2]).fixedBy("ground flat bar thickness");
      bar = b.label(b.block(length, width, thickness), "Ground bar");
    });
    var housing = F.transformPoint(F.inverse(frame), housingFrame(scaleAxis()).x, housingFrame(scaleAxis()).y, housingFrame(scaleAxis()).z);
    b.datum("housing", new Plane(new Vector(housing.x, housing.y, housing.z), Vector.X(), Vector.Z()));
    return b.label(b.holes(bar, housingHoles()), "Housing holes");
  }
  function scaleAxis():HobberScaleAxis return designId=="GE-X-scale-seat"?HobberScales.x():HobberScales.z();
  override public function machinable():Bool return false;
  /** The hole table: the recorded housing holes. */
  override public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return [for (hole in housingHoles()) RouteHoles.route(hole, hole.name)];
  override public function manufacturingRoute():Null<ManufacturingRoute> return route();

  static function route():ManufacturingRoute return {
    // Ground bar: the housing's mounting face parallelism comes from the datasheet (0.1 over the length), the grinding gives 0.01.
    ManufacturingRoute.groundStock(GroundStock.bar(HobberScales.SEAT_BAR_WIDTH, HobberScales.SEAT_BAR),
      [{name: "housing seat, tapped holes (ground after cutting)", tolerance: HobberScales.x().housing.mounting().housingParallelism, roughnessRa: null}]);
  };
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace>=[];
    faces.push(HobberScales.seatFace(scaleAxis(), [0.0, 0, 0], HobberScales.SEAT_BAR).at(recordedDatum("housing")));
    return faces;
  }
}
