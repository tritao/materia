package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.SocketHeadCapScrew;

/** Dimension-driven end-loaded nut. XY is the bearing plane; ridge top Z=0,
 * shoulders Z=-ridgeHeight and bottom Z=-height. X is across the slot.
 * Sharp corners conservatively retain unspecified chamfers. Threads are semantic.
 * A series name alone never establishes a fit or a load rating.
 */
class SlideInSlotNut extends MachineComponent {
  public final width:Float;
  public final length:Float;
  public final height:Float;
  public final ridgeWidth:Float;
  public final ridgeHeight:Float;
  public final size:String;
  static final recipe = new machinekit.cache.Once<ComponentType>();

  public function new(width:Float, length:Float, height:Float, ridgeWidth:Float, ridgeHeight:Float, size:String) {
    if (!Math.isFinite(width + length + height + ridgeWidth + ridgeHeight)
      || width <= 0 || length <= 0 || height <= 0 || ridgeWidth <= 0
      || ridgeWidth >= width || ridgeHeight <= 0 || ridgeHeight >= height) throw "Invalid stepped slot nut dimensions";
    var screw = SocketHeadCapScrew.metric(size, 10);
    if (screw.diameter >= width || screw.diameter >= length) throw "Thread exceeds nut body";
    super(
      'SLIDE-NUT-$size-${width}x${length}x${height}-R${ridgeWidth}x${ridgeHeight}',
      "Dimension-driven slide-in slot nut; fit requires review",
      "steel"
    );
    addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.width = width;
    this.length = length;
    this.height = height;
    this.ridgeWidth = ridgeWidth;
    this.ridgeHeight = ridgeHeight;
    this.size = size;
    addConnector("thread", Mount, Solids.axial(0, 0, 0));
  }
  /** Unperforated conservative envelope for interference checks. Caller owns it. */
  public function fitEnvelope():Part return Solids.building([], owned -> {var base = Part.box(
    width,
    length,
    height - ridgeHeight,
    Center,
    Center,
    Max
  );
  owned.push(base);
  var shifted = Solids.named(base.translated(new cadkit.modeling.Vector(0, 0, -ridgeHeight)), "base");
  owned.push(shifted);
  base.close();
  var ridge = Solids.named(Part.box(ridgeWidth, length, ridgeHeight, Center, Center, Max), "ridge");
  owned.push(ridge);
  return Solids.union([shifted, ridge]);
  });
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (detail == Envelope) return fitEnvelope();
    var screw = SocketHeadCapScrew.metric(size, 10);
    return Solids.building([], owned -> {var body = fitEnvelope();
    owned.push(body);
    var hole = Solids.named(TSlotTableParts.threadBore(screw.diameter / 2, -height - 0.1, 0.1, screw.thread.mating()), "thread");
    owned.push(hole);
    return Solids.cut(body, [hole]);
    });
  }
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.structural.slide-in-slot-nut",
      [
        ComponentRecipeSupport.length("width", 15.9),
        ComponentRecipeSupport.length("length", 15.5),
        ComponentRecipeSupport.length("height", 5.9),
        ComponentRecipeSupport.length("ridgeWidth", 8),
        ComponentRecipeSupport.length("ridgeHeight", 1.9),
        ComponentRecipeSupport.choice("size", ["M3", "M4", "M5", "M6", "M8"], "M6")
      ],
      v -> new SlideInSlotNut(
        v.number("width"),
        v.number("length"),
        v.number("height"),
        v.number("ridgeWidth"),
        v.number("ridgeHeight"),
        v.token("size")
      )
    ).numbered(v -> 'SLIDE-${v.token("size")}-${Dimension.format(v.number("width"))}x${Dimension.format(v.number("length"))}x${Dimension.format(v.number("height"))}-R${Dimension.format(v.number("ridgeWidth"))}x${Dimension.format(v.number("ridgeHeight"))}')
      .procured(_ -> Buy(null, null, null)).requiring(v -> {
        var terms = [machinekit.component.Procurement.ProcurementCodec.choice("thread", v.token("size"))];
        for (name in ["width", "length", "height", "ridgeWidth", "ridgeHeight"])
          terms.push(machinekit.component.Procurement.ProcurementCodec.requirement(name, "mm", null, null, v.number(name)));
        return terms;
      }));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setNumber("width", width).setNumber(
    "length",
    length
  ).setNumber(
    "height",
    height
  ).setNumber(
    "ridgeWidth",
    ridgeWidth
  ).setNumber(
    "ridgeHeight",
    ridgeHeight
  ).setToken(
    "size",
    size
  ).setToken(
    "material",
    materialSpec()
  );
}
