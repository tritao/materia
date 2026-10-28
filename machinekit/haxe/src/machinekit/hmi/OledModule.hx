package machinekit.hmi;

import cadkit.modeling.Footprint;
import cadkit.modeling.FootprintGeometry;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as R;

/**
 * A 2.42 in 128x64 OLED module behind a window in the face plate: the glass against the plate's back, four
 * standoffs to the carrier PCB, and a 4-pin header behind it. Origin at the centre of the panel's front face,
 * so the glass starts `panelThickness` behind it. Every size is assumed (module 70 x 40, glass 62 x 34 with a
 * 56 x 28 viewing window, 12 mm deep) until a real module is measured.
 */
class OledModule extends HmiPart {
  static var recipe:Null<ComponentType>;

  static function inputs():Array<ComponentParameter> return [
    HmiPart.basisParameter("assumed"),
    R.length("windowWidth", 56), R.length("windowHeight", 28), R.length("bezelMargin", 3),
    R.length("glassWidth", 62), R.length("glassHeight", 34), R.length("glassThickness", 1.5),
    R.length("pcbWidth", 70), R.length("pcbHeight", 40), R.length("pcbThickness", 1.6),
    R.length("panelThickness", 3), R.length("standoffGap", 2), R.length("behindDepth", 12),
    R.length("headerWidth", 10.5), R.length("headerHeight", 2.5),
    R.scalar("massGrams", 30, 1)
  ];

  public function new(?v:ComponentValues) {
    super("hmi-oled-module", "2.42 in 128x64 OLED module", "plastic", inputs(), v);
    positive(["windowWidth", "windowHeight", "glassWidth", "glassHeight", "glassThickness", "pcbWidth", "pcbHeight", "pcbThickness", "panelThickness", "standoffGap", "behindDepth", "headerWidth", "headerHeight", "massGrams"]);
    require(n("windowWidth") <= n("glassWidth") && n("windowHeight") <= n("glassHeight"), "the glass is larger than the viewing window");
    require(n("glassWidth") <= n("pcbWidth") && n("glassHeight") <= n("pcbHeight"), "the carrier is larger than the glass");
    require(n("behindDepth") > n("panelThickness") + n("glassThickness") + n("standoffGap") + n("pcbThickness"), "the header has room behind the carrier");
    addConnector("panel", Mount, Solids.axial(0, 0, 0));
    mount(panelCutout(), panelKeepOut(), [], aboveFace(), behindFace(), rearBody(), 1e9);
    declareMass(n("massGrams") / 1000.0, new Vector(0, 0, -behindFace() / 2));
  }

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.hmi.oled-module", inputs(), v -> new OledModule(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();

  function panelCutout():Footprint return Rect(n("windowWidth"), n("windowHeight"));
  function panelKeepOut():Footprint return Rect(n("windowWidth") + 2 * n("bezelMargin"), n("windowHeight") + 2 * n("bezelMargin"));
  function aboveFace():Float return 0.0;
  function behindFace():Float return n("behindDepth");
  function rearBody():Footprint return Rect(n("pcbWidth"), n("pcbHeight"));

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var t = n("panelThickness"), g = n("glassThickness"), gap = n("standoffGap"), p = n("pcbThickness");
    var parts:Array<Part> = [];
    parts.push(Solids.named(box(n("glassWidth"), n("glassHeight"), g, 0.0, 0.0, -t - g), "glass"));
    var pcbTop = -t - g - gap;
    parts.push(Solids.named(box(n("pcbWidth"), n("pcbHeight"), p, 0.0, 0.0, pcbTop - p), "pcb"));
    for (sx in [-1.0, 1.0]) for (sy in [-1.0, 1.0])
      parts.push(Part.cylinderSpan(2.0, pcbTop, -t - g, sx * (n("glassWidth") / 2 - 3.0), sy * (n("glassHeight") / 2 - 3.0)));
    // The header stands on the back of the carrier down to the module's full depth, near the bottom edge.
    var headerTop = pcbTop - p;
    var headerDepth = headerTop + n("behindDepth");
    parts.push(Solids.named(box(n("headerWidth"), n("headerHeight"), headerDepth, 0.0, -n("pcbHeight") / 2 + 4.0, -n("behindDepth")), "header"));
    return Solids.union(parts);
  }

  /** A box centred on (x, y) with its lower face at z0. */
  static function box(w:Float, h:Float, d:Float, x:Float, y:Float, z0:Float):Part {
    var raw = Part.box(w, h, d);
    var moved = raw.translated(new Vector(x, y, z0));
    raw.close();
    return moved;
  }
}
