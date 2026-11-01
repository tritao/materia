package machinekit.routing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Assumed catalogue-class ER32 collet. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class Collet extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("diameter", 33),
    P.length("length", 40),
    P.length("toolDiameter", 8)
  ];
  public function new(?v:ComponentValues) {
    super("collet", "ER32 collet", "steel", inputs(), v);
    positive(["diameter", "length", "toolDiameter"]);
    require(n("toolDiameter") < n("diameter"), "tool bore exceeds collet body");
    addConnector("holder", Mount, Solids.axial(0, 0, n("length")));
    addConnector("tool", Axis, Solids.axial(0, 0, 0));
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.routing.collet", inputs(), v -> new Collet(v))
      .numbered(v -> 'COLLET-ASSUMED-ER32-${Dimension.format(v.number("toolDiameter"))}MM').procured(_ -> Buy(null, null, null)).requiringInputs(["diameter", "length", "toolDiameter"], [], []);
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  /**
   * An ER collet: the short front cone and the long rear cone either side of its widest ring, the tool bore, and
   * eight slots cut alternately from each end. Envelope detail is its cylinder, which also sets the mass.
   */
  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (detail == Envelope) return envelope();
    var r = n("diameter") / 2, l = n("length"), bore = n("toolDiameter") / 2;
    return Solids.building([], owned -> {var body = Solids.named(
      Part.revolve([{r: bore,
      z: 0.0}, {r: 0.9 * r, z: 0.0}, {r: r, z: 0.22 * l}, {r: 0.62 * r, z: l}, {r: bore, z: l}]),
      "collet"
    );
    owned.push(body);
    var slots:Array<Part> = [];
    for (i in 0...8) {
      var angle = Math.PI * i / 4, front = i % 2 == 0;
      var raw = Part.box(
        1.0,
        r + 1,
        0.75 * l,
        cadkit.modeling.Align.Center,
        cadkit.modeling.Align.Min,
        cadkit.modeling.Align.Min
      );
      var slot = raw.placed(new cadkit.modeling.Location(new cadkit.modeling.Plane(
        new Vector(0, 0, front ? -0.01 : 0.25 * l + 0.01),
        new Vector(Math.cos(angle), Math.sin(angle), 0),
        Vector.Z()
      )));
      raw.close();
      owned.push(slot);
      slots.push(Solids.named(slot, "slot"));
    }
    return Solids.cut(body, slots);
    });
  }

  override public function massGeometry():Null<Part> return envelope();

  function envelope():Part {
    return Solids.cut(
      Solids.named(Part.cylinder(n("diameter") / 2, n("length")), "collet-envelope"),
      [Solids.named(Part.cylinderSpan(n("toolDiameter") / 2, -1, n("length") + 1), "tool-bore")]
    );
  }

}
