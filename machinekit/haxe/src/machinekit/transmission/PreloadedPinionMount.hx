package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Assumed catalogue-class hinged spring-preloaded pinion mount. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class PreloadedPinionMount extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.choice("layout", ["plate", "radial-fork"], "plate"),
    P.length("width", 100),
    P.length("length", 140),
    P.length("thickness", 15),
    P.length("leverArm", 100),
    P.length("springArm", 70),
    P.quantity("springForce", 700, "N"),
    P.quantity("springRate", 40, "N/mm"),
    P.length("stroke", 4),
    P.length("pinionAxialOffset", 0),
    P.length("shaftClearance", 20),
    P.length("hingeBore", 10)
  ];
  public function new(?v:ComponentValues) {
    super("preloaded-pinion-mount", "hinged spring-preloaded pinion mount", "steel", inputs(), v);
    positive([
      "width",
      "length",
      "thickness",
      "leverArm",
      "springArm",
      "springForce",
      "springRate",
      "stroke",
      "shaftClearance",
      "hingeBore"
    ]);
    require(n("leverArm") < n("length") && n("springArm") < n("length"), "lever arms must lie within plate");
    require(n("shaftClearance") < n("width") && n("hingeBore") < n("width"), "mount holes consume plate");
    addConnector("hinge", Axis, Solids.axial(0, 0, 0));
    addConnector(
      "pinion",
      Axis,
      parameterToken("layout") == "radial-fork" ? Solids.axial(
        n("leverArm"),
        0,
        n("thickness") + n("pinionAxialOffset")
      ) : Solids.axial(
        0,
        n("leverArm"),
        n("thickness")
      )
    );
    addConnector(
      "spring",
      Mount,
      parameterToken("layout") == "radial-fork" ? Solids.axial(-n("springArm"), 0, n("thickness")) : Solids.axial(
        0,
        n("springArm"),
        n("thickness")
      )
    );
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.transmission.preloaded-pinion-mount",
      inputs(),
      v -> new PreloadedPinionMount(v)
    ).numbered(v -> 'PIN-MOUNT-ASSUMED-${v.token("layout").toUpperCase()}-${Dimension.format(v.number("springForce"))}N');
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (parameterToken("layout") == "radial-fork") {
      var arm = n("leverArm"), wall = 8.0, depth = n("thickness"), eye = n("shaftClearance") / 2 + 5;
      var spine = Part.box(arm, 12, depth).translated(new Vector(arm / 2, eye + 6, 0));
      var near = Part.box(12, eye + 12, depth).translated(new Vector(0, (eye + 12) / 2, 0));
      var far = Part.box(12, eye + 12, depth).translated(new Vector(arm, (eye + 12) / 2, 0));
      var hinge = Part.cylinder(n("hingeBore") / 2 + 6, depth);
      var shaft = Part.cylinder(eye, depth).translated(new Vector(arm, 0, 0));
      return Solids.cut(
        Solids.union([spine, near, far, hinge, shaft]),
        [
          Part.cylinder(n("hingeBore") / 2, depth + 2).translated(new Vector(0, 0, -1)),
          Part.cylinder(n("shaftClearance") / 2, depth + 2).translated(new Vector(arm, 0, -1))
        ]
      );
    }
    return Solids.cut(
      Solids.named(Part.box(n("width"), n("length"), n("thickness"), Center, Min), "hinged-plate"),
      [
        Solids.named(
          Part.cylinderSpan(n("shaftClearance") / 2, -1, n("thickness") + 1, 0, n("leverArm")),
          "pinion-shaft-clearance"
        ),
        Solids.named(Part.cylinderSpan(n("hingeBore") / 2, -1, n("thickness") + 1), "hinge-bore")
      ]
    );
  }
  /** N at the pinion for spring extension measured in mm. */
  public function preload(extension:Float = 0):Float {
    require(Math.isFinite(extension) && extension >= 0 && extension <= n("stroke"), "spring extension outside stroke");
    return (n("springForce") + n("springRate") * extension) * n("springArm") / n("leverArm");
  }
  public function resists(separatingForce:Float):Bool {
    require(Math.isFinite(separatingForce) && separatingForce >= 0, "invalid separating force");
    return preload() >= separatingForce;
  }
}
