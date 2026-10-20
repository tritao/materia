package machinekit.routing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class lifting dust shoe. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class DustShoe extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("diameter", 180),
    P.length("spindleClearance", 130),
    P.length("height", 30),
    P.length("brushLength", 60),
    P.length("hoseDiameter", 100),
    P.length("hoseOffset", 130),
    P.length("hoseRise", 40),
    P.quantity("hoseAngle", 0, "rad"),
    P.length("liftStroke", 120),
    P.length("liftBore", 25)
  ];
  public function new(?v:ComponentValues) {
    super("dust-shoe", "lifting dust shoe", "aluminium 6061", inputs(), v);
    positive(["diameter", "spindleClearance", "height", "brushLength", "hoseDiameter", "hoseOffset", "hoseRise", "liftStroke", "liftBore"]);
    require(n("hoseDiameter") > 6, "hose wall consumes bore");
    require(n("spindleClearance") < n("diameter"), "spindle opening exceeds shoe");
    addConnector("mount", Mount, Solids.axial(0, 0, n("height")));
    require(Math.isFinite(n("hoseAngle")), "hose angle must be finite");
    addConnector("hose", Mount, Solids.axial(n("hoseOffset")*Math.cos(n("hoseAngle")), n("hoseOffset")*Math.sin(n("hoseAngle")), n("height") + n("hoseRise")));
    addConnector("lift", Axis, Solids.axial(0, 0, n("height")));
    require(n("hoseOffset")-n("hoseDiameter")/2>=n("spindleClearance")/2+3 && n("hoseOffset")<n("diameter")/2+n("hoseDiameter")/2, "hose must clear spindle and overlap shoe ring");
    liftCylinder();
    addPort({name: "lift_air", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true}
    );
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.routing.dust-shoe", inputs(), v -> new DustShoe(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.building([], parts -> {parts.push(Solids.cut(
      Solids.named(
        Part.cylinderSpan(
          n("diameter") / 2,
          -n("brushLength"),
          n("height")
        ),
        "shoe-brush-envelope"
      ),
      [Solids.named(
        Part.cylinderSpan(
          n("spindleClearance") / 2,
          -n("brushLength") - 1,
          n("height") + 1
        ),
        "spindle-clearance"
      )]
    ));
    parts.push(Solids.cut(
      Solids.named(
        Part.cylinderSpan(
          n("hoseDiameter") / 2,
          n("height") / 2,
          n("height") + n("hoseRise"),
          n("hoseOffset")*Math.cos(n("hoseAngle")),
          n("hoseOffset")*Math.sin(n("hoseAngle"))
        ),
        "hose-neck"
      ),
      [Solids.named(
        Part.cylinderSpan(
          n("hoseDiameter") / 2 - 3,
          n("height") / 2 - 1,
          n("height") + n("hoseRise") + 1,
          n("hoseOffset")*Math.cos(n("hoseAngle")),
          n("hoseOffset")*Math.sin(n("hoseAngle"))
        ),
        "hose-bore"
      )]
    ));
    return Solids.union(parts);
    }
    );
  }
  public function liftCylinder():machinekit
    .pneumatic.PneumaticCylinder return new machinekit.pneumatic.PneumaticCylinder(
      "ISO6432",
      n("liftBore"),
      n("liftStroke"),
      200
    );
  public function liftedFrame():materia.assembly.AssemblyRecord.AssemblyFrame return F.translation(
    0,
    0,
    n("liftStroke")
  );
  override public function io():MachineIo return {inputs:["raised", "lowered"], outputs:["lift"], analogOutputs:[]};

}
