package machinekit.hobbing;

import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;

/** Dedicated cartridge parts. Dimensions and fits are assumed design values in mm,
    not catalogue interfaces. Coordinates share the housing datum. */
class HobSpindlePart extends MachineComponent {
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.spindle-part",EngineeringMetadata.parameters().concat([P.text("role","shaft")]),v->new HobSpindlePart(v.token("role"),v));
    return recipe;
  }
  public function new(role:String,?overrides:ComponentValues) {
    var v=P.resolve(EngineeringMetadata.parameters().concat([P.text("role",role)]),overrides);
    super("HOB-SPINDLE-"+v.token("role"),"Assumed spindle cartridge "+v.token("role"),"steel",false,v);
    if(["shaft","drawbar","bearingRetainer","bearingLocknut"].indexOf(v.token("role"))<0)throw "unknown spindle cartridge part";
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addFacet(new EngineeringMetadata(v));
    addFacet(machinekit.motion.PrecisionErrors.fromValues(v,[]));
  }
  override public function componentType():Null<ComponentType>return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part {
    return switch(values().token("role")) {
      case "shaft":
        var body=Solids.union([Part.cylinderSpan(12.5,0,163),Part.cylinderSpan(15,163,170),Part.cylinderSpan(20,170,180),
          Part.cylinderSpan(2,178,182,14,0),Part.cylinderSpan(2,178,182,-14,0)]);
        Solids.cut(body,[Part.cylinderSpan(3,-0.1,160.1),Part.revolve([{r:0.0,z:160.0},{r:8.0,z:160.0},{r:10.0,z:180.0},{r:10.0,z:180.1},{r:0.0,z:180.1}])]);
      case "drawbar": Solids.union([Part.cylinderSpan(3,0,172),Part.cylinderSpan(6,-4,0)]);
      // Threaded housing gland and shaft locknut; thread helices omitted from CAD.
      case "bearingRetainer": Solids.cut(Part.cylinderSpan(31,163,180),[Part.cylinderSpan(21,162.9,180.1)]);
      case "bearingLocknut": Solids.cut(Part.cylinderSpan(16,123,129),[Part.cylinderSpan(12.5,122.9,129.1)]);
      default: throw "unknown spindle cartridge part";
    }
  }
}
