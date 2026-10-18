package machinekit.robotics;

import cadkit.modeling.Part;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentFamily;
import machinekit.component.ManufacturingVariant;
import machinekit.component.ComponentRecipeSupport;
import materia.assembly.AssemblyFrames;

/** One functional part family; manufacturing variants own geometry and hardware. */
class OffsetGripperFinger extends MachineComponent {
  public final spec:OffsetFingerSpec;
  public final variant:ManufacturingVariant;
  static var familyCache:Null<ComponentFamily>;
  public static function variants():Array<ManufacturingVariant> return [
    new ManufacturingVariant("cnc-aluminium","cnc-milling","aluminium 6061","prototype",[],
      ["CAM and workholding", "tolerances and bolt preload", "stiffness and gripping-force derating"]),
    new ManufacturingVariant("sls-pa12","sls","PA12 nylon","prototype",
      [{partNumber:"CUSTOM-FINGER-COMPRESSION-SLEEVE-6-4.5-L4.5-F10X1",quantity:2,material:"steel",
        description:"Steel flanged sleeve: OD6 ID4.5, total length4.5, flange OD10 thickness1 mm"}],
      ["supplier print tolerances and powder removal", "sleeve length/seat tolerances and preload", "creep and deflection/load test"])
  ];
  static function lookup(id:String):ManufacturingVariant {
    for(v in variants()) if(v.id==id) return v;
    throw 'Unknown offset finger variant "$id"';
  }
  public function new(left:Bool=true,variantId:String="cnc-aluminium") {
    var definition=lookup(variantId);
    super('OFFSET-FINGER-${left ? "L" : "R"}-$variantId',"Offset gripper finger, "+definition.process,definition.material);
    spec=new OffsetFingerSpec(left);variant=definition;
    addConnector("mount",Mount,AssemblyFrames.translation(spec.mountX,OffsetFingerSpec.MOUNT_Y,spec.mountZ));
    addConnector("contact",Mount,AssemblyFrames.translation(
      OffsetFingerSpec.CENTRE_X+(left ? -13 : 13),40,OffsetFingerSpec.CENTRE_Z));
  }
  public static function family():ComponentFamily {
    if(familyCache==null) {
      var recipe=new ComponentType("machinekit.robotics.offset-gripper-finger",[
        ComponentRecipeSupport.choice("side",["left","right"],"left"),
        ComponentRecipeSupport.choice("variant",["cnc-aluminium","sls-pa12"],"cnc-aluminium")
      ],v -> new OffsetGripperFinger(v.token("side")=="left",v.token("variant")),true,false,null,false);
      familyCache=new ComponentFamily(recipe.id,OffsetFingerSpec.INTERFACE_ID,recipe,variants());
    }
    return familyCache;
  }
  override public function componentType():ComponentType return family().recipe;
  override public function values():ComponentValues return new ComponentValues()
    .setToken("side",spec.left ? "left" : "right").setToken("variant",variant.id);
  override public function setMaterial(material:String):Void {
    if(material!=variant.material) throw "Select a manufacturing variant to change its material";
    super.setMaterial(material);
  }
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part return OffsetFingerGeometry.make(spec.left,variant.id);
  /** Owned sleeve solids; explicitly instantiate them in an assembly/BOM as hardware. */
  public function compressionSleeves():Array<Part> {
    if(variant.id!="sls-pa12") return [];
    var result:Array<Part> = [];
    for(x in spec.holes()) {
      var shaft=Part.cylinderAlongY(3,OffsetFingerSpec.MOUNT_Y,OffsetFingerSpec.MOUNT_Y+3.5,x,spec.mountZ);
      var flange=Part.cylinderAlongY(5,OffsetFingerSpec.MOUNT_Y+3.5,OffsetFingerSpec.MOUNT_Y+4.5,x,spec.mountZ);
      var blank=Part.fuseAll([shaft,flange]);shaft.close();flange.close();
      var hole=Part.cylinderAlongY(2.25,OffsetFingerSpec.MOUNT_Y-1,OffsetFingerSpec.MOUNT_Y+5.5,x,spec.mountZ);
      var sleeve=blank.subtract(hole);blank.close();hole.close();
      result.push(sleeve);
    }
    return result;
  }
}
