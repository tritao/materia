package hobber;

import machinekit.hobbing.*;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.motion.*;
import machinekit.structural.GroundStock;
import materia.assembly.AssemblyFrames as F;

/** Host stock and hole patterns; the mechanism and scale remain library
 * module features. The head face stays on its existing work-frame datum. */
class HobberYPart extends MachineComponent {
  static final recipe=new machinekit.cache.Once<ComponentType>();
  public final role:String;
  public final spec:ScrewLinearModuleSpec;
  public static function parameters():Array<ComponentParameter> return [P.choice("role",["base","carriage"],"base")];
  public static function recipeType():ComponentType return recipe.get(()->new ComponentType("machinekit.hobbing.y-module-part",parameters(),v->new HobberYPart(v),false,false,null,false).versioned(22).numbered(v->"HOBBER-Y-MODULE-"+v.token("role").toUpperCase()).procured(v->Make("ground plate, joint drilling")).requiring(v->routeFor(v.token("role")).requirements()));
  /** Ground stock for the role, inspected on receipt; small faces the supplier misses are finished. */
  static function routeFor(role:String):ManufacturingRoute {
    var spec=HobberSkeleton.reference().moduleSpec();
    return ManufacturingRoute.groundStock(GroundStock.plate(role=="base"?GroundStock.STEEL:GroundStock.ALUMINIUM,role=="base"?spec.baseThickness:spec.carriageThickness),
      [{name:"inspect ground stock; supplier-finish failing small faces",tolerance:0.01,roughnessRa:0.8}]);
  }
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);role=v.token("role");spec=HobberSkeleton.reference().moduleSpec();
    super("HOBBER-Y-MODULE-"+role.toUpperCase(),"Y module "+role,role=="base"?"steel":"aluminium 6061",false,v);
    addFacet(routeFor(role));
    addFacet(new EngineeringMetadata(new ComponentValues().setToken("ratingsBasis","Assumed").setToken("errorsBasis","Assumed").setToken("source","Bought ground stock acceptance and small-face finishing remain unqualified")));
    addConnector("mount",Mount,F.identity());addConnector("interface",Mount,F.translation(spec.carriageCentre,0,spec.carriageBottom()+spec.carriageThickness));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function machinable():Bool return true;
  function body():MachineComponent return new ScrewLinearModulePart(spec.values().setToken("role",role));
  public static function actuatorPoint():{x:Float,y:Float,z:Float} return F.transformPoint(F.inverse(HobberSkeleton.reference().modulePose()),HobberSkeleton.reference().swivelNativeX,0,HobberSkeleton.reference().nativeHobCentreZ);
  override public function fasteningFaces():Array<BoltedFace> {
    var out=body().fasteningFaces().filter(face->face.id!="host");
    if(role=="base") {
      var at=actuatorPoint(),bolt=machinekit.standard.SocketHeadCapScrew.catalog().get("M4");
      var length=machinekit.standard.SocketHeadCapScrew.standardLengthAtLeast(spec.baseThickness-at.z-bolt.headHeight+1.25*bolt.diameter);if(length==null)throw "No actuator screw";
      var grip=length-1.25*bolt.diameter;
      out.push(BoltedFace.clamped("actuator-output",ScrewLinearModulePart.plane(at.x,at.y,at.z,1),[for(i in 0...ErobActuatorData.OUTPUT_BOLT_COUNT){x:ErobActuatorPart.outputBolt(i).x,y:-ErobActuatorPart.outputBolt(i).y}],ErobActuatorData.OUTPUT_CLEARANCE_HOLE,grip,"M4",{diameter:bolt.counterboreDiameter,depth:spec.baseThickness-at.z-grip},false));
    }
    if(role=="carriage")for(face in HobberYPart.yHeadFaces())switch face.role {
      case Clamped(size,clearance,grip,recess):
        var holes=face.holes;if(holes==null)throw "Head interface needs its pattern";
        out.push(BoltedFace.clamped(face.id,F.compose(HobberSkeleton.reference().headInModule(),face.frame),holes,clearance,spec.carriageThickness,size,recess,false,true));
      case _:throw "Unexpected head interface";
    }
    return out;
  }
  override public function pinnedFaces():Array<PinnedFace> {
    var out:Array<PinnedFace>=body().pinnedFaces();
    if(role=="carriage")for(face in HobberYPart.yHeadPins())out.push(new PinnedFace(face.id,F.compose(HobberSkeleton.reference().headInModule(),face.frame),face.holes,face.diameter,face.through?spec.carriageThickness:face.depth,face.through,false,face.fit));
    return out;
  }
  static function box(a:Float,b:Float,c:Float,d:Float,e:Float,f:Float):Part {
    var raw=Part.box(b-a,d-c,f-e),shape=raw.translated(new Vector((a+b)/2,(c+d)/2,e));raw.close();return shape;
  }
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var shape:Part;
    if(role=="base") {
      var at=actuatorPoint(),half=spec.wallY(1)+spec.wallWidth/2+16;
      // The lobe overlaps the continuous rail/floor strip; no adapter.
      // The rail and drive floor is the broad face of ONE ground blank.
      shape=Solids.union([box(spec.baseMin,spec.baseMax,-half,half,0,spec.baseThickness),Part.cylinderSpan(ErobActuatorData.BODY_DIAMETER/2,0,spec.baseThickness,at.x,at.y)]);
      shape=Solids.cut(shape,[Part.cylinderSpan(ErobActuatorData.BODY_DIAMETER/2,-1,at.z,at.x,at.y),Part.cylinderSpan((ErobActuatorData.BORE_DIAMETER+0.4)/2,-1,spec.baseThickness+1,at.x,at.y)]);
    } else {
      // The plain rectangle extends toward the head, rather than retaining
      // unused stock above the upper guide or adding a separate adapter.
      var high=spec.carriageMaxY();
      shape=box(spec.carriageCentre-spec.carriageLength/2,spec.carriageCentre+spec.carriageLength/2,high-spec.carriageWidth,high,spec.carriageBottom(),spec.carriageBottom()+spec.carriageThickness);
    }
    if(role=="carriage") {
      var bore=Part.cylinderAlong(Bt30SpindleData.FLANGE_DIAMETER/2+3,new Vector(0,-132,0),Vector.Y(),HobberSkeleton.reference().spindleNativeY-Bt30SpindleData.FLANGE_FROM+134);
      var tool=PosedParts.moved(bore,HobberSkeleton.reference().headInModule());bore.close();shape=Solids.cut(shape,[tool]);
    } else shape=Solids.cut(shape,[HobberSkeleton.reference().sweptDeckRelief()]);
    var made=machined(shape);
    if(made.solidCount()!=1) {
      var fragments=[];
      for(i in 0...made.solidCount()) {
        var piece=new Part(made.shape.subshape(CadKit.ShapeKind.Solid,i));
        fragments.push({volume:piece.volume(),box:PosedParts.boxOf(piece)});piece.close();
      }
      made.close();throw "Disconnected made Y member: "+role+" "+haxe.Json.stringify(fragments);
    }
    return made;
  }

  public static function yHeadFaces():Array<BoltedFace> return [
    BoltedFace.clamped("outboard-carrier",HobberOutboardCarrier.outboardCarrierFrame(1),[for(y in [-8.0,8.0])for(z in [-45.0,45.0]){x:y,y:z}],4.5,12,"M4",null,false,true),
    BoltedFace.clamped("seat",BoltedFace.frameOf({x:HobberSpindleSeat.SEAT_X1,y:0.0,z:0.0},{x:1.0,y:0,z:0},{x:0.0,y:1,z:0}),HobberSpindleSeat.seatBolts(),9,12,"M8",null,false)
  ];
  public static function yHeadPins():Array<PinnedFace> return [HobberOutboardCarrier.outboardCarrierPins(1),PinnedFace.atPoints("seat-pins",BoltedFace.frameOf({x:HobberSpindleSeat.SEAT_X1,y:0.0,z:0.0},{x:1.0,y:0,z:0},{x:0.0,y:1,z:0}),[for(z in [-HobberSpindleSeat.SEAT_PIN_Z,HobberSpindleSeat.SEAT_PIN_Z]){x:HobberSpindleSeat.SEAT_X1+10,y:HobberSpindleSeat.SEAT_PIN_Y,z:z}],4,6,true)];

}
