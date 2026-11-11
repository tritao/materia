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
  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    var skeleton = HobberSkeleton.reference();
    if (role == "base") {
      // The rail and drive floor is the broad face of ONE ground blank; the actuator lobe overlaps it, no adapter.
      var at = actuatorPoint(), half = spec.wallY(1) + spec.wallWidth / 2 + 16;
      var plate = b.label(b.boxBetween(spec.baseMin, spec.baseMax, -half, half, 0, spec.baseThickness), "Ground plate");
      var lobe = b.label(b.cylinderAlong(new Vector(at.x, at.y, 0), Vector.Z(), ErobActuatorData.BODY_DIAMETER / 2, 0, spec.baseThickness), "Actuator lobe");
      var blank = b.label(b.unionAll([plate, lobe]), "Base plate");
      var recess = b.cylinderAlong(new Vector(at.x, at.y, 0), Vector.Z(), ErobActuatorData.BODY_DIAMETER / 2, -1, at.z);
      var bore = b.cylinderAlong(new Vector(at.x, at.y, 0), Vector.Z(), (ErobActuatorData.BORE_DIAMETER + 0.4) / 2, -1, spec.baseThickness + 1);
      var seated = b.label(b.cutAll(blank, [recess, bore], ["actuator recess", "actuator bore"]), "Actuator seat");
      return b.label(b.cutAll(seated, [sweptDeckRelief(b, skeleton)], ["swept deck"]), "Swept deck relief");
    }
    // The plain rectangle extends toward the head, rather than retaining unused stock above the upper guide or adding an adapter.
    var high = spec.carriageMaxY();
    var plate = b.label(b.boxBetween(spec.carriageCentre - spec.carriageLength / 2, spec.carriageCentre + spec.carriageLength / 2, high - spec.carriageWidth, high,
      spec.carriageBottom(), spec.carriageBottom() + spec.carriageThickness), "Ground plate");
    var head = skeleton.headInModule(), o = F.transformPoint(head, 0, -132, 0), y = F.transformVector(head, 0, 1, 0);
    var flange = b.cylinderAlong(new Vector(o.x, o.y, o.z), new Vector(y.x, y.y, y.z), Bt30SpindleData.FLANGE_DIAMETER / 2 + 3, 0,
      skeleton.spindleNativeY - Bt30SpindleData.FLANGE_FROM + 134);
    return b.label(b.cutAll(plate, [flange], ["spindle flange"]), "Spindle flange relief");
  }

  /**
   * The X deck and its chip skirt swept through the Y module's whole relative Z passage and the swivel's +-range (13 positions),
   * in the module frame: what the base must clear. One keep-out body, placed at each swivel angle, fused.
   */
  function sweptDeckRelief(b:cadkit.parametric.recording.DocumentBuilder, skeleton:HobberSkeleton):cadkit.parametric.Feature {
    var drop = skeleton.zTravelMax - skeleton.zTravelMin + 2 * HobberScales.overtravel(), gap = ScrewLinearModuleSpec.GAP;
    var skirt = new HobberChipCover(new ComponentValues().setToken("role", "Xskirt")).geometry(), sb = PosedParts.boxOf(skirt);
    skirt.close();
    var low = skeleton.nativeHobCentreZ + skeleton.zTravelMin - HobberScales.overtravel();
    var hw = HobberXCarriage.xCarriageHalfWidth(), bottom = HobberXCarriage.xCarriageBottom() - low, top = HobberXCarriage.xCarriageTop() - low;
    var deck = b.boxBetween(60 - gap, 386 + gap, -hw - gap, hw + gap, bottom - gap - drop, top + gap);
    var window = b.boxBetween(55 + gap, 140 - gap, -150 + gap, 150 - gap, bottom - gap - drop - 1, top + gap + 1);
    var skirtKeep = b.boxBetween(sb.minX - gap, sb.maxX + gap, sb.minY - gap, sb.maxY + gap, sb.minZ - gap - drop - low, sb.maxZ + gap - low);
    var keep = b.label(b.unionAll([b.cutAll(deck, [window], ["deck window"]), skirtKeep]), "Deck and skirt keep-out");
    var head = skeleton.headInModule(), copies:Array<cadkit.parametric.Feature> = [];
    for (i in 0...13) {
      var angle = -skeleton.swivelRange + 2 * skeleton.swivelRange * i / 12, c = Math.cos(angle), s = Math.sin(angle);
      var pose = F.compose(head, F.fromRotationMatrix(0, 0, 0, [1.0, 0, 0, 0, c, -s, 0, s, c]));
      var o = F.transformPoint(pose, 0, 0, 0), x = F.transformVector(pose, 1, 0, 0), z = F.transformVector(pose, 0, 0, 1);
      copies.push(b.placeIn(keep, new cadkit.modeling.Plane(new Vector(o.x, o.y, o.z), new Vector(x.x, x.y, x.z), new Vector(z.x, z.y, z.z))));
    }
    return b.label(b.unionAll(copies), "Swept deck");
  }

  /** A Y member must be one solid: a relief that splits it is a design error, not a shape to accept. */
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var made = super.geometry(detail);
    if (made.solidCount() != 1) {
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
