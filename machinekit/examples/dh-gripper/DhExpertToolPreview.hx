import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import cadkit.modeling.Location;
import cadkit.modeling.Curve;
import cadkit.modeling.Sketch;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import haxe.io.Bytes;

/** Installed-envelope study. Preserves DH exit constraints; Expert flexible lead dimensions are estimates. */
class DhExpertToolPreview {
  static function remove(id:String):Bool return id.indexOf("electrical-cable")>=0 || id.indexOf("cable-end-connector")>=0
    || StringTools.startsWith(id,"support-") || StringTools.startsWith(id,"return-");
  static function digest(bytes:Bytes):String {
    var hash=haxe.crypto.Sha256.make(bytes),out=new StringBuf();
    for(i in 0...hash.length)out.add(StringTools.hex(hash.get(i),2).toLowerCase());return out.toString();
  }
  static function box(x:Float,y:Float,z:Float,w:Float,d:Float,h:Float):Part {
    var p=Part.box(w,d,h),r=p.translated(new Vector(x,y,z));p.close();return r;
  }
  static function sweep(curves:Array<Curve>,start:Vector,tangent:Vector,r:Float):Part {
    var wire=Curve.wire(curves),normal=Math.abs(tangent.z)<0.9 ? Vector.Z() : Vector.X();
    var profile=Sketch.circle(r,new Plane(start,normal,tangent));
    var p=profile.sweep(wire);profile.close();wire.close();for(c in curves)c.close();
    if(!p.valid() || p.solidCount()!=1) throw "Invalid service sweep";
    return p;
  }
  static var fastenerTrials:Array<{id:String,partNumber:String,overlapMm3:Float}> = [];
  static function screwTrial(part:Part,size:String,length:Float,seat:Vector,id:String,?cable:Shape):Void {
    var screw=machinekit.standard.SocketHeadCapScrew.metric(size,length),raw=screw.geometry(),bolt=raw.translated(seat);raw.close();
    var common=part.shape.common(bolt.shape),volume=common.subshapeCount(CadKit.ShapeKind.Solid)>0 ? common.volume() : 0;common.close();
    if(cable!=null){var hit=cable.common(bolt.shape);if(hit.subshapeCount(CadKit.ShapeKind.Solid)>0)volume+=hit.volume();hit.close();}
    bolt.close();fastenerTrials.push({id:id,partNumber:screw.bom.partNumber,overlapMm3:volume});
    if(volume>0.001)throw 'Fastener access or cable-bore interference at $id';
  }
  static var lastEnd:Vector = Vector.X();
  static var routeMetrics:Array<{id:String,minimumRadiusMm:Float,lengthMm:Float}> = [];
  static function arcLead(id:String,start:Vector,tangent:Vector,end:Vector):Part {
    var u=tangent.normalized(),delta=end.subtract(start),h=delta.dot(u),across=delta.subtract(u.scale(h)),q=across.length();
    var v=across.normalized(),radius=(h*h+q*q)/(2*q),angle=2*Math.atan2(q,h);
    if(radius<25) throw "Expert planning radius below R25";
    var middle=start.add(u.scale(radius*Math.sin(angle/2))).add(v.scale(radius*(1-Math.cos(angle/2))));
    var curve=Curve.arc(start,middle,end);lastEnd=Vector.fromNative(curve.shape.tangentAt(1)).normalized();
    routeMetrics.push({id:id,minimumRadiusMm:radius,lengthMm:radius*angle});
    return sweep([curve],start,u,3);
  }
  public static function preview():Bytes {
    var cached=DhGripperPreview.directory()+"/build/dual/dual-electric-tool.mtrg";
    routeMetrics=[];fastenerTrials=[];
    var original=SceneArtifact.decode(Sys.args().indexOf("--cached-dual")>=0 ? sys.io.File.getBytes(cached) : DhDualToolPreview.preview());
    var source=original.assemblyDefinition;if(source==null) throw "Missing dual definition";
    var directory=DhGripperPreview.directory()+"/build/expert";sys.FileSystem.createDirectory(directory);
    // Remove superseded generated service STEP files so audits cannot pick up retired brackets.
    for(file in sys.FileSystem.readDirectory(directory)) if(StringTools.endsWith(file,".step")) sys.FileSystem.deleteFile(directory+"/"+file);
    var model=new AssemblyModel("mm"),parts:Array<SceneArtifactPart> = [],solids:Array<{id:String,shape:Shape}> = [];
    for(o in source.occurrences) if(!remove(o.id)) model.add(o.id,o.initialPose);
    for(d in source.definitions) if(!remove(d.id)) for(c in d.connectors) if(!remove(c.name)) model.connector(d.id,c.name,c.frame);
    for(j in source.joints) if(!remove(j.parent) && !remove(j.child)) model.mateOnAxis(j.id,j.type,j.parent,j.parentConnector,j.child,j.childConnector,j.axis,j.defaultValue,j.limits);
    for(c in source.couplings) model.couple(c.id,c.source,c.target,c.ratio,c.offset);
    for(p in original.parts) if(!remove(p.id)) parts.push(p);
    function add(id:String,name:String,p:Part,color:Array<Float>):Void {
      if(!p.valid() || p.solidCount()!=1) throw 'Invalid or disconnected CAD $id';
      p.exportStep(directory+"/"+id+".step");
      solids.push({id:id,shape:p.shape.cloneShape()});parts.push(DhGripperPreview.render(id,name,p.shape,color));p.close();
      model.add(id);model.connector("adapter",id,AssemblyFrames.identity());model.connector(id,"mount",AssemblyFrames.identity());
      model.mate(id+"-fixed","fixed","adapter",id,id,"mount");
    }
    var dark=[0.08,0.09,0.11],grey=[0.5,0.55,0.6];
    // A: turn in +Y instead of down through the wrist. B: original horizontal return.
    var a=new Vector(-48,0,37),b=new Vector(96,48,60);
    add("a-electrical-cable","A DH lead: 100 mm exit, R55 horizontal return",sweep([
      Curve.line(a,new Vector(-148,0,37)),Curve.arc(new Vector(-148,0,37),new Vector(-203,55,37),new Vector(-148,110,37)),
      Curve.line(new Vector(-148,110,37),new Vector(-108,110,37))],a,new Vector(-1,0,0),3.2),dark);
    add("b-electrical-cable","B DH lead: 100 mm exit, R55 horizontal return",sweep([
      Curve.line(b,new Vector(96,148,60)),Curve.arc(new Vector(96,148,60),new Vector(41,203,60),new Vector(-14,148,60)),
      Curve.line(new Vector(-14,148,60),new Vector(-14,108,60))],b,Vector.Y(),3.2),dark);
    // Smooth connector planning envelopes; actual gender, key and seating not verified.
    add("a-service-coupler","A supplier tail / Expert socket estimated mating envelope",Part.cylinderAlong(8,new Vector(-108,110,37),Vector.X(),24),grey);
    add("b-service-coupler","B supplier tail / Expert socket estimated mating envelope",Part.cylinderAlong(8,new Vector(-14,108,60),new Vector(0,-1,0),24),grey);
    add("expert-splitter","Fairino ICACX1500GSCEXP molded splitter estimated envelope",box(-25,65,20,36,44,20),dark);
    add("expert-branch-a","Expert branch A: analytic circular layout",arcLead("branch-a",new Vector(-84,110,37),Vector.X(),new Vector(-30,87,30)),dark);
    add("expert-branch-b","Expert branch B: analytic circular layout",arcLead("branch-b",new Vector(-14,84,60),new Vector(0,-1,0),new Vector(-25,45,40)),dark);
    add("expert-power-lead","Expert external power: tool-side segment to provisional wrist handoff",arcLead("power",new Vector(-43,65,30),new Vector(-1,0,0),new Vector(-45,-42,20)),dark);
    var signalStart=new Vector(-7,65,30);
    add("expert-signal-lead","Expert wrist signal: flexible R30/R37.5 outboard route",sweep([
      Curve.arc(signalStart,new Vector(30.5,102.5,30),new Vector(-7,140,30)),
      Curve.line(new Vector(-7,140,30),new Vector(-90,140,30)),
      Curve.arc(new Vector(-90,140,30),new Vector(-111.2132034356,131.2132034356,30),new Vector(-120,110,30)),
      Curve.line(new Vector(-120,110,30),new Vector(-120,-35,30)),
      Curve.arc(new Vector(-120,-35,30),new Vector(-111.2132034356,-56.2132034356,30),new Vector(-90,-65,30)),
      Curve.line(new Vector(-90,-65,30),new Vector(-80,-65,30))],signalStart,Vector.X(),3),dark);
    routeMetrics.push({id:"signal",minimumRadiusMm:30,lengthMm:83+145+10+67.5*Math.PI});
    add("expert-wrist-elbow","Expert right-angle wrist plug estimated envelope; mating unresolved",
      machinekit.component.Solids.union([Part.cylinderAlong(7,new Vector(-65,-65,-12),Vector.Z(),42),Part.cylinderAlong(6,new Vector(-80,-65,30),Vector.X(),15)]),grey);
    // A low shelf on the exposed adapter edge replaces all cable-loop cantilevers.
    // Two recessed M3 screws use matching blind tap drills in the adapter.
    var saddle=box(-25,65,16,52,54,4),cuts:Array<Part> = [box(-25,65,19,36,44,2)];
    for(x in [-47.0,-3.0]) {
      cuts.push(Part.cylinderSpan(1.7,15,21,x,44));
      cuts.push(Part.cylinderSpan(3,18,21,x,44));
    }
    for(y in [56.0,76.0]) for(x in [-46.0,-4.0]) cuts.push(box(x,y,15,3,10,6));
    saddle=machinekit.component.Solids.cut(saddle,cuts);
    var splitterShape:Shape=null;for(s in solids)if(s.id=="expert-splitter")splitterShape=s.shape;
    for(x in [-47.0,-3.0]) screwTrial(saddle,"M3",10,new Vector(x,44,18),"saddle-M3-"+x,splitterShape);
    add("expert-saddle","Low splitter saddle: two M3 mounts and reusable strap slots",saddle,[0.65,0.5,0.25]);
    // Envelope of a soft 1 mm pad plus two reusable 10 mm straps. No clamp caps.
    add("expert-saddle-pad","Soft splitter pad, 1 mm planning envelope",box(-25,65,19,36,44,1),dark);
    for(y in [56.0,76.0]) {
      var band=machinekit.component.Solids.cut(box(-25,y,15,44,10,26),[box(-25,y,16,42,12,24)]);
      add("expert-strap-"+Std.int(y),"Reusable splitter retention strap: estimated envelope",band,dark);
    }
    var def=model.definition("dh-dual-expert-layout");def.exposedConnectors=source.exposedConnectors;
    for(d in def.definitions)d.collisionHulls=[];
    var state=new AssemblyState(def);state.setJoint("a-"+DhGripperJaws.MASTER,17);state.setJoint("b-"+DhGripperJaws.MASTER,17);
    var purchases=new machinekit.assembly.MachineAssembly();machinekit.robotics.DhDualGripperServices.addPurchases(purchases);
    sys.io.File.saveContent(directory+"/service-bom.json",haxe.Json.stringify(purchases.billOfMaterials().lines(),null,"  "));
    var bytes=SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:parts,assemblyDefinition:def,assemblyState:state.record()});
    sys.io.File.saveBytes(directory+"/dual-expert-tool.mtrg",bytes);
    var renderRows:Array<{id:String,pose:AssemblyFrame,color:Array<Float>}> = [];
    for(o in def.occurrences) for(p in parts) if(p.id==o.definition) {
      sys.io.File.saveBytes(directory+"/"+o.id+".vertices",p.vertices);sys.io.File.saveBytes(directory+"/"+o.id+".indices",p.indices);
      renderRows.push({id:o.id,pose:state.worldPose(o.id),color:[p.red,p.green,p.blue]});
    }
    sys.io.File.saveContent(directory+"/render.json",haxe.Json.stringify(renderRows));
    // Exact solid checks among new static pieces. Contacts/coupled cable ends are reported, not hidden.
    var checks:Array<{a:String,b:String,distanceMm:Float,overlapMm3:Float,expectedAttachment:Bool}> = [];
    var unexpected=0;
    for(i in 0...solids.length) for(j in i+1...solids.length) {
      var x=solids[i],y=solids[j],distance=x.shape.distance(y.shape),volume=0.0;
      if(distance<0.01) {var common=x.shape.common(y.shape);if(common.subshapeCount(CadKit.ShapeKind.Solid)>0) volume=common.volume();common.close();}
      var attachment=x.id=="expert-splitter" && (y.id=="expert-branch-a" || y.id=="expert-branch-b");
      if(volume>0.1 && (!attachment || volume>64))unexpected++;
      checks.push({a:x.id,b:y.id,distanceMm:distance,overlapMm3:volume,expectedAttachment:attachment});
    }
    sys.io.File.saveContent(directory+"/layout-report.json",haxe.Json.stringify({sourceSceneSha256:digest(bytes),status:"Provisional Expert installed-envelope study; not qualified",
      massBudgetStatus:"New harness and saddle mass not qualified; do not reuse baseline dual-tool payload result",
      removed:["Original downward A return","Old return saddles and deep support scaffolding","Tall carrier, exit supports and screw clamp caps"],
      fastenerTrials:fastenerTrials,expertPlanningRadiusTargetMm:25,routeMetrics:routeMetrics,dhStraightExitMm:100,dhBendRadiusMm:55,expertCableRadiusVerified:false,
      robotMatingFrameVerified:false,externalPowerEndpoint:[-45,-42,20],
      purchasedAdapter:machinekit.robotics.DhDualGripperServices.adapter().manufacturerPartNumber,
      unexpectedInterferences:unexpected,attachmentPolicy:"Only the two branch end disks enter the estimated overmold, limited to 64 mm3 each; all other pairs must remain below 0.1 mm3 overlap",
      checks:checks,pending:["Expert cable dimensions, bend radii, pinout and DH mating compatibility","Reusable strap material, retention and saddle strength; two M3x10 mounts are provisional",
        "Wrist OEM seating datum; this elbow is a planning position","External power dress pack across moving joints; no rigid full-arm cable asserted"]},null,"  "));
    for(s in solids)s.shape.close();
    if(unexpected>0)throw 'Unexpected Expert layout interferences: $unexpected; see layout-report.json';
    Sys.println("Expert layout: all new solids valid and connected; no unexpected static service overlap");
    return bytes;
  }
  public static function main():Void {preview();}
}
