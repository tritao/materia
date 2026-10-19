import haxe.io.Bytes;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import machinekit.robotics.CobotArm;
import machinekit.control.DigitalSensor;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneArtifact.SceneArtifactPart;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import collisionkit.native.NativeCollisionWorld;
import collisionkit.CollisionPose;
import collisionkit.CollisionGeometry;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.IkOptions;
import robotkit.model.Frame;
import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;

private typedef AccessStation = {var id:String; var pose:AssemblyFrame; var blank:Null<String>; var vise:Bool;}
private typedef AccessHit = {var first:String; var second:String; var distanceMm:Float;}
private typedef AccessResult = {
  var profile:String; var station:String; var side:String; var yawDegrees:Int;
  var geometricClear:Bool; var ikStatus:String; var samples:Int; var hits:Array<AccessHit>;
  var q:Array<Float>; var routeQ:Array<Array<Float>>; var wrist:AssemblyFrame; var path:Array<AssemblyFrame>;
  var blockedAt:Null<AssemblyFrame>; var openingAtSnapshot:Float; var geometryHits:Array<AccessHit>;
}
private typedef AccessBody = {var id:String; var body:Int; var objects:Array<Int>;}

/** Bounded, independent access study. Stored pneumatic layout supplies placement only, never collision answers. */
class TendingElectricAccess {
  static final MARGIN=robotkit.collision.CollisionClearance.MARGIN*1000;
  static final CONTACT_MARGIN=robotkit.collision.CollisionClearance.CONTACT_MARGIN*1000;
  static final STEP=10.0;
  static final VISE_GATE_LIFT=0.0;
  static final VISE_APPROACH=50.0;
  static final APERTURE_RESERVE=1.0;
  // Both members can have 0.15 mm mesh deviation; also cover the hull inflation tolerance.
  static final EXACT_QUERY_RESERVE=0.35;
  static function digest(bytes:Bytes):String {
    var value=haxe.crypto.Sha256.make(bytes),out=new StringBuf();
    for(i in 0...value.length) out.add(StringTools.hex(value.get(i),2).toLowerCase());return out.toString();
  }
  static function cp(f:AssemblyFrame):CollisionPose return new CollisionPose(f.x,f.y,f.z,f.qx,f.qy,f.qz,f.qw);
  static function metric(f:AssemblyFrame):Transform3 return new Transform3(new Vec3(f.x/1000,f.y/1000,f.z/1000),new Quat(f.qx,f.qy,f.qz,f.qw));
  static function yaw(a:Float):AssemblyFrame return {x:0,y:0,z:0,qx:0,qy:0,qz:Math.sin(a/2),qw:Math.cos(a/2)};
  static function shift(f:AssemblyFrame,x:Float,y:Float,z:Float):AssemblyFrame return AssemblyFrames.compose(AssemblyFrames.translation(x,y,z),f);
  static function find(scene:SceneArtifactData,id:String):SceneArtifactPart {
    for(p in scene.parts) if(p.id==id) return p;throw 'Missing scene mesh $id';
  }
  static function readNumbers(value:Dynamic):Array<Float> {
    var values:Array<Dynamic> = cast value;
    var result=[for(v in values)Std.parseFloat(Std.string(v))];
    for(v in result)if(!Math.isFinite(v))throw "Saved route contains a non-finite coordinate";
    return result;
  }
  static function readFrame(value:Dynamic):AssemblyFrame {
    function n(key:String):Float return Std.parseFloat(Std.string(Reflect.field(value,key)));
    return {x:n("x"),y:n("y"),z:n("z"),qx:n("qx"),qy:n("qy"),qz:n("qz"),qw:n("qw")};
  }
  static function requirePose(a:AssemblyFrame,b:AssemblyFrame,label:String):Void {
    var dx=a.x-b.x,dy=a.y-b.y,dz=a.z-b.z;
    var dot=Math.abs(a.qx*b.qx+a.qy*b.qy+a.qz*b.qz+a.qw*b.qw);
    var na=a.qx*a.qx+a.qy*a.qy+a.qz*a.qz+a.qw*a.qw,nb=b.qx*b.qx+b.qy*b.qy+b.qz*b.qz+b.qw*b.qw;
    if(!(Math.abs(na-1)<0.000001 && Math.abs(nb-1)<0.000001 &&
      Math.sqrt(dx*dx+dy*dy+dz*dz)<=0.06 && 2*Math.acos(Math.min(1,dot))<=0.0003))
      throw 'Saved approach needs new IK: $label';
  }
  static function progress(s:String):Void {Sys.println(s);Sys.stdout().flush();}
  static function nearSeat(a:AssemblyFrame,b:AssemblyFrame):Bool {
    var dx=a.x-b.x,dy=a.y-b.y,dz=a.z-b.z;
    return Math.sqrt(dx*dx+dy*dy+dz*dz)<=20+MARGIN;
  }
  static function mechanical(id:String):Bool return id!="powerSupply" && !StringTools.startsWith(id,"driver");
  static function mesh(p:SceneArtifactPart):CollisionGeometry {
    var vertices:Array<Float> = [],indices:Array<Int> = [];
    for(i in 0...p.vertexCount*3) vertices.push(p.vertices.getDouble(i*8));
    for(i in 0...p.indexCount) indices.push(p.indices.getInt32(i*4));
    return Mesh(vertices,indices);
  }
  static function copyPart(p:SceneArtifactPart,id:String,name:String,red:Bool):SceneArtifactPart return {
    id:id,name:name,red:red ? 0.9 : p.red,green:red ? 0.18 : p.green,blue:red ? 0.12 : p.blue,
    physicalMassKg:0,physicalMassDeclared:true,vertexCount:p.vertexCount,indexCount:p.indexCount,
    vertices:p.vertices,normals:p.normals,indices:p.indices,edgeSegments:p.edgeSegments,edgeIds:p.edgeIds,faceRanges:p.faceRanges
  };
  static function snapshot(path:String,fixtures:SceneArtifactData,fs:AssemblyState,arm:SceneArtifactData,as_:AssemblyState,
    placement:AssemblyFrame,tool:SceneArtifactData,ts:AssemblyState,wrist:AssemblyFrame,hits:Array<AccessHit>,includeArm:Bool,hidden:Null<String>,?work:TendingElectricApproach):Void {
    var parts:Array<SceneArtifactPart> = [],model=new AssemblyModel("mm");
    function add(scene:SceneArtifactData,state:AssemblyState,prefix:String,root:AssemblyFrame):Void {
      for(o in state.definition.occurrences) {
        if((prefix=="arm/" && !mechanical(o.id)) || (prefix=="cell/" && o.id==hidden)) continue;
        var id=prefix+o.id,red=false;
        for(hit in hits) if(hit.first==id || hit.second==id) red=true;
        var p=find(scene,o.definition);
        parts.push(copyPart(p,id,id,red));
        var placed=prefix=="tool/" && work!=null && work.stockFixed() && o.id==work.side+"-blank-60"
          ? work.stockPose(state) : AssemblyFrames.compose(root,state.worldPose(o.id));
        model.add(id,placed);model.ground(id);
      }
    }
    add(fixtures,fs,"cell/",AssemblyFrames.identity());
    if(includeArm) add(arm,as_,"arm/",placement);
    add(tool,ts,"tool/",wrist);
    var definition=model.definition("electric-access-static-snapshot");for(d in definition.definitions) d.collisionHulls=[];
    sys.io.File.saveBytes(path,SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:parts,
      assemblyDefinition:definition,assemblyState:model.initialState("electric-access-static-snapshot").record()}));
  }
  /** Same physical 60 x 60 x 20 blank, turned within the existing finger pair for top entry. */
  static function topEntry(tool:SceneArtifactData,offset:Array<Float>):Void {
    var blank=topBlank(),placed=blank.shape.translate(new Vector(offset[0],offset[1],offset[2]).native());blank.close();
    for(id in ["a-blank-60","b-blank-60"]) {
      var rendered=DhGripperPreview.render(id,"Top-entry 60 x 60 x 20 blank",placed,[0.55,0.67,0.78]);
      for(i in 0...tool.parts.length) if(tool.parts[i].id==id) tool.parts[i]=rendered;
    }
    placed.close();
  }
  public static function topBlank():Part {
    var raw=Part.box(60,20,60);
    var out=raw.translated(new Vector(DhCustomFingers.CENTRE_X,40,DhCustomFingers.CENTRE_Z-30));raw.close();return out;
  }
  static function path(from:AssemblyFrame,to:AssemblyFrame):Array<AssemblyFrame> {
    var dx=to.x-from.x,dy=to.y-from.y,dz=to.z-from.z;
    var n=Std.int(Math.ceil(Math.sqrt(dx*dx+dy*dy+dz*dz)/STEP));if(n<1) n=1;
    return [for(i in 0...n+1) shift(from,dx*i/n,dy*i/n,dz*i/n)];
  }
  static function route(wrist:AssemblyFrame,station:AccessStation,tool:SceneArtifactData,ts:AssemblyState,cell:EnclosedBenchMill,side:String):Array<AssemblyFrame> {
    var approach=AssemblyFrames.transformVector(ts.worldPose(side+"-pgc-140-50"),0,-1,0);
    var direction=AssemblyFrames.transformVector(wrist,approach.x,approach.y,approach.z);
    var approachDistance=station.vise ? VISE_APPROACH : 100.0;
    var start=shift(wrist,approachDistance*direction.x,approachDistance*direction.y,approachDistance*direction.z);
    if(!station.vise) return path(start,wrist);
    // Fit the complete open-jaw tool and service envelope through the actual
    // aperture, then sample its crossing and transfer to the vise approach.
    var low=[1e30,1e30,1e30],high=[-1e30,-1e30,-1e30];
    var relative=shift(wrist,-station.pose.x,-station.pose.y,-station.pose.z);
    for(o in ts.definition.occurrences) {
      var p=find(tool,o.definition),pose=AssemblyFrames.compose(relative,ts.worldPose(o.id));
      for(i in 0...p.vertexCount) {
        var point=AssemblyFrames.transformPoint(pose,p.vertices.getDouble(i*24),p.vertices.getDouble(i*24+8),p.vertices.getDouble(i*24+16));
        low[0]=Math.min(low[0],point.x);low[1]=Math.min(low[1],point.y);low[2]=Math.min(low[2],point.z);
        high[0]=Math.max(high[0],point.x);high[1]=Math.max(high[1],point.y);high[2]=Math.max(high[2],point.z);
      }
    }
    // Mesh extrema can understate analytic cable bounds by the tessellation tolerance.
    // Keep a 1 mm planning reserve beyond the required 5 mm air clearance.
    var apertureMargin=MARGIN+APERTURE_RESERVE;
    var x0=cell.opening.x-cell.openingWidth/2-low[0]+apertureMargin,x1=cell.opening.x+cell.openingWidth/2-high[0]-apertureMargin;
    var z0=cell.opening.z-low[2]+apertureMargin,z1=cell.opening.z+cell.openingHeight-high[2]-apertureMargin;
    if(x0>x1 || z0>z1) return []; // conservative whole-envelope aperture screen
    var x=(x0+x1)/2,z=Math.max(z0,Math.min(station.pose.z+VISE_GATE_LIFT,z1));
    function at(x:Float,y:Float,z:Float):AssemblyFrame return shift(wrist,x-station.pose.x,y-station.pose.y,z-station.pose.z);
    var outside=at(x,cell.opening.y-high[1]-50,z),inside=at(x,cell.opening.y-low[1]+50,z);
    // Align laterally beside the parked head before advancing. A diagonal transfer
    // brings the companion blank past the head before it has reached this clearance.
    var insideY=cell.opening.y-low[1]+50;
    var lateral=at(station.pose.x,insideY,z);
    // Lift only to the seated stock datum while the wrist is under the header.
    // That clears the bare table; defer the full approach lift until the arm is inside.
    var stockSafeZ=Math.max(z,station.pose.z),clearTable=at(station.pose.x,insideY,stockSafeZ);
    var stageY=Math.max(insideY,station.pose.y-100);
    var stage=at(station.pose.x,stageY,stockSafeZ),transferZ=Math.max(stockSafeZ,station.pose.z+approachDistance);
    var lift=at(station.pose.x,stageY,transferZ),over=at(station.pose.x,station.pose.y,transferZ);
    var points=[outside,inside,lateral,clearTable,stage,lift,over,start,wrist],out:Array<AssemblyFrame> = [];
    for(i in 0...points.length-1) {
      var segment=path(points[i],points[i+1]);for(j in 0...segment.length) if(i==0 || j>0) out.push(segment[j]);
    }
    return out;
  }
  public static function main():Void {
    var clearanceFirst=Sys.args().indexOf("--clearance-first")>=0;
    var snapshotRoot=DhGripperPreview.directory()+"/../machine-tending/build/";
    if(clearanceFirst && Sys.args().indexOf("--plan-saved")<0)throw "Clearance-first planning requires reviewed proposal samples";
    var viseOnly=Sys.args().indexOf("--vise-only")>=0,exactVise=Sys.args().indexOf("--coarse-only")<0;
    if(Sys.args().indexOf("--audit-process")>=0 && Sys.args().indexOf("--plan-saved")<0)
      throw "--audit-process requires --plan-saved and a saved proposal report";
    var setback=0.0;
    for(arg in Sys.args())if(StringTools.startsWith(arg,"--tray-setback="))setback=Std.parseFloat(arg.substr("--tray-setback=".length));
    if(!Math.isFinite(setback) || setback<0 || setback>80)throw "Tray setback study range is 0 to 80 mm";
    var selectedCase:Null<String> = null;
    for(arg in Sys.args())if(StringTools.startsWith(arg,"--case="))selectedCase=arg.substr("--case=".length);
    var sides=["a","b"];
    if(selectedCase!=null) {
      var split=selectedCase.split("/");
      if(split.length!=2 || ["a","b"].indexOf(split[1])<0 || split[0].indexOf("..")>=0)throw "Case must be station/a or station/b";
      sides=[split[1]];
    }
    var directory=DhGripperPreview.directory()+"/../machine-tending/build/"+(viseOnly ? "electric-expert-vise" : "electric-expert-access")
      +(setback==0 ? "" : "-setback-"+setback)+(Sys.args().indexOf("--trays-only")>=0 ? "-trays" : "");
    var baseDirectory=directory;
    if(clearanceFirst)directory+="-clearance-first";
    if(selectedCase!=null)directory+="-case-"+selectedCase.split("/").join("-");
    sys.FileSystem.createDirectory(directory);
    var proposalFile=(clearanceFirst ? baseDirectory : directory)+"/report.json";
    for(arg in Sys.args())if(StringTools.startsWith(arg,"--proposal-report="))proposalFile=arg.substr("--proposal-report=".length);
    var priorRows:Array<Dynamic> = [];
    if(Sys.args().indexOf("--reuse-endpoints")>=0)
      for(folder in ["electric-expert-vise","electric-expert-access",haxe.io.Path.withoutDirectory(baseDirectory)]) {
        var path=DhGripperPreview.directory()+"/../machine-tending/build/"+folder+"/report.json";
        if(sys.FileSystem.exists(path)) {
          var prior:Dynamic=haxe.Json.parse(sys.io.File.getContent(path));var rows:Array<Dynamic> = cast prior.results;
          for(row in rows)priorRows.push(row);
        }
      }
    if(Sys.args().indexOf("--reuse-endpoints")>=0 && sys.FileSystem.exists(directory+"/progress.json")) {
      var progress:Dynamic=haxe.Json.parse(sys.io.File.getContent(directory+"/progress.json"));var rows:Array<Dynamic> = cast progress.results;
      for(row in rows)priorRows.unshift(row);
    }
    var geometryStarted=Sys.time(),geometryCache=new TendingGeometryCache(true);
    progress("Building current dual tool and live cell geometry");
    var cachedExpert=Sys.args().indexOf("--cached-expert")>=0;
    var toolBytes=cachedExpert ? sys.io.File.getBytes(DhGripperPreview.directory()+"/build/expert/dual-expert-tool.mtrg") : DhExpertToolPreview.preview();
    var reference=DhGripperPreview.referenceFrame;
    if(cachedExpert) {
      var saved:Dynamic=haxe.Json.parse(sys.io.File.getContent(DhGripperPreview.directory()+"/build/reference-frame.json"));
      function numbers(name:String):Array<Float> {
        var values:Array<Dynamic> = cast Reflect.field(saved,name);
        return [for(value in values)Std.parseFloat(Std.string(value))];
      }
      reference={offset:numbers("offset"),flange:numbers("flange"),grasp:numbers("grasp")};
    }
    if(reference==null) throw "DH frame was not generated";
    var layout=TendingReachLayout.decode(TendingReachCertificate.JSON).result;
    if(setback!=0) {
      for(tray in layout.trays) {
        tray.frame=shift(tray.frame,0,-setback,0);
        for(slot in tray.slots)slot.frame=shift(slot.frame,0,-setback,0);
      }
      layout.tableFrame=shift(layout.tableFrame,0,-setback,0);
    }
    var cell=new EnclosedBenchMill(false),fixtures=new MachineAssembly();fixtures.include("mill",cell);fixtures.exposePort("mains","mill/mill/powerSupply","mains");
    var vise:machinekit.milling.PneumaticVise=cell.mill.vise;
    var stations:Array<AccessStation> = [];
    var fillOutfeed=Sys.args().indexOf("--plan-saved")>=0 || Sys.args().indexOf("--geometry-only")>=0;
    for(tray in layout.trays) {
      var inventory=new MachineAssembly();
      fixtures.include(tray.id,new PartTray(2,3,60,60,tray.pitchX,tray.pitchY),tray.frame);
      for(slot in tray.slots) {
        var name=slot.name.substr(5),local=AssemblyFrames.compose(AssemblyFrames.inverse(tray.frame),slot.frame);
        if(tray.id=="infeedTray" || fillOutfeed) inventory.addComponent("blank-"+name,new TendingBlank(60,60,20),local);
        fixtures.addComponent(tray.id+"-sensor-"+name,new DigitalSensor("presence"),TendingReach.presenceHeadFrame(slot.frame));
        stations.push({id:tray.id+"-"+name,pose:slot.frame,blank:tray.id=="infeedTray" ? "infeed/blank-"+name : fillOutfeed ? "outfeed/blank-"+name : null,vise:false});
      }
      if(tray.id=="infeedTray" || fillOutfeed) fixtures.include(tray.id=="infeedTray" ? "infeed" : "outfeed",inventory,tray.frame);
    }
    fixtures.include("cellTable",new TendingCellTable(layout.tableWidth,layout.tableDepth,layout.tableTop,layout.tableOffset),layout.tableFrame);
    fixtures.addComponent("armRiser",new TendingCellRiser(layout.riser),AssemblyFrames.translation(layout.armPlacement.x,layout.armPlacement.y,0));
    var fixtureScene=clearanceFirst ? SceneArtifact.decode(sys.io.File.getBytes(snapshotRoot+"electric-simulation-inputs/fixtures.mtrg"))
      : geometryCache.scene("electric-access-fixtures-setback-"+setback+"-inventory-"+fillOutfeed,()->AssemblyPreview.scene(fixtures,"electric-access-fixtures")).data;
    var fs=new AssemblyState(fixtureScene.assemblyDefinition);
    for(i in 0...3) fs.setJoint("mill/mill/"+cell.mill.specs[i].id,cell.loadPosition[i]);
    fs.setJoint("mill/door",cell.doorStroke);fs.setJoint("mill/mill/vise/jaw",0);
    stations.unshift({id:"vise-load",pose:fs.worldConnector("mill/mill/vise/body","load"),blank:null,vise:true});
    stations.unshift({id:"vise-seat",pose:fs.worldConnector("mill/mill/vise/body","seat"),blank:null,vise:true});
    if(viseOnly)stations=[for(station in stations)if(station.vise)station];
    if(Sys.args().indexOf("--trays-only")>=0)stations=[for(station in stations)if(!station.vise)station];
    if(selectedCase!=null) {
      stations=[for(station in stations)if(station.id==selectedCase.split("/")[0])station];
      if(stations.length!=1)throw "Unknown selected station";
    }
    var arm=new CobotArm(Reach1300),armScene=clearanceFirst ? SceneArtifact.decode(sys.io.File.getBytes(snapshotRoot+"electric-simulation-inputs/arm.mtrg"))
      : geometryCache.scene("electric-access-bare-arm",()->AssemblyPreview.scene(arm,"electric-access-bare-arm")).data;
    if(clearanceFirst) {
      var reviewed:Dynamic=haxe.Json.parse(sys.io.File.getContent(snapshotRoot+"electric-cycle-review.json"));
      if(digest(toolBytes)!=reviewed.provenance.toolSha256 || digest(SceneArtifact.encode(armScene))!=reviewed.provenance.armSha256 ||
        digest(SceneArtifact.encode(fixtureScene))!=reviewed.provenance.fixtureSha256)
        throw "Clearance-first planning needs the exact baseline review geometry";
    }
    var armState=new AssemblyState(armScene.assemblyDefinition),physical=AssemblyPhysicalPartView.fromSceneArtifact(armScene);
    var converted=AssemblySimulationBridge.toRobotModel(armScene.assemblyDefinition,physical);
    var carrier=converted.partLinks.get("toolFlange"),base=converted.partLinks.get("baseFlange");
    if(carrier==null || base==null) throw "Bare arm has no flange/base carrier";
    var flange=converted.model.addFrame(new Frame("electric-study-flange",converted.model.links[carrier.link]));
    flange.position=[carrier.offset.x,carrier.offset.y,carrier.offset.z];flange.rotation=[carrier.offset.qx,carrier.offset.qy,carrier.offset.qz,carrier.offset.qw];
    var manipulator=new Manipulator(converted.model,converted.model.links[base.link].id,flange.id);
    var geometrySeconds=Sys.time()-geometryStarted;
    progress('Scene construction: $geometrySeconds seconds');
    var results:Array<AccessResult> = [];
    var fixturePhysical=AssemblyPhysicalPartView.fromSceneArtifact(fixtureScene);
    var topStock=topBlank();DhCustomFingers.checkBlankOrientation(topStock.shape,604);topStock.close();
    var hullCache:Map<String,Array<Array<Float>>> = [];
    var profiles=Sys.args().indexOf("--all-profiles")>=0 ? ["reference-side-entry","tending-top-entry"] : ["tending-top-entry"];
    for(profile in profiles) {
      progress('Screening $profile: ${stations.length} stations, ${sides.length} grippers, four headings');
      var tool=SceneArtifact.decode(toolBytes);if(profile=="tending-top-entry") topEntry(tool,reference.offset);
      var ts=new AssemblyState(tool.assemblyDefinition,tool.assemblyState);
      var exact=exactVise ? new TendingExactCad(fixtures,fs,arm,armState,ts,layout.armPlacement,reference.offset,profile=="tending-top-entry",Sys.args().indexOf("--plan-saved")>=0 ? directory+"/planner-native" : directory) : null;
      var world=new NativeCollisionWorld(),names:Map<Int,String> = [],env:Array<AccessBody> = [],armBodies:Array<AccessBody> = [];
      var toolBody=world.addBody(1),toolObjects:Map<String,Int> = [];
      function toolObject(id:String):Int {
        var value=toolObjects.get(id);if(value==null) throw 'Missing tool collision object $id';return value;
      }
      for(o in ts.definition.occurrences) {
        var rendered=find(tool,o.definition);
        // Stock is a solid cuboid: convex/convex signed penetration is reliable.
        // A mesh surface's zero distance cannot distinguish contact from overlap.
        var geometry=StringTools.endsWith(o.id,"blank-60")
          ? Convex(cadkit.ConvexHullVertices.fromMesh(rendered.vertices,rendered.vertexCount)) : mesh(rendered);
        var object=world.add(toolBody,cp(ts.worldPose(o.id)),geometry);
        names.set(object,"tool/"+o.id);toolObjects.set(o.id,object);
      }
      for(o in fs.definition.occurrences) {
        var p=[for(p in fixturePhysical.parts) if(p.id==o.definition) p][0];
        var d=[for(d in fs.definition.definitions) if(d.id==o.definition) d][0];
        var cached=hullCache.get("cell:"+o.definition),pieces:Array<Array<Float>> = [];
        if(cached!=null) pieces=cached; else {
          var rendered=find(fixtureScene,o.definition);
          pieces=d.collisionHulls==null ? [cadkit.ConvexHullVertices.enclosingFromMesh(rendered.vertices,rendered.vertexCount,0.001,0.001).vertices]
            : AssemblySimulationBridge.collisionPieces(o.id,d,p);
          hullCache.set("cell:"+o.definition,pieces);
        }
        var objects:Array<Int> = [];
        if(pieces.length==0) continue;
        var body=world.addBody(0);world.setBodyStatic(body,true);
        for(piece in pieces) {var object=world.add(body,CollisionPose.identity(),Convex(piece));objects.push(object);names.set(object,"cell/"+o.id);}
        env.push({id:o.id,body:body,objects:objects});
      }
      for(o in armState.definition.occurrences) if(mechanical(o.id)) {
        var p=[for(p in physical.parts) if(p.id==o.definition) p][0],d=[for(d in armState.definition.definitions) if(d.id==o.definition) d][0];
        var cached=hullCache.get("arm:"+o.definition),pieces:Array<Array<Float>> = [];
        if(cached!=null) pieces=cached; else {
          var rendered=find(armScene,o.definition);
          pieces=d.collisionHulls==null ? [cadkit.ConvexHullVertices.enclosingFromMesh(rendered.vertices,rendered.vertexCount,0.001,0.001).vertices]
            : AssemblySimulationBridge.collisionPieces(o.id,d,p);
          hullCache.set("arm:"+o.definition,pieces);
        }
        var objects:Array<Int> = [];
        if(pieces.length==0) continue;
        var body=world.addBody(2);world.setBodyStatic(body,true);world.setBodyRule(toolBody,body,Allow,Allowed);
        for(piece in pieces) {var object=world.add(body,CollisionPose.identity(),Convex(piece));objects.push(object);names.set(object,"arm/"+o.id);}
        for(other in armBodies) world.setBodyRule(body,other.body,Allow,Allowed);
        armBodies.push({id:o.id,body:body,objects:objects});
      }
      // Only the adapter/flange mounting interface is excluded. Cables and the
      // unused gripper still check the wrist and every other arm member.
      for(body in armBodies) if(body.id=="toolFlange") for(object in body.objects)
        world.setObjectRule(toolObject("adapter"),object,Allow,DeclaredContact);
      for(fixture in env) if(fixture.id=="armRiser") for(body in armBodies) if(body.id=="baseFlange")
        world.setBodyRule(fixture.body,body.body,Allow,DeclaredContact);
      function fixturePoses(station:AccessStation):Void {
        fs.setJoint("mill/door",station.vise ? cell.doorStroke : 0);
        fs.setJoint("mill/mill/vise/jaw",Sys.args().indexOf("--plan-saved")>=0 && station.id=="vise-seat" ? vise.grippedPosition : 0);
        for(body in env) {
          world.setBodyPose(body.body,cp(fs.worldPose(body.id)));
          world.setBodyRule(toolBody,body.body,body.id==station.blank ? Allow : Check,Allowed);
        }
      }
      var activeSide="a",lastJawState="";
      function toolPose(wrist:AssemblyFrame,opening:Float):Void {
        var key=activeSide+":"+opening;
        if(key!=lastJawState) {
          ts.setJoint("a-jaw-opening",activeSide=="a" ? opening : 17);
          ts.setJoint("b-jaw-opening",activeSide=="b" ? opening : 17);
          // Only the active tool opens; its companion keeps the held blank.
          // Retaining active-blank occupancy while opening is conservative.
          for(side in ["a","b"]) for(member in ["jaw-left","jaw-right","finger-left","finger-right"]) {
            var id=side+"-"+member;world.attach(toolObject(id),toolBody,cp(ts.worldPose(id)));
          }
          lastJawState=key;
        }
        world.setBodyPose(toolBody,cp(wrist));if(exact!=null)exact.toolPose(wrist);
      }
      function hits(station:AccessStation,side:String,contact:Bool):Array<AccessHit> {
        var out:Array<AccessHit> = [];
        // Cover the tool mesh tessellation error before the native CAD narrow phase.
        var meshReserve=exact!=null && station.vise ? EXACT_QUERY_RESERVE : 0.0;
        for(distance in world.distances(MARGIN+meshReserve)) {
          var a=names.get(distance.a),b=names.get(distance.b),active="tool/"+side+"-blank-60";
          var other=a==active ? b : b==active ? a : "";
          var intentional=contact && other!="" && (station.vise
            ? [for(member in vise.stockContacts()) "cell/mill/mill/vise/"+member].indexOf(other)>=0
            : StringTools.startsWith(other,"cell/"+station.id.substr(0,station.id.lastIndexOf("-",station.id.lastIndexOf("-")-1))+"/seat-"));
          if(intentional && distance.distance>=-0.02) continue;
          var member=StringTools.startsWith(a,"tool/"+side+"-") ? a.substr(("tool/"+side+"-").length)
            : StringTools.startsWith(b,"tool/"+side+"-") ? b.substr(("tool/"+side+"-").length) : "";
          var fixture=StringTools.startsWith(a,"cell/") ? a : StringTools.startsWith(b,"cell/") ? b : "";
          var selectedFixture=station.vise ? StringTools.startsWith(fixture,"cell/mill/mill/vise/")
            : StringTools.startsWith(fixture,"cell/"+station.id.split("-")[0]+"/");
          var closeMember=["pgc-140-50","jaw-left","jaw-right","finger-left","finger-right","blank-60"].indexOf(member)>=0;
          // Reuse the tending planner's scoped working clearance. Unused tools,
          // cables, neighbouring stock and all other fixtures keep the air margin.
          if(contact && closeMember && selectedFixture && distance.distance>=CONTACT_MARGIN+meshReserve) continue;
          if(exact!=null && station.vise) {
            var precise=exact.query(a,b,distance.distance,station.id,side);
            var required=contact && closeMember && selectedFixture ? CONTACT_MARGIN : MARGIN;
            if(precise.overlapMm3<=0.1 && (precise.distanceMm+0.000001>=required || (intentional && precise.distanceMm<0.001)))continue;
            out.push({first:a,second:b,distanceMm:precise.distanceMm});
          } else out.push({first:a,second:b,distanceMm:distance.distance});
          if(out.length>=12) break;
        }
        return out;
      }
      if(Sys.args().indexOf("--plan-saved")>=0) {
        var sourceBytes=sys.io.File.getBytes(proposalFile),saved:Dynamic=haxe.Json.parse(sourceBytes.toString());
        var currentProvenance={toolSha256:digest(toolBytes),fixtureSha256:digest(SceneArtifact.encode(fixtureScene)),
          armSha256:digest(SceneArtifact.encode(armScene)),placementSeedSha256:haxe.crypto.Sha256.encode(TendingReachCertificate.JSON)};
        for(key in ["toolSha256","placementSeedSha256"])
          if(Reflect.field(saved.provenance,key)!=Reflect.field(currentProvenance,key))
            throw 'Saved access geometry is stale ($key): regenerate before planning';
        var planned:Array<Dynamic> = [],rows:Array<Dynamic> = cast saved.results;
        var transportWorlds:Map<String,TendingElectricPlanner> = [];
        var transportRoutes:Map<String,Array<Array<Array<Float>>>> = [];
        for(raw in rows)if(raw.profile==profile && raw.ikStatus=="sampled route solved and clear" && sides.indexOf(Std.string(raw.side))>=0) {
          var rawQ:Array<Dynamic> = cast raw.routeQ,rawPath:Array<Dynamic> = cast raw.path;
          var row={station:Std.string(raw.station),side:Std.string(raw.side),routeQ:[for(q in rawQ)readNumbers(q)],
            path:[for(f in rawPath)readFrame(f)],wrist:readFrame(raw.wrist)};
          if(row.routeQ.length<2 || row.routeQ.length!=row.path.length)throw "Saved approach has inconsistent samples";
          for(q in row.routeQ)if(q.length!=6)throw "Saved approach needs six arm coordinates";
          var matching=[for(value in stations)if(value.id==row.station)value];
          if(matching.length==0)continue;
          var station=matching[0];
          var locatingQ:Array<Array<Float>> = [],clearanceOffset:Null<AssemblyFrame> = null;
          var stopWrist=row.wrist;
          if(clearanceFirst && row.station=="vise-load") {
            var axis=AssemblyFrames.transformVector(fs.worldConnector("mill/mill/vise/body","seat"),1,0,0);
            clearanceOffset={x:axis.x,y:axis.y,z:axis.z,qx:0,qy:0,qz:0,qw:1};
            station={id:station.id,pose:shift(station.pose,axis.x*0.5,axis.y*0.5,axis.z*0.5),blank:station.blank,vise:station.vise};
            for(i in 0...row.path.length) {
              row.path[i]=shift(row.path[i],axis.x*0.5,axis.y*0.5,axis.z*0.5);
              var answer=manipulator.solve(metric(layout.armPlacement).inverse().compose(metric(row.path[i])),row.routeQ[i],
                new IkOptions(0.000000001,0.00000001,200).reaching());
              if(!answer.converged)throw "Clearance insertion IK failed";
              row.routeQ[i]=answer.q;
            }
            row.wrist=shift(row.wrist,axis.x*0.5,axis.y*0.5,axis.z*0.5);
            var seed=row.routeQ[row.routeQ.length-1];locatingQ.push(seed.copy());
            for(i in 1...14) {
              var at=shift(row.wrist,-axis.x*0.05*i,-axis.y*0.05*i,-axis.z*0.05*i);
              var answer=manipulator.solve(metric(layout.armPlacement).inverse().compose(metric(at)),seed,
                new IkOptions(0.000000001,0.00000001,200).reaching());
              if(!answer.converged)throw "Locating envelope IK failed";
              seed=answer.q;locatingQ.push(seed.copy());
            }
            // Publish these as new proposals; the original zero-clearance review is retained.
            raw.routeQ=[for(q in row.routeQ)q.copy()];raw.path=row.path;raw.wrist=row.wrist;raw.locatingQ=[for(q in locatingQ)q.copy()];raw.q=row.routeQ[row.routeQ.length-1].copy();
          }
          // A bounded revolute arm can represent the same posture with an
          // unnecessary extra turn. Shift the WHOLE route, not only its goal,
          // to the nearest equivalent parking branch while retaining all limits.
          var turnOffsets=[for(_ in 0...6)0.0];
          for(j in [0,5]) {
            var limits=manipulator.group.limitsOf(j),best=0.0,cost=Math.abs(row.routeQ[0][j]-layout.parkPose[j]);
            for(offset in [-2*Math.PI,2*Math.PI]) {
              var valid=true;for(q in row.routeQ)if(q[j]+offset<limits.lower || q[j]+offset>limits.upper)valid=false;
              var distance=Math.abs(row.routeQ[0][j]+offset-layout.parkPose[j]);
              if(valid && distance<cost-0.000000001){cost=distance;best=offset;}
            }
            turnOffsets[j]=best;for(q in row.routeQ)q[j]+=best;for(q in locatingQ)q[j]+=best;
          }
          // Saved joint samples are proposals. Validate limits, current CAD FK,
          // compiled-model FK and the live target before fresh collision checks.
          var localBlank=TendingStockDatum.local(reference,profile);
          var desiredWrist=AssemblyFrames.compose(AssemblyFrames.compose(station.pose,yaw(Std.parseFloat(Std.string(raw.yawDegrees))*Math.PI/180)),
            AssemblyFrames.inverse(AssemblyFrames.compose(ts.worldPose(row.side+"-blank-60"),localBlank)));
          requirePose(row.wrist,desiredWrist,row.station+" target changed");
          for(i in 0...row.routeQ.length) {
            var q=row.routeQ[i];
            for(j in 0...6) {
              var limits=manipulator.group.limitsOf(j);
              if(q[j]<limits.lower || q[j]>limits.upper)throw "Saved approach violates current joint limits";
              armState.setJoint("j"+(j+1),q[j]);
            }
            var actual=AssemblyFrames.compose(layout.armPlacement,armState.worldPose("toolFlange"));
            requirePose(actual,row.path[i],row.station+" sample "+i);
            var computed=metric(layout.armPlacement).compose(manipulator.forwardKinematics(q));
            requirePose(actual,{x:computed.translation.x*1000,y:computed.translation.y*1000,z:computed.translation.z*1000,
              qx:computed.rotation.x,qy:computed.rotation.y,qz:computed.rotation.z,qw:computed.rotation.w},"planner/assembly FK");
          }
          fixturePoses(station);var states:Array<TendingElectricPlanner> = [],workingStates:Array<TendingElectricPlanner> = [];
          var prefix=station.vise ? "mill/mill/vise/" : station.id.split("-")[0]+"/";
          var contacts=station.vise ? [for(member in vise.stockContacts())"mill/mill/vise/"+member]
            : [for(o in fs.definition.occurrences)if(StringTools.startsWith(o.id,prefix+"seat-"))o.id];
          var work=new TendingElectricApproach(station.id,row.side,row.wrist,converted.model.links[carrier.link].id);
          if(Sys.args().indexOf("--audit-process")>=0) {
            for(j in 0...6)armState.setJoint("j"+(j+1),row.routeQ[row.routeQ.length-1][j]);
            var jaw=TendingJawStrokeAudit.run(tool,ts,row.wrist,armState,layout.armPlacement,fs,hullCache,station.blank,row.side,prefix,reference.offset);
            planned.push({station:row.station,side:row.side,jawStroke:jaw});
            progress('Jaw stroke ${row.station}/${row.side}: ${jaw.clear}; ${jaw.failure}');continue;
          }
          if(work.stockFixed() && exact!=null) {
            exact.toolPose(row.wrist);
            for(member in contacts) {
              var fit=exact.query("tool/"+row.side+"-blank-60","cell/"+member,0,station.id,row.side);
              if(fit.overlapMm3>0.1)throw 'Stock target overlaps fixture: $member (${fit.overlapMm3} mm3)';
            }
          }
          for(opening in [work.jawOpeningMm]) {
            ts.setJoint("a-jaw-opening",row.side=="a" ? opening : 17);
            ts.setJoint("b-jaw-opening",row.side=="b" ? opening : 17);
            states.push(new TendingElectricPlanner(manipulator,converted,layout.armPlacement,armState,fs,tool,ts,hullCache,station.blank,row.side,false,contacts,prefix,work));
            workingStates.push(new TendingElectricPlanner(manipulator,converted,layout.armPlacement,armState,fs,tool,ts,hullCache,station.blank,row.side,true,contacts,prefix,work));
          }
          progress('Automatic transfer and continuous approach: ${row.station}/${row.side}');
          var insertionStart=row.path.length-Std.int(Math.ceil((station.vise ? VISE_APPROACH : 100.0)/STEP))-1;
          var failedEdge=-1,failedOpening=work.jawOpeningMm,reason:Null<String> = null,started=Sys.time();
          for(i in 1...row.routeQ.length) {
            // The authored final straight approach is a process phase. Do not
            // switch to an air policy halfway through a segment entering that phase.
            var local=i>insertionStart;
            for(state in (local ? workingStates : states))if(!state.edge(row.routeQ[i-1],row.routeQ[i])) {
              failedEdge=i-1;failedOpening=work.jawOpeningMm;reason=state.lastEdgeReason;if(reason==null)reason=state.invalid(row.routeQ[i]);if(reason==null)reason="continuous edge could not be proven clear";break;
            }
            if(failedEdge>=0)break;
          }
          var approachSeconds=Sys.time()-started;
          // A blocked mandatory approach makes free-space search unnecessary.
          var guarded=reason!=null && StringTools.startsWith(reason,"intended stock/fixture contact");
          var key=station.vise ? "door-open" : "door-closed",transport=transportWorlds.get(key);
          if(transport==null) {
            var envelopeState=new AssemblyState(ts.definition,ts.record());envelopeState.setJoint("a-jaw-opening",17);envelopeState.setJoint("b-jaw-opening",17);
            transport=new TendingElectricPlanner(manipulator,converted,layout.armPlacement,armState,fs,tool,envelopeState,hullCache,
              null,"a",false,[],"",TendingElectricApproach.transport(converted.model.links[carrier.link].id));
            transportWorlds.set(key,transport);transportRoutes.set(key,[]);
          }
          var proposals:Array<Array<Array<Float>>> = [],savedRoutes=transportRoutes.get(key);
          for(saved in savedRoutes)proposals.push(saved.concat([row.routeQ[0].copy()]));
          for(pose in layout.poses)if(pose.name=="outside staging" || StringTools.endsWith(pose.name,"-clearance")) {
            var q=pose.position.copy();
            for(j in [0,5])for(offset in [-2*Math.PI,2*Math.PI]) {
              var limits=manipulator.group.limitsOf(j),value=q[j]+offset;
              if(value>=limits.lower && value<=limits.upper && Math.abs(value-layout.parkPose[j])<Math.abs(q[j]-layout.parkPose[j]))q[j]=value;
            }
            proposals.push([layout.parkPose.copy(),q,row.routeQ[0].copy()]);
          }
          var transfer:Dynamic=(failedEdge<0 || guarded) && Sys.args().indexOf("--approaches-only")<0 ? transport.plan(layout.parkPose,row.routeQ[0],transport,proposals)
            : {found:false,failure:failedEdge<0 || guarded ? "transfer search deferred by --approaches-only" : "transfer search skipped: mandatory approach is blocked",waypoints:null,
              iterations:0,nodes:0,edgeChecks:0,checkSeconds:0.0,totalSeconds:0.0,buildSeconds:transport.buildSeconds};
          if(transfer.found==true) {var path:Array<Array<Float>> = cast transfer.waypoints;savedRoutes.push(path);}
          var otherStateClear=false;
          if(transfer.found==true) {
            otherStateClear=true;
            var waypoints:Array<Array<Float>> = cast transfer.waypoints;
            for(i in 1...waypoints.length)if(!states[states.length-1].edge(waypoints[i-1],waypoints[i]))otherStateClear=false;
          }
          var smoke:Dynamic=null;
          if(planned.length==0 && Sys.args().indexOf("--planner-smoke")>=0) {
            var nearby=layout.parkPose.copy();nearby[0]+=0.02;
            smoke=states[0].plan(layout.parkPose,nearby,states[states.length-1]);
            if(smoke.found!=true)throw 'Parking-motion planner control failed: ${smoke.failure}';
            progress("Parking-motion positive control passed in the declared approach state");
          }
          var withdrawal:Dynamic=null,locatingReview:Dynamic=null;
          if(Sys.args().indexOf("--check-withdrawal")>=0) {
            var releasedWrist=row.wrist;
            if(clearanceOffset!=null) {
              var seat=fs.worldConnector("mill/mill/vise/body","seat"),load=fs.worldConnector("mill/mill/vise/body","load");
              var up=AssemblyFrames.transformVector(seat,0,0,1);
              var lift=(load.x-seat.x)*up.x+(load.y-seat.y)*up.y+(load.z-seat.z)*up.z;
              releasedWrist=shift(stopWrist,-up.x*lift,-up.y*lift,-up.z*lift);
            }
            var exitWork=TendingElectricApproach.withdrawal(station.id,row.side,releasedWrist,converted.model.links[carrier.link].id);
            var exitTool=new AssemblyState(ts.definition,ts.record()),exitFixture=new AssemblyState(fs.definition,fs.record());
            exitTool.setJoint(row.side+"-jaw-opening",exitWork.jawOpeningMm);
            // Pickup unclamps before leaving. Placement releases and withdraws
            // before clamping, so the vise cannot drag stock inside closed fingers.
            exitFixture.setJoint("mill/mill/vise/jaw",0);
            var air=new TendingElectricPlanner(manipulator,converted,layout.armPlacement,armState,exitFixture,tool,exitTool,hullCache,station.blank,row.side,false,contacts,prefix,exitWork);
            var local=new TendingElectricPlanner(manipulator,converted,layout.armPlacement,armState,exitFixture,tool,exitTool,hullCache,station.blank,row.side,true,contacts,prefix,exitWork);
            if(locatingQ.length>0) {
              var searchTool=new AssemblyState(ts.definition,ts.record());searchTool.setJoint(row.side+"-jaw-opening",17);
              var search=new TendingElectricPlanner(manipulator,converted,layout.armPlacement,armState,fs,tool,searchTool,hullCache,
                station.blank,row.side,true,contacts,prefix,work,"mill/mill/vise/endStop");
              for(i in 1...locatingQ.length)if(!search.edge(locatingQ[i-1],locatingQ[i]))
                throw "Locating command corridor has a non-stop collision: "+search.invalid(locatingQ[i]);
              for(i in 1...locatingQ.length)if(!local.edge(locatingQ[i],locatingQ[i-1]))
                throw "Locating return corridor is blocked: "+local.invalid(locatingQ[i]);
              search.close();
              locatingReview={commandCorridorClear:true,returnCorridorClear:true,returnPolicy:"reverse executed search samples",clearanceMm:0.5,
                commandTravelMm:0.65,contact:"mill/mill/vise/endStop",forceLimitN:5.0,
                contactThresholdN:0.05,restGapLimitMm:0.01,commandSpeedMmPerSecond:0.02,speedLimitMmPerSecond:0.1,totalPointSpeedLimitMmPerSecond:0.5,timeoutSeconds:90.0,
                numericalPenetrationLimitMm:0.001,stableSamples:10,simulationOnly:true,hardwareQualified:false};
            }
            var guardedEdges:Array<Int> = [],exitFailure:Null<String> = null,exitFailed=-1;
            for(reverse in 0...row.routeQ.length-1) {
              var i=row.routeQ.length-1-reverse,space=i>insertionStart ? local : air;
              if(!space.edge(row.routeQ[i],row.routeQ[i-1])) {
                var failure=space.lastEdgeReason;if(failure==null)failure=space.invalid(row.routeQ[i-1]);
                if(failure!=null && StringTools.startsWith(failure,"intended stock/fixture contact"))guardedEdges.push(i);
                else {exitFailure=failure==null ? "continuous withdrawal edge could not be proven clear" : failure;exitFailed=i;break;}
              }
            }
            withdrawal={stockOwner:exitWork.stockFixed() ? "fixture" : "tool",jawOpeningMm:exitWork.jawOpeningMm,
              viseJawMm:0,ordering:work.stockFixed() ? "grasp, unclamp, withdraw" : "release, withdraw, then clamp",
              freeEdgesClear:exitFailed<0,guardedEdges:guardedEdges,failedEdge:exitFailed,failure:exitFailure,
              complete:exitFailed<0 && guardedEdges.length==0,graspReleaseCertified:false};
            air.close();local.close();
          }
          planned.push({station:row.station,side:row.side,operation:work.stockFixed() ? "pickup" : "place",activeStockOwner:work.stockFixed() ? "fixture" : "tool",jawOpeningMm:work.jawOpeningMm,jointTurnOffsetsRad:turnOffsets,transfer:transfer,approachStateTransferClear:otherStateClear,
            constrainedApproachClear:failedEdge<0,contactPhaseStartSample:insertionStart,failedApproachEdge:failedEdge,approachFailure:reason,failedJawOpeningMm:failedOpening,
            requiresGuardedContact:guarded,withdrawal:withdrawal,locating:locatingReview,rigidQualifications:states[0].rigidQualifications,transportRigidQualifications:transport.rigidQualifications,approachCheckSeconds:approachSeconds,plannerPositiveControl:smoke,routeQ:row.routeQ,
            complete:(transfer.found==true) && otherStateClear && failedEdge<0});
          if(failedEdge>=0) {
            ts.setJoint("a-jaw-opening",row.side=="a" ? failedOpening : 17);
            ts.setJoint("b-jaw-opening",row.side=="b" ? failedOpening : 17);
            var blocked=row.routeQ[failedEdge+1];
            for(j in 0...6)armState.setJoint("j"+(j+1),blocked[j]);
            var wrist=AssemblyFrames.compose(layout.armPlacement,armState.worldPose("toolFlange"));
            snapshot(directory+"/planner-approach-"+row.station+"-"+row.side+".mtrg",fixtureScene,fs,armScene,armState,
              layout.armPlacement,tool,ts,wrist,[],true,station.blank,work);
          }
          for(state in states.concat(workingStates))state.close();
          progress('Transfer: ${transfer.failure}; constrained approach failed edge: $failedEdge; $reason');
        }
        if(clearanceFirst) {
          proposalFile=directory+"/clearance-proposal.json";
          saved.placementStrategy="clearance-first with bounded end-stop locating";
          sourceBytes=haxe.io.Bytes.ofString(haxe.Json.stringify(saved,null,"  "));
          sys.io.File.saveBytes(proposalFile,sourceBytes);
        }
        if(Sys.args().indexOf("--audit-process")>=0) {
          var door=TendingDoorSweepAudit.run(fs,armState,layout.armPlacement,tool,ts,hullCache,layout.parkPose,cell.doorStroke);
          sys.io.File.saveContent(directory+"/process-report.json",haxe.Json.stringify({provenance:currentProvenance,sourceReport:proposalFile,
            sourceReportSha256:digest(sourceBytes),traySetbackMm:setback,results:planned,doorSweep:door},null,"  "));
          progress('Door full sweep at park: ${door.clear}; ${door.failure}');
          if(exact!=null)exact.close();world.dispose();return;
        }
        for(space in transportWorlds)space.close();
        if(planned.length==0)throw "No current solved routes available for automatic planning";
        sys.io.File.saveContent(directory+(Sys.args().indexOf("--approaches-only")>=0 ? "/approach-report.json" : "/planner-report.json"),haxe.Json.stringify({planner:"RRT-Connect",seed:17,vampDeferred:true,
          sourceReport:proposalFile,sourceReportSha256:digest(sourceBytes),geometrySeconds:geometrySeconds,traySetbackMm:setback,inventoryPolicy:"all non-target pockets occupied; transport also includes both held stock and both jaw endpoints",marginMm:MARGIN,workingContactMarginMm:CONTACT_MARGIN,armProxyPolicy:"authored construction pieces, 0.03 mm mesh / 0.05 mm support offset",provenance:currentProvenance,proposalProvenance:saved.provenance,results:planned,
          limitations:["Conservative continuous edges in the modeled geometry; supplier meshes remain surfaces",
            "Pickup: active stock fixed and jaws 25 mm; placement: active stock held and jaws 17 mm; companion stock held conservatively",
            "Grasp closure, release, contact separation and withdrawal are separate transitions, not certified by an approach",
            "Flexible cable motion, real connector dimensions, grip retention, dynamics and execution remain unqualified"]},null,"  "));
        if(exact!=null) {
          sys.io.File.saveContent(directory+"/planner-stock-targets.json",haxe.Json.stringify({purpose:"Exact static stock/fixture fit before pickup",nativeCadDirectory:"planner-native",sources:exact.sources,checks:exact.checks},null,"  "));
          exact.close();
        }
        world.dispose();return;
      }
      for(station in stations) {
        fixturePoses(station);
        for(side in sides) for(heading in 0...4) {
          activeSide=side;
          var centreX=reference.grasp[0],centreZ=reference.grasp[2];
          var blankLocal=TendingStockDatum.local(reference,profile);
          var blankInTool=AssemblyFrames.compose(ts.worldPose(side+"-blank-60"),blankLocal);
          var desiredBlank=AssemblyFrames.compose(station.pose,yaw(heading*Math.PI/2));
          var wrist=AssemblyFrames.compose(desiredBlank,AssemblyFrames.inverse(blankInTool));
          // Independent mesh/unit check: both profiles must still place a flat
          // 60 x 60 x 20 mm blank on the station's bottom datum.
          var stockMesh=find(tool,side+"-blank-60"),stockPose=AssemblyFrames.compose(wrist,ts.worldPose(side+"-blank-60"));
          var lo=[1e30,1e30,1e30],hi=[-1e30,-1e30,-1e30];
          for(i in 0...stockMesh.vertexCount) {
            var p=AssemblyFrames.transformPoint(stockPose,stockMesh.vertices.getDouble(i*24),stockMesh.vertices.getDouble(i*24+8),stockMesh.vertices.getDouble(i*24+16));
            lo[0]=Math.min(lo[0],p.x);lo[1]=Math.min(lo[1],p.y);lo[2]=Math.min(lo[2],p.z);
            hi[0]=Math.max(hi[0],p.x);hi[1]=Math.max(hi[1],p.y);hi[2]=Math.max(hi[2],p.z);
          }
          var expectedLow=[station.pose.x-30,station.pose.y-30,station.pose.z],expectedHigh=[station.pose.x+30,station.pose.y+30,station.pose.z+20];
          for(i in 0...3) if(Math.abs(lo[i]-expectedLow[i])>0.001 || Math.abs(hi[i]-expectedHigh[i])>0.001)
            throw "Stock mesh/target frames or units disagree";
          var result:AccessResult={profile:profile,station:station.id,side:side,yawDegrees:heading*90,
            geometricClear:true,ikStatus:"not attempted",samples:0,hits:[],q:[],routeQ:[],wrist:wrist,path:[],blockedAt:null,openingAtSnapshot:17,geometryHits:[]};
          toolPose(wrist,17);result.hits=hits(station,side,true);result.samples++;
          if(result.hits.length==0) {
            toolPose(wrist,25);result.hits=hits(station,side,true);result.samples++;
          }
          if(result.hits.length==0) {
            result.path=route(wrist,station,tool,ts,cell,side);
            if(result.path.length==0) result.hits=[{first:"screen/complete-tool-envelope",second:"cell/door-aperture",distanceMm:0}];
            for(opening in [17.0,25.0]) {
              for(i in 0...result.path.length) {
                toolPose(result.path[i],opening);var found=hits(station,side,nearSeat(result.path[i],wrist));result.samples++;
                if(found.length>0) {result.hits=found;break;}
              }
              if(result.hits.length>0) break;
            }
          }
          result.geometryHits=result.hits.copy();result.geometricClear=result.hits.length==0;results.push(result);
        }
        var clear=[for(r in results) if(r.profile==profile && r.station==station.id && r.geometricClear) r].length;
        progress('Geometry $profile/${station.id}: $clear of ${sides.length*4} headings clear');
      }
      if(Sys.args().indexOf("--geometry-only")>=0) {
        for(j in 0...6)armState.setJoint("j"+(j+1),layout.parkPose[j]);
        var parked=AssemblyFrames.compose(layout.armPlacement,armState.worldPose("toolFlange"));toolPose(parked,17);
        snapshot(directory+"/layout.mtrg",fixtureScene,fs,armScene,armState,layout.armPlacement,tool,ts,parked,[],true,null);
        sys.io.File.saveContent(directory+"/geometry-report.json",haxe.Json.stringify({status:"Tool/fixture screen only; no IK or arm motion approval",traySetbackMm:setback,results:results},null,"  "));
        if(exact!=null)exact.close();world.dispose();return;
      }
      // Bounded IK and arm/environment checks on the clearest heading at each station.
      // Failures do not trigger placement search or rewrite the pneumatic certificate.
      for(station in stations) for(side in sides) {
        var candidates=[for(r in results) if(r.profile==profile && r.station==station.id && r.side==side) r];
        candidates.sort((a,b)-> a.hits.length-b.hits.length);
        var selected=candidates[0];activeSide=side;fixturePoses(station);
        for(body in armBodies) {world.setBodyStatic(body.body,false);world.setBodyRule(toolBody,body.body,Check,Allowed);}
        var seeds:Array<Array<Float>> = [];
        var wanted=station.id=="vise-seat" ? "vise" : station.id=="vise-load" ? "vise-load" : station.id.split("Tray").join("");
        for(pose in layout.poses) if(pose.name==wanted) seeds.push(pose.position);
        var preferredRows=priorRows.copy();preferredRows.reverse();
        for(previous in preferredRows)if(previous.profile==profile && previous.station==station.id && previous.side==side) {
          var values:Array<Dynamic> = cast previous.q;
          if(values.length==6 && previous.ikStatus!="not attempted")seeds.unshift(readNumbers(previous.q));
        }
        seeds.push(arm.ready());
        var alternative=arm.ready();alternative[0]=Math.PI;seeds.push(alternative);
        var elbow=arm.ready();for(j in 1...5)elbow[j]=-elbow[j];seeds.push(elbow);
        var elbowBack=elbow.copy();elbowBack[0]=Math.PI;seeds.push(elbowBack);
        if(Sys.args().indexOf("--extended-ik-seeds")>=0) {
          var relative=metric(layout.armPlacement).inverse().compose(metric(candidates[0].wrist));
          var bearing=Math.atan2(relative.translation.y,relative.translation.x);
          for(base in [bearing,bearing+Math.PI])for(shoulder in [-Math.PI/2,0.0,Math.PI/2])
            for(elbow in [-Math.PI/2,Math.PI/2])for(roll in [-Math.PI/2,Math.PI/2])for(pitch in [-Math.PI/2,Math.PI/2])
              seeds.push([Math.atan2(Math.sin(base),Math.cos(base)),shoulder,elbow,roll,pitch,0.0]);
        }
        var seed=seeds[0],solved=false,bestHitCount=1000000;
        // A fixture-clear heading may still fold the forearm through the tool.
        // Explore all headings before accepting a colliding endpoint from one seed.
        for(candidate in candidates) {
        if(bestHitCount==0)break;
        var endpointSeeds=seeds.copy(),branchSeedsAdded=false;
        for(initial in endpointSeeds) {
          var target=metric(layout.armPlacement).inverse().compose(metric(candidate.wrist));
          var solution=manipulator.solve(target,initial,new IkOptions(0.0000001,0.000001,200).reaching());
          if(!solution.converged) continue;
          for(j in 0...6) armState.setJoint("j"+(j+1),solution.q[j]);
          var mounted=AssemblyFrames.compose(layout.armPlacement,armState.worldPose("toolFlange"));
          for(body in armBodies) world.setBodyPose(body.body,cp(AssemblyFrames.compose(layout.armPlacement,armState.worldPose(body.id))));
          toolPose(mounted,17);var count=hits(station,side,true).length;
          if(count>0 && !branchSeedsAdded) {
            branchSeedsAdded=true;
            var mirror=solution.q.copy();mirror[1]+=mirror[2];mirror[3]+=mirror[2];mirror[2]=-mirror[2];endpointSeeds.push(mirror);
            var wristFlip=solution.q.copy();wristFlip[3]+=Math.PI;wristFlip[4]=-wristFlip[4];wristFlip[5]+=Math.PI;endpointSeeds.push(wristFlip);
            var both=mirror.copy();both[3]+=Math.PI;both[4]=-both[4];both[5]+=Math.PI;endpointSeeds.push(both);
          }
          if(count<bestHitCount) {selected=candidate;seed=solution.q;solved=true;bestHitCount=count;}
          if(count==0) break;
        }
        }
        var selectedIndex=results.indexOf(selected);
        if(solved) {
          selected.q=seed;selected.ikStatus="endpoint solved";
          for(i in 0...6) armState.setJoint("j"+(i+1),seed[i]);
          var actual=AssemblyFrames.compose(layout.armPlacement,armState.worldPose("toolFlange"));
          var dx=actual.x-selected.wrist.x,dy=actual.y-selected.wrist.y,dz=actual.z-selected.wrist.z;
          var dot=Math.abs(actual.qx*selected.wrist.qx+actual.qy*selected.wrist.qy+actual.qz*selected.wrist.qz+actual.qw*selected.wrist.qw);
          if(Math.sqrt(dx*dx+dy*dy+dz*dz)>0.06 || 2*Math.acos(Math.min(1,dot))>0.0003)
            throw "IK/assembly flange position or orientation disagree";
          for(body in armBodies) world.setBodyPose(body.body,cp(AssemblyFrames.compose(layout.armPlacement,armState.worldPose(body.id))));
          toolPose(actual,17);
          var checked=hits(station,side,true);if(checked.length>0) {selected.hits=checked;selected.geometricClear=false;selected.ikStatus="endpoint solved; conservative clearance flag";}
          if(selected.geometricClear && selected.path.length>0) {
            var template:AccessResult=cast Reflect.copy(selected),endSeeds=[selected.q.copy()];
            // A clear endpoint does not establish a clear approach. Test the other
            // bounded elbow/base seeds too, deduplicating the resulting joint branches.
            var routeInitials=seeds.copy();
            if(station.vise) {
              var mirror=selected.q.copy();mirror[1]+=mirror[2];mirror[3]+=mirror[2];mirror[2]=-mirror[2];routeInitials.unshift(mirror);
            }
            var bestProgress=-1,seedIndex=0,initialIndex=0;
            while(seedIndex<endSeeds.length || (station.vise && initialIndex<routeInitials.length)) {
              if(seedIndex>=endSeeds.length) {
                var initial=routeInitials[initialIndex++];
                var solution=manipulator.solve(metric(layout.armPlacement).inverse().compose(metric(selected.wrist)),initial,
                  new IkOptions(0.0000001,0.000001,200).reaching());
                if(!solution.converged)continue;
                var duplicate=false;
                for(existing in endSeeds) {var difference=0.0;for(j in 0...6)difference=Math.max(difference,Math.abs(Math.atan2(Math.sin(existing[j]-solution.q[j]),Math.cos(existing[j]-solution.q[j]))));if(difference<0.01)duplicate=true;}
                if(duplicate)continue;
                endSeeds.push(solution.q.copy());
              }
              var endSeed=endSeeds[seedIndex++];
              var trial:AccessResult=cast Reflect.copy(template);trial.hits=[];trial.q=endSeed.copy();trial.routeQ=[];
              seed=endSeed.copy();
              if(seedIndex==1)for(previous in priorRows)
                if(previous.profile==profile && previous.station==station.id && previous.side==side && previous.yawDegrees==trial.yawDegrees) {
                  var savedRoute:Dynamic=Reflect.field(previous,"routeQ");
                  if(savedRoute!=null) {
                    var values:Array<Dynamic> = cast savedRoute;
                    if(values.length>0){seed=readNumbers(values[0]);break;}
                  }
                }
              var routeClear=true,progressIndex=-1;
              for(i in 0...trial.path.length) {
                progressIndex=i;var target=metric(layout.armPlacement).inverse().compose(metric(trial.path[i]));
                var solution=manipulator.solve(target,seed,new IkOptions(0.0000001,0.000001,200).reaching());
                if(!solution.converged) {trial.ikStatus='no route IK at sample $i';routeClear=false;break;}
                var jump=0.0;for(j in 0...6) jump=Math.max(jump,Math.abs(solution.q[j]-seed[j]));
                if(i>0 && jump>0.35) {trial.ikStatus='joint branch jump at sample $i';routeClear=false;break;}
                seed=solution.q;trial.routeQ.push(seed.copy());
                for(j in 0...6) armState.setJoint("j"+(j+1),seed[j]);
                var mounted=AssemblyFrames.compose(layout.armPlacement,armState.worldPose("toolFlange"));
                for(body in armBodies) world.setBodyPose(body.body,cp(AssemblyFrames.compose(layout.armPlacement,armState.worldPose(body.id))));
                for(opening in [17.0,25.0]) {
                  toolPose(mounted,opening);var found=hits(station,side,nearSeat(mounted,trial.wrist));trial.samples++;
                  if(found.length>0) {
                    trial.hits=found;trial.geometricClear=false;trial.ikStatus='conservative clearance flag at route sample $i';
                    trial.blockedAt=mounted;trial.openingAtSnapshot=opening;trial.q=seed;routeClear=false;break;
                  }
                }
                if(!routeClear) break;
              }
              if(routeClear) {trial.ikStatus="sampled route solved and clear";trial.q=seed.copy();}
              if(routeClear || progressIndex>bestProgress) {selected=trial;bestProgress=progressIndex;}
              if(routeClear)break;
              if(station.vise)progress('Vise route posture rejected at sample $progressIndex');
            }
          }
        } else selected.ikStatus="no endpoint IK found within bounded seeds";
        results[selectedIndex]=selected;
        if(station.vise || station.id=="infeedTray-0-0") {
          var wrist=selected.wrist;
          if(solved) {
            for(j in 0...6) armState.setJoint("j"+(j+1),selected.q[j]);
            wrist=AssemblyFrames.compose(layout.armPlacement,armState.worldPose("toolFlange"));
          }
          toolPose(wrist,selected.openingAtSnapshot);
          snapshot(directory+"/"+profile+"-"+station.id+"-"+side+".mtrg",fixtureScene,fs,armScene,armState,
            layout.armPlacement,tool,ts,wrist,selected.hits,solved,station.blank);
        }
        for(body in armBodies) {world.setBodyStatic(body.body,true);world.setBodyRule(toolBody,body.body,Allow,Allowed);}
        sys.io.File.saveContent(directory+"/progress.json",haxe.Json.stringify({status:"partial sampled study; proposals only",traySetbackMm:setback,results:results},null,"  "));
        progress('$profile ${station.id}/$side: ${selected.geometricClear ? "geometry clear" : selected.hits.length+" conservative clearance flags"}; ${selected.ikStatus}');
      }
      if(exact!=null){sys.io.File.saveContent(directory+"/exact-vise-checks.json",haxe.Json.stringify({toolSha256:digest(toolBytes),sources:exact.sources,checks:exact.checks},null,"  "));exact.close();}
      world.dispose();
    }
    var report={status:"sampled access study; not a production reach certificate",conservativeScreen:true,exactViseNarrowPhase:exactVise,marginMm:MARGIN,workingContactMarginMm:CONTACT_MARGIN,maximumSampleStepMm:STEP,
      stockMm:[60,60,20],traySetbackMm:setback,inventoryPolicy:fillOutfeed ? "all non-target pockets occupied" : "input pockets occupied",millLoadPosition:cell.loadPosition,viseGateLiftMm:VISE_GATE_LIFT,viseApproachMm:VISE_APPROACH,apertureReserveMm:APERTURE_RESERVE,exactQueryReserveMm:EXACT_QUERY_RESERVE,armPlacement:layout.armPlacement,results:results,
      provenance:{toolSha256:digest(toolBytes),fixtureSha256:digest(SceneArtifact.encode(fixtureScene)),
        armSha256:digest(SceneArtifact.encode(armScene)),placementSeedSha256:haxe.crypto.Sha256.encode(TendingReachCertificate.JSON)},
      limitations:["Fixture convex hulls are conservative; tool meshes are surfaces with 0.15 mm tessellation tolerance",
        "10 mm Cartesian samples: 50 mm vise and 100 mm tray approaches, staged full-envelope doorway routes; no continuous-motion certificate",
        "IK bounded to saved placement and finite seeds; failures do not prove global unreachability",
        "Arm self-collision, electrical control, grip retention and dynamics remain unchecked",
        "Open-jaw state retains blank occupancy conservatively; no simulated grasp is claimed",
        "Hull flags need exact-CAD confirmation; mesh signed values are not physical penetration depths",
        "Cable assembly, connector compatibility, lead fixation and flexible motion remain unverified"]};
    sys.io.File.saveContent(directory+"/report.json",haxe.Json.stringify(report,null,"  "));
    progress('Access study exported $directory/report.json');
  }
}
