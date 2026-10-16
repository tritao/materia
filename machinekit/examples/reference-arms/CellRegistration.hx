import haxe.io.Bytes;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import robotkit.manipulation.Manipulator;
import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;
import robotkit.collision.CollisionSceneBuilder;
import robotkit.collision.CollisionSceneBuilder.CollisionOwner;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.IkTolerance;
import motionkit.path.PoseMath;

private typedef RegisteredPiece = {var name:String;var vertices:Array<Float>;var frame:AssemblyFrame;}
private typedef RegisteredTarget = {var name:String;var frame:AssemblyFrame;}
private typedef RegisteredPose = {var name:String;var channel:String;var q:Null<Array<Float>>;var reason:String;var witness:Dynamic;var yawRad:Float;}

private class RegisteredRouteSpace implements motionkit.planner.PlannerSpace {
  final arm:Manipulator;
  final worlds:Array<robotkit.collision.CollisionClearance>;
  public function new(arm:Manipulator,worlds:Array<robotkit.collision.CollisionClearance>) {this.arm=arm;this.worlds=worlds;}
  public function dimension():Int return arm.group.count();
  public function lower():Array<Float> return [for(i in 0...dimension()) arm.group.limitsOf(i).lower];
  public function upper():Array<Float> return [for(i in 0...dimension()) arm.group.limitsOf(i).upper];
  public function invalid(q:Array<Float>):Null<String> {
    for(i in 0...dimension()) if(q[i]<arm.group.limitsOf(i).lower || q[i]>arm.group.limitsOf(i).upper) return "Joint limit";
    for(world in worlds) {var found=world.violation(q);if(found!=null) return found.a+" / "+found.b;}
    return null;
  }
  public function edgesClear(edges:Array<Float>,count:Int):Array<Bool> {
    var n=dimension();
    return [for(edge in 0...count) {
      var start=[for(j in 0...n) edges[(edge*2)*n+j]],end=[for(j in 0...n) edges[(edge*2+1)*n+j]];
      CellRegistration.curveReview(worlds,[start,end]).clear;
    }];
  }
}

/** Candidate CAD cell registration, separate from the generic-arm reach certificate. */
class CellRegistration {
  static final ROOT = ReferenceAssetLocation.DIRECTORY+"/cell-registration";
  public static function readFrame(raw:Dynamic):AssemblyFrame {
    function n(key:String):Float {
      var value=Std.parseFloat(Std.string(Reflect.field(raw,key)));
      if(!Math.isFinite(value)) throw "Nonfinite registration frame "+key;
      return value;
    }
    return {x:n("x"),y:n("y"),z:n("z"),qx:n("qx"),qy:n("qy"),qz:n("qz"),qw:n("qw")};
  }
  static function digest(bytes:Bytes):String {
    var hash = haxe.crypto.Sha256.make(bytes), hex = new StringBuf();
    for (i in 0...hash.length) hex.add(StringTools.hex(hash.get(i),2).toLowerCase());
    return hex.toString();
  }
  static function scaled(f:AssemblyFrame,s:Float):AssemblyFrame
    return {x:f.x*s,y:f.y*s,z:f.z*s,qx:f.qx,qy:f.qy,qz:f.qz,qw:f.qw};
  static function transform(f:AssemblyFrame):Transform3
    return new Transform3(new Vec3(f.x,f.y,f.z),new Quat(f.qx,f.qy,f.qz,f.qw));
  static function pose(f:AssemblyFrame):Pose3 return new Pose3(f.x,f.y,f.z,f.qx,f.qy,f.qz,f.qw);
  static function pieces(scene:SceneArtifactData,?overrideDoor:Bool):Array<RegisteredPiece> {
    var definition:AssemblyDefinition = AssemblyDefinitionFlattener.flatten(scene.assemblyDefinition);
    var state = new cadkit.modeling.AssemblyState(scene.assemblyDefinition,scene.assemblyState);
    if (overrideDoor != null && !overrideDoor) state.setJoint("door",0);
    var physical = AssemblyPhysicalPartView.fromSceneArtifact(scene);
    var result:Array<RegisteredPiece> = [];
    for (o in definition.occurrences) {
      var component = [for (d in definition.definitions) if (d.id==o.definition) d][0];
      var part = [for (p in physical.parts) if (p.id==o.definition) p][0];
      var hulls:Array<Array<Float>> = component.collisionHulls;
      if (hulls==null || hulls.length==0) hulls=[part.collisionHull];
      for (i in 0...hulls.length) result.push({name:o.id+(hulls.length>1 ? '/piece$i' : ''),
        vertices:[for (value in hulls[i]) value*scene.metresPerUnit],frame:scaled(state.worldPose(o.id),scene.metresPerUnit)});
    }
    return result;
  }
  static function box(w:Float,d:Float,h:Float):Array<Float> {
    var result:Array<Float> = [];
    for (x in [-w/2,w/2]) for (y in [-d/2,d/2]) for (z in [0.0,h]) {result.push(x);result.push(y);result.push(z);}
    return result;
  }
  static function readScene(file:String,expected:String):SceneArtifactData {
    var bytes = sys.io.File.getBytes(ROOT+"/"+file);
    if (digest(bytes)!=expected) throw "Changed registered fixture artifact: "+file;
    return SceneArtifact.decode(bytes);
  }
  public static function properties():Void {
    var raw=sys.io.File.getContent(ROOT+"/fixtures.json"),data:Dynamic=haxe.Json.parse(raw);
    var states:Array<Dynamic> = data.toolStates,output:Array<Dynamic> = [];
    for(entry in states) {
      var scene=readScene(Std.string(entry.file),Std.string(entry.sha256));
      var physical=AssemblyPhysicalPartView.fromSceneArtifact(scene);
      var state=new cadkit.modeling.AssemblyState(scene.assemblyDefinition,scene.assemblyState);
      var definition=AssemblyDefinitionFlattener.flatten(scene.assemblyDefinition),rows:Array<Dynamic> = [];
      for(o in definition.occurrences) if(StringTools.startsWith(o.id,"tool/")) {
        var part=[for(p in physical.parts) if(p.id==o.definition) p][0];
        var scale=scene.metresPerUnit,baseMass=part.volume*part.density*Math.pow(scale,3);
        var mass:Float=part.physicalMassKg==null ? baseMass : part.physicalMassKg;
        var inertia:Array<Float> = part.physicalInertiaKgM2;
        if(inertia==null) inertia=[for(v in part.inertia) v*part.density*Math.pow(scale,5)*mass/baseMass];
        rows.push({id:o.id,massKg:mass,centerLocalMetres:[for(v in part.centerOfMass) v*scale],inertiaLocalKgM2:inertia,
          frameMetres:scaled(state.worldPose(o.id),scale),declaredMass:part.physicalMassDeclared});
      }
      output.push({holding:entry.holding,artifactSha256:entry.sha256,parts:rows});
    }
    var mill=readScene("mill.mtrg",Std.string(data.millArtifactSha256));
    var stock=[for(p in AssemblyPhysicalPartView.fromSceneArtifact(mill).parts) if(p.id=="mill/stock") p][0];
    var unit=mill.metresPerUnit,geometricMass=stock.volume*stock.density*Math.pow(unit,3);
    var stockMass:Float=stock.physicalMassKg==null ? geometricMass : stock.physicalMassKg;
    var stockInertia:Array<Float> = stock.physicalInertiaKgM2;
    if(stockInertia==null) stockInertia=[for(v in stock.inertia) v*stock.density*Math.pow(unit,5)*stockMass/geometricMass];
    sys.io.File.saveContent(ROOT+"/tool-properties.json",haxe.Json.stringify({fixtureSha256:haxe.crypto.Sha256.encode(raw),states:output,
      stock:{massKg:stockMass,centerLocalMetres:[for(v in stock.centerOfMass) v*unit],inertiaLocalKgM2:stockInertia},
      scope:"Component declared/CAD masses and inertias; external air supply excluded; unmeasured hardware"},null,"  "));
    Sys.println("Exported carried dual-tool component mass/inertia in all four jaw states");
  }
  public static function previews():Void {
    var fixtureText=sys.io.File.getContent(ROOT+"/fixtures.json"),data:Dynamic=haxe.Json.parse(fixtureText);
    var configText=sys.io.File.getContent(ReferenceAssetLocation.DIRECTORY+"/../tending-registration-config.json"),config:Dynamic=haxe.Json.parse(configText);
    var prefix=Sys.args().indexOf("--original-reference")>=0 ? "original-" : Sys.args().indexOf("--unrestricted-modified-reference")>=0 ? "unrestricted-" : "";
    var report:Dynamic=haxe.Json.parse(sys.io.File.getContent(ROOT+"/"+prefix+"registration-planned.json"));
    if(Std.string(report.fixtureSha256)!=haxe.crypto.Sha256.encode(fixtureText) || Std.string(report.configSha256)!=haxe.crypto.Sha256.encode(configText))
      throw "Stale registered cell review";
    var models:Array<Dynamic> = report.models,seats:Array<Dynamic> = data.traySeats;
    for(model in models) {
      var id=Std.string(model.model);
      if(Std.string(model.sourceFingerprint)!=ComparisonReview.fingerprint(ComparisonProject.source(id))) throw "Changed registered arm";
      var candidates:Array<Dynamic> = model.candidates;
      var candidate=candidates[0],poses:Array<Dynamic> = candidate.poses;
      var stage=[for(p in poses) if(Std.string(p.name)=="outside staging" && Std.string(p.channel)=="A" && p.q!=null) p];
      var q:Array<Float> = stage.length>0 ? stage[0].q : StringTools.startsWith(id,"zeroerr") ? [0.2,-0.8,1.0,0.1,-0.6,0.2] : [0.0,-1.0,1.3,-1.5,-1.0,0.0];
      RegisteredCellPreview.save(id,readFrame(candidate.baseWorldMetres),q,readFrame(data.frames.toolMountZ),readFrame(data.frames.toolA),
        (cast config.pedestalWidthMm:Float)/1000,seats);
    }
  }
  static function channel(value:String):String {
    if (value.indexOf("gripperA/")>=0) return "A";
    if (value.indexOf("gripperB/")>=0) return "B";
    if (StringTools.endsWith(value,"A") || StringTools.endsWith(value,"a")) return "A";
    if (StringTools.endsWith(value,"B") || StringTools.endsWith(value,"b")) return "B";
    throw "Unknown tending tool channel "+value;
  }
  public static function curveReview(worlds:Array<robotkit.collision.CollisionClearance>,path:Array<Array<Float>>):{clear:Bool,queries:Int,failedWorld:Int,failedSpan:Int,violation:Dynamic,error:Null<String>} {
    var knots=[for(i in 0...path.length) i*1.0],zeros=[for(q in path) [for(_ in q) 0.0]];
    var curve=new motionkit.planner.JointPathSamples(knots,path,zeros,zeros),queries=0;
    for(index in 0...worlds.length) {
      var proof:motionkit.robot.JointCurveClearance=null;
      try {
        proof=new motionkit.robot.JointCurveClearance(worlds[index],curve,1e-6,false);
        var clear=proof.check();queries+=proof.queries;
        if(!clear) return {clear:false,queries:queries,failedWorld:index,failedSpan:proof.failedSpan,violation:proof.failure,error:null};
      } catch(error:Dynamic) return {clear:false,queries:queries,failedWorld:index,failedSpan:proof==null ? -1 : proof.failedSpan,violation:null,error:Std.string(error)};
    }
    return {clear:true,queries:queries,failedWorld:-1,failedSpan:-1,violation:null,error:null};
  }
  public static function run():Void {
    var original=Sys.args().indexOf("--original-reference")>=0;
    var refining=Sys.args().indexOf("--refine-routes")>=0;
    if(refining && !original) throw "Route refinement is isolated to the original reference";
    var unrestricted=Sys.args().indexOf("--unrestricted-modified-reference")>=0;
    var numericOnly=Sys.args().indexOf("--numeric-only")>=0;
    var candidateIndex:Null<Int> = null;
    for(arg in Sys.args()) if(StringTools.startsWith(arg,"--plan-candidate-index=")) {
      candidateIndex=Std.parseInt(arg.substr(23));
      if(candidateIndex==null || candidateIndex<0 || candidateIndex>8 || !original) throw "Original candidate index must be 0..8";
    }
    if(numericOnly && !original) throw "Numeric control is isolated to the original reference";
    if(original && unrestricted) throw "Choose one isolated reference study";
    var reportStem=original ? (numericOnly ? "/original-numeric-registration" : "/original-registration") : unrestricted ? "/unrestricted-registration" : "/registration";
    var rawText = sys.io.File.getContent(ROOT+"/fixtures.json"), data:Dynamic = haxe.Json.parse(rawText);
    var configText = sys.io.File.getContent(ReferenceAssetLocation.DIRECTORY+"/../tending-registration-config.json");
    if (Std.string(data.configSha256)!=haxe.crypto.Sha256.encode(configText)) throw "Changed candidate registration inputs; rerun fixture export";
    var config:Dynamic = haxe.Json.parse(configText);
    var planning=Sys.args().indexOf("--plan-transfers")>=0 || refining;
    if(refining && (numericOnly || candidateIndex!=null)) throw "Refine the selected original base only";
    if(candidateIndex!=null && (!planning || numericOnly)) throw "Candidate workers require ordinary original transfer planning";
    var candidateCount=1;
    for(arg in Sys.args()) if(StringTools.startsWith(arg,"--plan-candidate-count=")) {
      var parsed=Std.parseInt(arg.substr(23));
      if(parsed==null || parsed<1 || parsed>9) throw "Planning candidate count must be 1..9";
      candidateCount=parsed;
    }
    var selected:Null<String> = null,reused:Dynamic=null;
    for(arg in Sys.args()) if(StringTools.startsWith(arg,"--registration-model=")) selected=arg.substr(21);
    if(selected!=null) {
      if(original || unrestricted) throw "Reference studies use isolated reports; omit --registration-model";
      if(["zeroerr","ur5e","fairino5_v6"].indexOf(selected)<0) throw "Unknown registration model";
      reused=haxe.Json.parse(sys.io.File.getContent(ROOT+(planning ? "/registration-planned.json" : "/registration-review.json")));
      if(Std.string(reused.fixtureSha256)!=haxe.crypto.Sha256.encode(rawText) || Std.string(reused.configSha256)!=haxe.crypto.Sha256.encode(configText))
        throw "Cannot reuse registration results with changed fixtures/configuration";
    }
    var previous:Dynamic=null;
    if(planning) {
      previous=haxe.Json.parse(sys.io.File.getContent(ROOT+reportStem+(refining ? "-planned.json" : "-review.json")));
      if(Std.string(previous.fixtureSha256)!=haxe.crypto.Sha256.encode(rawText) || Std.string(previous.configSha256)!=haxe.crypto.Sha256.encode(configText))
        throw "Run the current candidate registration scan before transfer planning";
    }
    var mill = readScene("mill.mtrg",Std.string(data.millArtifactSha256));
    var trays = readScene("trays.mtrg",Std.string(data.trayArtifactSha256));
    var toolStates:Array<Dynamic> = data.toolStates;
    var tools:Map<String,Array<RegisteredPiece>> = [];
    for (entry in toolStates) tools.set(Std.string(entry.holding),[for (p in pieces(readScene(Std.string(entry.file),Std.string(entry.sha256))))
      if (StringTools.startsWith(p.name,"tool/")) p]);
    var mount = readFrame(data.frames.toolMountZ);
    var inverseMount = AssemblyFrames.inverse(scaled(mount,0.001));
    var frameA = readFrame(data.frames.toolA),frameB = readFrame(data.frames.toolB);
    var tcp:Map<String,AssemblyFrame> = [];
    tcp.set("A",AssemblyFrames.compose(inverseMount,scaled(frameA,0.001)));
    tcp.set("B",AssemblyFrames.compose(inverseMount,scaled(frameB,0.001)));
    var graspLocal = readFrame(data.graspLocal);
    var grasp = scaled(graspLocal,0.001),inverseGrasp = AssemblyFrames.inverse(grasp);
    var seats:Array<Dynamic> = data.traySeats;
    var targets:Array<RegisteredTarget> = [];
    for (entry in seats) {
      var seat = readFrame(entry.frame);
      var target = AssemblyFrames.compose(scaled(seat,0.001),grasp);
      for (suffix in ["","-approach","-clearance"]) {
        var lift:Float = suffix=="" ? 0 : suffix=="-approach" ? config.approachMm : config.clearanceLiftMm;
        var lifted = AssemblyFrames.compose(AssemblyFrames.translation(0,0,lift/1000),target);
        targets.push({name:Std.string(entry.name)+suffix,frame:lifted});
      }
    }
    var load = readFrame(data.frames.millLoad);
    var stock = readFrame(data.frames.stock);
    for (station in ["vise","vise-load"]) {
      var target = AssemblyFrames.compose(scaled(station=="vise" ? stock : load,0.001),grasp);
      for (suffix in ["","-approach"]) {
        var lift:Float = suffix=="" ? 0 : config.approachMm;
        targets.push({name:station+suffix,frame:AssemblyFrames.compose(AssemblyFrames.translation(0,0,lift/1000),target)});
      }
    }
    var staging:Array<Float> = config.stagingMm;
    var stageFrame = {x:staging[0]/1000,y:staging[1]/1000,z:staging[2]/1000,qx:grasp.qx,qy:grasp.qy,qz:grasp.qz,qw:grasp.qw};
    targets.push({name:"outside staging",frame:stageFrame});
    var environmentOpen = pieces(mill,true),environmentClosed = pieces(mill,false),trayGeometry = pieces(trays);
    var stockPiece = [for (p in environmentOpen) if (p.name=="mill/stock") p][0];
    var candidates:Array<Array<Float>> = config.baseCandidatesMm;
    var transfers:Array<Dynamic> = data.transfers;
    var reports:Array<Dynamic> = [];
    for (id in (original ? ["zeroerr-original"] : unrestricted ? ["zeroerr-unrestricted"] : ["zeroerr","ur5e","fairino5_v6"])) {
      var source = ComparisonProject.source(id), sourceHash = ComparisonReview.fingerprint(source);
      if((original || unrestricted) && candidateIndex==null && !refining) sys.io.File.saveBytes(ROOT+"/"+id+"-study-source.mtrg",SceneArtifact.encode(source));
      if(selected!=null && selected!=id) {
        var oldModels:Array<Dynamic> = reused.models;
        var old=[for(m in oldModels) if(Std.string(m.model)==id) m];
        if(old.length!=1 || Std.string(old[0].sourceFingerprint)!=sourceHash) throw "Cannot reuse changed model: "+id;
        reports.push(old[0]);continue;
      }
      var converted = AssemblySimulationBridge.toRobotModel(source.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(source));
      var flange = [for (f in converted.model.frames) if (StringTools.endsWith(f.name," robot flange")) f][0];
      for (joint in converted.model.joints) if (joint.limits.lower< -1e8 || joint.limits.upper>1e8) {
        joint.limits.lower=-Math.PI;joint.limits.upper=Math.PI;
      }
      var flangePose:AssemblyFrame = {x:flange.position[0],y:flange.position[1],z:flange.position[2],
        qx:flange.rotation[0],qy:flange.rotation[1],qz:flange.rotation[2],qw:flange.rotation[3]};
      var arms:Map<String,Manipulator> = [],solvers:Map<String,ReferenceKinematics> = [];
      var numerical:Map<String,motionkit.robot.ManipulatorKinematics> = [];
      for (c in ["A","B"]) {
        var arm = new Manipulator(converted.model,converted.model.links[0].id,flange.id,transform(tcp.get(c)));
        arms.set(c,arm);solvers.set(c,new ReferenceKinematics(arm));
        if(numericOnly) numerical.set(c,new motionkit.robot.ManipulatorKinematics(arm));
      }
      var modelReports:Array<Dynamic> = [];
      var scanCandidates=candidates;
      if(planning) {
        var models:Array<Dynamic> = previous.models;
        var model=[for(m in models) if(Std.string(m.model)==id) m][0];
        if(Std.string(model.sourceFingerprint)!=sourceHash) throw "Changed arm since candidate scan";
        var choices:Array<Dynamic> = model.candidates;
        function score(choice:Dynamic):Int {
          var routes:Array<Dynamic> = choice.routes,clear=0;
          for(route in routes) {
            var states:Array<Dynamic> = route.states;
            if(states.length==4 && [for(s in states) if(s.clear==true) s].length==4) clear++;
          }
          return clear*1000+Std.int(choice.clearPoses);
        }
        if(!refining) choices.sort((a,b) -> score(b)-score(a));
        scanCandidates=[];
        var first=candidateIndex==null ? 0 : candidateIndex;
        for(choice in choices.slice(first,candidateIndex==null ? candidateCount : first+1)) {
          var base=readFrame(choice.baseWorldMetres);
          scanCandidates.push([base.x*1000,base.y*1000,base.z*1000]);
        }
      }
      var roadmap=new OriginalRouteRefinement();
      for (candidate in scanCandidates) {
        var yaw:Float = config.baseYawRad;
        var base = {x:candidate[0]/1000,y:candidate[1]/1000,z:candidate[2]/1000,qx:0.0,qy:0.0,qz:Math.sin(yaw/2),qw:Math.cos(yaw/2)};
        var baseInverse = AssemblyFrames.inverse(base);
        var reference = StringTools.startsWith(id,"zeroerr") ? [0.2,-0.8,1.0,0.1,-0.6,0.2] : [0.0,-1.0,1.3,-1.5,-1.0,0.0];
        var worlds:Map<String,robotkit.collision.CollisionClearance> = [];
        var rigidContacts:Array<Dynamic> = [];
        // Physical worlds include open/loaded jaws, both possible payloads, and
        // all other tray pockets occupied as a conservative inventory envelope.
        function world(door:String,holding:String):robotkit.collision.CollisionClearance {
          var key=door+"/"+holding, existing=worlds.get(key);
          if (existing!=null) return existing;
          var arm=arms.get("A"),builder=new CollisionSceneBuilder(arm,reference,id,cast config.collisionMarginMetres);
          var geometry=(door=="Open" ? environmentOpen : environmentClosed).concat(trayGeometry);
          for (piece in geometry) if (piece.name!="mill/stock") builder.add("fixture/"+piece.name,Fixture,piece.vertices,
            transform(AssemblyFrames.compose(baseInverse,piece.frame)));
          builder.add("seated/mill-stock",Loose,stockPiece.vertices,transform(AssemblyFrames.compose(baseInverse,stockPiece.frame)));
          for (piece in tools.get(holding)) {
            var local=AssemblyFrames.compose(flangePose,AssemblyFrames.compose(inverseMount,piece.frame));
            var member=piece.name.indexOf("gripperA/finger")>=0 ? "fingers/A" : piece.name.indexOf("gripperB/finger")>=0 ? "fingers/B" : "dual-tool";
            builder.add("dual/"+piece.name,Mechanism(id,flange.link.id),piece.vertices,transform(local),true,member);
          }
          for (c in ["A","B"]) if (holding.indexOf(c)>=0) {
            var local=AssemblyFrames.compose(flangePose,AssemblyFrames.compose(tcp.get(c),inverseGrasp));
            builder.add("held/"+c,Held(id,flange.link.id),stockPiece.vertices,transform(local),false,"held/"+c);
            var fingers=[for (object in builder.description.objects)
              if (StringTools.startsWith(object.name,"dual/tool/gripper"+c+"/finger")) object.name];
            builder.payload("held/"+c,fingers,collisionkit.CollisionPairRule.Touch,collisionkit.CollisionPairStatus.DeclaredContact,0);
          }
          for (entry in seats) {
            var frame = readFrame(entry.frame);
            builder.add("seated/"+Std.string(entry.name),Loose,stockPiece.vertices,transform(AssemblyFrames.compose(baseInverse,scaled(frame,0.001))));
          }
          var width:Float=config.pedestalWidthMm;
          builder.add("pedestal",Fixture,box(width/1000,width/1000,candidate[2]/1000),
            transform(AssemblyFrames.compose(baseInverse,{x:base.x,y:base.y,z:0.0,qx:0.0,qy:0.0,qz:0.0,qw:1.0})));
          var result=builder.clearance(() -> new collisionkit.native.NativeCollisionWorld());
          for(c in ["A","B"]) if(holding.indexOf(c)>=0) {
            var contacts=[for(object in builder.description.objects) if(object.member=="fingers/"+c) object.name];
            if(contacts.length<2) throw "Rigid grasp needs both modelled fingers";
            var distances:Array<Float> = [];
            for(contact in contacts) {
              var distance=result.distanceBetween(reference,"held/"+c,contact);
              if(distance < -1e-8 || distance>1e-6) throw "Held blank does not touch its intended rigid finger: "+contact+" distance="+distance;
              distances.push(distance);
            }
            // Both groups have constant offsets on the same flange link. Their
            // relative transform is invariant for every arm configuration and
            // every lowered arm trajectory. Validate contact once, then exempt
            // exactly these finger/blank pairs from independent motion inflation.
            result.apply(robotkit.manipulation.ClearanceChange.OpenWindow("held/"+c,"fingers/"+c,null),reference);
            rigidContacts.push({world:key,holding:holding,channel:c,carrier:flange.link.id,contacts:contacts,distancesMetres:distances,
              reason:"Validated zero-penetration contacts at invariant relative transforms; ideal rigid grasp assumption"});
          }
          worlds.set(key,result);return result;
        }
        var baseline:Dynamic=null;
        if(refining) {
          var oldModels:Array<Dynamic> = previous.models,oldCandidates:Array<Dynamic> = oldModels[0].candidates;
          baseline=oldCandidates[0];
        }
        var solved:Array<RegisteredPose> = [];
        if(refining) {
          var oldPoses:Array<Dynamic> = baseline.poses;
          for(p in oldPoses) solved.push({name:Std.string(p.name),channel:Std.string(p.channel),q:p.q,
            reason:Std.string(p.reason),witness:p.witness,yawRad:p.yawRad});
        }
        if(!refining) for (target in targets) for (c in ["A","B"]) {
          var available:ReferenceKinematics=solvers.get(c);
          var solver:motionkit.robot.ManipulatorKinematics=numericOnly ? numerical.get(c) : available;
          var exact:Null<motionkit.robot.EaikKinematics> = numericOnly ? null : available.exact;
          var selected:Null<Array<Float>> = null,reason="unreachable_from_study_seeds",witness:Dynamic=null;
          var selectedYaw=0.0;
          for (roll in [0.0,Math.PI/2,Math.PI,3*Math.PI/2]) {
            var orientation=AssemblyFrames.compose({x:0.0,y:0.0,z:0.0,qx:0.0,qy:0.0,qz:Math.sin(roll/2),qw:Math.cos(roll/2)},
              {x:0.0,y:0.0,z:0.0,qx:target.frame.qx,qy:target.frame.qy,qz:target.frame.qz,qw:target.frame.qw});
            var rolled={x:target.frame.x,y:target.frame.y,z:target.frame.z,qx:orientation.qx,qy:orientation.qy,qz:orientation.qz,qw:orientation.qw};
            var wanted=pose(AssemblyFrames.compose(baseInverse,rolled));
            var tolerance=new IkTolerance(1e-6,1e-6,400,0.005);
            var options:Array<Array<Float>> = exact==null ? solver.sampleCandidates(wanted,8,tolerance) :
              [for(branch in exact.branches(wanted,reference)) branch.q];
            if(options.length==0) for(seed in [reference,[1.7,0.6,0.5,2.6,-1.57,1.4],[-1.7,0.6,0.5,2.6,-1.57,-1.4]]) {
              var q=solver.solvePose(wanted,seed,tolerance);if(q!=null) options.push(q);
            }
            var freeTransfer=StringTools.endsWith(target.name,"-approach") || StringTools.endsWith(target.name,"-clearance") || target.name=="outside staging";
            for(q in options) {
              var actual=solver.forward(q);
              if(PoseMath.distance(actual,wanted)>1e-6 || PoseMath.angle(actual,wanted)>1e-6) throw "Registered IK residual";
              var clear=true;
              for(holding in (freeTransfer ? ["empty","A","B","AB"] : ["empty"])) {
                var found=world(StringTools.startsWith(target.name,"vise") ? "Open" : "Closed",holding).violation(q);
                if(found!=null) {clear=false;reason="collision";witness={holding:holding,a:found.a,b:found.b,distance:found.distance,required:found.required};break;}
              }
              if(clear) {selected=q;selectedYaw=roll;reason=freeTransfer ? "all_holding_states_clear" : "empty_tool_contact_pose_clear";break;}
            }
            if(selected!=null) break;
          }
          solved.push({name:target.name,channel:c,q:selected,reason:reason,witness:witness,yawRad:selectedYaw});
        }
        var waypointPool:Array<Array<Float>> = [],portal:Dynamic=null,branchReviews:Array<Dynamic> = [];
        if(refining) {
          for(spec in [{station:"outfeed-0-1",channel:"B",from:"A"},{station:"outfeed-0-2",channel:"A",from:"B"}]) {
            var upper=[for(p in solved) if(p.name==spec.station+"-clearance" && p.channel==spec.channel) p][0];
            var lower=[for(p in solved) if(p.name==spec.station+"-approach" && p.channel==spec.channel) p][0];
            var stage=[for(p in solved) if(p.name=="outside staging" && p.channel==spec.from) p][0];
            var oldUpper=upper.q.copy(),oldLower=lower.q.copy();
            var selection=OriginalOutfeedBranches.select(solvers.get(spec.channel),stage.q,upper.q,lower.q,
              [for(holding in ["empty","A","B","AB"]) world("Closed",holding)]);
            if(selection.clearance!=null) {upper.q=selection.clearance;lower.q=selection.approach;}
            branchReviews.push({station:spec.station,channel:spec.channel,originalClearance:oldUpper,originalApproach:oldLower,selection:selection});
            Sys.println("Original outfeed branch selection "+spec.station+": "+(selection.clearance!=null ? "both segments clear" : "unproven"));Sys.stdout().flush();
          }
          for(p in solved) if(p.q!=null && (StringTools.endsWith(p.name,"-approach") || StringTools.endsWith(p.name,"-clearance") || p.name=="outside staging")) waypointPool.push(p.q);
          var goals:Map<String,Array<Float>> = [];
          for(p in solved) if(p.name=="vise-approach" && p.q!=null) goals.set(p.channel,p.q);
          portal=OriginalPortalWaypoints.generate(solvers,goals,base,[for(holding in ["empty","A","B","AB"]) world("Open",holding)]);
          var portalPoints:Array<Array<Float>> = portal.points;
          for(q in portalPoints) waypointPool.push(q);
          Sys.println("Original front-entry waypoint IK: "+portalPoints.length+" clear configurations");Sys.stdout().flush();
          var oldRoutes:Array<Dynamic> = baseline.routes;
          for(r in oldRoutes) if(r.continuous!=null && r.continuous.clear==true) {
            var path:Array<Array<Float>> = r.path;for(q in path) waypointPool.push(q);
          }
        }
        var routes:Array<Dynamic> = [];
        for (request in transfers) {
          var c=channel(Std.string(request.tool)),fromC=channel(Std.string(request.fromTool));
          var fromName=Std.string(request.fromPose),toName=Std.string(request.toPose);
          var from=[for(p in solved) if(p.name==fromName && p.channel==fromC) p];
          var to=[for(p in solved) if(p.name==toName && p.channel==c) p];
          var parked=[for(p in solved) if(p.name=="outside staging" && p.channel==fromC) p];
          var start:Null<Array<Float>> = fromName=="ready" ? (parked.length==0 ? null : parked[0].q) : from.length==0 ? null : from[0].q;
          var end:Null<Array<Float>> = to.length==0 ? null : to[0].q;
          var states:Array<Dynamic> = [];
          if(!refining && start!=null && end!=null) for(holding in ["empty","A","B","AB"]) {
            var clearance=world(Std.string(request.door),holding),violation:Dynamic=null;
            for(index in 0...33) {
              var u=index/32.0,blend=10*u*u*u-15*u*u*u*u+6*u*u*u*u*u;
              var q=[for(j in 0...6) start[j]+blend*(end[j]-start[j])];
              var found=clearance.violation(q);
              if(found!=null){violation={sample:index,a:found.a,b:found.b,distance:found.distance,required:found.required};break;}
            }
            states.push({holding:holding,clear:violation==null,violation:violation});
          }
          var routePath:Null<Array<Array<Float>>> = null,proof:Dynamic=null,planner:Dynamic=null;
          if(refining) {
            var oldRoutes:Array<Dynamic> = baseline.routes;
            var old:Dynamic=oldRoutes[routes.length];
            states=old.states;
            var checks=[for(holding in ["empty","A","B","AB"]) world(Std.string(request.door),holding)];
            if(old.continuous!=null && old.continuous.clear==true) {
              var oldPath:Array<Array<Float>> = old.path;
              routePath=[for(q in oldPath) q.copy()];routePath[0]=start;routePath[routePath.length-1]=end;
              proof=curveReview(checks,routePath);
              if(proof.clear!=true) throw "Previously clear original route failed fresh review";
              planner={method:"baseline_path_freshly_rechecked"};
            } else if(start!=null && end!=null) {
              routePath=[start,end];proof=curveReview(checks,routePath);
              if(proof.clear==true) planner={method:"compatible_endpoint_branches"};
              else {
                var improved=roadmap.refine(Std.string(request.door),checks,start,end,waypointPool);
                routePath=improved.path;proof=improved.continuous;planner=improved.planner;
                if(proof==null) proof=old.continuous;
              }
            }
            Sys.println("Original route "+routes.length+": "+(proof!=null && proof.clear==true ? "continuously clear" : "unproven")+"; "+roadmap.edgeChecks+" distinct roadmap edges");Sys.stdout().flush();
          }
          if(planning && !refining && start!=null && end!=null) {
            var checks=[for(holding in ["empty","A","B","AB"]) world(Std.string(request.door),holding)];
            var space=new RegisteredRouteSpace(arms.get("A"),checks);
            if(space.invalid(start)==null && space.invalid(end)==null) {
              routePath=[start,end];proof=curveReview(checks,routePath);
              if(proof.clear!=true) {
                var search=new motionkit.planner.RrtConnect(space,20261016+routes.length);
                search.maxIterations=256;search.shortcutRounds=16;search.visibilityCheckLimit=128;
                var result=search.plan(start,end);
                planner={iterations:result.iterations,nodes:result.nodes,edgeChecks:result.edgeChecks,failure:result.failure};
                routePath=result.waypoints;
                if(routePath!=null) proof=curveReview(checks,routePath);
              }
            } else proof={clear:false,error:"A selected endpoint fails one required physical state"};
          }
          routes.push({station:request.station,tool:c,fromTool:fromC,fromPose:fromName,toPose:toName,
            door:request.door,status:start==null || end==null ? "endpoint_unproven" : "sampled_joint_route_reviewed",states:states,
            path:routePath,continuous:proof,planner:planner});
          if(refining) sys.io.File.saveContent(ROOT+"/original-refinement-progress.json",haxe.Json.stringify({routes:routes,edgeChecks:roadmap.edgeChecks,cacheHits:roadmap.cacheHits},null,"  "));
        }
        for(value in worlds) value.world.dispose();
        var report={baseWorldMetres:base,poses:solved,routes:routes,
          rigidGraspContacts:rigidContacts,portalWaypoints:portal,outfeedBranchReviews:branchReviews,
          clearPoses:[for(p in solved) if(p.q!=null) p].length,totalPoses:solved.length,
          reviewedRoutes:[for(r in routes) if(r.status=="sampled_joint_route_reviewed") r].length};
        modelReports.push(report);
        sys.io.File.saveContent(ROOT+"/"+id+(refining ? "-refinement" : "")+(numericOnly ? "-numeric" : "")+(candidateIndex==null ? "" : "-candidate"+candidateIndex)+(planning ? "-planned-progress.json" : "-registration-progress.json"),haxe.Json.stringify({model:id,sourceFingerprint:sourceHash,
          fixtureSha256:haxe.crypto.Sha256.encode(rawText),configSha256:haxe.crypto.Sha256.encode(configText),candidates:modelReports},null,"  "));
        Sys.println(id+" registration base "+candidate+": "+report.clearPoses+"/"+report.totalPoses+" target poses; "+report.reviewedRoutes+"/"+routes.length+" sampled routes");Sys.stdout().flush();
      }
      var examined:Array<Dynamic> = [];
      if(planning) {
        function continuousCount(candidate:Dynamic):Int {
          var routes:Array<Dynamic> = candidate.routes;
          return [for(route in routes) if(route.continuous!=null && route.continuous.clear==true) route].length;
        }
        examined=[for(candidate in modelReports) {baseWorldMetres:candidate.baseWorldMetres,
          continuousRoutes:continuousCount(candidate),clearPoses:candidate.clearPoses,reviewedRoutes:candidate.reviewedRoutes}];
        modelReports.sort((a,b) -> (continuousCount(b)-continuousCount(a))*1000+Std.int(b.clearPoses)-Std.int(a.clearPoses));
        modelReports=[modelReports[0]];
        var best=modelReports[0],poses:Array<RegisteredPose> = cast best.poses;
        var stage=[for(p in poses) if(p.name=="outside staging" && p.channel=="A" && p.q!=null) p];
        var visible=stage.length>0 ? stage[0].q : StringTools.startsWith(id,"zeroerr") ? [0.2,-0.8,1.0,0.1,-0.6,0.2] : [0.0,-1.0,1.3,-1.5,-1.0,0.0];
        if(candidateIndex==null && !refining) RegisteredCellPreview.save(id,readFrame(best.baseWorldMetres),visible,mount,frameA,(cast config.pedestalWidthMm:Float)/1000,seats);
      }
      var selectedSolver:ReferenceKinematics=solvers.get("A");
      reports.push({model:id,sourceFingerprint:sourceHash,candidates:modelReports,examinedCandidates:examined,
        solverMode:numericOnly ? "numeric_only_control" : selectedSolver.exact==null ? "numeric_fallback" : "analytic_with_numeric_fallback",
        unsupportedDiagnostic:selectedSolver.unsupportedDiagnostic});
    }
    sys.io.File.saveContent(ROOT+reportStem+(refining ? "-refinement" : "")+(candidateIndex==null ? "" : "-candidate"+candidateIndex)+(planning ? "-planned.json" : "-review.json"),haxe.Json.stringify({status:"candidate_review_not_cell_certificate",
      fixtureSha256:haxe.crypto.Sha256.encode(rawText),configSha256:haxe.crypto.Sha256.encode(configText),models:reports,
      baselineSha256:refining ? digest(sys.io.File.getBytes(ROOT+"/original-registration-planned.json")) : null,
      refinementProducerSha256:refining ? digest(sys.io.File.getBytes(ReferenceAssetLocation.DIRECTORY+"/../OriginalRouteRefinement.hx")) : null,
      outfeedProducerSha256:refining ? digest(sys.io.File.getBytes(ReferenceAssetLocation.DIRECTORY+"/../OriginalOutfeedBranches.hx")) : null,
      portalProducerSha256:refining ? digest(sys.io.File.getBytes(ReferenceAssetLocation.DIRECTORY+"/../OriginalPortalWaypoints.hx")) : null,
      registrationProducerSha256:refining ? digest(sys.io.File.getBytes(ReferenceAssetLocation.DIRECTORY+"/../CellRegistration.hx")) : null,
      scope:"Current CAD fixtures and dual gripper; nine proposed bases, or best per model for planning; source 23 transfer requests; empty/A/B/AB jaw and payload states. 33 quintic route samples; optional continuous proof and bounded RRT. Standard articulation exclusions. All tray pockets and mill stock conservatively present. No physical mount qualification, contact insertion, thermal or hardware qualification."},null,"  "));
  }
}
