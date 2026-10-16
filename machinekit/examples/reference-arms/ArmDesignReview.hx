import materia.assembly.AssemblyDefinition;
import materia.project.SceneArtifact;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import robotkit.manipulation.Manipulator;
import robotkit.collision.CollisionSceneBuilder;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.IkTolerance;
import motionkit.path.PoseMath;

/** Standalone arm diagnostics; no cell, trays, production plan or physical gripper. */
class ArmDesignReview {
  static final ROOT=ReferenceAssetLocation.DIRECTORY+"/arm-design";
  static function pose(p:Pose3):Dynamic return {x:p.x,y:p.y,z:p.z,qx:p.qx,qy:p.qy,qz:p.qz,qw:p.qw};
  static function wanted(p:Dynamic):Pose3 return new Pose3(p.x,p.y,p.z,p.qx,p.qy,p.qz,p.qw);
  static function proof(clearance:robotkit.collision.CollisionClearance,path:Array<Array<Float>>):Dynamic {
    var zeros=[for(q in path) [for(_ in q) 0.0]],curve=new motionkit.planner.JointPathSamples([for(i in 0...path.length) i*1.0],path,zeros,zeros);
    var review:motionkit.robot.JointCurveClearance=null;
    try {
      review=new motionkit.robot.JointCurveClearance(clearance,curve,1e-6,false);
      var clear=review.check();
      return {clear:clear,queries:review.queries,failedSpan:review.failedSpan,witness:review.failure,error:null};
    } catch(error:Dynamic) return {clear:false,error:Std.string(error),queries:review==null ? 0 : review.queries};
  }
  public static function run():Void {
    var inputText=sys.io.File.getContent(ROOT+"/inputs.json"),input:Dynamic=haxe.Json.parse(inputText);
    var rows:Array<Dynamic> = input.models,targets:Array<Dynamic> = input.targets,tasks:Array<Dynamic> = input.paths;
    var moduleHash=haxe.crypto.Sha256.make(sys.io.File.getBytes(Sys.programPath())),moduleHex=new StringBuf();
    for(i in 0...moduleHash.length) moduleHex.add(StringTools.hex(moduleHash.get(i),2).toLowerCase());
    var dependencies=[for(name in ["ComparisonProject.hx","ReferenceRobot.hx","ReferenceKinematics.hx"]) {
      name:name,sha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(ReferenceAssetLocation.DIRECTORY+"/../"+name))
    }];
    for(row in rows) {
      var id=Std.string(row.id),raw=ComparisonProject.source(id),sourceHash=ComparisonReview.fingerprint(raw);
      if(sourceHash!=Std.string(row.sourceFingerprint)) throw "Changed standalone source: "+id;
      var scene=ComparisonProject.withTool(raw),definition:AssemblyDefinition=scene.assemblyDefinition;
      var converted=AssemblySimulationBridge.toRobotModel(definition,AssemblyPhysicalPartView.fromSceneArtifact(scene));
      var frames=[for(f in converted.model.frames) if(StringTools.endsWith(f.name," robot flange")) f];
      if(frames.length!=1) throw "Standalone arm needs one flange";
      var limits:Array<Array<Float>> = row.limits;
      var moving=[for(j in converted.model.joints) if(j.type!=robotkit.model.JointType.Fixed) j];
      if(moving.length!=6) throw "Standalone arm needs six joints";
      for(i in 0...6) {moving[i].limits.lower=limits[i][0];moving[i].limits.upper=limits[i][1];}
      var arm=new Manipulator(converted.model,converted.model.links[0].id,frames[0].id,
        new Transform3(new Vec3(0,0,0.15),Quat.identity()));
      var solver=new ReferenceKinematics(arm),tolerance=new IkTolerance(1e-6,1e-6,400,0.005);
      function bounded(q:Array<Float>):Bool {
        if(q==null || q.length!=6) return false;
        for(i in 0...6) if(!Math.isFinite(q[i]) || q[i]<limits[i][0]-1e-9 || q[i]>limits[i][1]+1e-9) return false;
        return true;
      }
      var probes:Array<Array<Float>> = row.probes;
      var builder=new CollisionSceneBuilder(arm,probes[0],id,cast input.collisionMarginMetres);
      var clearance=builder.clearance(() -> new collisionkit.native.NativeCollisionWorld());
      var observations:Array<Dynamic> = [],queries:Array<Dynamic> = [],paths:Array<Dynamic> = [],wraps:Array<Dynamic> = [];
      var initial:Array<Float> = null;
      for(index in 0...probes.length) {
        var q=probes[index];if(!bounded(q)) throw "Invalid standalone probe";
        var p=solver.forward(q),violation=clearance.violation(q),ik:Dynamic=null;
        if(violation==null && initial==null) initial=q.copy();
        if(index%4==0 && violation==null) {
          var seed=[for(i in 0...6) Math.max(limits[i][0],Math.min(limits[i][1],q[i]+0.02*Math.sin(i+1)))];
          var solved=solver.solvePose(p,seed,tolerance);
          if(solved==null || !bounded(solved) || PoseMath.distance(solver.forward(solved),p)>1e-6 || PoseMath.angle(solver.forward(solved),p)>1e-6)
            throw "Standalone reachable FK/IK regression: "+id+" probe "+index;
          ik={positionError:PoseMath.distance(solver.forward(solved),p),angleError:PoseMath.angle(solver.forward(solved),p)};
        }
        observations.push({q:q,tcp:pose(p),clear:violation==null,witness:violation,jacobian:arm.tcpJacobian(q),ik:ik});
      }
      function candidates(target:Pose3,seed:Null<Array<Float>>):Array<Array<Float>> {
        var options:Array<Array<Float>> = solver.exact==null ? solver.sampleCandidates(target,8,tolerance) :
          [for(b in solver.exact.branches(target,seed==null ? [0.0,0.0,0.0,0.0,0.0,0.0] : seed)) b.q];
        if(options.length==0) {
          var q=solver.solvePose(target,seed==null ? [0.2,-0.8,1.0,0.1,-0.6,0.2] : seed,tolerance);
          if(q!=null) options.push(q);
        }
        return options;
      }
      for(target in targets) {
        var goal=wanted(target.pose),selected:Array<Float> = null,witness:Dynamic=null;
        var options=candidates(goal,null);
        for(q in options) if(bounded(q)) {
          if(PoseMath.distance(solver.forward(q),goal)>1e-6 || PoseMath.angle(solver.forward(q),goal)>1e-6) throw "Standalone target IK residual";
          witness=clearance.violation(q);if(witness==null) {selected=q;break;}
        }
        queries.push({id:target.id,q:selected,clear:selected!=null,witness:witness,
          reason:selected!=null ? "clear_solution_found" : options.length==0 ? "no_solution_found" : "no_clear_candidate_found"});
      }
      for(task in tasks) {
        var samples:Array<Dynamic> = task.poses,path:Array<Array<Float>> = [],failure:Dynamic=null,maximumJump=0.0;
        for(index in 0...samples.length) {
          var goal=wanted(samples[index]),seed=path.length==0 ? null : path[path.length-1],options=candidates(goal,seed),chosen:Array<Float> = null;
          if(seed!=null) options.sort((a,b) -> {
            var da=0.0,db=0.0;for(i in 0...6) {da+=(a[i]-seed[i])*(a[i]-seed[i]);db+=(b[i]-seed[i])*(b[i]-seed[i]);}return da<db ? -1 : da>db ? 1 : 0;
          });
          for(q in options) if(bounded(q) && clearance.violation(q)==null) {chosen=q;break;}
          if(chosen==null) {failure={sample:index,reason:"No clear IK candidate found"};break;}
          if(PoseMath.distance(solver.forward(chosen),goal)>1e-6 || PoseMath.angle(solver.forward(chosen),goal)>1e-6) throw "Standalone path IK residual";
          if(seed!=null) for(i in 0...6) maximumJump=Math.max(maximumJump,Math.abs(chosen[i]-seed[i]));
          path.push(chosen);
        }
        var continuous:Dynamic=failure==null ? proof(clearance,path) : {clear:false,error:null};
        paths.push({id:task.id,path:path,complete:failure==null,failure:failure,maxJointStep:maximumJump,continuous:continuous,
          scope:"Joint quintics through Cartesian knots; straight Cartesian motion between knots and timed dynamics are not certified"});
      }
      for(j in 0...6) if(limits[j][1]-limits[j][0]>=2*Math.PI+0.2) {
        var a=probes[0].copy(),b=a.copy();a[j]=limits[j][0]+0.1;b[j]=a[j]+2*Math.PI;
        var pa=solver.forward(a),pb=solver.forward(b);
        if(PoseMath.distance(pa,pb)>1e-8 || PoseMath.angle(pa,pb)>1e-7) throw "Joint periodicity regression";
        wraps.push({joint:j,a:a[j],b:b[j],positionError:PoseMath.distance(pa,pb),angleError:PoseMath.angle(pa,pb),scope:"FK equivalence only; not a clear full-turn path or harness permission"});
      }
      clearance.world.dispose();
      if(initial==null) throw "No hull-clear standalone observation";
      var state=new cadkit.modeling.AssemblyState(definition);
      var joints=[for(j in definition.joints) if(j.type!="fixed") j];
      for(i in 0...6) state.setJoint(joints[i].id,initial[i]);scene.assemblyState=state.record();
      sys.io.File.saveBytes(ROOT+"/"+id+".mtrg",SceneArtifact.encode(scene));
      var result={model:id,collisionMarginMetres:input.collisionMarginMetres,sourceFingerprint:sourceHash,moduleSha256:moduleHex.toString(),dependencies:dependencies,inputSha256:haxe.crypto.Sha256.encode(inputText),
        producerSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(ReferenceAssetLocation.DIRECTORY+"/../ArmDesignReview.hx")),
        observations:observations,targets:queries,paths:paths,periodicity:wraps,numericFallbacks:solver.numericFallbacks,
        ikBackend:solver.exact==null ? "numeric_fallback" : "analytic_with_numeric_fallback",scope:"Standalone arm with common 40x40x150 mm tool box, configured margin, standard articulation exclusions. No fixtures, gripper, cell or hardware qualification."};
      sys.io.File.saveContent(ROOT+"/"+id+"-native.json",haxe.Json.stringify(result,null,"  "));
      Sys.println(id+": "+observations.length+" standalone probes; "+[for(q in queries) if(q.clear==true) q].length+"/"+queries.length+" target solutions; "+[for(p in paths) if(p.continuous.clear==true) p].length+"/"+paths.length+" continuous basic paths");Sys.stdout().flush();
    }
  }
}
