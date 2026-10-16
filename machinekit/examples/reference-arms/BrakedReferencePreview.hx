import materia.project.SceneArtifact;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinition;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;

/** Frozen supplier geometry with whole-module inertials and brake-capable drives. */
class BrakedReferencePreview {
  public static function slowMove():Void {
    var root=ReferenceAssetLocation.DIRECTORY+"/arm-design";
    var scene=SceneArtifact.decode(sys.io.File.getBytes(root+"/zeroerr-braked.mtrg"));
    var definition:AssemblyDefinition=scene.assemblyDefinition;
    var joints=[for(j in definition.joints) if(j.type!="fixed") j];
    var distance=5*Math.PI/180;
    for(j in joints) {j.limits.velocity=2*Math.PI/180;j.limits.acceleration=4*Math.PI/180;}
    var out=[for(_ in joints) 0.0];out[1]=distance;
    // Runtime coordinates start at the saved inspection pose, not the supplier zero.
    scene.machineMotion={belts:[],program:{axes:[for(j in joints) j.id],
      waypoints:[out,[for(_ in joints) 0.0]],loop:false,positionTolerance:0.005}};
    sys.io.File.saveBytes(root+"/zeroerr-slow.mtrg",SceneArtifact.encode(scene));
    Sys.println("Single J2 excursion: +5 degrees and return; 2 deg/s, 4 deg/s^2; one pass, then powered hold");
  }

  public static function run():Void {
    var root=ReferenceAssetLocation.DIRECTORY, vendor=root+"/../../zeroerr-arm";
    var baseline:Dynamic=haxe.Json.parse(sys.io.File.getContent(vendor+"/reference-baseline.json"));
    var layout:Dynamic=haxe.Json.parse(sys.io.File.getContent(vendor+"/reference-layout.json"));
    var config:Dynamic=haxe.Json.parse(sys.io.File.getContent(vendor+"/original-comparison-config.json"));
    if(haxe.crypto.Sha256.encode(sys.io.File.getContent(vendor+"/reference-layout.json"))!=baseline.layoutSha256)
      throw "Frozen layout changed";
    var scene=SceneArtifact.decode(sys.io.File.getBytes(root+"/arm-design/zeroerr-original.mtrg"));
    var definition:AssemblyDefinition=scene.assemblyDefinition;
    var occurrences:Array<Dynamic> = layout.occurrences;
    var densities:Map<String,Float> = ["base"=>2810,"baseCover"=>2700,"upperArm"=>2810,"forearm"=>2700,
      "elbowOutputAdapter"=>2810,"elbowHousingAdapter"=>2810,"toolFlange"=>2810];
    for(part in scene.parts) {
      var module:Dynamic=null;
      var members=[for(o in definition.occurrences) if(o.definition==part.id) o.id];
      if(members.length==0) throw "Unplaced reference part";
      for(row in occurrences) if(row.statorSolid!=null && StringTools.startsWith(members[0],Std.string(row.id)+"-")) module=row;
      if(module!=null) {
        part.physicalMassKg=0;part.physicalMassDeclared=true;
        part.physicalInertiaKgM2=[for(_ in 0...9) 0.0];
        if(StringTools.endsWith(members[0],"-stator")) {
          var family=module.asset=="asset2" ? "eRob110T" : "eRob70T";
          var lump:Dynamic=Reflect.field(baseline.wholeModuleLumps,family);
          var mass:Float=lump.massKg,cog:Array<Float> = lump.cogMm;
          var rotation:Array<Array<Float>> = lump.manualToAssetRotation;
          var origin:Array<Float> = lump.outputOriginAssetMm, packed:Array<Float> = lump.inertiaAtOutputGmm2;
          var tensor=[packed[0],packed[3],packed[4],packed[3],packed[1],packed[5],packed[4],packed[5],packed[2]];
          var squared=0.0;for(v in cog) squared+=v*v*1e-6;
          for(i in 0...3) for(j in 0...3)
            tensor[i*3+j]=tensor[i*3+j]*1e-9-mass*((i==j ? squared : 0.0)-cog[i]*cog[j]*1e-6);
          var center=origin.copy();for(i in 0...3) for(j in 0...3) center[i]+=rotation[i][j]*cog[j];
          part.centerOfMass=[for(v in center) v*0.001/scene.metresPerUnit];
          var rotated=[for(_ in 0...9) 0.0];
          for(i in 0...3) for(j in 0...3) for(k in 0...3) for(l in 0...3)
            rotated[i*3+j]+=rotation[i][k]*tensor[k*3+l]*rotation[j][l];
          part.physicalMassKg=mass;part.physicalInertiaKgM2=rotated;
        }
      } else if(part.id!="comparison-tool") {
        var density=densities.exists(members[0]) ? densities.get(members[0]) : 2700.0;
        part.physicalMassKg=(cast part.volume:Float)*density*Math.pow(scene.metresPerUnit,3);
        part.physicalMassDeclared=true;
        part.physicalInertiaKgM2=[for(v in part.inertia) v*density*Math.pow(scene.metresPerUnit,5)];
      }
    }
    definition.actuators=[];definition.sensors=[];
    var joints=[for(j in definition.joints) if(j.type!="fixed") j];
    if(joints.length!=6) throw "Expected six frozen joints";
    for(i in 0...6) {
      var joint=joints[i],family=i<3 ? "eRob110T" : "eRob70T";
      var rating:Dynamic=Reflect.field(baseline.ratingsAtAssumedNominal100,family);
      var catalog:Dynamic=Reflect.field(config.catalog,family);
      // The native runtime owns servo holding and its brake release interlock.
      joint.passive=false;
      var inertia:Float=catalog.motorRotorInertiaGmm2*1e-9*Math.pow(baseline.nominalRatioForDiagnostics+1,2);
      definition.actuators.push({id:joint.id+"-module",joint:joint.id,drive:"servo",
        maxEffort:rating.ratedNm,maxRate:Math.PI,rotorInertia:inertia,
        // Existing simulator defaults, not measured vendor gains.
        servoStiffness:rating.repeatedStartStopPeakNm/0.01,servoDamping:rating.repeatedStartStopPeakNm,
        ratedTorque:rating.ratedNm,peakTorque:rating.repeatedStartStopPeakNm,ratedSpeed:Math.PI,maxSpeed:Math.PI,
        assumed:["Offline preview only; nominal ratio 100 and simulator servo gains; not hardware tuning"],
        brake:{id:joint.id+"-brake",releaseChannel:joint.id+"-release",releasedSensor:joint.id+"-released",
          holdingEffort:rating.brakeStaticAcceptanceNm,releaseDelay:0.0,applicationDelay:0.0,
          releaseTimeoutMs:1000,feedbackMaxAgeMs:100}});
      definition.sensors.push({id:joint.id+"-released",kind:"brake_released",joint:joint.id});
    }
    // Dedicated offline cabinet; native brake interlock, no hardware connection.
    scene.controllers=[{id:"zeroerr-inspection",kind:"robot",joints:[for(j in joints) j.id],
      sensors:[for(s in definition.sensors) s.id],channels:[for(j in joints) j.id+"-release"]}];
    scene.project={dynamicParts:[],motions:[]};
    // Round-trip before reporting to exercise the same artifact validation as the app.
    var bytes=SceneArtifact.encode(scene),decoded=SceneArtifact.decode(bytes);
    var converted=AssemblySimulationBridge.toRobotModel(decoded.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(decoded),decoded.assemblyState);
    var mass=0.0;for(link in converted.model.links) mass+=link.mass;
    if(Math.abs(mass-24.827373568836893)>1e-8) throw "Whole-module mass mismatch: "+mass;
    var out=root+"/arm-design/zeroerr-braked.mtrg";
    sys.io.File.saveBytes(out,bytes);
    var zero=new cadkit.modeling.AssemblyState(definition),bodies:Array<Dynamic> = [];
    for(o in definition.occurrences) {
      var part=[for(p in scene.parts) if(p.id==o.definition) p][0];
      if(part.physicalMassKg==0) continue;
      var pose=zero.worldPose(o.id),c=part.centerOfMass;
      var p=AssemblyFrames.transformPoint(pose,c[0],c[1],c[2]);
      bodies.push({id:o.id,massKg:part.physicalMassKg,centerMetres:[p.x*scene.metresPerUnit,p.y*scene.metresPerUnit,p.z*scene.metresPerUnit],
        rotation:AssemblyFrames.toRotationMatrix(pose),localInertia:part.physicalInertiaKgM2});
    }
    sys.io.File.saveContent(root+"/arm-design/zeroerr-braked-inertials.json",haxe.Json.stringify({totalMassKg:mass,bodies:bodies},null,"  "));
    Sys.println("Stationary original preview: "+out+"; total mass="+mass+" kg; brakes="+converted.model.brakes.length);
  }
}
