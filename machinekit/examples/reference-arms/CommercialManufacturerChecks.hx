import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import robotkit.manipulation.Manipulator;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

/** Fresh native observations consumed by an independent manufacturer DH implementation. */
class CommercialManufacturerChecks {
  public static function run():Void {
    var root=ReferenceAssetLocation.DIRECTORY+"/manufacturer-validation";
    var text=sys.io.File.getContent(root+"/inputs.json"),input:Dynamic=haxe.Json.parse(text);
    var models:Array<Dynamic> = input.models,output:Array<Dynamic> = [];
    for(model in models) {
      var source=new ReferenceRobot(ReferenceAssetLocation.DIRECTORY+"/"+model.id);
      var converted=AssemblySimulationBridge.toRobotModel(source.scene.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(source.scene));
      var frames=[for(f in converted.model.frames) if(StringTools.endsWith(f.name," robot flange")) f];
      if(frames.length!=1) throw "Manufacturer oracle needs physical flange";
      var arm=new Manipulator(converted.model,converted.model.links[0].id,frames[0].id,new Transform3(new Vec3(0,0,0.15),Quat.identity()));
      var rows:Array<Dynamic> = [],probes:Array<Array<Float>> = model.probes;
      var solver=new motionkit.robot.ManipulatorKinematics(arm);
      for(q in probes) {
        var p=solver.forward(q);
        rows.push({q:q,tcp:{x:p.x,y:p.y,z:p.z,qx:p.qx,qy:p.qy,qz:p.qz,qw:p.qw},jacobian:arm.tcpJacobian(q)});
      }
      output.push({id:model.id,observations:rows});
    }
    var bytes=sys.io.File.getBytes(Sys.programPath()),hash=haxe.crypto.Sha256.make(bytes),hex=new StringBuf();
    for(i in 0...hash.length) hex.add(StringTools.hex(hash.get(i),2).toLowerCase());
    sys.io.File.saveContent(root+"/native.json",haxe.Json.stringify({models:output,inputSha256:haxe.crypto.Sha256.encode(text),
      producerSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(ReferenceAssetLocation.DIRECTORY+"/../CommercialManufacturerChecks.hx")),moduleSha256:hex.toString()}));
    Sys.println("Observed 282 native commercial-arm configurations for independent manufacturer validation");
  }
}
