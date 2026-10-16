import haxe.io.Bytes;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneArtifact.SceneArtifactPart;

/** Metre-scale comparison of prepared arms with an identical synthetic tool. */
class ComparisonProject {
  public static final ROOT = ReferenceAssetLocation.DIRECTORY;
  static function scaled(frame:AssemblyFrame, factor:Float):AssemblyFrame
    return {x:frame.x*factor,y:frame.y*factor,z:frame.z*factor,qx:frame.qx,qy:frame.qy,qz:frame.qz,qw:frame.qw};
  static function streamScaled(bytes:Bytes,factor:Float):Bytes {
    if (bytes == null) return null;
    var output = Bytes.alloc(bytes.length);
    for (i in 0...Std.int(bytes.length/8)) output.setDouble(i*8,bytes.getDouble(i*8)*factor);
    return output;
  }
  public static function source(id:String):SceneArtifactData {
    if (id == "zeroerr") {
      var directory = ROOT+"/../../zeroerr-arm";
      var stem = directory+"/build/straight/zeroerr5-straight80";
      var bytes = sys.io.File.getBytes(stem+".mtrg");
      var record:Dynamic = haxe.Json.parse(sys.io.File.getContent(stem+".provenance.json"));
      var current:Dynamic = haxe.Json.parse(sys.io.File.getContent(directory+"/build/straight/current.json"));
      var preparation:Dynamic = haxe.Json.parse(sys.io.File.getContent(Std.string(current.preparationPath)));
      var hash = haxe.crypto.Sha256.make(bytes), hex = new StringBuf();
      for (i in 0...hash.length) hex.add(StringTools.hex(hash.get(i),2).toLowerCase());
      if (Std.string(record.artifactSha256) != hex.toString() ||
          Std.string(record.layoutSha256) != haxe.crypto.Sha256.encode(sys.io.File.getContent(Std.string(current.layoutPath))) ||
          Std.string(record.exporterSha256) != haxe.crypto.Sha256.encode(sys.io.File.getContent(directory+"/ZeroErrArmPreview.hx")) ||
          Std.string(record.planningWindowSha256) != haxe.crypto.Sha256.encode(sys.io.File.getContent(directory+"/straight-planning-window.json")) ||
          Std.string(preparation.inputs.configSha256) != haxe.crypto.Sha256.encode(sys.io.File.getContent(directory+"/straight-link-config.json")) ||
          Std.string(preparation.inputs.generatorSha256) != haxe.crypto.Sha256.encode(sys.io.File.getContent(directory+"/straight-link.cpp")) ||
          Std.string(preparation.inputs.preparerSha256) != haxe.crypto.Sha256.encode(sys.io.File.getContent(directory+"/prepare-straight.py")) ||
          Std.string(preparation.inputs.baselineLayoutSha256) != haxe.crypto.Sha256.encode(sys.io.File.getContent(directory+"/revised-layout.json")))
        throw "Straight comparison source is stale; prepare and export it again";
      return SceneArtifact.decode(bytes);
    }
    return new ReferenceRobot(ROOT+"/"+id).scene;
  }
  public static function taskSamples(id:String):Array<Array<Float>> {
    var raw:Dynamic = haxe.Json.parse(sys.io.File.getContent(ROOT+"/comparison/"+id+"-task.json"));
    var rows:Array<Array<Float>> = raw.samples;
    if (rows == null || rows.length != 21) throw "Comparison task requires 21 samples";
    for (row in rows) {
      if (row.length != 6) throw "Comparison task requires six coordinates";
      for (value in row) if (!Math.isFinite(value)) throw "Nonfinite comparison coordinate";
    }
    return rows;
  }
  /** The tool body is a 40 x 40 x 150 mm envelope, not a released mounting adapter. */
  static function tool(id:String):SceneArtifactPart {
    var values = [-0.02,-0.02,0.0, 0.02,-0.02,0.0, 0.02,0.02,0.0, -0.02,0.02,0.0,
      -0.02,-0.02,0.15, 0.02,-0.02,0.15, 0.02,0.02,0.15, -0.02,0.02,0.15];
    var faces = [0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7];
    var vertices = Bytes.alloc(24*8), normals = Bytes.alloc(24*8), indices = Bytes.alloc(36*4);
    for (i in 0...24) vertices.setDouble(i*8,values[i]);
    for (i in 0...8) {
      var x = values[i*3],y = values[i*3+1],z = values[i*3+2]-0.075;
      var length = Math.sqrt(x*x+y*y+z*z);
      normals.setDouble(i*24,x/length);normals.setDouble(i*24+8,y/length);normals.setDouble(i*24+16,z/length);
    }
    for (i in 0...36) indices.setInt32(i*4,faces[i]);
    var volume = 0.04*0.04*0.15;
    var inertia = [(0.04*0.04+0.15*0.15)/12,0.0,0.0,0.0,(0.04*0.04+0.15*0.15)/12,0.0,0.0,0.0,2*0.04*0.04/12];
    return {id:id,name:"Common synthetic tool envelope",red:0.92,green:0.56,blue:0.19,
      materialId:"comparison-tool",materialDensity:1/volume,materialSpec:"1 kg uniform comparison envelope; mounting hardware unspecified",
      volume:volume,centerOfMass:[0.0,0.0,0.075],inertia:[for (v in inertia) v*volume],
      physicalMassKg:1,physicalMassDeclared:true,physicalInertiaKgM2:inertia,
      vertexCount:8,indexCount:36,vertices:vertices,normals:normals,indices:indices,
      faceRanges:[{faceIndex:0,firstIndex:0,indexCount:36}]};
  }
  public static function withTool(scene:SceneArtifactData):SceneArtifactData {
    var definition:AssemblyDefinition = scene.assemblyDefinition;
    var terminal = [for (occurrence in definition.occurrences) for (component in definition.definitions)
      if (component.id == occurrence.definition && component.robotFlangeConnector != null) occurrence];
    if (terminal.length != 1) throw "Comparison source needs exactly one flange";
    var flange = terminal[0];
    var component = [for (d in definition.definitions) if (d.id == flange.definition) d][0];
    var connector = [for (c in component.connectors) if (c.name == component.robotFlangeConnector) c][0];
    var factor = 1/scene.metresPerUnit;
    var part = tool("comparison-tool");
    part.vertices = streamScaled(part.vertices,factor);
    part.volume = (cast part.volume:Float)*Math.pow(factor,3);
    part.centerOfMass = [0.0,0.0,0.075*factor];
    part.inertia = [for (v in part.inertia) v*Math.pow(factor,5)];
    // The physical mass/inertia fields are already SI and remain unchanged.
    scene.parts.push(part);
    definition.definitions.push({id:part.id,connectors:[{name:"origin",frame:AssemblyFrames.identity()},
      {name:"tcp",frame:scaled({x:0,y:0,z:0.15,qx:0,qy:0,qz:0,qw:1},factor)}]});
    definition.occurrences.push({id:part.id,definition:part.id,
      initialPose:AssemblyFrames.compose(flange.initialPose,connector.frame)});
    definition.joints.push({id:"comparison-tool-mount",type:"fixed",role:"tree",parent:flange.id,
      parentConnector:component.robotFlangeConnector,child:part.id,childConnector:"origin",axis:{x:0,y:0,z:1},
      limits:{lower:null,upper:null,velocity:null,effort:null},defaultValue:0});
    return scene;
  }
  public static function append(target:SceneArtifactData,source:SceneArtifactData,prefix:String,mount:AssemblyFrame):Void {
    var definition:AssemblyDefinition = source.assemblyDefinition;
    var out:AssemblyDefinition = target.assemblyDefinition;
    var factor = source.metresPerUnit;
    if (definition.assemblies != null && definition.assemblies.length>0 ||
        definition.couplings != null && definition.couplings.length>0 ||
        definition.actuators != null && definition.actuators.length>0 ||
        definition.mates != null && definition.mates.length>0)
      throw "Comparison merger supports flat uncoupled geometry sources only";
    for (part in source.parts) {
      part.id = prefix+part.id;
      part.vertices = streamScaled(part.vertices,factor);
      part.edgeSegments = streamScaled(part.edgeSegments,factor);
      if (part.volume != null) part.volume = (cast part.volume:Float)*Math.pow(factor,3);
      if (part.centerOfMass != null) part.centerOfMass = [for (v in part.centerOfMass) v*factor];
      if (part.inertia != null) part.inertia = [for (v in part.inertia) v*Math.pow(factor,5)];
      // B-rep face and thread metadata refers to source units and is not used in this comparison.
      part.faceDescriptors = null; part.threads = null;
      target.parts.push(part);
    }
    for (component in definition.definitions) {
      component.id = prefix+component.id;
      for (connector in component.connectors) connector.frame = scaled(connector.frame,factor);
      if (component.collisionHulls != null)
        component.collisionHulls = [for (hull in component.collisionHulls) [for (value in hull) value*factor]];
      out.definitions.push(component);
    }
    for (occurrence in definition.occurrences) {
      occurrence.id = prefix+occurrence.id;occurrence.definition = prefix+occurrence.definition;
      occurrence.initialPose = AssemblyFrames.compose(mount,scaled(occurrence.initialPose,factor));
      out.occurrences.push(occurrence);
    }
    for (joint in definition.joints) {
      if (joint.type == "prismatic") throw "Comparison sources require angular or fixed joints";
      joint.id = prefix+joint.id;joint.parent = prefix+joint.parent;joint.child = prefix+joint.child;
      if (joint.closureTolerance != null) joint.closureTolerance *= factor;
      out.joints.push(joint);
    }
  }
  public static function scene(mounting:String = "floor"):SceneArtifactData {
    var rotation = switch mounting {
      case "floor": AssemblyFrames.identity();
      case "wall": {x:0.0,y:0.0,z:1.1,qx:Math.sqrt(0.5),qy:0.0,qz:0.0,qw:Math.sqrt(0.5)};
      case "ceiling": {x:0.0,y:0.0,z:1.5,qx:1.0,qy:0.0,qz:0.0,qw:0.0};
      default: throw "Unknown comparison mounting";
    };
    var target:SceneArtifactData = {lengthUnit:"m",metresPerUnit:1,parts:[],
      assemblyDefinition:{schemaVersion:AssemblyDefinitionCodec.VERSION,id:"arm-comparison-"+mounting,lengthUnit:"m",
        definitions:[],occurrences:[],joints:[]},
      recipeDiagnostics:["Actual-scale side-by-side geometry; 1.2 m display spacing, not a registered cell.",
        "Identical 40 x 40 x 150 mm / 1 kg tool envelope; mounting adapter and drive dynamics unqualified."]};
    var ids = ["zeroerr","ur5e","fairino5_v6"];
    for (i in 0...ids.length) {
      var raw = source(ids[i]);
      var report:Dynamic = haxe.Json.parse(sys.io.File.getContent(ROOT+"/comparison/"+ids[i]+"-task.json"));
      if (Std.string(report.sourceFingerprint) != ComparisonReview.fingerprint(raw)) throw "Stale comparison task; rerun --comparison";
      var source = withTool(raw);
      var placement = {x:rotation.x+(i-1)*1.2,y:rotation.y,z:rotation.z,
        qx:rotation.qx,qy:rotation.qy,qz:rotation.qz,qw:rotation.qw};
      append(target,source,ids[i]+"/",placement);
    }
    AssemblyDefinitionCodec.validate(target.assemblyDefinition);
    var state = new cadkit.modeling.AssemblyState(target.assemblyDefinition);
    for (id in ids) {
      var q = taskSamples(id)[0];
      var merged:AssemblyDefinition = target.assemblyDefinition;
      var moving = [for (j in merged.joints) if (StringTools.startsWith(j.id,id+"/") && j.type != "fixed") j];
      for (i in 0...moving.length) state.setJoint(moving[i].id,q[i]);
    }
    target.assemblyState = state.record();
    return target;
  }
  public static function floor():Bytes return SceneArtifact.encode(scene("floor"));
  public static function wall():Bytes return SceneArtifact.encode(scene("wall"));
  public static function ceiling():Bytes return SceneArtifact.encode(scene("ceiling"));
}
