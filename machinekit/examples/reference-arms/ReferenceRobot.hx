import haxe.io.Bytes;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneArtifact.SceneArtifactPart;

@:wire typedef ReferenceDigests = {
  @:id(1) var vertices:String;
  @:id(2) var indices:String;
  @:id(3) @:optional var normals:String;
}
@:wire typedef ReferenceLimits = {
  @:id(1) var lower:Null<Float>;
  @:id(2) var upper:Null<Float>;
  @:id(3) var velocity:Null<Float>;
  @:id(4) var effort:Null<Float>;
}
@:wire typedef ReferenceProbeLink = {
  @:id(1) var id:String;
  @:id(2) var pose:AssemblyFrame;
}
@:wire typedef ReferenceProbe = {
  @:id(1) var q:Array<Float>;
  @:id(2) var tip:AssemblyFrame;
  @:id(3) var links:Array<ReferenceProbeLink>;
}
@:wire typedef ReferenceMesh = {
  @:id(1) var stem:String;
  @:id(2) var vertexCount:Int;
  @:id(3) var indexCount:Int;
  @:id(4) @:optional var volume:Float;
  @:id(5) @:optional var color:Array<Float>;
  @:id(6) var sha256:ReferenceDigests;
  @:id(7) @:optional var metallic:Null<Float>;
  @:id(8) @:optional var roughness:Null<Float>;
}
@:wire typedef ReferenceLink = {
  @:id(1) var id:String;
  @:id(2) var pose:AssemblyFrame;
  @:id(3) var massKg:Null<Float>;
  @:id(4) var centerOfMass:Array<Float>;
  @:id(5) var inertiaKgM2:Null<Array<Float>>;
  @:id(6) var visuals:Array<ReferenceMesh>;
  @:id(7) var collisions:Array<ReferenceMesh>;
}
@:wire typedef ReferenceJoint = {
  @:id(1) var id:String;
  @:id(2) var type:String;
  @:id(3) var parent:String;
  @:id(4) var child:String;
  @:id(5) var origin:AssemblyFrame;
  @:id(6) var axis:Array<Float>;
  @:id(7) var limits:ReferenceLimits;
}
@:wire typedef ReferenceManifest = {
  @:id(1) var schema:Int;
  @:id(2) var id:String;
  @:id(3) var family:String;
  @:id(4) var root:String;
  @:id(5) var tip:String;
  @:id(6) var tipStatus:String;
  @:id(7) var metresPerUnit:Float;
  @:id(8) var urdfSha256:String;
  @:id(9) var links:Array<ReferenceLink>;
  @:id(10) var joints:Array<ReferenceJoint>;
  @:id(11) var probes:Array<ReferenceProbe>;
  @:id(12) @:optional var flangeOffset:AssemblyFrame;
  @:id(13) @:optional var sourceWarnings:Array<String>;
  @:id(14) @:optional var wristInterfaceCatalogSha256:String;
}

/** Official URDF compiled to the same scene/assembly path as authored CAD robots.
 * Source inertias remain declared data. No motor/drive or hardware connection is added.
 */
class ReferenceRobot {
  public final manifest:ReferenceManifest;
  public final scene:SceneArtifactData;
  public final directory:String;
  public final tipOccurrence:String;
  public final wristInterfaces:Array<ReferenceWristInterfaces.WristProfile>;
  final linkAnchors:Map<String,{id:String,offset:AssemblyFrame}> = [];

  function anchorOf(id:String):{id:String,offset:AssemblyFrame} {
    var result = linkAnchors.get(id);
    if (result == null) throw 'Unknown source link $id';
    return result;
  }

  public function linkPose(state:cadkit.modeling.AssemblyState,id:String):AssemblyFrame {
    var anchor = linkAnchors.get(id);
    if (anchor == null) throw 'Unknown source link $id';
    return AssemblyFrames.compose(state.worldPose(anchor.id),anchor.offset);
  }

  static function digest(bytes:Bytes):String {
    var hash = haxe.crypto.Sha256.make(bytes), result = new StringBuf();
    for (i in 0...hash.length) result.add(StringTools.hex(hash.get(i),2).toLowerCase());
    return result.toString();
  }

  function stream(mesh:ReferenceMesh, extension:String, size:Int):Bytes {
    if (!~/^[a-zA-Z0-9_-]+$/.match(mesh.stem)) throw "Unsafe mesh stream path";
    var bytes = sys.io.File.getBytes(directory + "/" + mesh.stem + "." + extension);
    var expected = extension == "vertices" ? mesh.sha256.vertices : extension == "normals" ? mesh.sha256.normals : mesh.sha256.indices;
    if (bytes.length != size || digest(bytes) != expected)
      throw 'Changed mesh stream: ${mesh.stem}.$extension';
    return bytes;
  }

  public function new(directory:String) {
    this.directory = directory;
    manifest = haxeon.wire.JsonWire.decode(sys.io.File.getContent(directory + "/manifest.wire.json"));
    if (manifest.schema != 1 || manifest.metresPerUnit != 1 ||
        digest(sys.io.File.getBytes(directory + "/robot.urdf")) != manifest.urdfSha256)
      throw "Changed or unsupported reference description";
    wristInterfaces=manifest.wristInterfaceCatalogSha256==null ? [] :
      ReferenceWristInterfaces.load(directory,manifest.wristInterfaceCatalogSha256,manifest.id);
    var definition:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION,
      id: manifest.id, lengthUnit: "m", definitions: [], occurrences: [], joints: []};
    var visible = [for (link in manifest.links) if (link.visuals.length>0) link];
    if (visible.length == 0) throw "Reference has no visual links";
    var root = visible[0];
    for (link in manifest.links) {
      var cursor = link.id, found = false;
      while (!found) {
        for (candidate in visible) if (candidate.id == cursor) {found = true; break;}
        if (found) break;
        var incoming = [for (joint in manifest.joints) if (joint.child == cursor) joint];
        if (incoming.length == 0) {cursor = root.id; break;}
        cursor = incoming[0].parent;
      }
      var parent = [for (candidate in visible) if (candidate.id == cursor) candidate][0];
      linkAnchors.set(link.id,{id:cursor,offset:AssemblyFrames.compose(AssemblyFrames.inverse(parent.pose),link.pose)});
      if (link.visuals.length == 0 && link.massKg != null) throw "Mass-bearing invisible links need explicit geometry support";
    }
    var tipAnchor = anchorOf(manifest.tip);
    tipOccurrence = tipAnchor.id;
    var parts:Array<SceneArtifactPart> = [];
    for (link in visible) {
      var component:AssemblyComponentDefinition = {id: link.id,
        connectors: [{name: "origin", frame: AssemblyFrames.identity()}]};
      var hulls:Array<Array<Float>> = [];
      if (link.id == tipOccurrence) {
        var flange = manifest.flangeOffset == null ? AssemblyFrames.identity() : manifest.flangeOffset;
        component.connectors.push({name: "tip", frame: AssemblyFrames.compose(tipAnchor.offset,flange)});
        component.robotFlangeConnector = "tip";
      }
      for (joint in manifest.joints) if (joint.type != "fixed" && anchorOf(joint.parent).id == link.id) {
        component.connectors.push({name: joint.id,
          frame: AssemblyFrames.compose(anchorOf(joint.parent).offset,joint.origin)});
      }
      for (mesh in link.collisions) {
        var vertices = stream(mesh,"vertices",mesh.vertexCount * 24);
        stream(mesh,"indices",mesh.indexCount * 4);
        hulls.push(cadkit.ConvexHullVertices.enclosingFromMesh(vertices,mesh.vertexCount,0.00001,0).vertices);
      }
      if (hulls.length > 0) component.collisionHulls = hulls;
      definition.definitions.push(component);
      definition.occurrences.push({id: link.id, definition: link.id, initialPose: link.pose,
        grounded: link.id == root.id});
      if (link.visuals.length == 0) continue;
      if (link.massKg == null || link.inertiaKgM2 == null) throw 'Visible link lacks source mass/inertia: ${link.id}';
      for (index in 0...link.visuals.length) {
        var mesh=link.visuals[index],id=index==0 ? link.id : link.id+"-visual"+index;
        if(index>0) {
          definition.definitions.push({id:id,connectors:[{name:"origin",frame:AssemblyFrames.identity()}],collisionHulls:[]});
          definition.occurrences.push({id:id,definition:id,initialPose:link.pose});
          definition.joints.push({id:id+"-mount",type:"fixed",role:"tree",parent:link.id,parentConnector:"origin",
            child:id,childConnector:"origin",axis:{x:0,y:0,z:1},
            limits:{lower:null,upper:null,velocity:null,effort:null},defaultValue:0});
        }
        var color=mesh.color,volume:Float=cast mesh.volume;
        var vertices=stream(mesh,"vertices",mesh.vertexCount*24),normals=stream(mesh,"normals",mesh.vertexCount*24),
          indices=stream(mesh,"indices",mesh.indexCount*4);
        for(i in 0...mesh.indexCount) if(indices.getInt32(i*4)<0 || indices.getInt32(i*4)>=mesh.vertexCount)
          throw "Mesh index out of bounds";
        parts.push({id:id,name:manifest.id+" / "+id,red:color[0],green:color[1],blue:color[2],
          appearance:{finish:"vendor",metallic:mesh.metallic==null ? 0.0 : mesh.metallic,roughness:mesh.roughness==null ? 0.65 : mesh.roughness},
          materialId:"neutral",materialSpec:"Source URDF mass/inertia, assigned once per link; vendor visual material regions",
          materialDensity:1,volume:volume,centerOfMass:link.centerOfMass.copy(),
          inertia:[for(v in link.inertiaKgM2) v*volume/(cast link.massKg:Float)],
          physicalMassKg:index==0 ? link.massKg : 0.0,physicalMassDeclared:true,
          physicalInertiaKgM2:index==0 ? link.inertiaKgM2.copy() : [for(_ in 0...9) 0.0],
          vertexCount:mesh.vertexCount,indexCount:mesh.indexCount,vertices:vertices,normals:normals,indices:indices,
          faceRanges:[{faceIndex:0,firstIndex:0,indexCount:mesh.indexCount}]});
      }
    }
    for (joint in manifest.joints) if (joint.type != "fixed") definition.joints.push({id: joint.id, type: cast joint.type, role: "tree",
      parent: anchorOf(joint.parent).id, parentConnector: joint.id, child: joint.child, childConnector: "origin",
      axis: {x:joint.axis[0],y:joint.axis[1],z:joint.axis[2]},
      limits: {lower:joint.limits.lower,upper:joint.limits.upper,velocity:joint.limits.velocity,effort:joint.limits.effort}, defaultValue: 0});
    AssemblyDefinitionCodec.validate(definition);
    scene = {lengthUnit: "m", metresPerUnit: 1, parts: parts, assemblyDefinition: definition,
      recipeDiagnostics: [manifest.tipStatus, "Collision geometry uses enclosing convex hulls of source collision meshes; dynamics not qualified."]};
    scene.recipeDiagnostics.push(wristInterfaces.length==0 ? "Electrical wrist interface not prepared/qualified for this source model." :
      "Source-backed wrist interface profiles available: "+[for(p in wristInterfaces) p.id].join(", ")+". Select hardware revision and rating edition explicitly; socket mating poses remain unqualified.");
    if (manifest.sourceWarnings != null) for (warning in manifest.sourceWarnings) scene.recipeDiagnostics.push(warning);
  }
}
