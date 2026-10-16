import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;

/** A diagnostic scene at the chosen candidate base, with frozen mill and jaw states. */
class RegisteredCellPreview {
  static final ROOT=ReferenceAssetLocation.DIRECTORY+"/cell-registration";
  static function scaled(f:AssemblyFrame,s:Float):AssemblyFrame return {
    x:f.x*s,y:f.y*s,z:f.z*s,qx:f.qx,qy:f.qy,qz:f.qz,qw:f.qw};
  static function frozen(file:String,toolOnly:Bool=false):SceneArtifactData {
    var scene=SceneArtifact.decode(sys.io.File.getBytes(ROOT+"/"+file));
    var state=new cadkit.modeling.AssemblyState(scene.assemblyDefinition,scene.assemblyState);
    var flat=AssemblyDefinitionFlattener.flatten(scene.assemblyDefinition);
    var definition:AssemblyDefinition={schemaVersion:AssemblyDefinitionCodec.VERSION,id:flat.id+"-snapshot",lengthUnit:flat.lengthUnit,
      definitions:[],occurrences:[],joints:[]};
    var used:Map<String,Bool> = [];
    for(o in flat.occurrences) if(!toolOnly || StringTools.startsWith(o.id,"tool/")) {
      o.initialPose=state.worldPose(o.id);o.grounded=!toolOnly;
      definition.occurrences.push(o);used.set(o.definition,true);
    }
    for(d in flat.definitions) if(used.exists(d.id)) {
      d.connectors.push({name:"registrationOrigin",frame:AssemblyFrames.identity()});
      definition.definitions.push(d);
    }
    scene.parts=[for(p in scene.parts) if(used.exists(p.id)) p];
    scene.assemblyDefinition=definition;scene.assemblyState=null;scene.missions=null;scene.controllers=null;
    scene.machineMotion=null;scene.robotTools=null;scene.project=null;scene.signals=null;scene.machining=null;
    return scene;
  }
  static function pedestal(width:Float,height:Float,base:AssemblyFrame):SceneArtifactData {
    var points:Array<Float> = [],faces=[0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7];
    for(z in [0.0,height]) for(p in [[-width/2,-width/2],[width/2,-width/2],[width/2,width/2],[-width/2,width/2]]) {
      points.push(p[0]);points.push(p[1]);points.push(z);
    }
    var vertices=haxe.io.Bytes.alloc(24*8),normals=haxe.io.Bytes.alloc(24*8),indices=haxe.io.Bytes.alloc(36*4);
    for(i in 0...24) vertices.setDouble(i*8,points[i]);
    for(i in 0...8) normals.setDouble(i*24+16,points[i*3+2]==0 ? -1 : 1);
    for(i in 0...36) indices.setInt32(i*4,faces[i]);
    var volume=width*width*height;
    return {lengthUnit:"m",metresPerUnit:1,parts:[{id:"pedestal",name:"Pedestal clearance envelope",red:0.35,green:0.4,blue:0.45,
      materialId:"steel",materialDensity:7850,materialSpec:"Solid clearance envelope; pedestal fabrication and anchoring pending",
      volume:volume,centerOfMass:[0.0,0.0,height/2],inertia:[volume*(width*width+height*height)/12,0.0,0.0,0.0,volume*(width*width+height*height)/12,0.0,0.0,0.0,volume*width*width/6],
      vertexCount:8,indexCount:36,vertices:vertices,normals:normals,indices:indices,faceRanges:[{faceIndex:0,firstIndex:0,indexCount:36}]}],
      assemblyDefinition:{schemaVersion:AssemblyDefinitionCodec.VERSION,id:"candidate-pedestal",lengthUnit:"m",
        definitions:[{id:"pedestal",connectors:[]}],occurrences:[{id:"pedestal",definition:"pedestal",grounded:true,
          initialPose:{x:base.x,y:base.y,z:0.0,qx:0.0,qy:0.0,qz:0.0,qw:1.0}}],joints:[]}};
  }
  public static function save(id:String,base:AssemblyFrame,q:Array<Float>,mountZ:AssemblyFrame,tcpA:AssemblyFrame,pedestalWidth:Float,seats:Array<Dynamic>):Void {
    var output:SceneArtifactData={lengthUnit:"m",metresPerUnit:1,parts:[],
      assemblyDefinition:{schemaVersion:AssemblyDefinitionCodec.VERSION,id:id+"-registered-cell-review",lengthUnit:"m",definitions:[],occurrences:[],joints:[]},
      recipeDiagnostics:["Candidate CAD cell registration. Frozen mill loading state and open gripper jaws; no tending mission or hardware qualification.",
        "Trays/table are proposed study placements. Arm mounting adapter and physical installation remain unverified."]};
    ComparisonProject.append(output,ComparisonProject.source(id),"arm/",base);
    ComparisonProject.append(output,frozen("mill.mtrg"),"mill/",AssemblyFrames.identity());
    ComparisonProject.append(output,frozen("trays.mtrg"),"trays/",AssemblyFrames.identity());
    ComparisonProject.append(output,pedestal(pedestalWidth,base.z,base),"stand/",AssemblyFrames.identity());
    var definition:AssemblyDefinition=output.assemblyDefinition;
    var stock=[for(o in definition.occurrences) if(o.id=="mill/mill/stock") o][0];
    for(seat in seats) definition.occurrences.push({id:"inventory/"+Std.string(seat.name),definition:stock.definition,grounded:true,
      initialPose:scaled(CellRegistration.readFrame(seat.frame),0.001)});
    var moving=[for(j in definition.joints) if(j.type!="fixed") j];
    var state=new cadkit.modeling.AssemblyState(definition);
    for(i in 0...moving.length) state.setJoint(moving[i].id,q[i]);
    var terminal=[for(o in definition.occurrences) for(d in definition.definitions)
      if(d.id==o.definition && d.robotFlangeConnector!=null) o];
    if(terminal.length!=1) throw "Registered preview requires one robot flange";
    var root=terminal[0],component=[for(d in definition.definitions) if(d.id==root.definition) d][0];
    var connector=[for(c in component.connectors) if(c.name==component.robotFlangeConnector) c][0];
    var worldFlange=state.worldConnector(root.id,component.robotFlangeConnector);
    var tool=frozen("tool-empty.mtrg",true);
    var inverseMount=AssemblyFrames.inverse(scaled(mountZ,0.001));
    var localPoses=[for(o in (cast tool.assemblyDefinition:AssemblyDefinition).occurrences) {
      id:"eoat/"+o.id,frame:AssemblyFrames.compose(inverseMount,scaled(o.initialPose,tool.metresPerUnit))
    }];
    ComparisonProject.append(output,tool,"eoat/",AssemblyFrames.compose(worldFlange,inverseMount));
    for(i in 0...localPoses.length) {
      var entry=localPoses[i],name='registeredTool$i';
      component.connectors.push({name:name,frame:AssemblyFrames.compose(connector.frame,entry.frame)});
      definition.joints.push({id:'tool-fixed-$i',type:"fixed",role:"tree",parent:root.id,parentConnector:name,
        child:entry.id,childConnector:"registrationOrigin",axis:{x:0,y:0,z:1},limits:{lower:null,upper:null,velocity:null,effort:null},defaultValue:0});
    }
    state=new cadkit.modeling.AssemblyState(definition);
    for(i in 0...moving.length) state.setJoint(moving[i].id,q[i]);
    output.assemblyState=state.record();
    var bytes=SceneArtifact.encode(output),saved=SceneArtifact.decode(bytes);
    var actual=new cadkit.modeling.AssemblyState(saved.assemblyDefinition,saved.assemblyState);
    var expectedTcp=AssemblyFrames.compose(worldFlange,AssemblyFrames.compose(inverseMount,scaled(tcpA,0.001)));
    var savedTcp=actual.worldConnector("eoat/tool/gripperA/body","tcp");
    if(Math.abs(expectedTcp.x-savedTcp.x)+Math.abs(expectedTcp.y-savedTcp.y)+Math.abs(expectedTcp.z-savedTcp.z)>1e-8)
      throw "Registered saved tool TCP differs";
    sys.io.File.saveBytes(ROOT+"/"+id+"-cell.mtrg",bytes);
    var meshDirectory=ROOT+"/"+id+"-preview";
    sys.FileSystem.createDirectory(meshDirectory);
    var entries:Array<Dynamic> = [];
    for(part in saved.parts) {
      var stem=part.id.split("/").join("_");
      sys.io.File.saveBytes(meshDirectory+"/"+stem+".vertices",part.vertices);
      sys.io.File.saveBytes(meshDirectory+"/"+stem+".indices",part.indices);
    }
    for(o in (cast saved.assemblyDefinition:AssemblyDefinition).occurrences) {
      var part=[for(p in saved.parts) if(p.id==o.definition) p][0];
      entries.push({id:o.id,stem:part.id.split("/").join("_"),pose:actual.worldPose(o.id),color:[part.red,part.green,part.blue]});
    }
    sys.io.File.saveContent(meshDirectory+"/meshes.json",haxe.Json.stringify(entries));
    Sys.println(id+" saved registered CAD review with checked physical tool TCP");
  }
}
