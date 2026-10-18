import cadkit.Shape;
import cadkit.modeling.AssemblyModel;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;

/** Read-only inspection of the pinned supplier STEP; subshape IDs are evidence, not part semantics. */
class WristGeometryInspection {
  static function vector(v:CadKit.Vec3):Array<Float> return [v.get_x(),v.get_y(),v.get_z()];
  public static function main():Void {
    var root=ReferenceAssetLocation.DIRECTORY;
    var file=root+"/cad/fairino-v6-step/FRCobots-V6.0 STEP Models/FR5-V6.0 Model.STEP";
    var directory=root+"/wrist-geometry-inspection";sys.FileSystem.createDirectory(directory);
    if(ReferenceWristInterfaces.digest(sys.io.File.getBytes(file))!="d7a7e07f50a221dfba9ca807bc2a4241518fdfe162233b7c0d404341e78687be") throw "FR5 source STEP changed; review subshape evidence before inspection";
    var source=Shape.importStep(file),model=new AssemblyModel("mm");
    var parts:Array<SceneArtifactPart> = [],records:Array<Dynamic> = [];
    var count=source.subshapeCount(CadKit.ShapeKind.Solid);
    Sys.println('Inspecting $count FR5 STEP solids');Sys.stdout().flush();
    for(i in 0...count) {
      var solid=source.subshape(CadKit.ShapeKind.Solid,i),bounds=solid.bounds();
      var cylinders:Array<Dynamic> = [],planes:Array<Dynamic> = [],edges:Array<Dynamic> = [];
      for(k in 0...solid.subshapeCount(CadKit.ShapeKind.Face)) {
        var face=solid.subshape(CadKit.ShapeKind.Face,k);
        if(face.surfaceKind()==CadKit.SurfaceKind.Cylinder) {
          var axis=face.faceAxis();
          var fb=face.bounds();
          cylinders.push({low:vector(fb.get_min()),high:vector(fb.get_max()),face:k,radius:axis.get_radius(),origin:vector(axis.get_origin()),direction:vector(axis.get_direction()),centre:vector(face.center())});
        }
        if(i==36 && face.surfaceKind()==CadKit.SurfaceKind.Plane) {
          var fb=face.bounds();planes.push({face:k,centre:vector(face.center()),normal:vector(face.faceNormal()),
            area:face.faceArea(),low:vector(fb.get_min()),high:vector(fb.get_max())});
        }
        face.close();
      }
      if(i==36) for(k in 0...solid.subshapeCount(CadKit.ShapeKind.Edge)) {
        var edge=solid.subshape(CadKit.ShapeKind.Edge,k),eb=edge.bounds();
        if(edge.curveKind()==CadKit.CurveKind.Circle) {
          var axis=edge.edgeAxis();edges.push({edge:k,kind:"circle",origin:vector(axis.get_origin()),direction:vector(axis.get_direction()),
            radius:axis.get_radius(),low:vector(eb.get_min()),high:vector(eb.get_max())});
        } else if(edge.curveKind()==CadKit.CurveKind.Line) edges.push({edge:k,kind:"line",low:vector(eb.get_min()),high:vector(eb.get_max())});
        edge.close();
      }
      var mesh=solid.tessellate(0.15),id="step-solid-"+i;
      sys.io.File.saveBytes(directory+"/"+id+".vertices",mesh.vertices);
      sys.io.File.saveBytes(directory+"/"+id+".indices",mesh.indices);
      records.push({solid:i,low:vector(bounds.get_min()),high:vector(bounds.get_max()),faces:solid.subshapeCount(CadKit.ShapeKind.Face),
        cylinders:cylinders,planes:planes,edges:edges,vertexCount:mesh.vertexCount,indexCount:mesh.indexCount});
      var shade=0.35+0.06*(i%7);
      parts.push({id:id,name:'Supplier FR5 solid $i',red:shade,green:shade,blue:shade+0.05,
        physicalMassKg:0,physicalMassDeclared:true,vertexCount:mesh.vertexCount,indexCount:mesh.indexCount,
        vertices:mesh.vertices,normals:mesh.normals,indices:mesh.indices,edgeSegments:mesh.edgeSegments,edgeIds:mesh.edgeIds,
        faceRanges:[for(r in mesh.faceRanges) {faceIndex:r.faceIndex,firstIndex:r.firstIndex,indexCount:r.indexCount}]});
      model.add(id);model.ground(id);
      if(i==36) {
        solid.exportStep(directory+"/fr5-terminal-wrist.step");
        var isolated=new AssemblyModel("mm");isolated.add(id);isolated.ground(id);
        var terminal=isolated.definition("fr5-source-terminal-wrist");for(d in terminal.definitions) d.collisionHulls=[];
        sys.io.File.saveBytes(directory+"/fr5-terminal-wrist.mtrg",SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,
          parts:[parts[parts.length-1]],assemblyDefinition:terminal,assemblyState:isolated.initialState("fr5-source-terminal-wrist").record()}));
      }
      solid.close();
    }
    source.close();
    var definition=model.definition("supplier-fr5-step-inspection");for(d in definition.definitions) d.collisionHulls=[];
    sys.io.File.saveBytes(directory+"/fr5-step.mtrg",SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:parts,
      assemblyDefinition:definition,assemblyState:model.initialState("supplier-fr5-step-inspection").record()}));
    sys.io.File.saveContent(directory+"/fr5-step.json",haxe.Json.stringify({source:file,
      sha256:ReferenceWristInterfaces.digest(sys.io.File.getBytes(file)),solids:records},null,"  "));
    Sys.println("Exported STEP geometry and exact cylindrical face observations");
  }
}
