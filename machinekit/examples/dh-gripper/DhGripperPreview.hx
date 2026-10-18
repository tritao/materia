import cadkit.Shape;
import cadkit.modeling.Axis;
import cadkit.modeling.Curve;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import haxe.io.Bytes;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;

/** Pinned supplier reference with a separately routed electrical cable; synchronized reference jaw motion. */
class DhGripperPreview {
  public static final DIRECTORY = "machinekit/examples/dh-gripper";
  static final HASH = "8803e2416e133a9ec4766dc77e85595859bfa27d91081fb9154e1e0cebbc4a7c";
  static final EXIT = new Vector(242.503081202068, -79.27830199531, 307.089453582398);
  static final TAIL = new Vector(395.503081202068, EXIT.y, EXIT.z);
  public static final STRAIGHT_MM = 100.0;
  public static final BEND_RADIUS_MM = 55.0;
  public static final RETURN_MM = 80.0;
  public static final DIAMETER_MM = 6.4;
  static function directory():String {
    return sys.FileSystem.exists(DIRECTORY + "/build/vendor/PGC-140-50.step") ? DIRECTORY : ".";
  }
  static function render(id:String, name:String, shape:Shape, color:Array<Float>):SceneArtifactPart {
    var mesh = shape.tessellate(0.15);
    return {id:id, name:name, red:color[0], green:color[1], blue:color[2],
      physicalMassKg:0, physicalMassDeclared:true,
      vertexCount:mesh.vertexCount, indexCount:mesh.indexCount,
      vertices:mesh.vertices, normals:mesh.normals, indices:mesh.indices,
      edgeSegments:mesh.edgeSegments, edgeIds:mesh.edgeIds,
      faceRanges:[for (r in mesh.faceRanges) {faceIndex:r.faceIndex, firstIndex:r.firstIndex, indexCount:r.indexCount}]};
  }
  // Preserve supplier connector faces without falsely declaring its invalid shell a solid.
  static function renderFaces(id:String,name:String,faces:Array<Shape>,color:Array<Float>):SceneArtifactPart {
    var meshes:Array<cadkit.Mesh> = [];
    var empty=0;
    for(face in faces) {
      try {meshes.push(face.tessellate(0.15));}
      catch(error:Dynamic) {
        if(Std.string(error).indexOf("shape produced no triangles")<0) throw error;
        empty++;
      }
    }
    Sys.println('Connector: ${faces.length} supplier faces, $empty without renderable triangles');
    var nv=0, ni=0;
    for(mesh in meshes) {nv+=mesh.vertexCount; ni+=mesh.indexCount;}
    var vertices=Bytes.alloc(nv*24), normals=Bytes.alloc(nv*24), indices=Bytes.alloc(ni*4);
    var vo=0, io=0, ranges=[];
    for(mesh in meshes) {
      vertices.blit(vo*24,mesh.vertices,0,mesh.vertexCount*24);
      normals.blit(vo*24,mesh.normals,0,mesh.vertexCount*24);
      for(i in 0...mesh.indexCount) indices.setInt32((io+i)*4,mesh.indices.getInt32(i*4)+vo);
      for(r in mesh.faceRanges) ranges.push({faceIndex:ranges.length,firstIndex:io+r.firstIndex,indexCount:r.indexCount});
      vo+=mesh.vertexCount;io+=mesh.indexCount;
    }
    return {id:id,name:name,red:color[0],green:color[1],blue:color[2],physicalMassKg:0,
      physicalMassDeclared:true,vertexCount:nv,indexCount:ni,vertices:vertices,normals:normals,
      indices:indices,faceRanges:ranges};
  }
  static function ends():Array<Vector> {
    var bendStart = EXIT.add(new Vector(STRAIGHT_MM,0,0));
    var bendEnd = bendStart.add(new Vector(BEND_RADIUS_MM,-BEND_RADIUS_MM,0));
    return [bendStart,bendEnd,bendEnd.add(new Vector(0,-RETURN_MM,0))];
  }
  static function cable():Part {
    if (STRAIGHT_MM < 100 || BEND_RADIUS_MM < 50 || DIAMETER_MM >= 2*BEND_RADIUS_MM)
      throw "Cable exit clearance or bend radius is below specification";
    var e = ends(), centre = e[0].add(new Vector(0,-BEND_RADIUS_MM,0));
    var middle = centre.add(new Vector(BEND_RADIUS_MM/Math.sqrt(2), BEND_RADIUS_MM/Math.sqrt(2),0));
    var curves = [Curve.line(EXIT,e[0]), Curve.arc(e[0],middle,e[1]), Curve.line(e[1],e[2])];
    var path = Curve.wire(curves);
    var profile = Sketch.circle(DIAMETER_MM/2,new Plane(EXIT,Vector.Y(),Vector.X()));
    var result = profile.sweep(path);
    profile.close(); path.close(); for (curve in curves) curve.close();
    if (!result.valid() || result.solidCount()!=1) {result.close(); throw "Routed cable is not one valid solid";}
    var length = STRAIGHT_MM+BEND_RADIUS_MM*Math.PI/2+RETURN_MM;
    var expectedVolume = Math.PI*Math.pow(DIAMETER_MM/2,2)*length;
    if (Math.abs(result.shape.volume()/expectedVolume-1)>0.001) {
      result.close(); throw "Cable sweep volume disagrees with analytic centreline length";
    }
    Sys.println('Cable: diameter $DIAMETER_MM mm; straight $STRAIGHT_MM mm; bend R$BEND_RADIUS_MM mm; length $length mm');
    return result;
  }
  public static function preview():Bytes return build(false);
  public static function previewCustom():Bytes return build(true);
  public static function previewSls():Bytes return build(true,"sls-pa12");
  static function build(custom:Bool,variant:String="cnc-aluminium"):Bytes {
    var file = directory()+"/build/vendor/PGC-140-50.step";
    var digest=haxe.crypto.Sha256.make(sys.io.File.getBytes(file)), hex=new StringBuf();
    for (i in 0...digest.length) hex.add(StringTools.hex(digest.get(i),2).toLowerCase());
    if (hex.toString()!=HASH) throw "Supplier STEP revision changed";
    var source = Shape.importStep(file);
    var owned:Array<Shape> = [source];
    try {
      // This AP203 export mixes 30 free shells (housing, cable/connectors) with
      // ten jaw solids. Keep solids whole; never reconstruct the housing from solids alone.
      var shells = source.subshapeCount(CadKit.ShapeKind.Shell);
      var solids = source.subshapeCount(CadKit.ShapeKind.Solid);
      if (shells!=40 || solids!=10) throw 'Supplier topology changed: $shells shells, $solids solids';
      var rigid:Array<Shape> = [], tail:Array<Shape> = [], removed:Array<Shape> = [];
      var jawLeft:Array<Shape> = [], jawRight:Array<Shape> = [];
      for (i in 0...30) {
        var shell = source.subshape(CadKit.ShapeKind.Shell,i); owned.push(shell);
        if (i==18 || i==24) {
          for (j in 0...shell.subshapeCount(CadKit.ShapeKind.Face)) {
            var face = shell.subshape(CadKit.ShapeKind.Face,j); owned.push(face);
            var bounds=face.bounds(), low=bounds.get_min(), high=bounds.get_max();
            if (high.get_x()<=EXIT.x+0.001) rigid.push(face);
            else if (low.get_x()>=TAIL.x-0.001) tail.push(face);
            else removed.push(face);
          }
        } else {
          var target = shell.bounds().get_min().get_x()>400 ? tail : rigid;
          for(j in 0...shell.subshapeCount(CadKit.ShapeKind.Face)) {
            var face=shell.subshape(CadKit.ShapeKind.Face,j); owned.push(face); target.push(face);
          }
        }
      }
      // Two semicylinders per supplier cable span, plus any joining end faces.
      if (removed.length!=6) throw 'Unexpected supplier cable partition: ${removed.length} faces';
      for (i in 0...solids) {
        var solid = source.subshape(CadKit.ShapeKind.Solid,i); owned.push(solid);
        if(i<5) jawLeft.push(solid); else jawRight.push(solid);
        var check = new Part(solid.cloneShape()); var valid=check.valid(); check.close();
        if (!valid) throw 'Invalid supplier jaw solid $i';
      }
      Sys.println("Compounding rigid supplier faces");
      var body = Shape.compound(rigid); owned.push(body);
      if(custom) {
        // Remove supplier pad and fingertip; keep carriage and its two real screws.
        jawLeft=jawLeft.slice(1,4);jawRight=jawRight.slice(1,4);
      }
      var leftOpen=Shape.compound(jawLeft), rightOpen=Shape.compound(jawRight);
      owned.push(leftOpen);owned.push(rightOpen);
      // Source is fully open. Put meshes at closed; joint coordinates open them.
      var left=leftOpen.translate(new Vector(25,0,0).native());owned.push(left);
      var right=rightOpen.translate(new Vector(-25,0,0).native());owned.push(right);
      var padLeft:Shape, padRight:Shape;
      var fingerLeft:Shape=null, fingerRight:Shape=null;
      if(custom) {
        var fl=DhCustomFingers.make(true,variant),fr=DhCustomFingers.make(false,variant);
        fingerLeft=fl.shape.cloneShape();fingerRight=fr.shape.cloneShape();fl.close();fr.close();
        owned.push(fingerLeft);owned.push(fingerRight);
        var pl=DhCustomFingers.contact(true),pr=DhCustomFingers.contact(false);
        padLeft=pl.shape.cloneShape();padRight=pr.shape.cloneShape();pl.close();pr.close();
        var carriages:Array<Shape> = [], fasteners:Array<Shape> = [];
        for(i in 0...2) for(j in 0...3) {
          var shape=(i==0 ? jawLeft[j] : jawRight[j]).translate(new Vector(i==0 ? 25 : -25,0,0).native());owned.push(shape);
          if(j==0) carriages.push(shape);else fasteners.push(shape);
        }
        DhCustomFingers.check(fingerLeft,fingerRight,carriages,fasteners,variant);
        // Jaw motion is X only. A separating Y plane proves housing clearance
        // for every jaw coordinate, even though supplier housing is surface-only.
        for(finger in [fingerLeft,fingerRight])
          if(finger.bounds().get_min().get_y()-body.bounds().get_max().get_y()<1)
            throw "Custom finger has less than 1 mm housing clearance";
        Sys.println("Custom fingers clear the fixed housing by at least 1 mm throughout X travel");
        var l=Shape.compound([left,fingerLeft]),r=Shape.compound([right,fingerRight]);
        owned.push(l);owned.push(r);left=l;right=r;
      } else {
        padLeft=jawLeft[0].translate(new Vector(25,0,0).native());
        padRight=jawRight[0].translate(new Vector(-25,0,0).native());
      }
      owned.push(padLeft);owned.push(padRight);
      var removedCable = Shape.compound(removed); owned.push(removedCable);
      // The old cable must actually touch the retained gripper exit.
      if (body.distance(removedCable)>0.001)
        throw "Cable partition lost its supplier attachment";
      var e=ends();
      var placement = Location.translation(e[2]).compose(Location.rotation(Axis.Z(),-Math.PI/2))
        .compose(Location.translation(TAIL.scale(-1)));
      var route = cable(); owned.push(route.shape.cloneShape()); route.close();
      var cableShape=owned[owned.length-1];
      if (cableShape.distance(body)>0.001)
        throw "Routed cable does not meet the fixed connector exit";
      if (cableShape.bounds().get_min().get_x()<body.bounds().get_max().get_x()-0.001)
        throw "Cable route enters the rigid gripper envelope";

      // Both connector joints are intentional contacts; these are static references.
      var envelope=Shape.compound([body,leftOpen,rightOpen]);owned.push(envelope);
      var b=envelope.bounds(), lo=b.get_min(), hi=b.get_max();
      var offset = new Vector(-(lo.get_x()+hi.get_x())/2,-(lo.get_y()+hi.get_y())/2,-lo.get_z());
      var parts:Array<SceneArtifactPart> = [];
      var shapes=[body,cableShape];
      var ids=["pgc-140-50","electrical-cable","cable-end-connector"];
      var names=["PGC-140-50 fixed housing and cable exit","Routed electrical cable — R55 / 100 mm straight","Supplier cable-end connector"];
      var colors=[[0.48,0.50,0.53],[0.08,0.09,0.11],[0.3,0.32,0.35]];
      for(i in 0...shapes.length) {
        var displayed=shapes[i].translate(offset.native()); owned.push(displayed);
        parts.push(render(ids[i],names[i],displayed,colors[i]));
        shapes[i].exportStep(directory()+'/build/${ids[i]}.step');
      }
      parts.push(renderFaces(ids[2],names[2],tail,colors[2]));
      var model=new cadkit.modeling.AssemblyModel("mm");
      for(i in 0...ids.length) {
        if(i<2) model.add(ids[i]); else {
          var pose=Location.translation(offset).compose(placement).plane;
          // Assembly frame is derived from the same orthonormal cable tangent placement.
          model.add(ids[i],{x:pose.origin.x,y:pose.origin.y,z:pose.origin.z,
            qx:0,qy:0,qz:-Math.sqrt(0.5),qw:Math.sqrt(0.5)});
        }
      }
      for(i in 0...2) {
        var jaw=custom ? (i==0 ? leftOpen : rightOpen).translate(new Vector(i==0 ? 25 : -25,0,0).native()) : (i==0 ? left : right).cloneShape();owned.push(jaw);
        var shape=jaw.translate(offset.native());owned.push(shape);
        var id=i==0 ? "jaw-left" : "jaw-right";
        parts.push(render(id,i==0 ? "Left supplier jaw" : "Right supplier jaw",shape,[0.68,0.7,0.73]));
        shape.exportStep(directory()+'/build/$id.step');
      }
      DhGripperJaws.configure(model);
      if(custom) {
        sys.FileSystem.createDirectory(directory()+"/build/"+variant);
        for(i in 0...2) {
          var id=i==0 ? "finger-left" : "finger-right", jaw=i==0 ? "jaw-left" : "jaw-right";
          var finger=(i==0 ? fingerLeft : fingerRight).translate(offset.native());owned.push(finger);
          parts.push(render(id,(variant=="sls-pa12" ? "SLS PA12 " : "CNC aluminium ")+(i==0 ? "left finger" : "right finger"),finger,variant=="sls-pa12" ? [0.72,0.73,0.75] : [0.72,0.55,0.26]));
          finger.exportStep(directory()+'/build/$variant/$id.step');
          model.add(id);model.connector(jaw,"finger",materia.assembly.AssemblyFrames.identity());
          model.connector(id,"mount",materia.assembly.AssemblyFrames.identity());model.mate(id+"-mount","fixed",jaw,"finger",id,"mount");
        }
        if(variant=="sls-pa12") for(i in 0...2) {
          var component=new machinekit.robotics.OffsetGripperFinger(i==0,variant);
          var sleeves=component.compressionSleeves();
          for(j in 0...sleeves.length) {
            var sleeve=sleeves[j].shape.translate(offset.native());sleeves[j].close();owned.push(sleeve);
            var id='sleeve-$i-$j',jaw=i==0 ? "jaw-left" : "jaw-right";
            parts.push(render(id,"Steel compression sleeve",sleeve,[0.55,0.57,0.6]));
            model.add(id);model.connector(jaw,id,materia.assembly.AssemblyFrames.identity());
            model.connector(id,"mount",materia.assembly.AssemblyFrames.identity());model.mate(id+"-mount","fixed",jaw,id,id,"mount");
            sleeve.exportStep(directory()+'/build/$variant/$id.step');
          }
        }
        var blank=DhCustomFingers.blank(),stock=blank.shape.translate(offset.native());blank.close();owned.push(stock);
        parts.push(render("blank-60","Aluminium blank 60 x 60 x 20 mm",stock,[0.55,0.67,0.78]));model.add("blank-60");
      }
      DhGripperJaws.check(model,left,right,padLeft,padRight,custom ? 26 : 0);
      var definition=model.definition("dh-pgc-articulated-reference");
      for(component in definition.definitions) component.collisionHulls=[];
      var result=SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:parts,
        assemblyDefinition:definition, assemblyState:model.initialState("dh-pgc-articulated-reference").record()});
      for(shape in owned) shape.close();
      Sys.println(custom ? 'Retained six supplier carriage/screw solids and both connectors; fitted two custom fingers' : 'Retained ten supplier jaw solids and both connectors');
      return result;
    } catch(error:Dynamic) {for(shape in owned) shape.close(); throw error;}
  }
  public static function main():Void {
    var custom=Sys.args().indexOf("--supplier")<0;
    var sls=Sys.args().indexOf("--sls")>=0;
    var prefix=custom ? (sls ? "pgc-sls-fingers" : "pgc-custom-fingers") : "pgc-140-50";
    var output=directory()+'/build/$prefix.mtrg';
    if(custom) {
      var family=machinekit.robotics.OffsetGripperFinger.family(),id=sls ? "sls-pa12" : "cnc-aluminium";
      var variant=family.variant(id);
      var bom=[for(side in ["left","right"]) family.create(id,new machinekit.component.ComponentValues().setToken("side",side)).bom];
      sys.io.File.saveContent(directory()+'/build/$prefix-manufacturing.json',haxe.Json.stringify({
        family:family.id,interfaceId:family.interfaceId,variant:variant.id,process:variant.process,material:variant.material,
        status:variant.status,pendingChecks:variant.pendingChecks(),requiredHardwarePerFinger:variant.hardware(),bom:bom,
        hardwareBom:[for(item in variant.hardware()) {partNumber:item.partNumber,description:item.description,
          material:item.material,quantity:item.quantity*2}],
        fingerStepFiles:['build/$id/finger-left.step','build/$id/finger-right.step']},null,"  "));
    }
    var bytes=custom ? (sls ? previewSls() : previewCustom()) : preview();
    sys.io.File.saveBytes(output,bytes);Sys.println('Exported $output');
    for(opening in (custom ? [26,60,76] : [0,25,50])) {
      var scene=SceneArtifact.decode(bytes);
      var definition=scene.assemblyDefinition;
      if(definition==null) throw "Preview lost assembly definition";
      if(custom && opening<60) {
        scene.parts=[for(part in scene.parts) if(part.id!="blank-60") part];
        definition.definitions=[for(d in definition.definitions) if(d.id!="blank-60") d];
        definition.occurrences=[for(o in definition.occurrences) if(o.id!="blank-60") o];
      }
      var state=new cadkit.modeling.AssemblyState(definition);
      state.setJoint(DhGripperJaws.MASTER,(opening-(custom ? 26 : 0))/2);
      scene.assemblyState=state.record();
      sys.io.File.saveBytes(directory()+'/build/$prefix-opening-$opening.mtrg',SceneArtifact.encode(scene));
    }
  }
}
