import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.AssemblyModel;
import machinekit.component.Solids;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;

/** Exploded 3D review of the dimensioned proposal, deliberately separate from the working enclosure. */
class HobberTrayProposalPreview {
  static function polygon(loop:Array<Array<Float>>,z0:Float,z1:Float):Part
    return Part.prism([for(p in loop)new Vector(p[0],p[1],0)],z0,z1);
  static function shifted(loop:Array<Array<Float>>,offset:Float):Array<Array<Float>> {
    return [for(i in 0...loop.length) {
      var p=loop[i],a=loop[(i+loop.length-1)%loop.length],b=loop[(i+1)%loop.length];
      var vx=p[0]-a[0],vy=p[1]-a[1],wx=b[0]-p[0],wy=b[1]-p[1];
      var vl=Math.sqrt(vx*vx+vy*vy),wl=Math.sqrt(wx*wx+wy*wy);
      [p[0]+offset*(-vy/vl-wy/wl),p[1]+offset*(vx/vl+wx/wl)];
    }];
  }
  static function area(loop:Array<Array<Float>>):Float {
    var sum=0.0;for(i in 0...loop.length){var p=loop[i],q=loop[(i+1)%loop.length];sum+=p[0]*q[1]-q[0]*p[1];}return sum/2;
  }
  static function move(part:Part,x:Float,y:Float,z:Float):Part {var result=part.translated(new Vector(x,y,z));part.close();return result;}
  static function main():Void {
    var root=Sys.getCwd();
    while(!sys.FileSystem.exists(root+"/machinekit/examples/flexspline-hobber/tray-mounting-proposal.json")) {
      var parent=haxe.io.Path.directory(root);if(parent==root||parent.length==0)throw "Cannot find tray proposal";root=parent;
    }
    var folder=root+"/machinekit/examples/flexspline-hobber",p:Dynamic=haxe.Json.parse(sys.io.File.getContent(folder+"/tray-mounting-proposal.json"));
    var loops:Array<Array<Array<Float>>> = p.tray.outlineLoopsMm,points:Array<Array<Float>> = p.sockets.positionsMm;
    var outer=[for(l in loops)if(area(l)>0)l][0],inner=[for(l in loops)if(area(l)<0)l][0];
    var parts:Array<SceneArtifactPart> = [];
    function add(id:String,name:String,body:Part,color:Array<Float>):Void {
      var material=id=="tray-proposal"?"compact-laminate":StringTools.startsWith(id,"anchor-")?"stainless-sheet":StringTools.startsWith(id,"compression-")?"machined-steel":"rubber";
      var rendered=AssemblyPreview.part(id,name,body,material,null,null);
      rendered.red=color[0];rendered.green=color[1];rendered.blue=color[2];parts.push(rendered);
    }
    // Lift the sheet 40 mm and the gasket 15 mm to expose the fastening/sealing interfaces.
    var stack:Dynamic=p.stackZMm;
    var bottom:Float=stack.trayUnderside+40,top:Float=stack.trayTop+40,seat:Float=stack.socketMouthAndSeat;
    var cuts=[polygon(inner,bottom-1,top+1)];
    for(pt in points) {
      cuts.push(move(Part.cylinderSpan(5.25,bottom-1,top+1),pt[0],pt[1],0));
      cuts.push(move(Part.cylinderSpan(8,top-7,top+1),pt[0],pt[1],0));
    }
    var tray=Solids.cut(polygon(outer,bottom,top),cuts);
    if(tray.solidCount()!=1)throw "Proposal tray must be a single solid";
    add("tray-proposal","PROPOSAL: one-piece 13 mm tray; shown 40 mm above installed position",tray,[0.45,0.56,0.64]);
    add("inner-tape-proposal","PROPOSAL: 15 mm tape, 2 mm installed thickness; rear support still to detail",
      Solids.cut(polygon(shifted(inner,15),seat+15,seat+17),[polygon(inner,seat+14,seat+18)]),[0.15,0.65,0.40]);
    add("outer-tape-proposal","PROPOSAL: outer gasket loop; shown 15 mm above installed position",
      Solids.cut(polygon(shifted(outer,5),seat+15,seat+17),[polygon(shifted(outer,20),seat+14,seat+18)]),[0.15,0.65,0.40]);
    var socket=new machinekit.structural.CastInFixingSocket();
    var socketGeometry=socket.geometry();
    if(socketGeometry.solidCount()!=1)throw "Socket must be one solid";
    var freeProbe=Part.cylinderSpan(2.4,-24,-0.2),freeCommon=socketGeometry.intersect(freeProbe);
    if(freeCommon.volume()>0.001)throw "Socket blind insertion cavity is obstructed";
    freeCommon.close();freeProbe.close();
    var closedProbe=Part.cylinderSpan(0.5,-26,-25.1),closedCommon=socketGeometry.intersect(closedProbe);
    if(closedCommon.volume()<0.1)throw "Socket tail must close the insertion cavity";
    closedCommon.close();closedProbe.close();socketGeometry.close();
    var recipeAssembly=new machinekit.assembly.MachineAssembly();recipeAssembly.addComponent("socket",socket);
    var recreated=machinekit.assembly.MachineAssembly.decode(recipeAssembly.encode());
    var recreatedSocket:machinekit.structural.CastInFixingSocket=cast recreated.components()[0].component;
    if(recreatedSocket.parameterNumber("threadLength")!=6||recreatedSocket.parameterNumber("boreDepth")!=25)throw "Socket recipe lost dimensions";
    for(i in 0...points.length) {
      var pt=points[i];
      add("anchor-socket-"+i,"PROPOSAL: M6 x 40 socket, blind bore and crimped tail; OD/tail nominal",
        move(socket.geometry(),pt[0],pt[1],seat),[0.67,0.70,0.74]);
      add("compression-sleeve-"+i,"PROPOSAL: removable compression sleeve OD10 ID6.6 x 8 mm; lifted with sheet",
        move(Solids.cut(Part.cylinderSpan(5,seat+40,seat+48),[Part.cylinderSpan(3.3,seat+39,seat+49)]),pt[0],pt[1],0),[0.75,0.76,0.78]);
    }
    var model=new AssemblyModel("mm");for(part in parts){model.add(part.id);model.ground(part.id);}
    var definition=model.definition("tray-direct-mount-proposal");for(d in definition.definitions)d.collisionHulls=[];
    var scene:materia.project.SceneArtifact.SceneArtifactData={lengthUnit:"mm",metresPerUnit:0.001,parts:parts,assemblyDefinition:definition,
      assemblyState:model.initialState("tray-direct-mount-proposal").record(),recipeDiagnostics:[
        "EXPLODED DESIGN PROPOSAL. Sheet lifted 40 mm; gasket lifted 15 mm. M6 sockets have nominal OD and crimp dimensions; not vendor CAD.",
        "Rear gasket support, actual anchor fit, coolant grade and full machine travel remain unresolved. Separate from the accepted enclosure scene."
      ]};
    var out=folder+"/build/tray-mounting";sys.FileSystem.createDirectory(out);
    var bytes=SceneArtifact.encode(scene),decoded=SceneArtifact.decode(bytes);
    if(decoded.parts.length!=parts.length)throw "Tray proposal save/reopen lost parts";
    sys.io.File.saveBytes(out+"/tray-proposal.mtrg",bytes);
    var detailParts:Array<SceneArtifactPart> = [];
    var detailBody=AssemblyPreview.part("cast-in-socket","M6 x 40 crimped fixing socket; nominal OD/tail, a=25 e=6",socket.geometry(),"stainless-sheet");
    detailParts.push(detailBody);
    var detailModel=new AssemblyModel("mm");detailModel.add(detailBody.id);detailModel.ground(detailBody.id);
    var detailDefinition=detailModel.definition("cast-in-socket-detail");for(d in detailDefinition.definitions)d.collisionHulls=[];
    sys.io.File.saveBytes(out+"/anchor-detail.mtrg",SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:detailParts,
      assemblyDefinition:detailDefinition,assemblyState:detailModel.initialState("cast-in-socket-detail").record(),
      recipeDiagnostics:["PFEIFER M6 x 40 reference: a=25 / e=6. OD8 and tail shape are nominal study assumptions, not manufacturer CAD."]}));
    Sys.println('Tray proposal scene: ${parts.length} parts; one solid sheet, routed holes, gasket loops, sleeves and nominal crimped sockets; cavity, closed tail, recipe and scene save/reopen passed.');
  }
}
