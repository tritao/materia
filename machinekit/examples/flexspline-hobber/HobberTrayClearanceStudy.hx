import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.AssemblyModel;
import machinekit.assembly.PosedParts;

/** Rectangular sheet against existing machine at travel endpoints; no fabricated backing/strength claim. */
class HobberTrayClearanceStudy {
  static function main():Void {
    var root=Sys.getCwd();
    while(!sys.FileSystem.exists(root+"/machinekit/examples/flexspline-hobber/tray-mounting-proposal.json")) {
      var parent=haxe.io.Path.directory(root);if(parent==root||parent.length==0)throw "Cannot find tray proposal";root=parent;
    }
    var folder=root+"/machinekit/examples/flexspline-hobber",p:Dynamic=haxe.Json.parse(sys.io.File.getContent(folder+"/tray-mounting-proposal.json"));
    var blank:Array<Float> = p.tray.blankMm,opening:Array<Float> = p.tray.openingBoundsMm;
    var z=machinekit.hobbing.FoundationLayout.dimensions("bed")[2];
    var raw=Part.box(blank[0],blank[1],blank[2]),sheet=raw.translated(new Vector(0,0,z+p.stackZMm.trayUnderside));raw.close();
    var tool=Part.prism([new Vector(opening[0],opening[1]),new Vector(opening[2],opening[1]),
      new Vector(opening[2],opening[3]),new Vector(opening[0],opening[3])],z+p.stackZMm.trayUnderside-1,z+p.stackZMm.trayTop+1);
    var ring=machinekit.component.Solids.cut(sheet,[tool]);
    var machine=new FlexsplineHobber(),model=new AssemblyModel("mm");machine.addTo(model,"");var state=model.initialState();
    var posed=PosedParts.moved(ring,state.worldPose("bed/casting"));ring.close();
    var box=PosedParts.boxOf(posed),cache=new PosedParts(),samples=0,exact=0,verified:Map<String,Bool> = [];
    var conflicts:Array<Dynamic> = [];
    try {
      var ranges=[for(t in machine.travel)[t.lower,t.upper]];
      for(x in ranges[0])for(y in ranges[1])for(z in ranges[2])for(a in [-Math.PI/12,0.0,Math.PI/12]) {
        state.setJoint("X",x);state.setJoint("Y",y);state.setJoint("Z",z);state.setJoint(FlexsplineHobber.A,a);state.forwardKinematics();
        for(row in machine.components())if(!HobberEnclosurePreview.external(row.id)) {
          var pose=state.worldPose(row.id),key=row.id+haxe.Json.stringify(pose);
          if(verified.exists(key)||PosedParts.apart(box,cache.box(row.component,pose),0))continue;
          verified.set(key,true);exact++;
          var other=cache.posed(row.component,pose),common=posed.intersect(other),volume=common.volume();common.close();other.close();
          if(volume>0.001)conflicts.push({part:row.id,overlapMm3:volume,X:x,Y:y,Z:z,A:a});
        }
        samples++;
      }
      cache.close();posed.close();
    }catch(error:Dynamic){cache.close();posed.close();throw error;}
    sys.io.File.saveContent(folder+"/tray-clearance-screen.json",haxe.Json.stringify({revision:p.revision,samples:samples,exactPairs:exact,
      conflicts:conflicts,scope:"Rectangular sheet vs existing machine at X/Y/Z endpoints and three A positions. No continuous sweep, gasket, frame, supports or sleeve/anchor assembly acceptance."},null,"  "));
    if(conflicts.length>0)throw "Rectangular tray clashes: "+haxe.Json.stringify(conflicts);
    Sys.println('Rectangular tray clears machine in $samples sampled states, $exact unique exact pairs.');
  }
}
