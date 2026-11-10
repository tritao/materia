import hobber.*;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import hobber.HobberChipCover;
import materia.assembly.AssemblyFrames as F;

/** Actual-solid ingress samples complement the strict collision sweep. Includes
 * the carriage above the skirt aperture, so its running gap is not waived. */
class HobberChipGuardChecks {
  public static function run(machine:FlexsplineHobber):Array<String> {
    var model = new cadkit.modeling.AssemblyModel("mm"); machine.addTo(model, "");
    var definition = model.definition("chip-coverage");
    for (j in definition.joints) if (j.id == "X") { j.limits.lower=20; j.limits.upper=115; }
    var state = new cadkit.modeling.AssemblyState(definition), cache=new PosedParts();
    var members = machine.components(), back=F.inverse(hobber.HobberFrame.workToMachine()), count=0;
    var ids=[for (r in HobberChipCover.installations()) r.id].concat([for(p in HobberChipCover.plugs())p.id]).concat(["Xbridge","XnutBlock"]);
    for (x in [20.0,45.0,110.0,115.0]) {
      state.setJoint("X",x); state.forwardKinematics();
      var shields=[for (m in members) if(ids.indexOf(m.id)>=0) cache.posed(m.component,F.compose(back,state.worldPose(m.id)))];
      for (y in [-40.0,-20.0,0.0,20.0,40.0]) for (station in 0...45) {
        var xx=180.0+station*10, ray=Part.cylinderSpan(0.1,214,282,xx,y), hit=false;
        for(p in shields) { var common=p.intersect(ray); if(common.volume()>1e-5)hit=true; common.close(); if(hit)break; }
        ray.close(); count++; if(!hit)throw 'Unshielded X channel at X$x: ($xx,$y)';
      }
      for(p in shields)p.close();
    }
    var zguards=[for(m in members)if([for(c in machine.zBellows)for(id in c.members())id].indexOf(m.id)>=0) cache.posed(m.component,F.compose(back,state.worldPose(m.id)))];
    for(y in [-155.0,-145.0,-135.0,73.0,81.0,89.0,135.0,145.0,155.0])for(k in 0...22) {
      // Z guards travel with X: translate the ray by the same X displacement.
      var ray=Part.cylinderAlong(0.1,new Vector(120+115,y,320+k*20),Vector.X(),26), hit=false;
      for(p in zguards) {var common=p.intersect(ray);if(common.volume()>1e-5)hit=true;common.close();}
      ray.close();count++;if(!hit)throw 'Unshielded Z channel at Y$y Z${320+k*20}';
    }
    for(p in zguards)p.close(); cache.close();
    return HobberChipCover.coverage().concat(['Chip protection: $count actual-solid ingress samples passed; four closed tool ports.']);
  }
}
