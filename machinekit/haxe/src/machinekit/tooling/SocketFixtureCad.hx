package machinekit.tooling;

import materia.tooling.casting.SocketFixturePlanner;
import materia.tooling.casting.SocketFixturePlanner.SocketFixturePlan;
import materia.tooling.casting.SocketFixturePlanner.BlindSocketSpec;
import materia.tooling.panel.PanelAssemblyPlan;
import materia.tooling.panel.PanelJoints;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.structural.CastInFixingSocket;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** CAD adapter for explicit blind-socket fixtures, reusing neutral panel plans and standard hardware. */
class SocketFixtureCad {
  public static function spec(text:String):BlindSocketSpec {
    var d:Dynamic=haxe.Json.parse(text);
    return {length:d.length,outsideDiameter:d.outsideDiameter,boreDepth:d.boreDepth,threadLength:d.threadLength,
      tailWidth:d.tailWidth,tailThickness:d.tailThickness,tailWave:d.tailWave,thread:d.thread,source:d.source,exteriorVerified:d.exteriorVerified};
  }
  public static function socket(s:BlindSocketSpec):CastInFixingSocket {
    if(s.thread!="M6")throw "Only explicit M6 socket supported";
    return new CastInFixingSocket(new ComponentValues().setNumber("length",s.length).setNumber("outsideDiameter",s.outsideDiameter)
      .setNumber("boreDepth",s.boreDepth).setNumber("threadLength",s.threadLength).setNumber("tailWidth",s.tailWidth)
      .setNumber("tailThickness",s.tailThickness).setNumber("tailWave",s.tailWave));
  }
  public static function panels(p:SocketFixturePlan):materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord {
    var plan=PanelAssemblyPlan.pair(p.plates);
    plan.members[0].withdrawal={x:0,y:0,z:-1};plan.members[1].withdrawal={x:0,y:0,z:-1};
    return plan;
  }
  static function add(a:MachineAssembly,root:String,id:String,part:MachineComponent,pose:AssemblyFrame):Void {
    a.addComponent(id,part,pose);a.addMemberConnector(root,id+"-seat",pose);a.addMemberConnector(id,"fixture-seat",F.identity());
    a.addMate(id+"-position","fixed",root,id+"-seat",id,"fixture-seat");
  }
  public static function assembly(p:SocketFixturePlan,cured:Bool=false,exploded:Bool=false):MachineAssembly {
    if(Math.abs(machinekit.standard.FlatWasher.metric("M6").thickness-p.washerThickness)>1e-7)throw "Fixture washer differs from the selected catalogue hardware";
    var plan=panels(p),a=PanelKit.assembly(plan),root=p.plates.fixed.id,t=p.plateThickness;
    var pin=0;
    for(at in [p.primary,p.secondary])add(a,root,"locator-"+pin++,new ToolingPin(new ComponentValues().setNumber("diameter",p.pinDiameter).setNumber("length",2*t+2)),F.translation(at.x,at.y,0));
    for(i in 0...p.clampPoints.length){var at=p.clampPoints[i];ToolingHardwareCad.throughBolt(a,"plate-clamp-"+i,6,2*t,F.translation(at.x,at.y,2*t));}
    for(s in p.sites) {
      add(a,root,s.id,socket(p.socket),s.mouth);
      // Explicit standard length; helper's length excludes its washer thickness.
      ToolingHardwareCad.fixtureScrew(a,s.id+"-positioning",6,s.screwLength-p.washerThickness,
        F.fromRotationMatrix(s.x,s.y,t,[1,0,0,0,-1,0,0,0,-1]));
    }
    // Separate review-only walls. External clamps/seal system are intentionally pending, not invented.
    var c=p.cavity,stock=PanelJoints.stock(t);
    var wallRows=[
      {id:"south-wall",w:c.width+2*t,h:c.height,pose:F.fromRotationMatrix(c.x-t,c.y,c.z,[1,0,0,0,0,-1,0,1,0])},
      {id:"north-wall",w:c.width+2*t,h:c.height,pose:F.fromRotationMatrix(c.x-t,c.y+c.depth+t,c.z,[1,0,0,0,0,-1,0,1,0])},
      {id:"west-wall",w:c.depth,h:c.height,pose:F.fromRotationMatrix(c.x-t,c.y,c.z,[0,0,1,1,0,0,0,1,0])},
      {id:"east-wall",w:c.depth,h:c.height,pose:F.fromRotationMatrix(c.x+c.width,c.y,c.z,[0,0,1,1,0,0,0,1,0])}
    ];
    for(w in wallRows)add(a,root,w.id,ToolingPanel.fromPlan(PanelJoints.blank(w.id,w.w,w.h,stock)),w.pose);
    if(cured)add(a,root,"cured-coupon",SocketCoupon.fromPlan(p),F.translation(c.x+c.width/2,c.y+c.depth/2,c.z));
    var posesBefore=a.solvedPoses();
    for(row in a.components())if(StringTools.startsWith(row.id,"plate-clamp-")||row.id.indexOf("positioning")>=0) {
      a.addMemberConnector(root,row.id+"-seat",posesBefore.get(row.id));a.addMemberConnector(row.id,"fixture-seat",F.identity());
      a.addMate(row.id+"-position","fixed",root,row.id+"-seat",row.id,"fixture-seat");
    }
    if(!exploded)return a;
    var out=new MachineAssembly(),poses=a.solvedPoses();
    for(row in a.components()) {
      var pose=F.compose(F.identity(),poses.get(row.id));
      if(row.id==p.plates.fixed.id)pose.z-=75;
      else if(row.id==p.plates.moving.id)pose.z-=35;
      else if(row.id=="cured-coupon"){pose.x+=550;pose.z+=80;}
      else if(row.id=="south-wall")pose.y-=100;
      else if(row.id=="north-wall")pose.y+=100;
      else if(row.id=="west-wall")pose.x-=100;
      else if(row.id=="east-wall")pose.x+=100;
      else if(StringTools.startsWith(row.id,"socket")&&row.id.indexOf("positioning")<0)pose.z+=60;
      else if(row.id.indexOf("positioning")>=0)pose.z-=70;
      out.addComponent(row.id,row.component,pose);
    }
    return out;
  }
}
