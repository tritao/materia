package machinekit.motion;

import machinekit.assembly.*;
import machinekit.component.*;
import machinekit.cover.*;
import machinekit.cover.WayCoverMount.MountStock;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Uses the existing way-cover library. Its envelope includes the scale cable's
 * complete R75 bend and tail; the generic row and its mounting remain assumed. */
class ScrewLinearModuleBellows {
  public static function plan(s:ScrewLinearModuleSpec):WayCoverPlan {
    var row=WayCoverCatalog.get("BELLOWS-LAMELLA-8x8"),stroke=s.stroke+2*s.overtravel;
    var space=Math.ceil((stroke/(row.ratio-1)+2*row.flangeThickness+row.foldCompressed)/10)*10;
    var stub=s.scaleHead.cableStubEnd();
    var top=s.scaleHeadTop()-stub[1]+SealedScaleData.CABLE_BEND_RADIUS+s.scaleHead.series.cableStraight+s.scaleHead.series.cableDiameter/2+ScrewLinearModuleSpec.GAP;
    return new WayCoverPlan(row,s.baseWidth+4,top-s.baseThickness,stroke,space,WayCoverPlan.boxCover(0,100000,100000,true));
  }
  public static function pose(s:ScrewLinearModuleSpec,role:String):AssemblyFrame {
    var front=StringTools.endsWith(role,"Front"),fixed=StringTools.startsWith(role,"mountFixed"),sign=front?1.0:-1.0;
    var moving=s.centre-sign*(s.carriageLength/2+8),at=fixed?moving-sign*(plan(s).space+s.stroke/2+s.overtravel):moving;
    return AxisBuilder.orient(at,0,s.baseThickness,[0.0,0,1],[fixed?sign:-sign,0.0,0]);
  }
  public static function clamps(s:ScrewLinearModuleSpec,role:String):Array<BoltedFace> {
    if(StringTools.startsWith(role,"mountMoving"))return [BoltedFace.clamped("carriage",BoltedFace.frameOf({x:0.0,y:s.carriageBottom()-s.baseThickness+s.carriageThickness/2,z:-8.0},{x:0.0,y:0,z:1},{x:1.0,y:0,z:0}),
      [{x:-25.0,y:0.0},{x:25.0,y:0.0}],4.5,8,"M4",null,false,true)];
    var half=plan(s).plateHalf(),out:Array<BoltedFace>=[];
    for(side in [-1,1])out.push(BoltedFace.clamped("edge"+side,BoltedFace.frameOf({x:side*s.baseWidth/2,y:-s.baseThickness/2,z:4.0},{x:side*1.0,y:0,z:0},{x:0.0,y:0,z:1}),
      [{x:0.0,y:0.0}],4.5,half-s.baseWidth/2,"M4",null,false,true));
    return out;
  }
  public static function mount(s:ScrewLinearModuleSpec,role:String):WayCoverMount {
    var p=plan(s),fixed=StringTools.startsWith(role,"mountFixed"),half=p.plateHalf();
    var cuts:Array<WayCoverMount.MountCut>=[];
    if(fixed) {
      cuts.push({x0:-s.baseWidth/2,x1:s.baseWidth/2,y0:-s.baseThickness-1,y1:0});
      cuts.push({x0:-p.width/2,x1:p.width/2,y0:0.0,y1:p.height});
    } else {
      cuts.push({x0:-half-1,x1:half+1,y0:-1.0,y1:2.2});
      cuts.push({x0:-p.width/2,x1:p.width/2,y0:0.0,y1:s.carriageBottom()-s.baseThickness});
      // Two narrow struts retain the carriage bolts; clear the stops and blocks
      // instead of carrying a solid wall up to the bellows roof.
      for(band in [{lo:-p.width/2,hi:-32.0},{lo:32.0,hi:p.width/2}])
        cuts.push({x0:band.lo,x1:band.hi,y0:s.carriageBottom()-s.baseThickness,y1:p.height});
      cuts.push({x0:-19.0,x1:19.0,y0:s.carriageBottom()+s.carriageThickness-s.baseThickness,y1:p.height});
    }
    var attachment:Null<WayCoverMount.MountAttachment>=null;
    if(fixed) {
      var stock:Array<MountStock>=[];
      for(side in [-1,1])stock.push({x0:side<0?-half:s.baseWidth/2,x1:side<0?-s.baseWidth/2:half,y0:-14.0,y1:-2.0,z0:-8.0,z1:20.0});
      attachment={stock:stock,bolted:clamps(s,role),pinned:[]};
    }
    return new WayCoverMount("SCREW-MODULE-"+role,half,p.plateHigh(),8,[],cuts,fixed?14:0,attachment);
  }
  public static function faces(s:ScrewLinearModuleSpec,role:String):Array<BoltedFace> {
    var delegate=mount(s,role),out:Array<BoltedFace>=[];
    for(face in delegate.fasteningFaces())if(face.id=="cover")
      out.push(BoltedFace.drilledToSuit(face.id,face.frame,face.width,face.height,8,8,BoltedFace.ThreadMaterial.Steel,"M5",true));
    else out.push(face);
    if(StringTools.startsWith(role,"mountMoving"))out=out.concat(clamps(s,role));
    return out;
  }
  public static function baseReceivers(s:ScrewLinearModuleSpec):Array<BoltedFace> {
    var out:Array<BoltedFace>=[];
    if(s.stroke>20)for(front in [true,false]) {
      var role="mountFixed"+(front?"Front":"Rear"),world=pose(s,role);
      for(face in clamps(s,role)) {
        var frame=F.compose(F.compose(world,face.frame),F.fromRotationMatrix(0,0,0,[1.0,0,0,0,-1,0,0,0,-1]));
        out.push(BoltedFace.drilledToSuit(role+face.id,frame,24,16,10,16,BoltedFace.ThreadMaterial.Steel,"M4"));
      }
    }
    return out;
  }
  public static function members(s:ScrewLinearModuleSpec):Array<String> {
    var out:Array<String>=[],p=plan(s);
    for(suffix in ["Front","Rear"]) {
      out=out.concat(["mountFixed"+suffix,"mountMoving"+suffix,"bellows"+suffix+"-fixed","bellows"+suffix+"-moving"]);
      for(k in 1...p.levels)out.push("bellows"+suffix+"-s"+k);
    }
    return out;
  }
  public static function install(axis:ScrewLinearModule):Void {
    var s=axis.spec,p=plan(s),half=s.stroke/2+s.overtravel;
    axis.group("Chip bellows");
    for(front in [true,false]) {
      var suffix=front?"Front":"Rear",fixed="mountFixed"+suffix,moving="mountMoving"+suffix;
      for(id in [fixed,moving])axis.attach(id,new ScrewLinearModuleCover(s.values().setToken("role",id)),pose(s,id),id==fixed?"base":"carriage");
      var origin=pose(s,fixed);
      var cover=WayCover.install(axis,{id:"bellows"+suffix,row:p.row.id,fixedParent:fixed,movingParent:moving,
        leader:{id:"travel",lower:-half,upper:half,initial:0.0},origin:[origin.x,origin.y,origin.z],along:[front?1.0:-1.0,0,0],up:[0.0,0,1],
        width:p.width,height:p.height,space:p.space,shape:p.shape});
      // The last sleeve also guides the moving flange. This closure declares
      // their physical sliding pair without granting any overlap allowance.
      var last=cover.skinMembers.length==0?cover.fixedMember:cover.skinMembers[cover.skinMembers.length-1];
      var poses=axis.solvedPoses(),common=pose(s,moving),a=poses.get(last),b=poses.get(cover.movingMember);
      if(a==null||b==null)throw "Missing bellows guide poses";
      axis.addMemberConnector(last,"end-guide",F.compose(F.inverse(a),common));
      axis.addMemberConnector(cover.movingMember,"end-guide",F.compose(F.inverse(b),common));
      axis.addConstraintOnAxis("bellows"+suffix+"-end-guide","prismatic",last,"end-guide",cover.movingMember,"end-guide",{x:0.0,y:0,z:1});
      axis.addDesignRequirement(machinekit.design.DesignRequirement.ServiceRemoval("bellows"+suffix,[fixed,moving].concat(cover.members()),[{direction:[0.0,0,1],distance:p.outerHeight()+50}],[],[]));
    }
  }
}
