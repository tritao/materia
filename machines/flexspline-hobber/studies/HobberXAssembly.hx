import hobber.*;
import machinekit.hobbing.*;
import hobber.HobberSkeleton;
import hobber.HobberCarriageRib;

/** Assembly instructions are keyed to the actual derived joints, never a second bolt list.
 * Tool IDs are shared with the LM6 tooling BOM. Support seat layers are held by the same through-stack fasteners. */
typedef XAssemblyStep = {
  id:String, parts:Array<String>, tools:Array<String>, measurement:String,
  joints:Array<{member:String, face:String, method:String}>
}
class HobberXAssembly {
  public static final METHODS = ["reference edge", "indicate then match-pin", "shim or adjust", "drawing dowels", "fixture-replicated"];
  public static function steps():Array<XAssemblyStep> return [
    {id:"A1 bed", parts:["bed/casting"], tools:["precision-level"],
      measurement:"Level datum P in both directions; lock three feet without rocking.", joints:[]},
    {id:"A2 ground bars",parts:["XdriveFixedBar","XdriveFloatingBar"],tools:["dial-indicator","magnetic-base","torque-driver"],
      measurement:"Buy matched thickness from one ground parent bar; inspect pair within0.01 mm. Tighten the flush recessed bar screws before fitting supports. Their covered heads are deliberate: remove the support only for rare bar replacement/regrinding; ordinary motor service leaves the bars installed.",
      joints:[for(id in ["XdriveFixedBar","XdriveFloatingBar"]){member:id,face:"insert",method:"shim or adjust"}]},
    {id:"A2 rails", parts:["railXLeft","railXRight"], tools:["straightedge-granite","dial-indicator","magnetic-base"],
      measurement:"Reference rail against its edge; indicate second rail parallel over full travel.",
      joints:[for (id in ["railXLeft","railXRight"]) {member:id,face:"*",method:"reference edge"}]},
    {id:"A2 blocks", parts:["blockXLeft","blockXLeftFront","blockXRight","blockXRightFront"],
      tools:["block-transfer-sleeves","dial-indicator","magnetic-base"], measurement:"Slide the matched blocks from their sleeves onto clean rails; protect their wipers. Check free running before lowering the carriage.", joints:[]},
    {id:"A3 supports",parts:["Xfixed","Xfloating"],tools:["dial-indicator","magnetic-base","torque-driver"],
      measurement:"Indicate supports coaxial with the actual screw and parallel to the rails; check the nut at both travel ends, torque, then match-ream/pin the supports to their own ground bars. No alignment arbor or epoxy seat.",
      joints:[for(id in ["Xfixed","Xfloating"]){member:id,face:"*",method:"indicate then match-pin"}]},
    {id:"A4 screw and nut", parts:["Xscrew","XnutBlock","Xnut"], tools:["af4-long-key","dial-indicator","magnetic-base"],
      measurement:"Support screw during insertion; retain nut on transfer sleeve; check free rotation.",
      joints:[{member:"Xnut",face:"*",method:"indicate then match-pin"}]},
    {id:"A4 carriage", parts:["Xbridge"].concat([for (role in HobberCarriageRib.ROLES) "Xrib-"+role]),
      tools:["torque-driver","dial-indicator","magnetic-base"], measurement:"Inspect/select the ground plate or finish-grind its four block lands together to0.01 mm before assembly; no small epoxy pads. Mask those datums and blast-clean both steel rib lands. Qualify mu>=0.40 with a slip coupon; keep oil, paint and adhesive off the lands. Bolt ribs before lowering carriage. Indicate nut block to screw at BOTH travel ends, tighten, then match-drill/ream; never pre-drill its dowels from drawing coordinates.",
      joints:[for (face in ["blockL1","blockL2","blockR1","blockR2"]) {member:"Xbridge",face:face,method:"shim or adjust"}]
        .concat([{member:"Xbridge",face:"nut-block",method:"indicate then match-pin"}])
        .concat([for (role in HobberCarriageRib.ROLES) {member:"Xrib-"+role,face:"*",method:"drawing dowels"}])},
    {id:"A6 sole", parts:[for (pad in HobberSkeleton.reference().columnSolePads()) "column/"+pad.id],
      tools:["c-test-mandrel","dial-indicator","magnetic-base","square-granite"], measurement:"Indicate Z parallel to C mandrel in two planes; A6 sole matching/indication and match-pin release remains open. No pin or bond credit in slip/stiffness.",
      joints:[for (pad in HobberSkeleton.reference().columnSolePads()) {member:"Xbridge",face:"column-sole-"+pad.id,method:"shim or adjust"}]},
    {id:"A11 scale", parts:["XscaleSeat","Xscale","XscaleHead"], tools:["dial-indicator","magnetic-base","feeler-gauges","af3-low-profile-key"],
      measurement:"Adjust at head to scale supplier parallelism and 1 +/-0.3 mm gap; check full reading travel before guards. Service uses an ASSUMED AF3 key envelope: 6 mm projection plus 3.5 mm diameter bend/handle, 90 mm reach along X; a straight 20 mm driver hits the bed. Confirm the tool before release.",
      joints:[for (id in ["XscaleSeat","Xscale","XscaleHead"]) {member:id,face:"*",method:"shim or adjust"}]},
    {id:"guards and tray", parts:["XcoverFrontLeft","XcoverFrontRight","XcoverRearLeft","XcoverRearRight","XfrontCover","XrearCover","Xskirt"],
      tools:["torque-driver","af5.5-long-spanner","af2.5-key"], measurement:"Fit guard nuts on bench; hold rear nuts through the open rear channel with the long flat spanner for lid service. Fit tray last; check each blank and motor-bay lid removal path.", joints:[]}
  ];
  public static function main():Void {
    var machine=new FlexsplineHobber(); check(machine);
    var carriage=["Xbridge"].concat([for (role in HobberCarriageRib.ROLES) "Xrib-"+role]);
    Sys.println("XC measured deck+ribs mass kg: " + HobberJointLoads.massOf(machine,carriage)
      + "; sole pads kg: " + HobberJointLoads.massOf(machine,[for (pad in HobberSkeleton.reference().columnSolePads()) "column/"+pad.id]));
  }
  /** Installed scale-head screws must be reachable with guards removed,
   * including the stationary bed (which moves relative to the head). */
  static function headAccess(machine:FlexsplineHobber):Int {
    var derived=machine.derivedFasteners(); if (derived==null) throw "Missing X head joints";
    var poses=machine.solvedPoses(), members=machine.components(), cache=new machinekit.assembly.PosedParts(), count=0;
    var guards=hobber.HobberChipCover.fixedMembers().concat(["Xskirt"]);
    for (joint in derived.joints) if (joint.clampedMember=="XscaleHead") {
      var host=members.filter(p->p.id==joint.receivingMember)[0];
      var face=host.component.fasteningFaces().filter(f->f.id==joint.receivingFace)[0];
      var frame=materia.assembly.AssemblyFrames.compose(poses.get(host.id),face.frame);
      for (volume in joint.volumes) if (volume.access) for (hole in joint.hostHoles) {
        // ASSUMED low-profile AF3 key: circumscribed D3.5, 6 mm exposed
        // stub and a 90 mm handle along the scale, clear of the inner rib.
        var raw=cadkit.modeling.Part.cylinderSpan(1.75,volume.to-6,volume.to,hole.x,hole.y);
        var stub=machinekit.assembly.PosedParts.moved(raw,frame); raw.close();
        var elbow=materia.assembly.AssemblyFrames.transformPoint(frame,hole.x,hole.y,volume.to-6);
        var handle=cadkit.modeling.Part.cylinderAlong(1.75,new cadkit.modeling.Vector(elbow.x,elbow.y,elbow.z),cadkit.modeling.Vector.Y(),90);
        for (tool in [stub,handle]) {
        var box=machinekit.assembly.PosedParts.boxOf(tool);
        for (other in members) if (!guards.contains(other.id)) {
          var pose=poses.get(other.id);
          if (machinekit.assembly.PosedParts.apart(box,cache.box(other.component,pose),0)) continue;
          var part=cache.posed(other.component,pose), common=part.intersect(tool), overlap=common.volume();
          common.close(); part.close();
          if (overlap>0.001) {stub.close();handle.close();cache.close();throw 'X scale head driver meets ${other.id}: $overlap mm3';}
        }
        }
        stub.close(); handle.close(); count++;
      }
    }
    cache.close(); if (count!=2) throw "Expected two X scale head driver corridors";
    Sys.println('X scale head: $count low-profile AF3 key corridors clear with guards removed, including bed; ASSUMED D3.5 / 6 mm stub / 90 mm reach.');
    return count;
  }
  /** Ordinary nuts remain reachable from the open channel end: no invisible
   * captive nut or unmodelled retention hardware is credited. */
  static function bayNutAccess(machine:FlexsplineHobber):Int {
    var derived=machine.derivedFasteners(); if (derived==null) throw "Missing bay lid joints";
    var poses=machine.solvedPoses(), members=machine.components(), cache=new machinekit.assembly.PosedParts(), count=0;
    var back=materia.assembly.AssemblyFrames.inverse(hobber.HobberFrame.workToMachine());
    for (joint in derived.joints) if (joint.clampedMember=="XrearCover") {
      var host=members.filter(p->p.id==joint.receivingMember)[0];
      var face=host.component.fasteningFaces().filter(f->f.id==joint.receivingFace)[0];
      var frame=materia.assembly.AssemblyFrames.compose(poses.get(host.id),face.frame);
      for (volume in joint.volumes) if (volume.name=="nut") for (targetIndex=>hole in joint.hostHoles) {
        var centre=materia.assembly.AssemblyFrames.transformPoint(frame,hole.x,hole.y,(volume.from+volume.to)/2);
        var point=materia.assembly.AssemblyFrames.transformPoint(back,centre.x,centre.y,centre.z);
        var x0=point.x-5, x1=hobber.HobberChipCover.REAR_END+5;
        // ASSUMED AF5.5 offset spanner: D10 jaw, 5-wide handle offset 7, 2 thick, >=140 reach.
        var outward=point.y<0?-1:1, handleY=point.y+outward*7;
        function envelope(x0:Float,x1:Float,y0:Float,y1:Float):cadkit.modeling.Part {
          var raw=cadkit.modeling.Part.box(x1-x0,y1-y0,2);
          var out=machinekit.assembly.PosedParts.moved(raw,materia.assembly.AssemblyFrames.compose(
            hobber.HobberFrame.workToMachine(),materia.assembly.AssemblyFrames.translation((x0+x1)/2,(y0+y1)/2,point.z-1)));
          raw.close();return out;
        }
        var tools=[envelope(x0,point.x+5,point.y-5,point.y+5),
          envelope(point.x+5,point.x+15,Math.min(point.y,handleY)-2.5,Math.max(point.y,handleY)+2.5),
          envelope(point.x+10,x1,handleY-2.5,handleY+2.5)];
        for (tool in tools) {
        var box=machinekit.assembly.PosedParts.boxOf(tool);
        for (other in members) {
          var pose=poses.get(other.id);
          if (machinekit.assembly.PosedParts.apart(box,cache.box(other.component,pose),0)) continue;
          var part=cache.posed(other.component,pose), common=part.intersect(tool), overlap=common.volume(); common.close();part.close();
          if (overlap>0.001) {for(t in tools)t.close();cache.close();throw 'Bay lid spanner meets ${other.id}: $overlap mm3';}
        }
        // Other hardware also occupies the corridor, including the second nut
        // in this very joint. Only the engaged target nut/bolt is exempt.
        for (other in derived.joints) {
          var receiving=members.filter(p->p.id==other.receivingMember)[0];
          var otherFace=receiving.component.fasteningFaces().filter(f->f.id==other.receivingFace)[0];
          var otherFrame=materia.assembly.AssemblyFrames.compose(poses.get(receiving.id),otherFace.frame);
          for (physical in other.volumes) if (["head","head washer","nut","nut washer"].contains(physical.name))
            for (index=>at in other.hostHoles) {
              if(other.id==joint.id && index==targetIndex)continue;
              var raw=cadkit.modeling.Part.cylinderSpan(physical.diameter/2,physical.from,physical.to,at.x,at.y);
              var hardware=machinekit.assembly.PosedParts.moved(raw,otherFrame);raw.close();
              if(!machinekit.assembly.PosedParts.apart(box,machinekit.assembly.PosedParts.boxOf(hardware),0)) {
                var common=tool.intersect(hardware), overlap=common.volume();common.close();
                if(overlap>0.001){hardware.close();for(t in tools)t.close();cache.close();throw 'Bay spanner meets hardware ${other.id}: $overlap mm3';}
              }
              hardware.close();
            }
        }
        }
        for(tool in tools)tool.close();count++;
      }
    }
    cache.close();if(count!=4) throw "Expected four bay nut spanner corridors";
    Sys.println('Motor bay: $count rear-entry AF5.5 spanner corridors clear; ASSUMED D10 jaw / 5 x 2 mm handle / 7 mm offset / >=140 mm reach.');
    return count;
  }
  /** Long AF2.5 leg through the actual roof ports with only their plugs off.
   * Front/rear tool access uses the same opposite-end X parking as the manual. */
  static function guardFootAccess(machine:FlexsplineHobber):Int {
    var derived=machine.derivedFasteners();if(derived==null)throw "Missing guard joints";
    var members=machine.components(),cache=new machinekit.assembly.PosedParts(),count=0;
    var plugs=[for(p in hobber.HobberChipCover.plugs())p.id];
    var travel=machine.travel.filter(t->t.id=="X")[0];
    for(front in [true,false]) {
      var state=new cadkit.modeling.AssemblyState(machine.definition());
      state.setJoint("X",front?travel.upper:travel.lower);state.setJoint("Z",hobber.HobberSkeleton.reference().zTravelMax);state.forwardKinematics();
      var poses=machine.solvedPoses(state);
      for(joint in derived.joints)if(joint.clampedFace=="pad"&&StringTools.startsWith(joint.clampedMember,front?"XcoverFront":"XcoverRear")) {
        var host=members.filter(m->m.id==joint.receivingMember)[0],hp=poses.get(host.id);if(hp==null)throw "Missing guard receiver pose";
        var face=host.component.fasteningFaces().filter(f->f.id==joint.receivingFace)[0];
        var frame=materia.assembly.AssemblyFrames.compose(hp,face.frame);
        for(index=>point in joint.hostHoles)for(head in joint.volumes)if(head.name=="head") {
          var raw=cadkit.modeling.Part.cylinderSpan(1.75,head.from-100,head.from,point.x,point.y);
          var tool=machinekit.assembly.PosedParts.moved(raw,frame);raw.close();
          for(other in members)if(other.component.hasGeometry()&&!plugs.contains(other.id)) {
            var pose=poses.get(other.id);if(pose==null)throw "Missing guard tool obstacle";
            if(machinekit.assembly.PosedParts.apart(machinekit.assembly.PosedParts.boxOf(tool),cache.box(other.component,pose)))continue;
            var body=cache.posed(other.component,pose),common=tool.intersect(body),volume=common.volume();body.close();common.close();
            if(volume>0.001)throw 'Guard AF2.5 corridor ${joint.id} / ${other.id}: $volume mm3';
          }
          // The engaged target head is the sole hardware exemption.
          for(other in derived.joints) {
            var receiver=members.filter(m->m.id==other.receivingMember)[0],rp=poses.get(receiver.id);if(rp==null)throw "Missing hardware pose";
            var rf=receiver.component.fasteningFaces().filter(f->f.id==other.receivingFace)[0];
            var world=materia.assembly.AssemblyFrames.compose(rp,rf.frame);
            for(k=>p in other.hostHoles)if(other.id!=joint.id||k!=index)for(v in other.volumes)
              if(["head","head washer","nut","nut washer"].contains(v.name)) {
                var shape=cadkit.modeling.Part.cylinderSpan(v.diameter/2,v.from,v.to,p.x,p.y),body=machinekit.assembly.PosedParts.moved(shape,world);shape.close();
                if(!machinekit.assembly.PosedParts.apart(machinekit.assembly.PosedParts.boxOf(tool),machinekit.assembly.PosedParts.boxOf(body))) {
                  var common=tool.intersect(body),volume=common.volume();common.close();if(volume>0.001)throw "Guard key meets hardware: "+other.id;
                }
                body.close();
              }
          }
          tool.close();count++;
        }
      }
    }
    cache.close();if(count!=4)throw "Expected four guard-foot tool corridors";
    Sys.println('X guard feet: $count AF2.5 D3.5 x100 mm corridors clear, plugs removed, front at X retract/rear at X minimum; tool envelope ASSUMED.');
    return count;
  }

  /** Friction-only VQ/I screen. Charge the full X mass to the beam shear,
   * with 2x uneven sharing ASSUMED; no bond or assembly-dowel capacity. */
  static function ribSlip(machine:FlexsplineHobber):Int {
    var bottom=HobberXCarriage.xCarriageBottom(), thickness=HobberXCarriage.X_CARRIAGE_THICKNESS;
    var sections=[{width:2*HobberXCarriage.xCarriageHalfWidth(),height:thickness,z:bottom+thickness/2}];
    var ribs=[for (p in machine.components()) if (StringTools.startsWith(p.id,"Xrib-")) p];
    for (entry in ribs) {
      var rib:HobberCarriageRib=cast entry.component, band=rib.band();
      sections.push({width:band.high-band.low,height:bottom-band.bottom,z:(bottom+band.bottom)/2});
    }
    var area=0.0, first=0.0;
    for (section in sections) {area+=section.width*section.height;first+=section.width*section.height*section.z;}
    var neutral=first/area, inertia=0.0;
    for (section in sections) inertia+=section.width*Math.pow(section.height,3)/12
      +section.width*section.height*Math.pow(section.z-neutral,2);
    var crash=HobberCrashProtection.axis(machine,"X"), shear=crash.movingKg*(9.80665+0.5)+HobberJointLoads.CUTTING_FORCE;
    var derived=machine.derivedFasteners();if(derived==null)throw "Rib slip needs derived screws";
    var cache=new machinekit.assembly.PosedParts(), count=0;
    for (i=>entry in ribs) {
      var section=sections[i+1], box=cache.box(entry.component,materia.assembly.AssemblyFrames.identity());
      var force=2*shear*section.width*section.height*Math.abs(section.z-neutral)/inertia*(box.maxX-box.minX);
      var screws=[for (joint in derived.joints) if(joint.clampedMember==entry.id && joint.receivingMember=="Xbridge")
        {id:joint.id,size:joint.size,propertyClass:joint.propertyClass,points:joint.worldHoles}];
      var checks=machinekit.assembly.JointSlip.check({label:"Rib friction-only "+entry.id,between:[entry.id,"Xbridge"],
        shear:{peak:force,momentary:force+crash.limit.thrust+HobberJointLoads.CUTTING_FORCE},friction:HobberCarriageRib.ASSUMED_FAYING_FRICTION},screws,[]);
      if(checks.length!=1)throw "Missing rib friction check: "+entry.id;
      for (result in checks) {
        Sys.println(machinekit.assembly.JointSlip.describe(result));
        var momentary=result.momentaryFactor;
        if(!result.passes || result.peakFactor<3 || momentary==null || momentary<3) {cache.close();throw "Rib lacks factor 3: "+entry.id;}
        count++;
      }
    }
    cache.close();if(count!=4)throw "Expected four rib slip checks";return count;
  }
  public static function check(machine:FlexsplineHobber):Int {
    var parts = [for (p in machine.components()) p.id], declarations = steps(), count=0;
    for (step in declarations) {
      if (step.tools.length==0 || StringTools.trim(step.measurement)=="") throw "Assembly step needs tools and measurement: "+step.id;
      for (part in step.parts) if (!parts.contains(part)) throw 'Assembly step ${step.id} names missing $part';
      for (rule in step.joints) if (!METHODS.contains(rule.method)) throw 'Unknown alignment method ${rule.method}';
    }
    var derived=machine.derivedFasteners(); if (derived==null) throw "X sequence needs derived joints";
    var precision=["railXLeft","railXRight","XdriveFixedBar","XdriveFloatingBar","Xfixed","Xfloating","Xnut","Xbridge","XscaleSeat","Xscale","XscaleHead"]
      .concat([for (role in HobberCarriageRib.ROLES) "Xrib-"+role]);
    for (joint in derived.joints) if (precision.contains(joint.clampedMember)) {
      var matches=0;
      for (step in declarations) for (rule in step.joints)
        if (rule.member==joint.clampedMember && (rule.face=="*" || rule.face==joint.clampedFace)) matches++;
      if (matches!=1) throw 'Precision joint ${joint.id} needs exactly one alignment declaration, got $matches';
      count++;
    }
    Sys.println('X assembly: $count precision joints have alignment methods; ${declarations.length} steps name tools and measurement. Short matched ground bars; A6 release remains open.');
    return count + headAccess(machine) + bayNutAccess(machine) + guardFootAccess(machine) + ribSlip(machine);
  }
}
