import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.assembly.PosedParts;
import machinekit.component.MachineComponent;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import gearkit.GearSpec;

/** Geometry-derived, conservative loading corridors. mm, work radial X towards column.
    The robot is on -X. Y parks at zero; B stopped. A remains anywhere +/-15 deg.
    Angular sampling is enclosed by a displacement bound, not assumed continuous proof. */
class HobberLoadingSizing {
  public static inline final APPROACH=250.0;
  public static inline final FRONT_LIFT=35.0;
  public static inline final CLEARANCE=10.0;
  static function box(x0:Float,x1:Float,y0:Float,y1:Float,z0:Float,z1:Float):Part {
    var raw=Part.box(x1-x0,y1-y0,z1-z0),p=raw.translated(new Vector((x0+x1)/2,(y0+y1)/2,z0));raw.close();return p;
  }
  /** Head-clearance volumes include 10 mm clearance plus the all-angle enclosure.
      The filled cup locating volume is checked only against the X group here;
      actual hollow cup and jaw/fixture checks are separate. */
  static function corridor(radius:Float,top:Bool,pad:Float):Array<Part> {
    var z=363.0,r=radius+pad;
    if(top)return [Part.cylinderSpan(r,z-pad,z+120+40+pad),Part.cylinderSpan(radius+25+pad,z+30-pad,1400)];
    var parts=[Part.cylinderSpan(r,z-pad,z+FRONT_LIFT+40+pad),box(-600,0,-r,r,z+FRONT_LIFT-pad,z+FRONT_LIFT+40+pad)];
    // Two side fingers: rear extent +15 mm, 25 mm outside the cup,
    // 3 mm nominal contact engagement, gripping its upper 10 mm.
    for(side in [-1,1])parts.push(box(-600,15+pad,side<0?-radius-25-pad:radius-3-pad,side<0?-radius+3+pad:radius+25+pad,z+30-pad,z+FRONT_LIFT+55+pad));
    // Wrist/palm is entirely in front, away from the column.
    parts.push(box(-600,-radius-25+pad,-radius-25-pad,radius+25+pad,z+40-pad,z+FRONT_LIFT+80+pad));return parts;
  }
  static function group(machine:FlexsplineHobber,axis:String):Map<String,Bool> {
    var def=machine.definition(),ids:Map<String,Bool> = [];ids.set(def.joints.filter(j->j.id==axis)[0].child,true);
    var changed=true;while(changed){changed=false;for(j in def.joints)if(ids.exists(j.parent)&&!ids.exists(j.child)){ids.set(j.child,true);changed=true;}}return ids;
  }
  public static function measure(machine:FlexsplineHobber):Dynamic {
    var model=new cadkit.modeling.AssemblyModel("mm");machine.addTo(model,"");var state=model.initialState(),cache=new PosedParts(),canonical=F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    state.setJoint("X",machine.travel[0].upper);state.setJoint("Y",0);state.setJoint("Z",0);state.setJoint(FlexsplineHobber.A,0);state.forwardKinematics();
    var moving=group(machine,"X"),rotating=group(machine,FlexsplineHobber.A),members=machine.components().filter(m->moving.exists(m.id)),maxRadius=0.0;
    for(m in members)if(rotating.exists(m.id)){
      var b=cache.box(m.component,F.compose(canonical,state.worldPose(m.id)));
      for(y in [b.minY,b.maxY])for(z in [b.minZ,b.maxZ])maxRadius=Math.max(maxRadius,Math.sqrt(y*y+(z-363)*(z-363)));
    }
    var angularPad=maxRadius*Math.PI/360,poses:Array<{part:MachineComponent,id:String,pose:AssemblyFrame}> = [];
    for(deg in -15...16){state.setJoint(FlexsplineHobber.A,deg*Math.PI/180);state.forwardKinematics();
      for(m in members){var p=F.compose(canonical,state.worldPose(m.id));p.x-=machine.travel[0].upper;poses.push({part:m.component,id:m.id,pose:p});}}
    var radius=machine.spec.tipRadius(),answers:Array<Dynamic> = [];
    try {
      for(top in [false,true]){
        var volumes=corridor(radius,top,CLEARANCE+angularPad),last="",calls=0;
        function clear(x:Float):Bool {
          for(row in poses){var p=F.compose(F.translation(x,0,0),row.pose),b=cache.box(row.part,p);
            for(volume in volumes){var bb=volume.shape.bounds(),lo=bb.get_min(),hi=bb.get_max();
              if(b.minX>=hi.get_x()||b.maxX<=lo.get_x()||b.minY>=hi.get_y()||b.maxY<=lo.get_y()||b.minZ>=hi.get_z()||b.maxZ<=lo.get_z())continue;
              var solid=cache.posed(row.part,p),overlap=PosedParts.sharedVolume(solid,volume);solid.close();calls++;
              if(overlap>0.001){last=row.id;return false;}
            }
          }return true;
        }
        var low=machine.travel[0].lower,high=250.0;
        if(!clear(high))throw "loading corridor does not clear analysis upper bound";
        while(high-low>0.25){var middle=(low+high)/2;if(clear(middle))high=middle;else low=middle;}
        var retract=Math.ceil(high/5)*5;
        if(!clear(retract))throw "rounded loading retract is obstructed";
        answers.push({direction:top?"top":"front",minimumClearRetractMm:high,roundedRetractMm:retract,limitingPart:last,exactCommonChecks:calls});
        for(v in volumes)v.close();
      }
    }catch(e:Dynamic){cache.close();throw e;}cache.close();
    return {outsideMm:2*radius,faceMm:machine.spec.faceWidth,headZMm:0,hobShiftMm:0,aDegrees:[-15,15],aSampleStepDegrees:1,angularRadiusBoundMm:maxRadius,angularPaddingMm:angularPad,headClearanceMm:CLEARANCE,frontLiftMm:FRONT_LIFT,frontApproachMm:APPROACH,results:answers};
  }
  public static function run():Void {
    var rows:Array<Dynamic> = [];
    for(spec in [new GearSpec(0.5,48,20,20*Math.PI/180),new GearSpec(0.5,278,20,20*Math.PI/180)])rows.push(measure(new FlexsplineHobber(spec)));
    var frontMax=0.0,topMax=0.0;
    for(row in rows)for(result in (cast row.results:Array<Dynamic>)){var limit=result.direction=="front"?machinekit.hobbing.HobberXLayout.RETRACT:machinekit.hobbing.HobberXLayout.TOP_RETRACT;
      if(result.roundedRetractMm>limit)throw "loading retract obstructs a supported size";
      if(result.direction=="front")frontMax=Math.max(frontMax,result.roundedRetractMm);else topMax=Math.max(topMax,result.roundedRetractMm);}
    if(frontMax!=machinekit.hobbing.HobberXLayout.RETRACT||topMax!=machinekit.hobbing.HobberXLayout.TOP_RETRACT)throw "loading retract must be the geometry-derived 5 mm round-up for the largest required corridor";
    Sys.println(haxe.Json.stringify({mechanicalFront:machinekit.hobbing.HobberXLayout.front(),mechanicalTopAlternative:machinekit.hobbing.HobberXLayout.dimensions(machinekit.hobbing.HobberXLayout.TOP_RETRACT),basis:"Assumed side-jaw gripper and front wrist; actual head solids; all A enclosed by 1-degree samples plus R*half-angle displacement padding; B stopped, Y=Z=0; exact hollow-cup and unpadded jaw/fixture paths checked separately",configurations:rows}));
  }
}
