import machinekit.hobbing.FoundationLayout;
import materia.assembly.AssemblyFrames as F;
import machinekit.structural.CastingCells;
import machinekit.structural.CastingFeatures;
import machinekit.hobbing.HobberXLayout;

/** Preliminary assumed-stiffness evidence, not FEA or a structural certification. */
class FlexsplineHobberBaseReport {
  static function beam(x:Float,a:Float,force:Float,ei:Float,l:Float=860):Array<Float> {
    var b=l-a;
    return x<=a?[force*b*x*(l*l-b*b-x*x)/(6*l*ei),force*b*(l*l-b*b-3*x*x)/(6*l*ei)]:
      [force*a*(l-x)*(l*l-a*a-(l-x)*(l-x))/(6*l*ei),-force*a*(l*l-a*a-3*(l-x)*(l-x))/(6*l*ei)];
  }
  public static function measure(machine:FlexsplineHobber):Dynamic {
    var raw:Dynamic=haxe.Json.parse(sys.io.File.getContent("machinekit/examples/flexspline-hobber/BASELINE_FOUNDATIONS.json"));
    function number(row:Dynamic,key:String):Float {var value:Dynamic=Reflect.field(row,key);return cast value;}
    var oldRaw:Dynamic=Reflect.field(raw,"base"),old={massKg:number(oldRaw,"massKg"),width:number(oldRaw,"width"),depth:number(oldRaw,"depth"),height:number(oldRaw,"height"),voids:Std.string(Reflect.field(oldRaw,"voids"))};
    var oldMoving=number(raw,"movingXKg"),oldColumn=number(Reflect.field(raw,"column"),"massKg"),oldInserts=number(raw,"columnInsertMassKg");
    var body=machine.bed.body;
    var cells=CastingCells.material(body.parameterNumber("width"),body.parameterNumber("depth"),body.parameterNumber("height"),CastingFeatures.decode(body.values().token("voids")));
    var oldCells=CastingCells.material(old.width,old.depth,old.height,CastingFeatures.decode(old.voids));
    var nativeBody=body.geometry();var nativeVolume=nativeBody.volume();nativeBody.close();
    if(Math.abs(CastingCells.volume(cells)-nativeVolume)>nativeVolume*1e-9)throw "Casting collision/beam cells disagree with exact CAD volume";
    var span=HobberXLayout.front().footSpan,half=span/2;
    var oldI=CastingCells.minimumIy(oldCells,-430,430),newI=CastingCells.minimumIy(cells,-half,half),e=35000.0;
    var baseKg=0.0,columnKg=0.0,total=0.0;for(m in machine.components()){var mass=m.component.massProperties().mass;total+=mass;
      if(StringTools.startsWith(m.id,"bed/"))baseKg+=mass;if(StringTools.startsWith(m.id,"column/"))columnKg+=mass;}
    var def=machine.definition();
    function group(axis:String):Map<String,Bool> {
      var result:Map<String,Bool> = [];var key=axis;result.set(def.joints.filter(j->j.id==key)[0].child,true);
      var changed=true;while(changed){changed=false;for(j in def.joints)if(result.exists(j.parent)&&!result.exists(j.child)){result.set(j.child,true);changed=true;}}return result;
    }
    var ids=group("X"),ys=group("Y"),zs=group("Z"),movingY=0.0,movingZ=0.0;
    for(m in machine.components()){var mass=m.component.massProperties().mass;if(ys.exists(m.id))movingY+=mass;if(zs.exists(m.id))movingZ+=mass;}
    var moving=0.0;for(m in machine.components())if(ids.exists(m.id))moving+=m.component.massProperties().mass;
    var model=new cadkit.modeling.AssemblyModel("mm");machine.addTo(model,"");var state=model.initialState(),canonical=F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    var samples:Array<Dynamic> = [],oldLo=[1e30,1e30],oldHi=[-1e30,-1e30],newLo=[1e30,1e30],newHi=[-1e30,-1e30],worstFoot=0.0;
    for(i in 0...21){
      var x=HobberXLayout.MIN+(HobberXLayout.RETRACT-HobberXLayout.MIN)*i/20,legalX=Math.max(machine.travel[0].lower,x);
      state.setJoint("X",legalX);state.forwardKinematics();var loadX=0.0,allX=0.0,allY=0.0;
      for(m in machine.components()){var mass=m.component.massProperties(),p=F.compose(canonical,state.worldPose(m.id));
        var c=F.transformPoint(p,mass.centreOfMass.x,mass.centreOfMass.y,mass.centreOfMass.z);
        if(ids.exists(m.id)){c.x+=x-legalX;loadX+=mass.mass*c.x;}allX+=mass.mass*c.x;allY+=mass.mass*c.y;}
      loadX/=moving;var a=loadX+180;
      if(a<=0||a>=span)throw "Moving weight lies outside beam support span";
      var ov=beam(a,a,oldMoving*9.80665,e*oldI),ot=beam(180,a,oldMoving*9.80665,e*oldI),
        nv=beam(a,a,moving*9.80665,e*newI,span),nt=beam(180,a,moving*9.80665,e*newI,span);
      // 121 mm hob above X guide plane; 233 mm work above table mount.
      var oldError=[(121*ov[1]-233*ot[1])*1000,(ov[0]-ot[0])*1000],newError=[(121*nv[1]-(363-FoundationLayout.TABLE_SEAT)*nt[1])*1000,(nv[0]-nt[0])*1000];
      for(k in 0...2){oldLo[k]=Math.min(oldLo[k],oldError[k]);oldHi[k]=Math.max(oldHi[k],oldError[k]);newLo[k]=Math.min(newLo[k],newError[k]);newHi[k]=Math.max(newHi[k],newError[k]);}
      var rear=(allX/total+180)/span,front=1-rear,skew=(allY/total)/330;
      var reactions=[(front-skew)/2,(front+skew)/2,rear];
      for(r in reactions){if(r<=0)throw "Machine centre of mass leaves three-foot support triangle";worstFoot=Math.max(worstFoot,r*total*9.80665);}
      samples.push({xMm:x,movingLoadWorkXMm:loadX,oldRelativeRadialVerticalUm:oldError,newRelativeRadialVerticalUm:newError,footReactionN:[for(r in reactions)r*total*9.80665]});
    }
    var oldRange=Math.sqrt(Math.pow(oldHi[0]-oldLo[0],2)+Math.pow(oldHi[1]-oldLo[1],2)),newRange=Math.sqrt(Math.pow(newHi[0]-newLo[0],2)+Math.pow(newHi[1]-newLo[1],2));
    var allocation=gearkit.Iso1328.tolerance(machine.spec,"Falpha",6)*0.2;
    if(body.massProperties().mass/moving<5)throw "Base-to-X moving mass ratio below 5:1";
    if(2*newRange>allocation)throw "Base needs resizing: assumed 0.5x E deflection exceeds profile allocation";
    var work=machine.workOrigin.z;if(work<950||work>1100)throw "Work height outside target";
    if(worstFoot>15000)throw "Levelling foot exceeds assumed 15 kN design target";
    return {baseCastingKg:body.massProperties().mass,baseAssemblyKg:baseKg,machineKg:total,
      baseToMovingMassRatio:{X:body.massProperties().mass/moving,Y:body.massProperties().mass/movingY,Z:body.massProperties().mass/movingZ},
      column:{beforeCastingKg:oldColumn,afterCastingKg:machine.column.body.massProperties().mass,beforeAssemblyKg:oldColumn+oldInserts,afterAssemblyKg:columnKg},
      beforeLoadingRevision:Reflect.field(raw,"beforeLoadingRevision"),dimensionsMm:[body.parameterNumber("width"),body.parameterNumber("depth"),body.parameterNumber("height")],workingHeightMm:work,feet:{count:3,radialSpacingMm:span,frontTangentialSpacingMm:660,heightMm:47,worstReactionN:worstFoot,assumedTargetPerFootN:15000,vibrationMounts:false},
      table:{form:Std.isOfType(machine.table,machinekit.motion.PurchasedRotaryTable)?"purchased":"built",faceDiameterMm:machine.tableSpec.number("faceDiameter"),heightMm:machine.tableSpec.number("height"),throughBoreMm:machine.tableSpec.number("bore"),seatWorkZMm:FoundationLayout.TABLE_SEAT,fieldValues:machine.tableSpec.report(),basis:"per-field catalog/measured/assumed data; face/flange/centering provisional; bearing construction not claimed"},
      coolantReservedLitres:7.92,beam:{youngsNPerMm2:e,spanMm:span,oldMinimumIyMm4:oldI,newMinimumIyMm4:newI,
        oldChangeBoundUm:oldRange,newChangeBoundUm:newRange,atHalfModulusChangeBoundUm:2*newRange,assumedProfileAllocationUm:allocation,passed:true,samples:samples,
        basis:"Simply supported Euler-Bernoulli beam; minimum net orthogonal casting section over foot span, steel ignored; X-group weight lumped at its measured CAD mass centre; full mechanism X25..retract evaluated by exact prismatic COM translation, reference moving mass; old 210 mm bed uses its original 860 mm support span over the new X range; previous heavy-plinth/full-stroke result is retained separately; 121 mm guide Abbe height; current work-to-table-seat lever follows TABLE_SEAT; old table lever233 mm; bound combines radial and vertical peak-to-peak changes. Assumed E sensitivity 0.5x..2x. Excludes floor/foot compliance, torsion, joints, local contact and modes; external FEA remains open."}};
  }
}
