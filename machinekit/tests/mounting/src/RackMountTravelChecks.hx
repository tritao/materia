import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import machinekit.assembly.PosedParts;
import machinekit.assembly.PosedParts.PartBox;
import machinekit.gantry.Gantry;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import cadkit.modeling.Part;

private typedef TravelPart = {
 id:String, shape:Part, box:PartBox, motion:Array<Array<Float>>, hinge:Null<String>, hingeRadius:Float,rotary:Null<String>,
};

/** Continuous translational clearance certificate for the new mount against the
 * rest of the machine. Native distance is 1-Lipschitz under relative translation:
 * d(mid) - sum(norm(relative axis vector)*half interval) bounds an entire cell.
 * Unresolved cells fail; samples alone never certify a pass. Same-motion fixed
 * interfaces and the bounded pivot's internal fits are qualified separately. */
class RackMountTravelChecks {
 static function focused(id:String):Bool return StringTools.startsWith(id,"rackMount") ||
  StringTools.startsWith(id,"pinion") || StringTools.startsWith(id,"switchY") || StringTools.startsWith(id,"switchTriggerY");
 static function norm(v:Array<Float>):Float return Math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
 static function boxDistance(a:PartBox,b:PartBox):Float {
  var x=Math.max(0,Math.max(a.minX-b.maxX,b.minX-a.maxX));
  var y=Math.max(0,Math.max(a.minY-b.maxY,b.minY-a.maxY));
  var z=Math.max(0,Math.max(a.minZ-b.maxZ,b.minZ-a.maxZ));
  return Math.sqrt(x*x+y*y+z*z);
 }
 static function swept(p:TravelPart,lo:Array<Float>,hi:Array<Float>,inflate:Float):PartBox {
  var low=[p.box.minX-inflate,p.box.minY-inflate,p.box.minZ-inflate];
  var high=[p.box.maxX+inflate,p.box.maxY+inflate,p.box.maxZ+inflate];
  for(k in 0...p.motion.length)for(d in 0...3){low[d]+=Math.min(p.motion[k][d]*lo[k],p.motion[k][d]*hi[k]);high[d]+=Math.max(p.motion[k][d]*lo[k],p.motion[k][d]*hi[k]);}
  return {minX:low[0],minY:low[1],minZ:low[2],maxX:high[0],maxY:high[1],maxZ:high[2]};
 }
 public static function run(machine:Gantry,bed:Float,?onlyAxis:String):Void {
  var definition=machine.definition(),parents:Map<String,KinematicJoint>=[];
  for(j in definition.joints)if(j.role==AssemblyJointRole.Tree)parents.set(j.child,j);
  var axes=["x","y","z"],lo=[for(a in machine.axes)a.lower-machine.axisOvertravel(a.id)],hi=[for(a in machine.axes)a.upper+machine.axisOvertravel(a.id)];
  var directions=[for(id in axes)machine.axisDirection(id)],state=new cadkit.modeling.AssemblyState(definition);
  for(j in definition.joints)if(j.role==AssemblyJointRole.Tree && j.type==AssemblyJointType.Prismatic && axes.indexOf(j.id)<0) {
   var lower=j.limits.lower,upper=j.limits.upper;if(lower==null||upper==null)throw "Unbounded accessory slide: "+j.id;
   if(j.defaultValue!=0)throw "Accessory sweep needs a zero-coordinate geometry datum: "+j.id;
   var room=j.limits.overtravel==null?0:j.limits.overtravel;
   axes.push(j.id);lo.push(lower-room);hi.push(upper+room);
   var parent=state.worldConnector(j.parent,j.parentConnector),v=F.transformVector(parent,j.axis.x,j.axis.y,j.axis.z);
   var scale=Math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z);directions.push([v.x/scale,v.y/scale,v.z/scale]);
  }
  var parts:Array<TravelPart>=[],failures:Array<Dynamic>=[],proofs=0,nativeCalls=0,skippedFits=0,qualifiedFits:Array<Dynamic>=[];
  function ancestors(id:String):Array<KinematicJoint> {
   var rows:Array<KinematicJoint>=[],seen:Map<String,Bool>=[];
   while(parents.exists(id)) {if(seen.exists(id))throw "Joint cycle";seen.set(id,true);var j=parents.get(id);if(j==null)throw "Missing parent joint";rows.push(j);id=j.parent;}
   return rows;
  }
  PosedParts.scope(cache->{
   try {
    for(m in machine.components()) {
     var ancestry=ancestors(m.id),motion=[for(_ in axes)[0.0,0,0]],hinge:Null<String>=null;
     for(j in ancestry) {
      if(j.type==AssemblyJointType.Prismatic) {
       var k=axes.indexOf(j.id);if(k<0)throw "Unmodelled translating joint "+j.id;
       motion[k]=directions[k];
      }
      if(j.type==Revolute) {if(hinge!=null)throw "Nested pivot needs an explicit sweep";hinge=j.child;}
     }
     var local:Part=null,owned=false,rotary=[for(j in ancestry)if(j.type==Continuous)j];
     // A chain's Envelope contains every routed position, including overtravel.
     // It is deliberately conservative and does not rely on interpolation of CAD.
     for(binding in machine.cableChainBindings)if(binding.member==m.id){local=binding.chain.geometry(Envelope);owned=true;}
     if(local==null) {
      if(rotary.length>0) {
       owned=true;
       // All translational gantry rotary members are coaxial with their local Z.
       // Verify that fact from joint frames before using the enclosing cylinder.
       var own=cache.box(m.component,F.identity()),radius=0.0;
       for(x in [own.minX,own.maxX])for(y in [own.minY,own.maxY])radius=Math.max(radius,Math.sqrt(x*x+y*y));
       if(Std.isOfType(m.component,machinekit.motion.SteppedShaft)) {
        // Preserve actual shoulders; a single max-radius cylinder would fill
        // the gearbox's smaller bore with an imaginary shaft collar.
        var shaft:machinekit.motion.SteppedShaft=cast m.component,segments:Array<Part>=[],z0=0.0;
        for(section in shaft.sections){segments.push(Part.cylinderSpan(section.diameter/2,z0,z0+section.length));z0+=section.length;}
        local=machinekit.component.Solids.union(segments);
       } else {
        var raw=Part.cylinder(radius,own.maxZ-own.minZ);
        local=raw.translated(new cadkit.modeling.Vector(0,0,own.minZ));raw.close();
       }
       // Refuse a silently off-axis envelope if the joint's shaft is elsewhere.
       var shaft=machine.zeroPose(rotary[0].child),relative=F.compose(F.inverse(shaft),machine.zeroPose(m.id));
       var z=F.transformVector(relative,0,0,1);
       if(Math.abs(relative.x)+Math.abs(relative.y)>1e-6||Math.abs(z.x)+Math.abs(z.y)>1e-6)throw "Noncoaxial rotary member needs a sweep: "+m.id;
      } else local=cache.local(m.component);
     }
     var posed=PosedParts.moved(local,machine.zeroPose(m.id));
     if(owned)local.close();
     var radius=0.0;
     if(hinge!=null) {
      var relative=F.compose(F.inverse(machine.zeroPose(hinge)),machine.zeroPose(m.id));
      var bounds=PosedParts.carried(cache.box(m.component,F.identity()),relative);
      for(x in [bounds.minX,bounds.maxX])for(y in [bounds.minY,bounds.maxY])radius=Math.max(radius,Math.sqrt(x*x+y*y));
     }
     parts.push({id:m.id,shape:posed,box:PosedParts.boxOf(posed),motion:motion,hinge:hinge,hingeRadius:radius,rotary:rotary.length==0?null:rotary[0].id});
    }
    Sys.println('Mount continuous sweep prepared '+parts.length+' parts, bed '+bed);Sys.stdout().flush();
    var maximumAngle=machinekit.transmission.RadialPinionAssembly.upperAngle();
    var sweptBoxes=[for(p in parts)swept(p,lo,hi,0)];
    for(i in 0...parts.length)for(j in i+1...parts.length) {
     var a=parts[i],b=parts[j];if(!focused(a.id)&&!focused(b.id))continue;
     var relative=[for(k in 0...axes.length)[for(d in 0...3)a.motion[k][d]-b.motion[k][d]]];
     var weights=[for(v in relative)norm(v)];
     if(onlyAxis!=null && weights[axes.indexOf(onlyAxis)]==0)continue;
     var weightSum=0.0;for(w in weights)weightSum+=w;
     if(weightSum==0 && a.hinge==b.hinge && a.rotary==b.rotary) {skippedFits++;continue;}
     // Tooth engagement is intentional; the independent periodic mesh check
     // verifies the real teeth. It cannot be tested with an outer cylinder.
     if((a.id=="pinionX"&&b.id=="rackX")||(b.id=="pinionX"&&a.id=="rackX") ||
        (a.id=="pinionYLeft"&&b.id=="rackYLeft")||(b.id=="pinionYLeft"&&a.id=="rackYLeft") ||
        (a.id=="pinionYRight"&&b.id=="rackYRight")||(b.id=="pinionYRight"&&a.id=="rackYRight"))continue;
     var inflation=a.hinge==b.hinge?0:(a.hingeRadius+b.hingeRadius)*maximumAngle;
     var boxGap=boxDistance(sweptBoxes[i],sweptBoxes[j]);
     if(boxGap>=1.5+inflation){proofs++;continue;}
     var fit=RackMountInterfaces.boundedMotionFit(machine,a.id,b.id);
     if(fit!=null){qualifiedFits.push({a:a.id,b:b.id,qualification:fit});continue;}
     var tolerance=RackMountInterfaces.sameFitAssembly(machine,a.id,b.id)?0.15:1.5;
     var required=tolerance+inflation;
     if(boxGap>=required) {proofs++;continue;}
     var cells=[{low:lo.copy(),high:hi.copy()}],tested=0,failed=false,minDistance=Math.POSITIVE_INFINITY;
     while(cells.length>0 && !failed) {
      if(++tested>20000){failures.push({a:a.id,b:b.id,reason:"interval limit; clearance not certified"});failed=true;break;}
      var cell=cells.pop();if(cell==null)throw "Missing interval cell";
      var mid=[for(k in 0...axes.length)(cell.low[k]+cell.high[k])/2],delta=[0.0,0,0],radius=0.0,split=-1,largest=0.0;
      for(k in 0...axes.length){var span=weights[k]*(cell.high[k]-cell.low[k])/2;radius+=span;if(span>largest){largest=span;split=k;}for(d in 0...3)delta[d]+=relative[k][d]*mid[k];}
      var moved=a.shape.translated(new cadkit.modeling.Vector(delta[0],delta[1],delta[2]));
      var distance=moved.distance(b.shape);moved.close();nativeCalls++;minDistance=Math.min(minDistance,distance);
      if(!Math.isFinite(distance)||distance<0)throw "Native distance failed";
      if(distance-radius>=required)continue;
      if(distance<required || largest<0.01) {
       failures.push({a:a.id,b:b.id,poseMm:mid,distanceMm:distance,requiredMm:required,passiveAllowanceMm:inflation,
        reason:distance<required?"clearance below tolerance envelope":"unresolved interval"});failed=true;break;
      }
      var left={low:cell.low.copy(),high:cell.high.copy()},right={low:cell.low.copy(),high:cell.high.copy()};
      left.high[split]=mid[split];right.low[split]=mid[split];cells.push(left);cells.push(right);
     }
     if(!failed)proofs++;
    }
   } catch(e:Dynamic){for(p in parts)p.shape.close();throw e;}
   for(p in parts)p.shape.close();
  });
  var output={bedMm:bed,toleranceMm:1.5,rangeMm:{axes:axes,low:lo,high:hi},certifiedPairs:proofs,nativeDistances:nativeCalls,
   fixedInterfacePairsExcluded:skippedFits,qualifiedFits:qualifiedFits,failures:failures,status:failures.length==0?"continuous sweep passes; declared fits and mesh have separate qualifications":"FAIL"};
  sys.io.File.saveContent('build/router-mount-implementation/travel-'+Std.int(bed)+'.json',haxe.Json.stringify(output,null,"  "));
  Sys.println('Mount continuous sweep '+bed+': '+proofs+' certified pairs, '+nativeCalls+' distances, '+failures.length+' failures');Sys.stdout().flush();
  if(failures.length>0)throw "Mount travel clearance failed; see report";
 }
 public static function main():Void {
  var bed=Sys.args().length>0?Std.parseFloat(Sys.args()[0]):2600.0;
  if(bed!=2600 && bed!=3150)throw "Choose a supported bed";
  var machine=new SheetRouter(new SheetRouterSpec(bed));machine.check().throwIfErrors();run(machine,bed);
 }
}
