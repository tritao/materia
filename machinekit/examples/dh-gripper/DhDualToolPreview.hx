import cadkit.Shape;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import haxe.io.Bytes;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;

/** Isolated, visual-only dual electric wrist. No pneumatic controller or reach certificate is reused. */
class DhDualToolPreview {
  // Optimal OCCT bounds of imported spline faces are expensive. Cache source
  // bounds and transform their eight corners conservatively for each placement.
  static var envelopes:haxe.ds.ObjectMap<Shape,{low:Vector,high:Vector}> = new haxe.ds.ObjectMap();
  static function envelope(shape:Shape):{low:Vector,high:Vector} {
    var found=envelopes.get(shape);if(found!=null) return found;
    var bounds=shape.bounds(),a=bounds.get_min(),b=bounds.get_max();
    var result={low:new Vector(a.get_x(),a.get_y(),a.get_z()),high:new Vector(b.get_x(),b.get_y(),b.get_z())};
    envelopes.set(shape,result);return result;
  }
  static function box(x0:Float,x1:Float,y0:Float,y1:Float,z0:Float,z1:Float):Part {
    var raw=Part.box(x1-x0,y1-y0,z1-z0,Min,Min,Min);
    var out=raw.translated(new Vector(x0,y0,z0));raw.close();return out;
  }
  static function cut(part:Part,tools:Array<Part>):Part {
    var out=part.subtractAll(tools);part.close();for(tool in tools) tool.close();return out;
  }
  static function fuse(parts:Array<Part>):Part {
    var out=Part.fuseAll(parts);for(part in parts) part.close();return out;
  }
  static function location(frame:AssemblyFrame):Location {
    var m=AssemblyFrames.toRotationMatrix(frame);
    return new Location(new Plane(new Vector(frame.x,frame.y,frame.z),
      new Vector(m[0],m[3],m[6]),new Vector(m[2],m[5],m[8])));
  }
  static function seat(side:Int):AssemblyFrame {
    // Source +Y is approach. A: +Z, cable -X. B: +X, cable +Y.
    return side==0 ? AssemblyFrames.fromRotationMatrix(0,0,16,[-1,0,0, 0,0,1, 0,1,0])
      : AssemblyFrames.fromRotationMatrix(75,0,60,[0,1,0, 1,0,0, 0,0,-1]);
  }
  static function adapter():Part {
    // R3 internal root can be reached with a 6 mm cutter; the plate is machined
    // as one L-shaped billet rather than relying on an unmodelled bolted seam.
    var root=cut(box(60,63,-50,50,16,19),[Part.cylinderAlongY(3,-51,51,60,19)]);
    var result=fuse([box(-50,75,-50,50,0,16),box(63,75,-50,50,16,110),root]);
    var tools:Array<Part> = [];
    // Robot-side raised pilot in our existing RobotFlange reference.
    tools.push(Part.cylinderSpan(15.85,-1,3.2));
    var pin=25/Math.sqrt(2);
    tools.push(Part.cylinderSpan(3.1,-1,3.2,pin,pin));
    for(i in 0...4) {
      var a=i*Math.PI/2,x=25*Math.cos(a),y=25*Math.sin(a);
      tools.push(Part.cylinderSpan(3.3,-1,17,x,y));
      tools.push(Part.cylinderSpan(5.5,9.5,17,x,y));
    }
    // Supplier male centring boss: 31.5 diameter, protrudes 2.5 behind the seat.
    // M6 tap-drill geometry; drawings/tapping and screw lengths remain prototype checks.
    for(side in 0...2) {
      var mount=location(seat(side));
      for(i in 0...4) {
        var a=Math.PI/4+i*Math.PI/2;
        var hole=Part.cylinderAlongY(2.5,-8,0.5,25*Math.cos(a),25*Math.sin(a));
        var placed=mount.apply(hole.shape);hole.close();tools.push(new Part(placed));
      }
      var pocket=Part.cylinderAlongY(15.85,-2.7,0.5);
      tools.push(new Part(mount.apply(pocket.shape)));pocket.close();
    }
    // Low Expert splitter saddle on the exposed +Y edge: blind M3 tap drills.
    for(x in [-47.0,-3.0]) tools.push(Part.cylinderSpan(1.25,8,17,x,44));
    // Blind M4 tap drills for cable support A; opposing pin/tap holes retain a web.
    for(y in [-39.0,-29.0]) tools.push(Part.cylinderSpan(1.65,8,17,-45,y));
    // B support anchors normal to the side plate.
    for(z in [35.0,45.0]) tools.push(Part.cylinderAlong(1.65,new Vector(67,45,z),Vector.X(),9));
    result=cut(result,tools);var refined=result.refined();result.close();
    if(!refined.valid() || refined.solidCount()!=1) throw "Adapter is not one valid solid";
    return refined;
  }
  /** Lower split clamps include support arms all the way back to the adapter. */
  static function support(side:Int,cap:Bool):Part {
    var result:Part;
    // A is constructed at z=33, then raised 4 mm to its 16 mm adapter seat.
    // Final cable centres: A (-143,0,37), tangent -X; B (96,143,60), tangent +Y.
    if(side==0) result=cap ? box(-148,-138,-8,8,33.2,41)
      : fuse([box(-52,-40,-44,-24,12,27),box(-148,-40,-44,-26,20,25),box(-148,-138,-44,8,20,25),box(-148,-138,-8,8,25,33)]);
    else result=cap ? box(88,104,138,148,60.2,68)
      : fuse([box(75,104,40,50,30,50),box(88,104,40,148,43,50),box(88,104,138,148,50,60)]);
    if(!cap) {
      var rear=returnSupport(side,false);
      if(side==0) {var shifted=rear.translated(new Vector(0,0,-4));rear.close();rear=shifted;}
      result=fuse([result,rear]);
    }
    var tools:Array<Part> = [];
    tools.push(side==0 ? Part.cylinderAlong(3.3,new Vector(-149,0,33),Vector.X(),12)
      : Part.cylinderAlongY(3.3,137,149,96,60));
    for(radial in [-6.0,6.0]) {
      var x=side==0 ? -143.0 : 96+radial,y=side==0 ? radial : 143.0,z=side==0 ? 33.0 : 60.0;
      tools.push(Part.cylinderSpan(cap ? 1.7 : 1.25,z-6,z+9,x,y));
      if(cap) tools.push(Part.cylinderSpan(2.9,z+5,z+9,x,y));
    }
    if(!cap) for(y in [-6.0,6.0]) {
      if(side==0) {
        tools.push(Part.cylinderSpan(2.25,11,28,-45,y*5/6-34));
        tools.push(Part.cylinderSpan(3.75,23,28,-45,y*5/6-34));
      } else {
        tools.push(Part.cylinderAlong(2.25,new Vector(74,45,40+y*5/6),Vector.X(),31));
        tools.push(Part.cylinderAlong(3.75,new Vector(100,45,40+y*5/6),Vector.X(),5));
      }
    }
    result=cut(result,tools);var refined=result.refined();result.close();
    if(!refined.valid() || refined.solidCount()!=1) throw "Cable support is not one valid solid";
    if(side==0) {
      var raised=refined.translated(new Vector(0,0,4));refined.close();return raised;
    }
    return refined;
  }
  /** Second split saddle supports the return lead immediately before its connector.
      Both ends stay on the tool; no rigid cable is asserted across moving joints. */
  static function returnSupport(side:Int,cap:Bool):Part {
    // Supports are dimensioned for the shared route; fail rather than silently
    // leaving a saddle behind if the single-gripper service route changes.
    near(DhGripperPreview.STRAIGHT_MM,100,"Return support straight lead");
    near(DhGripperPreview.BEND_RADIUS_MM,55,"Return support bend radius");
    near(DhGripperPreview.RETURN_MM,40,"Return support return lead");
    // Cable tips A (-108,0,-73), B (-14,108,60). Saddles stop 10 mm
    // before those tips, leaving clearance for the supplier connector backshell.
    var result=side==0
      ? (cap ? box(-128,-118,-8,8,-72.8,-65)
        : fuse([box(-128,-118,-44,-26,-81,25),box(-128,-118,-44,8,-81,-76),box(-128,-118,-8,8,-81,-73)]))
      : (cap ? box(-22,-6,118,128,60.2,68)
        : fuse([box(-22,96,118,128,43,50),box(-22,-6,118,128,50,60)]));
    var tools:Array<Part> = [side==0 ? Part.cylinderAlong(3.3,new Vector(-129,0,-73),Vector.X(),12)
      : Part.cylinderAlongY(3.3,117,129,-14,60)];
    for(radial in [-6.0,6.0]) {
      var x=side==0 ? -123.0 : -14+radial,y=side==0 ? radial : 123.0,z=side==0 ? -73.0 : 60.0;
      tools.push(Part.cylinderSpan(cap ? 1.7 : 1.25,z-9,z+9,x,y));
      if(cap) tools.push(Part.cylinderSpan(2.9,z+5,z+9,x,y));
    }
    result=cut(result,tools);var refined=result.refined();result.close();
    if(!refined.valid() || refined.solidCount()!=1) throw "Return cable support is not one valid solid";
    return refined;
  }
  static function shapeAt(shape:Shape,frame:AssemblyFrame):Shape {
    var placed=location(frame).apply(shape),bounds=envelope(shape);
    var low=new Vector(1e30,1e30,1e30),high=new Vector(-1e30,-1e30,-1e30);
    for(x in [bounds.low.x,bounds.high.x]) for(y in [bounds.low.y,bounds.high.y]) for(z in [bounds.low.z,bounds.high.z]) {
      var p=AssemblyFrames.transformPoint(frame,x,y,z);
      low=new Vector(Math.min(low.x,p.x),Math.min(low.y,p.y),Math.min(low.z,p.z));
      high=new Vector(Math.max(high.x,p.x),Math.max(high.y,p.y),Math.max(high.z,p.z));
    }
    envelopes.set(placed,{low:low,high:high});return placed;
  }
  static function near(a:Float,b:Float,label:String):Void {
    if(!Math.isFinite(a) || Math.abs(a-b)>0.001) throw '$label: $a != $b';
  }
  static function overlap(a:Shape,b:Shape,label:String):Void {
    var common=a.common(b);
    var volume=common.subshapeCount(CadKit.ShapeKind.Solid)>0 ? common.volume() : 0;
    common.close();if(volume>0.001) throw '$label: overlap $volume mm3';
  }
  static function separation(a:Shape,b:Shape):Float {
    var ab=envelope(a),bb=envelope(b),al=ab.low,ah=ab.high,bl=bb.low,bh=bb.high;
    var dx=Math.max(0,Math.max(bl.x-ah.x,al.x-bh.x));
    var dy=Math.max(0,Math.max(bl.y-ah.y,al.y-bh.y));
    var dz=Math.max(0,Math.max(bl.z-ah.z,al.z-bh.z));
    return Math.sqrt(dx*dx+dy*dy+dz*dz);
  }
  public static function preview():Bytes {
    envelopes=new haxe.ds.ObjectMap();
    var source=SceneArtifact.decode(DhGripperPreview.previewCustom());
    var directory=DhGripperPreview.directory()+"/build/dual";sys.FileSystem.createDirectory(directory);
    var frame=DhGripperPreview.referenceFrame;
    if(frame==null) throw "Single-gripper build did not publish its reference frame";
    var flange=AssemblyFrames.translation(frame.flange[0],frame.flange[1],frame.flange[2]);
    var placements=[for(side in 0...2) AssemblyFrames.compose(seat(side),AssemblyFrames.inverse(flange))];
    var model=new AssemblyModel("mm"),parts:Array<SceneArtifactPart> = [],owned:Array<Shape> = [];
    var masses:Array<{id:String,massKg:Float,centreMm:Array<Float>,basis:String}> = [];
    function addMass(id:String,mass:Float,centre:Vector,basis:String):Void masses.push({id:id,massKg:mass,centreMm:[centre.x,centre.y,centre.z],basis:basis});
    function addMetal(id:String,name:String,part:Part,colour:Array<Float>):Void {
      var properties=part.massProperties(),mass=properties.volume*2700e-9;
      var shape=part.shape.cloneShape();owned.push(shape);part.close();
      shape.exportStep(directory+"/"+id+".step");
      var rendered=DhGripperPreview.render(id,name,shape,colour);parts.push(rendered);
      addMass(id,mass,properties.centerOfMass,"CAD volume, aluminium density 2700 kg/m3");
      model.add(id);
      if(id=="adapter") model.ground(id); else {
        model.connector("adapter",id,AssemblyFrames.identity());model.connector(id,"mount",AssemblyFrames.identity());
        model.mate(id+"-mount","fixed","adapter",id,id,"mount");
      }
    }
    try {
      addMetal("adapter","Prototype machined dual-wrist adapter",adapter(),[0.38,0.44,0.52]);
      for(side in 0...2) for(cap in [false,true]) addMetal('support-$side-'+(cap ? "cap" : "base"),
        'Cable ${side==0 ? "A" : "B"} '+(cap ? "split-clamp cap" : "support and lower clamp"),support(side,cap),[0.58,0.6,0.63]);
      for(side in 0...2) addMetal('return-$side-cap','Cable return saddle cap '+side,returnSupport(side,true),[0.58,0.6,0.63]);
      var metalCount=owned.length;
      var definition=source.assemblyDefinition;
      if(definition==null) throw "Single gripper lost its kinematic definition";
      for(side in 0...2) {
        var prefix=side==0 ? "a-" : "b-",placement=placements[side];
        for(part in source.parts) {
          var copy:SceneArtifactPart={id:prefix+part.id,name:(side==0 ? "A / " : "B / ")+part.name,
            red:part.red,green:part.green,blue:part.blue,physicalMassKg:0,physicalMassDeclared:true,
            vertexCount:part.vertexCount,indexCount:part.indexCount,vertices:part.vertices,normals:part.normals,
            indices:part.indices,edgeSegments:part.edgeSegments,edgeIds:part.edgeIds,faceRanges:part.faceRanges};
          parts.push(copy);
        }
        for(o in definition.occurrences) model.add(prefix+o.id);
        for(d in definition.definitions) for(c in d.connectors) model.connector(prefix+d.id,c.name,c.frame);
        for(j in definition.joints) model.mateOnAxis(prefix+j.id,j.type,prefix+j.parent,j.parentConnector,
          prefix+j.child,j.childConnector,j.axis,j.defaultValue,j.limits);
        for(c in definition.couplings) model.couple(prefix+c.id,prefix+c.source,prefix+c.target,c.ratio,c.offset);
        // Attach every independent supplier root, preserving connector placement exactly.
        for(o in definition.occurrences) {
          var attached=false;for(j in definition.joints) if(j.child==o.id) attached=true;
          if(!attached) {
            model.connector("adapter",prefix+o.id,AssemblyFrames.compose(placement,o.initialPose));
            model.connector(prefix+o.id,"dual-root",AssemblyFrames.identity());
            model.mate(prefix+"root-"+o.id,"fixed","adapter",prefix+o.id,prefix+o.id,"dual-root");
          }
        }
        model.connector(prefix+"pgc-140-50","grasp",AssemblyFrames.translation(frame.grasp[0],frame.grasp[1],frame.grasp[2]));
        // Supplier CoG is unavailable: centre of housing envelope is an explicit planning estimate.
        var centre=AssemblyFrames.transformPoint(placement,frame.flange[0],frame.flange[1]+70,frame.flange[2]);
        addMass(prefix+"gripper",1,new Vector(centre.x,centre.y,centre.z),"1 kg planning allowance; housing-centre CoG assumed; includes original fingers");
        var grasp=AssemblyFrames.transformPoint(placement,frame.grasp[0],frame.grasp[1],frame.grasp[2]);
        addMass(prefix+"blank",0.1944,new Vector(grasp.x,grasp.y,grasp.z),"60 x 60 x 20 mm aluminium blank");
        for(left in [true,false]) {
          var finger=machinekit.robotics.OffsetFingerGeometry.make(left);
          var translated=finger.shape.translate(new Vector(frame.offset[0],frame.offset[1],frame.offset[2]).native());finger.close();
          var world=shapeAt(translated,placement);translated.close();var properties=world.massProperties();world.close();
          // Mass is correct at any opening. CoG budget uses the held-blank (q=17) state below.
          var shift=location(placement).direction(new Vector(left ? -17 : 17,0,0));
          addMass(prefix+(left ? "finger-left" : "finger-right"),properties.volume*2700e-9,properties.centerOfMass.add(shift),"CNC finger CAD; conservatively added to full vendor allowance");
        }
      }
      var wrist=new machinekit.robotics.RobotFlange(50);
      model.connector("adapter","wrist-mount",wrist.pinAlignedFrame(0));
      var dual=model.definition("dh-dual-electric-reference");
      dual.exposedConnectors=[{name:"mount",occurrence:"adapter",connector:"wrist-mount"},{name:"grasp-a",occurrence:"a-pgc-140-50",connector:"grasp"},
        {name:"grasp-b",occurrence:"b-pgc-140-50",connector:"grasp"}];
      for(d in dual.definitions) d.collisionHulls=[];
      var state=new AssemblyState(dual);state.setJoint("a-"+DhGripperJaws.MASTER,17);state.setJoint("b-"+DhGripperJaws.MASTER,17);
      // Exhaustive sampled cross-tool envelope check. Bounds separation is conservative even for
      // supplier housing surfaces: it never claims a false clearance from a missing solid.
      var groups:Array<Array<Shape>> = [];
      {
        var group:Array<Shape> = [];
        for(file in ["pgc-140-50-local.step","electrical-cable-local.step","jaw-left-with-screws.step","jaw-right-with-screws.step","cnc-aluminium/finger-left.step","cnc-aluminium/finger-right.step","cable-end-connector-envelope.step","blank-60-local.step"]) {
          var s=Shape.importStep(DhGripperPreview.directory()+"/build/"+file);owned.push(s);group.push(s);
        }
        groups=[group,group];
      }
      for(i in 0...metalCount) for(j in i+1...metalCount) overlap(owned[i],owned[j],'Added metal $i / $j');
      var low=new Vector(1e30,1e30,1e30),high=new Vector(-1e30,-1e30,-1e30);
      function extend(shape:Shape):Void {
        var bounds=envelope(shape),a=bounds.low,b=bounds.high;
        low=new Vector(Math.min(low.x,a.x),Math.min(low.y,a.y),Math.min(low.z,a.z));
        high=new Vector(Math.max(high.x,b.x),Math.max(high.y,b.y),Math.max(high.z,b.z));
      }
      for(i in 0...metalCount) extend(owned[i]);
      var cached:Array<Array<Array<Shape>>> =[[],[]];
      Sys.println("Checking 121 dual-tool jaw combinations");
      var minimum=1e30;
      for(a in 0...11) for(b in 0...11) {
        state.setJoint("a-"+DhGripperJaws.MASTER,a*2.5);state.setJoint("b-"+DhGripperJaws.MASTER,b*2.5);
        var world:Array<Array<Shape>> = [];
        for(side in 0...2) {
          var prefix=side==0 ? "a-" : "b-",ids=["pgc-140-50","electrical-cable","jaw-left","jaw-right","finger-left","finger-right","cable-end-connector","blank-60"];
          var sample=side==0 ? a : b;
          if(cached[side].length<=sample) {
            var placed=[for(i in 0...ids.length) sample>0 && (i<2 || i>5)
              ? cached[side][0][i] : shapeAt(groups[side][i],state.worldPose(prefix+ids[i]))];
            cached[side].push(placed);
            for(i in 0...placed.length) if(sample==0 || (i>=2 && i<=5)) owned.push(placed[i]);
          }
          world.push(cached[side][sample]);
          // Supports must clear all gripper members and blanks. Adapter/body seat
          // contacts are intentional; jaws and fingers must clear the adapter.
          if((side==0 && b==0) || (side==1 && a==0)) for(i in 0...ids.length) {
            if(i==1 || (sample>0 && (i==0 || i>=6))) continue; // fixed roots checked once; cable below
            for(metal in (i==0 ? 1 : 0)...metalCount) {
              var distance=separation(world[side][i],owned[metal]);
              if(distance<0.5) distance=world[side][i].distance(owned[metal]);
              if(distance<0.05) throw 'Tool $prefix${ids[i]} / metal $metal clearance $distance mm';
            }
          }
          var left=state.worldPose(prefix+"finger-left"),jaw=state.worldPose(prefix+"jaw-left");
          var relativeLeft=AssemblyFrames.compose(AssemblyFrames.inverse(state.worldPose(prefix+"pgc-140-50")),left);
          var relativeRight=AssemblyFrames.compose(AssemblyFrames.inverse(state.worldPose(prefix+"pgc-140-50")),state.worldPose(prefix+"finger-right"));
          var expected=(side==0 ? a : b)*2.5;
          near(relativeLeft.x,-expected,"Independent left jaw opening");near(relativeRight.x,expected,"Synchronized right jaw opening");
          near(relativeLeft.y,0,"Local jaw Y drift");near(relativeLeft.z,0,"Local jaw Z drift");
          near(left.x,jaw.x,"Finger attachment X");near(left.y,jaw.y,"Finger attachment Y");near(left.z,jaw.z,"Finger attachment Z");
        }
        for(x in world[0]) for(y in world[1]) {
          var distance=separation(x,y);
          if(distance<0.5) {
            // Resolve a tight conservative envelope with the actual surface/solid distance.
            distance=x.distance(y);if(distance<0.5) throw 'Cross-gripper clearance below 0.5 mm at samples $a/$b: A ${world[0].indexOf(x)} / B ${world[1].indexOf(y)}';
          }
          minimum=Math.min(minimum,distance);
        }
        for(group in world) for(shape in group) extend(shape);
      }
      state.setJoint("a-"+DhGripperJaws.MASTER,17);state.setJoint("b-"+DhGripperJaws.MASTER,17);
      // Check routed cable against all added metal. A 0.1 mm nominal radial clamp gap
      // is geometric clearance only; compliant liners/preload need engineering validation.
      for(side in 0...2) {
        var cable=shapeAt(groups[side][1],placements[side]);
        for(i in 0...metalCount) overlap(cable,owned[i],'Cable $side / support metal $i');
        cable.close();
      }
      var mass=0.0,weighted=new Vector();for(item in masses) {mass+=item.massKg;weighted=weighted.add(new Vector(item.centreMm[0],item.centreMm[1],item.centreMm[2]).scale(item.massKg));}
      // Reserve includes mounting/clamp screws, liners and unweighed cable/connectors.
      var reserve=0.25;addMass("hardware-and-service-reserve",reserve,new Vector(0,0,60),"Planning allowance, not a detailed fastener/cable BOM");
      mass+=reserve;weighted=weighted.add(new Vector(0,0,60).scale(reserve));var cog=weighted.scale(1/mass);
      var moment=mass*9.81*cog.length()*0.001;
      var serviceAdapter=machinekit.robotics.DhDualGripperServices.adapter();
      var servicePurchases=new machinekit.assembly.MachineAssembly();
      machinekit.robotics.DhDualGripperServices.addPurchases(servicePurchases);
      sys.io.File.saveContent(directory+"/service-bom.json",haxe.Json.stringify(servicePurchases.billOfMaterials().lines(),null,"  "));
      var report={selectedServiceAdapter:{manufacturerPartNumber:serviceAdapter.manufacturerPartNumber,
          supplierPartNumber:serviceAdapter.supplierPartNumber,sourceUrl:serviceAdapter.sourceUrl,
          externalPower:serviceAdapter.externalPower,geometryInstalled:false,pinoutVerified:serviceAdapter.pinoutVerified,
          status:"Selected Expert purchase; external 24 V supply, DH wiring and physical installation pending"},
        status:"prototype reference; no reach or structural certification",jawSamples:121,
        approachAngleDegrees:90,
        cableRouting:{style:"Supported 180 degree returns toward wrist; no cable across moving joints yet",
          bendRadiusMm:55,straightExitMm:100,returnLengthMm:40,
          connectorStatus:"Supplier connector supported by adjacent return saddle; Expert adapter selected; mating compatibility and installed flexible harness unverified"},
        sampledEnvelopeMm:{min:[low.x,low.y,low.z],max:[high.x,high.y,high.z],size:[high.x-low.x,high.y-low.y,high.z-low.z]},minimumSampledCrossToolClearanceMm:minimum,
        loadedMassKg:mass,centreOfGravityMm:[cog.x,cog.y,cog.z],gravityMomentUpperBoundNm:moment,
        reach1300NominalPayloadKg:12.5,withinNominalMassLimit:mass<12.5,massBudget:masses,
        pendingChecks:["Actual vendor mass, CoG and wrist moment/inertia limits","Adapter stiffness, fatigue and tolerances",
          "Mounting screw lengths, engagement and access; clamp fasteners and liners",
          "Cable flex/abrasion, service-loop envelope and moving wrist clearance",
          "Electric driver, feedback, force control and physical grasp attachment",
          "Machine/vise access and full tending reach/collision certificate"]};
      sys.io.File.saveContent(directory+"/report.json",haxe.Json.stringify(report,null,"  "));
      var bytes=SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:parts,assemblyDefinition:dual,assemblyState:state.record()});
      sys.io.File.saveBytes(directory+"/dual-electric-tool.mtrg",bytes);
      Sys.println('Dual wrist: 121 jaw combinations checked; minimum cross-tool clearance >= $minimum mm; loaded budget $mass kg; gravity moment bound $moment Nm');
      for(shape in owned) shape.close();envelopes=new haxe.ds.ObjectMap();return bytes;
    } catch(error:Dynamic) {for(shape in owned) shape.close();envelopes=new haxe.ds.ObjectMap();throw error;}
  }
  public static function main():Void {DhExpertToolPreview.preview();}
}
