package machinekit.hobbing;

/** Derived stack and swept host interfaces for the hobber Y module. */
class HobberYLayout {
  public static final JOINT="Ymodule/travel";
  static final cached=new machinekit.cache.Once<machinekit.motion.ScrewLinearModuleSpec>();
  public static function moduleSpec():machinekit.motion.ScrewLinearModuleSpec return cached.get(()->{
    var first=new machinekit.motion.ScrewLinearModuleSpec(),face=HobberSaddle.headFaceX();
    var roof=machinekit.motion.ScrewLinearModuleCover.roof(first)-first.carriageBottom()-first.carriageThickness+machinekit.motion.ScrewLinearModuleCover.SHEET;
    var raw=cadkit.modeling.Part.box(roof,2000,2000),slab=raw.translated(new cadkit.modeling.Vector(face-roof/2,0,-1000));raw.close();
    var y=0.0,loZ=0.0,hiZ=0.0;
    for(role in ["GE-spindle-seat","GE-motor-cradle","GE-outboard-carrier"]) {
      var part=new HobberSaddle(role).geometry(),cut=part.intersect(slab);
      if(cut.volume()>0.001){var b=machinekit.assembly.PosedParts.boxOf(cut);y=Math.max(y,Math.max(Math.abs(b.minY),Math.abs(b.maxY)));loZ=Math.min(loZ,b.minZ);hiZ=Math.max(hiZ,b.maxZ);}
      cut.close();part.close();
    }
    slab.close();
    var bore=Bt30SpindleData.FLANGE_DIAMETER/2+3,bolt=machinekit.standard.SocketHeadCapScrew.catalog().get(first.guide.blockMountThread);
    var reach=Math.max(first.railSpacing/2+first.guide.blockHoleSpacingAcross/2,Math.abs(first.scaleCarrierY()));
    var lift:Float=Math.ceil((reach+1.5*bolt.diameter+Math.sqrt(Math.max(0.0,bore*bore-face*face)))/5)*5;
    var required=lift,length=2*(y+2*(first.stroke/2+first.overtravel)+10+4+machinekit.motion.ScrewLinearModuleSpec.GAP);
    for(i in 0...5) {
      var width=2*(Math.max(Math.abs(loZ-lift),Math.abs(hiZ-lift))+10+4+machinekit.motion.ScrewLinearModuleSpec.GAP);
      var s=new machinekit.motion.ScrewLinearModuleSpec("HGH15CA",20,150,80,5,length,width);
      var next=Math.max(required,s.baseWidth/2+machinekit.motion.ScrewLinearModuleCover.SHEET+machinekit.motion.ScrewLinearModuleSpec.GAP+HobberSaddle.xCarriageTop()-HobberZLayout.SEAT_Z);
      if(Math.abs(next-lift)<1e-6)return s;lift=next;
    }
    throw "Y stock/cover stack did not converge";
  });
  public static function moduleLift():Float return Math.max(requiredLift(),moduleSpec().baseWidth/2+machinekit.motion.ScrewLinearModuleCover.SHEET+machinekit.motion.ScrewLinearModuleSpec.GAP+HobberSaddle.xCarriageTop()-HobberZLayout.SEAT_Z);
  static function requiredLift():Float {
    var s=moduleSpec(),bore=Bt30SpindleData.FLANGE_DIAMETER/2+3,face=HobberSaddle.headFaceX(),bolt=machinekit.standard.SocketHeadCapScrew.catalog().get(s.guide.blockMountThread);
    return Math.ceil((Math.max(s.railSpacing/2+s.guide.blockHoleSpacingAcross/2,Math.abs(s.scaleCarrierY()))+1.5*bolt.diameter+Math.sqrt(Math.max(0.0,bore*bore-face*face)))/5)*5;
  }
  public static function modulePose():materia.assembly.AssemblyRecord.AssemblyFrame {
    var s=moduleSpec(),back=HobberSaddle.headFaceX()+s.carriageThickness+s.carriageBottom();
    return materia.assembly.AssemblyFrames.fromRotationMatrix(back,s.centre,HobberZLayout.SEAT_Z+moduleLift(),[0.0,0,-1,-1,0,0,0,1,0]);
  }
  public static function headInModule():materia.assembly.AssemblyRecord.AssemblyFrame return materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.inverse(modulePose()),materia.assembly.AssemblyFrames.translation(0,0,HobberZLayout.SEAT_Z));
  /** Host cover seals to the carriage BACK: its roof clears all moving
   * blocks/carriers while the head plate overhangs freely on the front. */
  public static function guardRoof():Float {
    var s=moduleSpec(),m3=machinekit.standard.SocketHeadCapScrew.catalog().get("M3"),nut=machinekit.standard.HexNut.catalog().get("M3"),washer=machinekit.standard.FlatWasher.catalog().get("M3");
    var reach=nut.height+washer.thickness+2*m3.pitch+machinekit.motion.ScrewLinearModuleSpec.GAP+1;
    for(face in HobberSaddle.yHeadFaces())switch face.role {
      case Clamped(size,_,_,_):
        var screw=machinekit.standard.SocketHeadCapScrew.catalog().get(size),extra=face.washer?machinekit.standard.FlatWasher.catalog().get(size).thickness:0.0;
        reach=Math.max(reach,screw.headHeight+extra+machinekit.motion.ScrewLinearModuleSpec.GAP+0.2); // ASSUMED0.2 assembly margin.
      case _:
    }
    return s.carriageBottom()-Math.ceil(reach);
  }
  public static function stopLift():Float {
    var part=new machinekit.motion.ScrewLinearModulePart(moduleSpec().values().setToken("role","stopFrontLeft")),depth=0.0;
    for(face in part.pinnedFaces())depth=Math.max(depth,face.depth);
    return depth+machinekit.motion.ScrewLinearModuleSpec.GAP;
  }
  public static function hoodRoof():Float {
    var s=moduleSpec();return Math.max(machinekit.motion.ScrewLinearModuleCover.roof(s),s.carriageBottom()+s.carriageThickness+stopLift()+machinekit.motion.ScrewLinearModuleSpec.GAP);
  }
  static function railClearance():Float {
    var rail=machinekit.motion.LinearRailSystem.catalog().get("HGH25CA");if(rail==null)throw "Missing X guide";
    var stroke=HobberScales.stroke("Y"),half=(stroke.upper-stroke.lower)/2+HobberScales.overtravel();
    return FoundationLayout.bedRailSeatTop()+rail.railHeight+machinekit.motion.ScrewLinearModuleSpec.GAP
      +half*Math.sin(HobberSwivelLayout.RANGE)-HobberZLayout.SEAT_Z-HobberZLayout.TRAVEL_MIN+HobberScales.overtravel();
  }
  /** Conservative floor above either X rail through Y/Z overtravel and A.
   * This removes free lower corners, never changes an operating limit. */
  public static function headClearance():cadkit.modeling.Part {
    var a=HobberSwivelLayout.RANGE,c=railClearance(),sn=Math.sin(a),cs=Math.cos(a);
    var raw=cadkit.modeling.Part.prism([new cadkit.modeling.Vector(-1000,(c+1000*sn)/cs),new cadkit.modeling.Vector(0,c/cs),new cadkit.modeling.Vector(1000,(c+1000*sn)/cs),new cadkit.modeling.Vector(1000,1000),new cadkit.modeling.Vector(-1000,1000)],-1000,1000);
    var out=machinekit.assembly.PosedParts.moved(raw,materia.assembly.AssemblyFrames.fromRotationMatrix(0,0,0,[0.0,0,1,1,0,0,0,1,0]));raw.close();return out;
  }
  static function clip(points:Array<{x:Float,y:Float}>,nx:Float,ny:Float,c:Float):Array<{x:Float,y:Float}> {
    var out:Array<{x:Float,y:Float}>=[];
    for(i in 0...points.length) {
      var a=points[i],b=points[(i+1)%points.length],da=nx*a.x+ny*a.y-c,db=nx*b.x+ny*b.y-c;
      if(da>=-1e-8)out.push(a);
      if((da<0)!=(db<0)){var t=da/(da-db);out.push({x:a.x+t*(b.x-a.x),y:a.y+t*(b.y-a.y)});}
    }
    return out;
  }
  static final windowSize=new machinekit.cache.Once<{a:Float,b:Float}>();
  static function movingWindow():{a:Float,b:Float} return windowSize.get(()->{
    var s=moduleSpec(),raw=machinekit.motion.LinearRailBlock.metric(s.profile,machinekit.motion.LinearRailBlock.SUPPLIER).geometry();
    var block=machinekit.assembly.PosedParts.moved(raw,machinekit.assembly.AxisBuilder.orient(0,0,0,[0.0,0,1],[1.0,0,0]));raw.close();
    var b=machinekit.assembly.PosedParts.boxOf(block);block.close();
    var carrier=new machinekit.motion.ScrewLinearModulePart(s.values().setToken("role","scaleHeadCarrier")).geometry(),head=machinekit.assembly.PosedParts.boxOf(carrier);carrier.close();
    var margin=4+machinekit.motion.ScrewLinearModuleSpec.GAP,half=s.stroke/2+s.overtravel;
    var along=Math.max(s.blockSpacing/2+Math.max(Math.abs(b.minX),Math.abs(b.maxX)),Math.max(s.centre-head.minX,head.maxX-s.centre))+half+margin;
    var across=Math.max(s.railSpacing/2+Math.max(Math.abs(b.minY),Math.abs(b.maxY)),Math.max(Math.abs(head.minY),Math.abs(head.maxY)))+margin;
    // Keep the swept seal clear of through holes and protruding heads.
    var obstacles:Array<{x:Float,y:Float,radius:Float}>=[];
    for(face in HobberSaddle.yHeadFaces())switch face.role {
      case Clamped(size,_,_,_):
        var screw=machinekit.standard.SocketHeadCapScrew.catalog().get(size),radius=(face.washer?machinekit.standard.FlatWasher.catalog().get(size).outerDiameter:screw.headDiameter)/2+margin;
        var frame=materia.assembly.AssemblyFrames.compose(headInModule(),face.frame),holes=face.holes;
        if(holes!=null)for(h in holes){var p=materia.assembly.AssemblyFrames.transformPoint(frame,h.x,h.y,0);obstacles.push({x:Math.abs(p.x-s.centre),y:Math.abs(p.y),radius:radius});}
      case _:
    }
    for(face in HobberSaddle.yHeadPins())if(face.through) {
      var frame=materia.assembly.AssemblyFrames.compose(headInModule(),face.frame);
      for(h in face.holes){var p=materia.assembly.AssemblyFrames.transformPoint(frame,h.x,h.y,0);obstacles.push({x:Math.abs(p.x-s.centre),y:Math.abs(p.y),radius:face.diameter/2+margin});}
    }
    for(i in 0...(2*obstacles.length))for(p in obstacles) {
      if(p.y<=across+p.radius&&Math.abs(p.x-along)<p.radius+half)along=Math.max(along,p.x+p.radius+half);
      if(p.x<=along+p.radius+half&&Math.abs(p.y-across)<p.radius)across=Math.max(across,p.y+p.radius);
    }
    return {a:along,b:across};
  });
  public static function window(dx:Float=0,dy:Float=0):Array<{x:Float,y:Float}> {
    var s=moduleSpec(),size=movingWindow(),a=size.a,b=size.b;
    return [{x:s.centre-a-dx,y:-b-dy},{x:s.centre+a+dx,y:-b-dy},{x:s.centre+a+dx,y:b+dy},{x:s.centre-a-dx,y:b+dy}];
  }
  public static function outline(base:Bool,offset:Float=0):Array<{x:Float,y:Float}> {
    var s=moduleSpec(),x0=base?s.baseMin:s.centre-s.carriageLength/2,x1=base?s.baseMax:s.centre+s.carriageLength/2,b=(base?s.baseWidth:s.carriageWidth)/2;
    var sn=Math.sin(HobberSwivelLayout.RANGE),cs=Math.cos(HobberSwivelLayout.RANGE),c=railClearance()-cs*moduleLift()+(base?1:0)-offset;
    return clip(clip([{x:x0-offset,y:-b-offset},{x:x1+offset,y:-b-offset},{x:x1+offset,y:b+offset},{x:x0-offset,y:b+offset}],sn,cs,c+sn*s.centre),-sn,cs,c-sn*s.centre);
  }
  public static function prism(points:Array<{x:Float,y:Float}>,z0:Float,z1:Float):cadkit.modeling.Part return
    cadkit.modeling.Part.prism([for(p in points)new cadkit.modeling.Vector(p.x,p.y)],z0,z1);
  static final housingBox=new machinekit.cache.Once<machinekit.assembly.PosedParts.PartBox>();
  static function scaleBox():machinekit.assembly.PosedParts.PartBox return housingBox.get(()->{
    var s=moduleSpec(),raw=s.scaleHousing.geometry(),placed=machinekit.assembly.PosedParts.moved(raw,s.scalePose());raw.close();
    var bounds=machinekit.assembly.PosedParts.boxOf(placed);placed.close();return bounds;
  });
  public static function sealBolts(upper:Bool):Array<{x:Float,y:Float}> {
    var s=moduleSpec(),points=window(6,6),out:Array<{x:Float,y:Float}>=[];
    for(i in 0...points.length) {
      var a=points[i],b=points[(i+1)%points.length],dx=b.x-a.x,dy=b.y-a.y,n=Std.int(Math.ceil(Math.sqrt(dx*dx+dy*dy)/150));
      for(j in 0...n) {
        var t=(j+0.5)/n,y=a.y+t*dy,seam=1.5*machinekit.standard.SocketHeadCapScrew.catalog().get("M3").diameter+machinekit.motion.ScrewLinearModuleSpec.GAP;
        for(shift in Math.abs(y)<1e-8?[-seam,seam]:[0.0])if((y+shift>0)==upper)out.push({x:a.x+t*dx,y:y+shift});
      }
    }
    var housing=scaleBox(),screw=machinekit.standard.SocketHeadCapScrew.catalog().get("M3"),margin=screw.headDiameter/2+machinekit.motion.ScrewLinearModuleSpec.GAP;
    return out.filter(p->p.x<housing.minX-margin||p.x>housing.maxX+margin||p.y<housing.minY-margin||p.y>housing.maxY+margin);
  }
  static final reliefCache=new machinekit.cache.Once<cadkit.modeling.Part>();
  public static function sweptDeckRelief():cadkit.modeling.Part {
    var raw=reliefCache.get(()->{
      var drop=HobberZLayout.TRAVEL_MAX-HobberZLayout.TRAVEL_MIN+2*HobberScales.overtravel();
      var gap=machinekit.motion.ScrewLinearModuleSpec.GAP,skirt=new HobberChipCover(new machinekit.component.ComponentValues().setToken("role","Xskirt")).geometry();
      var b=machinekit.assembly.PosedParts.boxOf(skirt);skirt.close();
      // Sweep the stationary deck/skirt through the complete relative Z
      // passage. A single low-Z slab misses material near its lower face.
      var skirtBox=cadkit.modeling.Part.box(b.maxX-b.minX+2*gap,b.maxY-b.minY+2*gap,b.maxZ-b.minZ+2*gap+drop);
      var skirtKeep=skirtBox.translated(new cadkit.modeling.Vector((b.minX+b.maxX)/2,(b.minY+b.maxY)/2,b.minZ-gap-drop));skirtBox.close();
      var deck=machinekit.component.Solids.union([HobberSaddle.xDeckKeepout(gap,drop),skirtKeep]);
      var low=HobberZLayout.SEAT_Z+HobberZLayout.TRAVEL_MIN-HobberScales.overtravel();
      var local=machinekit.assembly.PosedParts.moved(deck,materia.assembly.AssemblyFrames.translation(0,0,-low));deck.close();
      var copies:Array<cadkit.modeling.Part>=[];
      for(i in 0...13) {
        var angle=-HobberSwivelLayout.RANGE+2*HobberSwivelLayout.RANGE*i/12,c=Math.cos(angle),s=Math.sin(angle);
        copies.push(machinekit.assembly.PosedParts.moved(local,materia.assembly.AssemblyFrames.fromRotationMatrix(0,0,0,[1.0,0,0,0,c,-s,0,s,c])));
      }
      local.close();return machinekit.component.Solids.union(copies);
    });
    return machinekit.assembly.PosedParts.moved(raw,headInModule());
  }
}
