package machinekit.hobbing;

/** Derived stack and swept host interfaces for the hobber Y module. */
class HobberYLayout {
  public static final JOINT="Ymodule/travel";
  static final cached=new machinekit.cache.Once<machinekit.motion.ScrewLinearModuleSpec>();
  public static function moduleSpec():machinekit.motion.ScrewLinearModuleSpec return cached.get(()->{
    var first=new machinekit.motion.ScrewLinearModuleSpec(),face=HobberSaddle.headFaceX();
    var roof=machinekit.motion.ScrewLinearModuleCover.roof(first)-first.carriageBottom()-first.carriageThickness+1.5;
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
      var next=Math.max(required,s.baseWidth/2+1.5+machinekit.motion.ScrewLinearModuleSpec.GAP+HobberSaddle.xCarriageTop()-HobberZLayout.SEAT_Z);
      if(Math.abs(next-lift)<1e-6)return s;lift=next;
    }
    throw "Y stock/cover stack did not converge";
  });
  public static function moduleLift():Float return Math.max(requiredLift(),moduleSpec().baseWidth/2+1.5+machinekit.motion.ScrewLinearModuleSpec.GAP+HobberSaddle.xCarriageTop()-HobberZLayout.SEAT_Z);
  static function requiredLift():Float {
    var s=moduleSpec(),bore=Bt30SpindleData.FLANGE_DIAMETER/2+3,face=HobberSaddle.headFaceX(),bolt=machinekit.standard.SocketHeadCapScrew.catalog().get(s.guide.blockMountThread);
    return Math.ceil((Math.max(s.railSpacing/2+s.guide.blockHoleSpacingAcross/2,Math.abs(s.scaleCarrierY()))+1.5*bolt.diameter+Math.sqrt(Math.max(0.0,bore*bore-face*face)))/5)*5;
  }
  public static function modulePose():materia.assembly.AssemblyRecord.AssemblyFrame {
    var s=moduleSpec(),back=HobberSaddle.headFaceX()+s.carriageThickness+s.carriageBottom();
    return materia.assembly.AssemblyFrames.fromRotationMatrix(back,s.centre,HobberZLayout.SEAT_Z+moduleLift(),[0.0,0,-1,-1,0,0,0,1,0]);
  }
  public static function headInModule():materia.assembly.AssemblyRecord.AssemblyFrame return materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.inverse(modulePose()),materia.assembly.AssemblyFrames.translation(0,0,HobberZLayout.SEAT_Z));
  static final reliefCache=new machinekit.cache.Once<cadkit.modeling.Part>();
  public static function sweptDeckRelief():cadkit.modeling.Part {
    var raw=reliefCache.get(()->{
      var deck=HobberSaddle.xDeckKeepout(machinekit.motion.ScrewLinearModuleSpec.GAP),local=machinekit.assembly.PosedParts.moved(deck,materia.assembly.AssemblyFrames.translation(0,0,-HobberZLayout.SEAT_Z));deck.close();
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
