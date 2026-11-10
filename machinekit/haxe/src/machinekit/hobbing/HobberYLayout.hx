package machinekit.hobbing;

/** Derived stack and swept host interfaces for the hobber Y module. */
class HobberYLayout {
  public static final JOINT="Ymodule/travel";
  // A short plain plate carries the refitted spindle seat and existing outboard arm.
  public static function moduleSpec():machinekit.motion.ScrewLinearModuleSpec return new machinekit.motion.ScrewLinearModuleSpec("HGH15CA",30,140,64,5,140,290);
  public static function moduleLift():Float return 145;
  public static function modulePose():materia.assembly.AssemblyRecord.AssemblyFrame {
    var s=moduleSpec(),back=HobberSaddle.headFaceX()+s.carriageThickness+s.carriageBottom();
    return materia.assembly.AssemblyFrames.fromRotationMatrix(back,s.carriageCentre+60,HobberZLayout.SEAT_Z+moduleLift(),[0.0,0,-1,-1,0,0,0,1,0]);
  }
  public static function headInModule():materia.assembly.AssemblyRecord.AssemblyFrame return materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.inverse(modulePose()),materia.assembly.AssemblyFrames.translation(0,0,HobberZLayout.SEAT_Z));
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
