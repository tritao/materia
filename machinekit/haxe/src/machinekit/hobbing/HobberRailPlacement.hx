package machinekit.hobbing;

import machinekit.motion.LinearRail;
import machinekit.assembly.AxisBuilder;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

typedef HobberPlacedRail = {
 var memberId:String;
 var insertId:String;
 var rail:LinearRail;
 var pose:AssemblyFrame;
 var contact:AssemblyFrame;
}

/** One placement declaration feeds both the machine and its source seat holes.
 * Rail origins follow the actual catalogue height; hole positions remain the
 * bought rail's mount connectors, never a second pitch calculation. */
class HobberRailPlacement {
 public static function placements(kind:String,rows:Array<CastInsertPlacement>):Array<HobberPlacedRail> {
  var rail=switch kind {
   case "bed":LinearRail.metric("HGH25CA",HobberXLayout.front().railLength,LinearRail.MEASURED);
   case "column":LinearRail.metric("HGH15CA",480);
   default:throw "Unknown hobber foundation rail family: "+kind;
  };
  var frame=FoundationLayout.worldPose(kind),alongSign=kind=="bed" ? 1.0 : -1.0;
  return [for(side in [-1,1]) {
   var suffix=side<0 ? "Left" : "Right",id="rail"+suffix;
   var matches=rows.filter(row->row.id==id);
   if(matches.length!=1)throw "Bought rail needs one declared source seat: "+kind+"/"+id;
   var row=matches[0],size=row.part.boxDimensions(id);
   var face=F.compose(F.compose(frame,row.pose),F.translation(0,0,size.height));
   var start=F.transformPoint(face,0,-alongSign*rail.length/2,0);
   var normal=F.transformVector(face,0,0,1),along=F.transformVector(face,0,alongSign,0);
   {memberId:(kind=="bed" ? "railX" : "railZ")+suffix,insertId:id,rail:rail,
    pose:AxisBuilder.orient(start.x+normal.x*rail.spec.railHeight,start.y+normal.y*rail.spec.railHeight,
     start.z+normal.z*rail.spec.railHeight,[normal.x,normal.y,normal.z],[along.x,along.y,along.z]),
    contact:F.alongY(start.x,start.y,start.z,normal.x,normal.y,normal.z)};
  }];
 }

 public static function fingerprint(placed:HobberPlacedRail):String {
  var rail=placed.rail;
  return haxe.crypto.Sha256.encode(materia.tooling.ToolingCodec.canonical({
   definition:machinekit.assembly.MachineAssembly.definitionKey(placed.memberId,rail),
   specification:rail.spec,length:rail.length,mountingHole:rail.mountingHole(),connectors:rail.mountingConnectors()}));
 }
}
