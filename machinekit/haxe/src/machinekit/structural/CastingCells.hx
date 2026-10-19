package machinekit.structural;
import materia.assembly.AssemblyFrames as F;
import machinekit.structural.CastingFeatures.CastingBox;
/** Exact disjoint boxes for orthogonal casting recipes. Shared by collision and
    preliminary section stiffness checks; rejects rotated non-orthogonal cores. */
class CastingCells {
  public static function remove(boxes:Array<Array<Float>>,cut:Array<Float>):Array<Array<Float>> {
    var result:Array<Array<Float>> = [];
    for(a in boxes){
      var lo=[for(i in 0...3)Math.max(a[i],cut[i])],hi=[for(i in 0...3)Math.min(a[i+3],cut[i+3])];
      if(lo[0]>=hi[0]||lo[1]>=hi[1]||lo[2]>=hi[2]){result.push(a);continue;}
      var b=a.copy();
      for(axis in 0...3){
        if(b[axis]<lo[axis]){var p=b.copy();p[axis+3]=lo[axis];result.push(p);b[axis]=lo[axis];}
        if(b[axis+3]>hi[axis]){var p=b.copy();p[axis]=hi[axis];result.push(p);b[axis+3]=hi[axis];}
      }
    }
    return result;
  }
  public static function orthogonal(row:CastingBox):Bool {
    for(axis in 0...3){var v=F.transformVector(row.pose,axis==0?1:0,axis==1?1:0,axis==2?1:0);
      if(Math.abs(Math.abs(v.x)+Math.abs(v.y)+Math.abs(v.z)-1)>1e-8)return false;
    }
    return true;
  }
  public static function bounds(row:CastingBox):Array<Float> {
    if(!orthogonal(row))throw "Casting cells require orthogonal core frames";
    var low=[1e30,1e30,1e30],high=[-1e30,-1e30,-1e30];
    for(x in [-row.width/2,row.width/2])for(y in [-row.depth/2,row.depth/2])for(z in [0.0,row.height]){
      var p=F.transformPoint(row.pose,x,y,z),v=[p.x,p.y,p.z];for(i in 0...3){low[i]=Math.min(low[i],v[i]);high[i]=Math.max(high[i],v[i]);}
    }
    return low.concat(high);
  }
  public static function material(width:Float,depth:Float,height:Float,voids:Array<CastingBox>):Array<Array<Float>> {
    var boxes=[[-width/2,-depth/2,0.0,width/2,depth/2,height]];
    for(row in voids)boxes=remove(boxes,bounds(row));return coalesce(boxes);
  }
  /** Remove decomposition seams without changing the exact material union. */
  public static function coalesce(input:Array<Array<Float>>):Array<Array<Float>> {
    var boxes=[for(b in input)b.copy()],changed=true;
    while(changed){changed=false;
      for(i in 0...boxes.length){for(j in i+1...boxes.length){var a=boxes[i],b=boxes[j];
        for(axis in 0...3){var same=true;for(k in 0...3)if(k!=axis&&(a[k]!=b[k]||a[k+3]!=b[k+3]))same=false;
          if(!same||(a[axis+3]!=b[axis]&&b[axis+3]!=a[axis]))continue;
          a[axis]=Math.min(a[axis],b[axis]);a[axis+3]=Math.max(a[axis+3],b[axis+3]);boxes.splice(j,1);changed=true;break;
        }
        if(changed)break;
      }if(changed)break;}
    }
    return boxes;
  }
  /** Reserve full-height backing beneath precision seats, instead of cutting a core through it. */
  public static function outsideColumns(cores:Array<CastingBox>,footprints:Array<Array<Float>>):Array<CastingBox> {
    var result:Array<CastingBox> = [];
    for(core in cores){var cells=[bounds(core)];for(foot in footprints)cells=remove(cells,[foot[0],foot[1],-1e9,foot[2],foot[3],1e9]);
      for(i in 0...cells.length){var b=cells[i];result.push(CastingFeatures.box(core.id+"-backing-"+i,b[3]-b[0],b[4]-b[1],b[5]-b[2],(b[0]+b[3])/2,(b[1]+b[4])/2,b[2]));}
    }
    return result;
  }
  public static function volume(cells:Array<Array<Float>>):Float {var v=0.0;for(b in cells)v+=(b[3]-b[0])*(b[4]-b[1])*(b[5]-b[2]);return v;}
  /** Minimum centroidal I about Y over every constant X-section, in mm^4. */
  public static function minimumIy(cells:Array<Array<Float>>,x0:Float,x1:Float):Float {
    var cuts=[x0,x1];for(b in cells)for(x in [b[0],b[3]])if(x>x0&&x<x1)cuts.push(x);cuts.sort((a,b)->a<b?-1:a>b?1:0);
    var minimum=1e30;
    for(i in 0...cuts.length-1)if(cuts[i+1]-cuts[i]>1e-8){
      var x=(cuts[i]+cuts[i+1])/2,area=0.0,first=0.0,second=0.0;
      for(b in cells)if(x>b[0]&&x<b[3]){var w=b[4]-b[1],h=b[5]-b[2],a=w*h,z=(b[2]+b[5])/2;area+=a;first+=a*z;second+=w*h*h*h/12+a*z*z;}
      if(area<=0)throw "Casting has no continuous beam section";minimum=Math.min(minimum,second-first*first/area);
    }
    return minimum;
  }
}
