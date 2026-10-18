package stockkit.hobbing;
import gearkit.GearRings;
import stockkit.RingGrid;
import stockkit.RingStock;

/** Adds physically cut Z=0 and Z=faceWidth circles to an existing inspection.
    Interior coordinates are unchanged. These planes are sampling domains, not
    extra material or an extrapolation of unseen face ends. */
class FaceEndpointStock implements GearRings {
  public final stocks:Array<RingStock>;
  final base:GearRings;
  final endpointRadii:Array<Int> = [];
  final planes:Array<RingStock> = [];
  final nodes:Array<{z:Float,base:Int,plane:Int}> = [];
  public function new(base:GearRings,baseStocks:Array<RingStock>,grid:RingGrid,faceWidth:Float) {
    if(!Math.isFinite(faceWidth)||faceWidth<=0)throw "invalid inspected face width";
    this.base=base;stocks=baseStocks.copy();
    var zs=base.axialPositions(),rs=base.radii();
    // Endpoints are used by lead metrology only. An exact inspection cross
    // already has the required radial strip; retain its original coordinates.
    if(Std.isOfType(base,InspectionRingStock)){var cross:InspectionRingStock=cast base;grid=cross.stocks[1].grid;}
    for(r in 0...grid.radialCount){var found=rs.indexOf(grid.radius(r));if(found<0)throw "endpoint radius absent from base inspection";endpointRadii.push(found);}
    for(i in 0...zs.length)nodes.push({z:zs[i],base:i,plane:-1});
    try{
      for(z in [0.0,faceWidth]){
        var present=false;for(value in zs)if(value==z)present=true;if(present)continue;
        var half=grid.axialStep/2;
        var planeGrid=new RingGrid(grid.radiusMin,grid.radiusMax,z-half,z+half,grid.radialStep,2*half);
        if(planeGrid.axialCount!=1||planeGrid.z(0)!=z||planeGrid.radialCount!=endpointRadii.length)
          throw "face-end sampling grid must retain exact endpoint and radial coordinates";
        for(r in 0...endpointRadii.length)if(planeGrid.radius(r)!=rs[endpointRadii[r]])throw "face-end radial coordinates differ from inspection";
        var stock=RingStock.annulus(planeGrid);nodes.push({z:z,base:-1,plane:planes.length});planes.push(stock);stocks.push(stock);
      }
      nodes.sort((a,b)->a.z<b.z?-1:a.z>b.z?1:0);
    }catch(e:Dynamic){for(stock in planes)stock.dispose();throw e;}
  }
  public function radii():Array<Float> return base.radii();
  public function axialPositions():Array<Float> return [for(node in nodes)node.z];
  public function boundaries(r:Int,z:Int):Array<Float> {
    if(z<0||z>=nodes.length)throw "face-end axial index outside inspection";
    var node=nodes[z];if(node.plane<0)return base.boundaries(r,node.base);
    var index=endpointRadii.indexOf(r);if(index<0)throw "uncomputed face-end inspection radius";
    return planes[node.plane].boundaries(index,0);
  }
}
