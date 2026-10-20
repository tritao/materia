package stockkit;

/** One material component on the dexel lattice. Areas/volumes are sampled,
 * just as Stock.volume is; supportArea is material crossing supportPlaneZ.
 */
class StockComponent {
    public final seedRay:Int;
    public final seedZ:Float;
    public final supportArea:Float;
    public final volume:Float;
    public final min:Array<Float>;
    public final max:Array<Float>;
    public final intervalSamples:Int;
    public final supportByRegion:Array<Float>;
    public function new(seedRay:Int,seedZ:Float,supportArea:Float,volume:Float,min:Array<Float>,max:Array<Float>,samples:Int,regions:Array<Float>) {
        this.seedRay=seedRay;this.seedZ=seedZ;this.supportArea=supportArea;this.volume=volume;
        this.min=min;this.max=max;intervalSamples=samples;supportByRegion=regions;
    }
}
private typedef LabelledInterval = {var lo:Float;var hi:Float;var label:Int;}
private class ComponentAccum {
    public var parent:Int;
    public var seedRay:Int;
    public var seedZ:Float;
    public var area:Float=0;
    public var volume:Float=0;
    public var samples:Int=0;
    public final supportSamples:Array<Int>;
    public var min:Array<Float> = [Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY];
    public var max:Array<Float> = [Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY];
    public function new(id:Int,ray:Int,z:Float,regions:Int) {parent=id;seedRay=ray;seedZ=z;supportSamples=[for(i in 0...regions)0];}
    public function absorb(other:ComponentAccum):Void {
        area+=other.area;volume+=other.volume;samples+=other.samples;
        for(i in 0...supportSamples.length)supportSamples[i]+=other.supportSamples[i];
        for(i in 0...3) {min[i]=Math.min(min[i],other.min[i]);max[i]=Math.max(max[i],other.max[i]);}
        if(other.seedRay<seedRay||(other.seedRay==seedRay&&other.seedZ<seedZ)) {seedRay=other.seedRay;seedZ=other.seedZ;}
    }
}

/** Four-neighbour XY connectivity, joining only intervals with positive Z
 * overlap. A thin onion skin connects a part; stacked layers and diagonal-only
 * contact do not. Streams two rows instead of materialising a sheet's rays.
 */
class StockComponents {
    /** Call at a completed cut checkpoint; the stock must not change mid-scan. */
    public static function measure(stock:Stock,supportPlaneZ:Float=0,tolerance:Float=1e-12,regionCount:Int=1,regionAt:Null<Float->Float->Int> = null):Array<StockComponent>
        return fromRows(stock.grid,row->stock.rays(0,row,stock.grid.countU,1),supportPlaneZ,tolerance,regionCount,regionAt);

    public static function fromRows(grid:StockGrid,readRow:Int->Array<Array<StockInterval>>,supportPlaneZ:Float=0,tolerance:Float=1e-12,regionCount:Int=1,regionAt:Null<Float->Float->Int> = null):Array<StockComponent> {
        if(grid.axis!=Z||grid.countU<1||grid.countV<1||!Math.isFinite(grid.spacing)||grid.spacing<=0
            ||!Math.isFinite(grid.originU)||!Math.isFinite(grid.originV)||regionCount<1
            ||!Math.isFinite(supportPlaneZ)||!Math.isFinite(tolerance)||tolerance<0)throw "Component scan needs a valid Z grid, support plane and tolerance";
        var nodes:Array<ComponentAccum> = [],previous:Array<Array<LabelledInterval>> = [for(i in 0...grid.countU)[]];
        function root(label:Int):Int {
            var found=label;while(nodes[found].parent!=found)found=nodes[found].parent;
            while(nodes[label].parent!=label) {var next=nodes[label].parent;nodes[label].parent=found;label=next;}
            return found;
        }
        function join(a:Int,b:Int):Int {
            a=root(a);b=root(b);if(a==b)return a;
            var keep=Std.int(Math.min(a,b)),lose=Std.int(Math.max(a,b));nodes[keep].absorb(nodes[lose]);nodes[lose].parent=keep;return keep;
        }
        var cellArea=grid.spacing*grid.spacing,half=grid.spacing/2;
        for(j in 0...grid.countV) {
            var rays=readRow(j);if(rays.length!=grid.countU)throw "Component row width does not match the grid";
            var current:Array<Array<LabelledInterval>> = [];
            for(i in 0...grid.countU) {
                var ray:Array<LabelledInterval> = [],lastHi=Math.NEGATIVE_INFINITY;
                for(interval in rays[i]) {
                    if(!Math.isFinite(interval.lo)||!Math.isFinite(interval.hi)||interval.hi<=interval.lo||interval.lo<lastHi-tolerance)throw "Component input intervals are invalid or overlapping";
                    lastHi=interval.hi;
                    if(interval.hi-interval.lo<=tolerance)continue;
                    var label=ray.length>0&&Math.abs(ray[ray.length-1].hi-interval.lo)<=tolerance?root(ray[ray.length-1].label):-1;
                    if(i>0)for(other in current[i-1])if(Math.min(interval.hi,other.hi)-Math.max(interval.lo,other.lo)>tolerance)label=label<0?root(other.label):join(label,other.label);
                    for(other in previous[i])if(Math.min(interval.hi,other.hi)-Math.max(interval.lo,other.lo)>tolerance)label=label<0?root(other.label):join(label,other.label);
                    if(label<0) {label=nodes.length;nodes.push(new ComponentAccum(label,j*grid.countU+i,interval.lo,regionCount));}
                    label=root(label);var component=nodes[label];
                    component.volume+=(interval.hi-interval.lo)*cellArea;component.samples++;
                    if(interval.lo<=supportPlaneZ+tolerance&&interval.hi>supportPlaneZ+tolerance) {
                        var region=regionAt==null?0:regionAt(grid.u(i),grid.v(j));
                        if(region<0||region>=regionCount)throw "Support-region index is outside the requested range";
                        component.area+=cellArea;component.supportSamples[region]++;
                    }
                    component.min[0]=Math.min(component.min[0],grid.u(i)-half);component.max[0]=Math.max(component.max[0],grid.u(i)+half);
                    component.min[1]=Math.min(component.min[1],grid.v(j)-half);component.max[1]=Math.max(component.max[1],grid.v(j)+half);
                    component.min[2]=Math.min(component.min[2],interval.lo);component.max[2]=Math.max(component.max[2],interval.hi);
                    ray.push({lo:interval.lo,hi:interval.hi,label:label});
                }
                current.push(ray);
            }
            previous=current;
        }
        var result:Array<StockComponent> = [];
        for(i in 0...nodes.length)if(root(i)==i) {
            var n=nodes[i],areas=[for(samples in n.supportSamples)samples*cellArea];
            var area=0.0;for(value in areas)area+=value;
            result.push(new StockComponent(n.seedRay,n.seedZ,area,n.volume,n.min.copy(),n.max.copy(),n.samples,areas));
        }
        result.sort((a,b)->a.seedRay==b.seedRay?(a.seedZ<b.seedZ?-1:a.seedZ>b.seedZ?1:0):a.seedRay-b.seedRay);
        return result;
    }
}
