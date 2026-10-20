import stockkit.*;
import stockkit.StockComponents.StockComponent;
import toolpathkit.tool.Tool;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;

class StockComponentsChecks {
    static var assertions=0;
    static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw 'Stock components: $label';}
    static function interval(lo:Float,hi:Float):StockInterval return new StockInterval(lo,hi,-1,-1,[0,0,-1],[0,0,1]);
    static function scan(rows:Array<Array<Array<StockInterval>>>):Array<StockComponent> {
        var grid=new StockGrid(Z,0.5,0.5,1,rows[0].length,rows.length);
        return StockComponents.fromRows(grid,j->rows[j]);
    }
    static function synthetic():Void {
        var full=[interval(0,1)],empty:Array<StockInterval> = [];
        var diagonal=scan([[full,empty],[empty,full]]);
        check(diagonal.length==2,"diagonal point contact is disconnected");
        check(diagonal[0].seedRay==0&&diagonal[1].seedRay==3,"deterministic component order");
        check(scan([[full,[interval(0,0.0005)],full]]).length==1,"thin onion skin retains connectivity");
        check(scan([[full,empty,full]]).length==2,"through kerf separates the piece");
        var layers=scan([[[interval(0,1),interval(2,3)]]]);
        check(layers.length==2&&layers[0].supportArea==1&&layers[1].supportArea==0,"floating layers receive no support footprint");
        var bridge=scan([[[interval(0,1),interval(2,3)],[interval(0,3)]]]);
        check(bridge.length==1&&bridge[0].volume==5&&bridge[0].supportArea==2,"neighbouring full-height material joins both layers");
        check(scan([[[interval(0,1),interval(1,2)]]]).length==1,"vertically shared material faces connect");
        var grid=new StockGrid(Z,0.5,0.5,1,3,1);
        var regional=StockComponents.fromRows(grid,j->[full,full,full],0,1e-12,2,(x,y)->x<1.5?0:1);
        check(regional.length==1&&regional[0].supportByRegion[0]==1&&regional[0].supportByRegion[1]==2,"support area partitions without counting empty footprint");
        var invalid=false;try {scan([[[interval(0,2),interval(1,3)]]]);}catch(error:Dynamic) {invalid=true;}
        check(invalid,"overlapping intervals are rejected");
    }
    public static function contour(stock:Stock,bottom:Float):Void {
        var tool=new Tool(3,0.06,0.006,null,0.04,0.06);
        var xy=[[0.047,0.047],[0.103,0.047],[0.103,0.103],[0.047,0.103],[0.047,0.047]];
        var moves:Array<CutMove> = [];
        for(i in 0...4)moves.push(new CutMove(tool,Path(Line(new Point3(xy[i][0],xy[i][1],bottom),new Point3(xy[i+1][0],xy[i+1][1],bottom))),Cut,i,new Provenance(40+i,1,1)));
        stock.cut(moves);
    }
    static function native():Void {
        var lattice=StockLattice.covering(0,0,0,0.4,0.4,0.018,0.001,false);
        var stock=Stock.box(lattice,0,0,0,0.4,0.4,0.018);stock.setThreads(1);
        var snapshot=stock.snapshot();
        try {
            check(StockComponents.measure(stock).length==1,"native initial sheet is connected");
            contour(stock,-0.001);var parts=StockComponents.measure(stock);
            check(parts.length==2,"native 50 mm through-cut part separates");
            parts.sort((a,b)->a.supportArea<b.supportArea?-1:1);
            check(Math.abs(parts[0].supportArea-0.0025)<1e-9,"native part has a 50 x 50 mm support footprint");
            var volume=0.0;for(part in parts)volume+=part.volume;
            check(Math.abs(volume-stock.volume())<1e-11,"component volumes conserve the native stock volume");
            var repeat=StockComponents.measure(stock);
            check(repeat[0].seedRay<repeat[1].seedRay,"repeat scan has stable seed order");
            stock.restore(snapshot);check(StockComponents.measure(stock).length==1,"restored snapshot restores connectivity");
            contour(stock,0.0005);check(StockComponents.measure(stock).length==1,"native 0.5 mm onion skin retains the part");
            stock.dispose();snapshot.dispose();
        }catch(error:Dynamic) {stock.dispose();snapshot.dispose();throw error;}
    }
    public static function main():Void {
        synthetic();if(Sys.args().indexOf("--synthetic-only")<0)native();
        Sys.println('Stock components: $assertions assertions passed');
    }
}
