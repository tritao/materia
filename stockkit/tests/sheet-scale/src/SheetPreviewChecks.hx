import stockkit.*;
import stockkit.StockPreviewWorker.StockPreviewUpdate;
import toolpathkit.tool.Tool;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;

class SheetPreviewChecks {
  static var assertions=0;
  static function check(ok:Bool,label:String):Void { assertions++; if(!ok) throw label; }
  static function same(update:StockPreviewUpdate,preview:StockPreview):Bool {
    for(i in 0...update.chunks.length) {
      var a=update.meshes[i], b=preview.meshes[update.chunks[i]];
      if(b==null || a.positions.compare(b.positions)!=0 || a.normals.compare(b.normals)!=0 ||
        a.indices.compare(b.indices)!=0 || a.colors.compare(b.colors)!=0 || a.triangleSources.compare(b.triangleSources)!=0) return false;
    }
    return true;
  }
  static function main():Void {
    var lattice=StockLattice.covering(0,0,0,0.08,0.096,0.018,0.0005,true);
    var stock=Stock.box(lattice,0,0,0,0.08,0.096,0.018);
    check(stock.bytes()<200000,"uniform tiles are active in the loaded native core");
    var coloring:StockColoring=BySource(_->0xFFFF00FF,0x777777FF);
    var preview=new StockPreview(stock,coloring,2), worker=new StockPreviewWorker(stock,coloring,2);
    preview.update(); check(worker.refresh(),"initial worker refresh"); worker.wait();
    var updates=worker.take();
    check(updates.length==1 && updates[0].chunks.length==preview.chunksX*preview.chunksY && same(updates[0],preview),"worker matches all synchronous contour streams");
    var tool=new Tool(3,0.06,0.006,null,0.04,0.06);
    var saved=stock.snapshot();
    stock.cut([new CutMove(tool,Path(Line(new Point3(0.01,0.01,0.002),new Point3(0.025,0.01,0.002))),Cut,0,new Provenance(1,1,1))]);
    var changed=preview.update(); check(changed.length>0 && changed.length<preview.chunksX*preview.chunksY,"local cut rebuilds only nearby tiles");
    check(worker.refresh(),"changed worker refresh"); worker.wait(); updates=worker.take();
    check(updates.length==1 && updates[0].chunks.length<preview.chunksX*preview.chunksY && [for(chunk in changed) updates[0].chunks.indexOf(chunk)>=0].indexOf(false)<0 && same(updates[0],preview),"snapshot worker preserves changed-chunk geometry and provenance");
    stock.restore(saved); changed=preview.update();
    check(worker.refresh(),"restored worker refresh"); worker.wait(); updates=worker.take();
    check(updates.length==1 && updates[0].chunks.join(",")==changed.join(",") && same(updates[0],preview),"restoring compact tiles invalidates worker contours");
    worker.dispose(); saved.dispose(); stock.dispose();
    Sys.println('Sheet preview checks passed ($assertions assertions)');
  }
}
