package stockkit;

import sys.thread.Condition;
import sys.thread.Thread;

/** Single-flight, bounded snapshot preview. Worker owns its contour stock and only transfers buffers. */
class RingPreviewWorker {
  final source:RingStock;
  final condition=new Condition();
  var pending:Null<RingSnapshot> = null;
  var result:Null<StockMesh> = null;
  var failure:Null<String> = null;
  var busy=false;var stopping=false;var stopped=false;
  public function new(source:RingStock){this.source=source;Thread.create(work);}
  public function refresh():Bool {
    condition.acquire();if(busy||stopping){condition.release();return false;}busy=true;condition.release();
    var snapshot:RingSnapshot;
    try{snapshot=source.snapshot();}catch(e:Dynamic){condition.acquire();busy=false;condition.release();throw e;}
    condition.acquire();pending=snapshot;condition.broadcast();condition.release();return true;
  }
  public function take():Null<StockMesh>{condition.acquire();var value=result,problem=failure;result=null;condition.release();if(problem!=null)throw problem;return value;}
  public function wait():Void{condition.acquire();while(busy&&!stopped)condition.wait();condition.release();}
  public function dispose():Void{condition.acquire();stopping=true;condition.broadcast();while(!stopped)condition.wait();condition.release();}
  function work():Void {
    var display:Null<RingStock> = null;
    try{
      display=RingStock.annulus(source.grid);
      while(true){condition.acquire();while(pending==null&&!stopping)condition.wait();var snapshot=pending;pending=null;var stop=stopping;condition.release();
        if(snapshot==null){if(stop)break;continue;}
        var mesh:StockMesh;try{display.restore(snapshot);mesh=display.contour();}catch(e:Dynamic){snapshot.dispose();throw e;}snapshot.dispose();
        condition.acquire();result=mesh;busy=false;condition.broadcast();condition.release();
        if(stop)break;
      }
    }catch(e:Dynamic){condition.acquire();failure=Std.string(e);condition.release();}
    if(display!=null)display.dispose();condition.acquire();if(pending!=null)pending.dispose();pending=null;busy=false;stopped=true;condition.broadcast();condition.release();
  }
}
