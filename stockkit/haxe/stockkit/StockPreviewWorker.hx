package stockkit;

import sys.thread.Condition;
import sys.thread.Thread;

/** The chunks one refresh rebuilt: chunk `chunks[k]` now has mesh `meshes[k]`. */
class StockPreviewUpdate {
  public final chunks:Array<Int>;
  public final meshes:Array<StockMesh>;
  public final chunksX:Int;
  public final chunksY:Int;

  public function new(chunks:Array<Int>, meshes:Array<StockMesh>, chunksX:Int, chunksY:Int) {
    this.chunks = chunks;
    this.meshes = meshes;
    this.chunksX = chunksX;
    this.chunksY = chunksY;
  }
}

/**
  A `StockPreview` contoured on a worker thread, so cutting never waits for
  meshing. `refresh` hands the worker a snapshot of `source`, which costs a
  pointer per tile since tiles are shared until the next cut changes them;
  the worker restores it into a stock of its own, contours the chunks that
  changed and posts their meshes, which `take` returns on the caller's
  thread. Only the caller's thread touches `source`.
**/
class StockPreviewWorker {
  public final source:Stock;
  final coloring:StockColoring;
  final chunkTiles:Int;
  final condition = new Condition();
  var pending:Null<StockSnapshot> = null;
  var finished:Array<StockPreviewUpdate> = [];
  var busy = false;
  var stopping = false;
  var stopped = false;
  var failure:Null<String> = null;

  public function new(source:Stock, coloring:StockColoring, chunkTiles:Int = 4) {
    this.source = source;
    this.coloring = coloring;
    this.chunkTiles = chunkTiles;
    var self = this;
    Thread.create(function() self.work());
  }

  /** Starts a refresh of what `source` is now, unless one is under way; returns whether it started. */
  public function refresh():Bool {
    condition.acquire();
    if (busy || stopping) {
      condition.release();
      return false;
    }
    busy = true;
    condition.release();
    var snapshot:Null<StockSnapshot> = null;
    try {
      snapshot = source.snapshot();
    } catch (error:Dynamic) {
      condition.acquire();
      busy = false;
      condition.release();
      throw error;
    }
    condition.acquire();
    pending = snapshot;
    condition.broadcast();
    condition.release();
    return true;
  }

  /** The refreshes finished since the last call, oldest first; throws the worker's failure. */
  public function take():Array<StockPreviewUpdate> {
    condition.acquire();
    var updates = finished;
    finished = [];
    var problem = failure;
    condition.release();
    if (problem != null) throw 'stock preview failed: $problem';
    return updates;
  }

  /** Waits for the refresh under way, if any. */
  public function wait():Void {
    condition.acquire();
    while (busy && !stopped) condition.wait();
    condition.release();
  }

  /** Stops the worker and releases its stock. Returns once it has stopped. */
  public function dispose():Void {
    condition.acquire();
    stopping = true;
    condition.broadcast();
    while (!stopped) condition.wait();
    condition.release();
  }

  function work():Void {
    var display:Null<Stock> = null;
    var preview:Null<StockPreview> = null;
    while (true) {
      condition.acquire();
      while (pending == null && !stopping) condition.wait();
      var snapshot = pending;
      pending = null;
      var stop = stopping;
      condition.release();
      if (stop || snapshot == null) {
        if (snapshot != null) snapshot.dispose();
        break;
      }
      var taken:StockSnapshot = cast snapshot;
      var update:Null<StockPreviewUpdate> = null;
      var problem:Null<String> = null;
      try {
        var shown = display;
        if (shown == null) {
          // Any stock on the lattice will do: the snapshot replaces all of it.
          var lattice = taken.lattice;
          shown = Stock.box(lattice, lattice.x(0), lattice.y(0), lattice.z(0),
            lattice.x(1), lattice.y(1), lattice.z(1));
          display = shown;
        }
        shown.restore(taken);
        var contoured = preview;
        if (contoured == null) {
          contoured = new StockPreview(shown, coloring, chunkTiles);
          preview = contoured;
        }
        var chunks = contoured.update();
        update = new StockPreviewUpdate(chunks, [for (chunk in chunks) contoured.meshes[chunk]],
          contoured.chunksX, contoured.chunksY);
      } catch (error:Dynamic) {
        problem = Std.string(error);
      }
      taken.dispose();
      condition.acquire();
      if (update != null) finished.push(update);
      if (problem != null) failure = problem;
      busy = false;
      condition.broadcast();
      condition.release();
    }
    if (display != null) display.dispose();
    condition.acquire();
    stopped = true;
    condition.broadcast();
    condition.release();
  }
}
