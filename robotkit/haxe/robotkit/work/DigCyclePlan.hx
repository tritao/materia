package robotkit.work;

import robotkit.process.Toolpath;

/** A planned dig cycle's `Toolpath`, plus the sweep to apply to a `HeightMap` once it executes. */
class DigCyclePlan {
  public final toolpath:Toolpath;
  public final sweepFrom:Point2;
  public final sweepTo:Point2;
  public final sweepHalfWidth:Float;
  public final sweepEdgeHeight:Float;
  /** Cutting-edge path used by the engaged toolpath, inset from trench walls. */
  public final cuttingEdgeFrom:Point2;
  public final cuttingEdgeTo:Point2;

  public function new(toolpath:Toolpath, sweepFrom:Point2, sweepTo:Point2, sweepHalfWidth:Float,
      sweepEdgeHeight:Float, ?cuttingEdgeFrom:Point2, ?cuttingEdgeTo:Point2) {
    this.toolpath = toolpath;
    this.sweepFrom = sweepFrom;
    this.sweepTo = sweepTo;
    this.sweepHalfWidth = sweepHalfWidth;
    this.sweepEdgeHeight = sweepEdgeHeight;
    this.cuttingEdgeFrom = cuttingEdgeFrom == null ? sweepFrom : cuttingEdgeFrom;
    this.cuttingEdgeTo = cuttingEdgeTo == null ? sweepTo : cuttingEdgeTo;
  }
}
