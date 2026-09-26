package robotkit.work;

import robotkit.process.Toolpath;

/** A planned dig cycle's `Toolpath`, plus the sweep to apply to a `HeightMap` once it executes. */
class DigCyclePlan {
  public final toolpath:Toolpath;
  public final sweepFrom:Point2;
  public final sweepTo:Point2;
  public final sweepHalfWidth:Float;
  public final sweepEdgeHeight:Float;

  public function new(toolpath:Toolpath, sweepFrom:Point2, sweepTo:Point2, sweepHalfWidth:Float, sweepEdgeHeight:Float) {
    this.toolpath = toolpath;
    this.sweepFrom = sweepFrom;
    this.sweepTo = sweepTo;
    this.sweepHalfWidth = sweepHalfWidth;
    this.sweepEdgeHeight = sweepEdgeHeight;
  }
}
