package cadkit.drawing;

import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;

/** Exact planar curves and upright annotations, in millimetres. */
enum DrawingEntity {
	Curve(layer:DrawingLayer, segment:Segment);
	Label(layer:DrawingLayer, position:Point2, height:Float, value:String);
}
