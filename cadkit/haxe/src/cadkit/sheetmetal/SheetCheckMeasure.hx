package cadkit.sheetmetal;

/** Finding measurements carry their units and the rule they were compared with. */
enum SheetCheckMeasure {
	DistanceMm(actual:Float, minimum:Float);
	AngleRadians(actual:Float, minimum:Float);
	AreaMm2(overlap:Float);
	InteriorPoint(position:Point2);
}
