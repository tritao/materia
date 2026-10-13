package cadkit.sheetmetal;

import cadkit.sheetmetal.FlatDomain.FlatOwner;

/** Exact finding geometry is expressed in the report's exported flat plane. */
enum SheetCheckGeometry {
	Material(owner:FlatOwner, outline:WallOutline);
	BendMark(bend:Int, line:Segment);
	BendEnd(bend:Int, position:Point2);
	Boundary(identity:String, origin:BoundaryOrigin, curve:Segment);
}
