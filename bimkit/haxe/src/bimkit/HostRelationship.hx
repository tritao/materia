package bimkit;

import cadkit.parametric.ElementId;

/** Authoritative authored coordinates for one opening on one wall baseline. */
class HostRelationship {
	public final openingId:ElementId;
	public final wallId:ElementId;
	public final along:Float;
	public final sill:Float;
	public final outputName:String;

	public function new(openingId:ElementId, wallId:ElementId, along:Float, sill:Float, outputName:String = "opening") {
		this.openingId = openingId;
		this.wallId = wallId;
		this.along = along;
		this.sill = sill;
		this.outputName = outputName;
	}
}
