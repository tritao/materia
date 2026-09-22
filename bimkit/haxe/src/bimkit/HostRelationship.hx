package bimkit;

import cadkit.parametric.ElementId;
import cadkit.parametric.ElementReference;
import cadkit.parametric.Placement;

/** Authoritative authored coordinates for one opening on one wall baseline. */
class HostRelationship {
	public final openingId:ElementId;
	public final wallId:ElementId;
	public final along:Float;
	public final sill:Float;
	public final outputName:String;
	public final unhostPlacement:Placement;
	public final unhostParent:Null<ElementReference>;
	public final unhostDepth:Null<Float>;

	public function new(openingId:ElementId, wallId:ElementId, along:Float, sill:Float, outputName:String = "opening", ?unhostPlacement:Placement,
			?unhostParent:ElementReference, ?unhostDepth:Float) {
		this.openingId = openingId;
		this.wallId = wallId;
		this.along = along;
		this.sill = sill;
		this.outputName = outputName;
		this.unhostPlacement = unhostPlacement == null ? Placement.identity() : unhostPlacement;
		this.unhostParent = unhostParent;
		this.unhostDepth = unhostDepth;
	}
}
