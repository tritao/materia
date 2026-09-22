package bimkit;

import cadkit.parametric.ElementId;
import cadkit.parametric.Feature;
import cadkit.parametric.features.BoxFeature;

/** BIM metadata for a straight vertical wall; geometry stays in CadKit. */
class WallRole {
	public final elementId:ElementId;
	public final uncutOutput:Feature;
	public final body:BoxFeature;

	public function new(elementId:ElementId, uncutOutput:Feature) {
		this.elementId = elementId;
		this.uncutOutput = uncutOutput;
		this.body = cast uncutOutput;
	}

	public var length(get, never):Float;

	private function get_length():Float
		return body.width.value;

	public var thickness(get, never):Float;

	private function get_thickness():Float
		return body.depth.value;

	public var height(get, never):Float;

	private function get_height():Float
		return body.height.value;
}
