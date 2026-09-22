package bimkit;

import cadkit.parametric.ElementId;
import cadkit.parametric.Feature;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.LevelBoxFeature;

/** BIM metadata for a straight vertical wall; geometry stays in CadKit. */
class WallRole {
	public final elementId:ElementId;
	public final uncutOutput:Feature;
	public final body:Null<BoxFeature>;
	public final levelBody:Null<LevelBoxFeature>;

	public function new(elementId:ElementId, uncutOutput:Feature) {
		this.elementId = elementId;
		this.uncutOutput = uncutOutput;
		this.body = uncutOutput.serializationType() == "box" ? cast uncutOutput : null;
		this.levelBody = uncutOutput.serializationType() == "level-box" ? cast uncutOutput : null;
		if (body == null && levelBody == null)
			throw new BimError("straight wall requires a box or level-box feature");
	}

	public var length(get, never):Float;

	private function get_length():Float {
		if (body != null) {
			var box:BoxFeature = cast body;
			return box.width.value;
		}
		var level:LevelBoxFeature = cast levelBody;
		return level.width.value;
	}

	public var thickness(get, never):Float;

	private function get_thickness():Float {
		if (body != null) {
			var box:BoxFeature = cast body;
			return box.depth.value;
		}
		var level:LevelBoxFeature = cast levelBody;
		return level.depth.value;
	}

	public var height(get, never):Float;

	private function get_height():Float {
		if (body != null) {
			var box:BoxFeature = cast body;
			return box.height.value;
		}
		var level:LevelBoxFeature = cast levelBody;
		return level.height(uncutOutput.document);
	}

	public function baseElevation():Float {
		if (levelBody == null)
			return 0;
		var level:LevelBoxFeature = cast levelBody;
		return level.baseElevation(uncutOutput.document);
	}
}
