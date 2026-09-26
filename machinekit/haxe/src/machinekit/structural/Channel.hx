package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Dimension;
import machinekit.component.Solids;

/** C-section channel, extruded along local +Z from z=0 to z=`length`. The web sits at local
 * x=0..thickness spanning the full `height`; the flanges open toward +X.
 */
class Channel implements StructuralProfile {
	public final designation:String;
	public final description:String;
	public final height:Float;
	public final flangeWidth:Float;
	public final thickness:Float;

	public function new(height:Float, flangeWidth:Float, thickness:Float) {
		if (!(height > 0) || !(flangeWidth > 0) || !(thickness > 0))
			throw "Channel needs a positive height, flange width, and thickness";
		if (!(2 * thickness < height)) throw "Channel thickness is too thick for its height";
		if (!(thickness < flangeWidth)) throw "Channel thickness must be less than the flange width";
		this.height = height;
		this.flangeWidth = flangeWidth;
		this.thickness = thickness;
		var size = '${Dimension.format(height)}x${Dimension.format(flangeWidth)}x${Dimension.format(thickness)}';
		designation = 'CHANNEL-$size';
		description = 'Channel $size';
	}

	public function geometry(length:Float):Part {
		if (!(length > 0)) throw "Channel needs a positive length";
		var w = flangeWidth, h = height, t = thickness;
		return Solids.prism([
			new Vector(0, 0), new Vector(w, 0), new Vector(w, t), new Vector(t, t),
			new Vector(t, h - t), new Vector(w, h - t), new Vector(w, h), new Vector(0, h),
		], 0, length);
	}

	public function profileDesignation():String return designation;
	public function profileDescription():String return description;
	public function sectionBounds():SectionBounds
		return {minX: 0, maxX: flangeWidth, minY: 0, maxY: height};
}
