package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.Dimension;
import machinekit.component.Solids;

/** Round hollow structural tube, extruded along local +Z from z=0 to z=`length`. */
class RoundTube implements StructuralProfile {
	public final designation:String;
	public final description:String;
	public final outerDiameter:Float;
	public final wall:Float;

	public function new(outerDiameter:Float, wall:Float) {
		if (!(outerDiameter > 0) || !(wall > 0)) throw "Round tube needs a positive outer diameter and wall";
		if (!(2 * wall < outerDiameter)) throw "Round tube wall is too thick for its diameter";
		this.outerDiameter = outerDiameter;
		this.wall = wall;
		var od = Dimension.format(outerDiameter), wallText = Dimension.format(wall);
		designation = 'ROUND-${od}x$wallText';
		description = 'Round tube $od OD x $wallText wall';
	}

	public function geometry(length:Float):Part {
		if (!(length > 0)) throw "Round tube needs a positive length";
		return Solids.cut(Solids.cylinder(outerDiameter / 2, 0, length),
			[Solids.cylinder(outerDiameter / 2 - wall, -0.1, length + 0.1)]);
	}

	public function profileDesignation():String return designation;
	public function profileDescription():String return description;
	public function sectionBounds():SectionBounds
		return {minX: -outerDiameter / 2, maxX: outerDiameter / 2, minY: -outerDiameter / 2, maxY: outerDiameter / 2};
}
