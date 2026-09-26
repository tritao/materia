package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Timing belt pulley with an explicit belt family and a simplified tooth outline. The
 * groove is a straight-flank preview, not a belt manufacturer's tooth profile. The pitch line
 * lies in the belt cords outside the pulley, so OD = pitch diameter - 2 * PLD.
 */
class TimingPulley extends MachineComponent {
	public final beltProfile:TimingBeltProfile;
	public final pitch:Float;
	public final pitchLineDifferential:Float;
	public final teeth:Int;
	public final boreDiameter:Float;
	public final thickness:Float;
	public final pitchDiameter:Float;
	public final outsideDiameter:Float;
	public final grooveDiameter:Float;

	public function new(beltProfile:TimingBeltProfile, teeth:Int, boreDiameter:Float, thickness:Float) {
		var dimensions = profileDimensions(beltProfile);
		var pitch = dimensions.pitch;
		var pld = dimensions.pld;
		if (!(pitch > 0) || !Math.isFinite(pitch)) throw "Timing pulley needs a positive belt pitch";
		if (teeth < 8) throw "Timing pulley needs at least 8 teeth";
		if (!(boreDiameter > 0)) throw "Timing pulley needs a positive bore diameter";
		if (!(thickness > 0)) throw "Timing pulley needs a positive thickness";

		if (!(pld >= 0) || !(pld < 0.25 * pitch)) throw "Timing pulley pitch line differential must be between 0 and a quarter pitch";
		var pitchDia = pitch * teeth / Math.PI;
		var grooveDepth = 0.2 * pitch;
		var outsideDia = pitchDia - 2 * pld;
		var grooveDia = outsideDia - 2 * grooveDepth;
		if (!(grooveDia > boreDiameter))
			throw "Timing pulley groove diameter must clear the bore; use more teeth or a smaller bore";
		var pitchText = Dimension.format(pitch);
		super('PULLEY-${dimensions.name}-${teeth}T', '${dimensions.name} timing pulley, $pitchText mm pitch, ${teeth} teeth', "aluminium 6061");
		this.beltProfile = beltProfile;
		this.pitch = pitch;
		this.pitchLineDifferential = pld;
		this.teeth = teeth;
		this.boreDiameter = boreDiameter;
		this.thickness = thickness;
		pitchDiameter = pitchDia;
		outsideDiameter = outsideDia;
		grooveDiameter = grooveDia;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, thickness / 2));
		addConnector("back", Face, Solids.axial(0, 0, thickness));
	}

	static function profileDimensions(profile:TimingBeltProfile):{name:String, pitch:Float, pld:Float}
		return switch (profile) {
			case GT2: {name: "GT2", pitch: 2.0, pld: 0.254};
			case HTD3M: {name: "HTD3M", pitch: 3.0, pld: 0.381};
			case HTD5M: {name: "HTD5M", pitch: 5.0, pld: 0.5715};
			case HTD8M: {name: "HTD8M", pitch: 8.0, pld: 0.686};
			case HTD14M: {name: "HTD14M", pitch: 14.0, pld: 1.397};
			case T5: {name: "T5", pitch: 5.0, pld: 0.5};
			case XL: {name: "XL", pitch: 5.08, pld: 0.254};
			case Custom(family, pitch, pld):
				if (family == null || family.length == 0) throw "Custom timing belt needs a family name";
				{name: 'CUSTOM-${family}-P${Dimension.format(pitch)}-PLD${Dimension.format(pld)}', pitch: pitch, pld: pld};
		};

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.prism(profile(), 0, thickness);
		return Solids.cut(body, [Solids.cylinder(boreDiameter / 2, -0.1, thickness + 0.1)]);
	}

	function profile():Array<Vector> {
		var ra = outsideDiameter / 2, rf = grooveDiameter / 2;
		var angleStep = 2 * Math.PI / teeth;
		var halfLand = 0.3 * angleStep / 2, halfGroove = 0.15 * angleStep / 2;
		var points:Array<Vector> = [];
		for (i in 0...teeth) {
			var center = i * angleStep;
			points.push(new Vector(ra * Math.cos(center - halfLand), ra * Math.sin(center - halfLand)));
			points.push(new Vector(rf * Math.cos(center - halfGroove), rf * Math.sin(center - halfGroove)));
			points.push(new Vector(rf * Math.cos(center + halfGroove), rf * Math.sin(center + halfGroove)));
			points.push(new Vector(ra * Math.cos(center + halfLand), ra * Math.sin(center + halfLand)));
		}
		return points;
	}
}
