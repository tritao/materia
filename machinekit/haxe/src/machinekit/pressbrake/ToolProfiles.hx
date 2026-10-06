package machinekit.pressbrake;

import cadkit.modeling.Part;
import cadkit.sheetmetal.WallOutline;

/** Typed operations shared by tooling segments; no runtime class casts are required. */
class ToolProfiles {
	public static function identity(profile:ToolProfile):String return switch profile {
		case Punch(p): p.id;
		case SingleV(p): p.id;
		case FourV(p, face):
			var groove = p.groove(face);
			p.id + '-face-$face';
	};

	public static function catalogueProfiles():Array<ToolProfile> {
		var result:Array<ToolProfile> = [];
		for (punch in ToolingCatalogue.punches()) result.push(Punch(punch));
		for (die in ToolingCatalogue.dies()) result.push(SingleV(die));
		result.push(SingleV(ToolingCatalogue.raisedV12()));
		var block = ToolingCatalogue.fourV();
		for (face in 0...4) result.push(FourV(block, face));
		return result;
	}

	public static function catalogueIds():Array<String> return [for (profile in catalogueProfiles()) identity(profile)];

	public static function fromCatalogue(id:String):ToolProfile {
		for (profile in catalogueProfiles()) if (identity(profile) == id) return profile;
		throw 'Unknown tooling catalogue profile "$id"';
	}

	/** A custom section does not become a catalogue part merely by reusing its identity. */
	public static function catalogueId(profile:ToolProfile):Null<String> {
		var key = specificationKey(profile);
		for (candidate in catalogueProfiles()) if (specificationKey(candidate) == key) return identity(candidate);
		return null;
	}

	public static function sameSpecification(a:ToolProfile, b:ToolProfile):Bool
		return specificationKey(a) == specificationKey(b);

	static function specificationKey(profile:ToolProfile):String return switch profile {
		case Punch(p):
			var key = 'punch|${p.id}|${p.kind}|${p.tipRadius}|${p.includedAngle}|${p.height}';
			for (point in p.upperSection.points()) key += '|${point.y},${point.z}';
			key;
		case SingleV(p): 'die|${p.id}|${p.width}|${p.height}|' + grooveKey(p.groove);
		case FourV(p, face):
			var selected = p.groove(face);
			var key = 'four-v|${p.id}|${p.size}|$face';
			for (i in 0...4) key += '|' + grooveKey(p.groove(i));
			key;
	};

	static function grooveKey(groove:DieGroove):String
		return '${groove.vWidth},${groove.includedAngle},${groove.shoulderRadius}';

	public static function geometry(profile:ToolProfile, start:Float, end:Float):Part return switch profile {
		case Punch(p): p.geometry(start, end);
		case SingleV(p): p.geometry(start, end);
		case FourV(p, face): p.geometry(start, end, face);
	};

	public static function collisionSection(profile:ToolProfile, maximumError:Float):ToolProfilePolygon return switch profile {
		case Punch(p): p.collisionSection(maximumError);
		case SingleV(p): p.collisionSection(maximumError);
		case FourV(p, face): p.collisionSection(maximumError, face);
	};
	public static function contactSection(profile:ToolProfile):WallOutline return switch profile {
		case Punch(p): p.contactSection();
		case SingleV(p): p.contactSection();
		case FourV(p, face): p.contactSection(face);
	};

	/** Height between the contact and mounting datums. */
	public static function height(profile:ToolProfile):Float return switch profile {
		case Punch(p): p.height;
		case SingleV(p): p.height;
		case FourV(p, face):
			var groove = p.groove(face);
			p.size;
	};

	public static function isPunch(profile:ToolProfile):Bool return switch profile {
		case Punch(_): true;
		case _: false;
	};
}
