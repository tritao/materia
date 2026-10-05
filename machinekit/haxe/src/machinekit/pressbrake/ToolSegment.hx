package machinekit.pressbrake;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** One standard-length tooling part. Its CAD frame runs from x=0 to x=length along the bed. */
class ToolSegment extends MachineComponent {
	public final profile:ToolProfile;
	public final length:Float;
	public final horn:Null<ToolHorn>;
	final catalogueProfile:Null<String>;
	static var recipe:Null<ComponentType>;

	public static function standardLengths():Array<Float> return [10, 15, 20, 40, 50, 100, 200, 300];

	public function new(profile:ToolProfile, length:Float, ?horn:ToolHorn) {
		if (profile == null) throw "Tool segment needs a profile";
		var standard = false;
		for (value in standardLengths()) if (length == value) standard = true;
		if (!standard) throw "Tool segment length is outside the standard set";
		if (horn != null) switch profile {
			case Punch(p):
				if (!(horn.reliefLength < length) || !(horn.remainingHeight > p.tipRadius && horn.remainingHeight < p.height))
					throw "Tool horn must retain its nose and leave an unrelieved mounting length";
			case _: throw "End horns are supported on punch segments";
		}
		var profileId = ToolProfiles.identity(profile);
		var hornId = horn == null ? "" : '-${horn.end}-${Dimension.format(horn.reliefLength)}x${Dimension.format(horn.remainingHeight)}';
		super('TOOL-$profileId-${Dimension.format(length)}$hornId', "Reference press-brake tooling segment", "steel", true);
		catalogueProfile = ToolProfiles.catalogueId(profile);
		this.profile = profile;
		this.length = length;
		this.horn = horn;
		var height = ToolProfiles.height(profile);
		var mountX = horn == null ? length / 2 : switch horn.end {
			case Left: (length + horn.reliefLength) / 2;
			case Right: (length - horn.reliefLength) / 2;
		};
		addConnector("mount", Mount, Solids.axial(mountX, 0, ToolProfiles.isPunch(profile) ? height : -height));
		addConnector("contact", Face, Solids.axial(length / 2, 0, 0));
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.pressbrake.tool-segment", [
			ComponentRecipeSupport.choice("profile", ToolProfiles.catalogueIds(), ToolingCatalogue.straight().id),
			ComponentRecipeSupport.length("length", 100),
			ComponentRecipeSupport.choice("horn", ["None", "Left", "Right"], "None"),
			ComponentRecipeSupport.length("reliefLength", 20), ComponentRecipeSupport.length("remainingHeight", 25)
		], values -> {
			var horn = switch values.token("horn") {
				case "None": null;
				case "Left": new ToolHorn(Left, values.number("reliefLength"), values.number("remainingHeight"));
				case "Right": new ToolHorn(Right, values.number("reliefLength"), values.number("remainingHeight"));
				case value: throw 'Unknown tool horn "$value"';
			};
			return new ToolSegment(ToolProfiles.fromCatalogue(values.token("profile")), values.number("length"), horn);
		});
		return recipe;
	}

	override public function componentType():Null<ComponentType> return catalogueProfile == null ? null : recipeType();

	override public function values():ComponentValues {
		if (catalogueProfile == null) throw "Custom tooling section has no catalogue recipe";
		return new ComponentValues().setToken("profile", catalogueProfile).setNumber("length", length)
			.setToken("horn", horn == null ? "None" : Std.string(horn.end))
			.setNumber("reliefLength", horn == null ? 20 : horn.reliefLength)
			.setNumber("remainingHeight", horn == null ? 25 : horn.remainingHeight).setToken("material", materialSpec());
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var part = ToolProfiles.geometry(profile, 0, length);
		if (horn == null) return part;
		var cutter:Null<Part> = null;
		try {
			var section = switch profile {
				case Punch(p): p.upperSection.points();
				case _: throw "Tool horn needs a punch profile";
			};
			var minimum = section[0].y, maximum = section[0].y;
			for (point in section) { minimum = Math.min(minimum, point.y); maximum = Math.max(maximum, point.y); }
			var start = horn.end == Left ? 0 : length - horn.reliefLength;
			var rectangle = new ToolProfilePolygon([new ToolProfilePoint(minimum - 1, horn.remainingHeight),
				new ToolProfilePoint(maximum + 1, horn.remainingHeight),
				new ToolProfilePoint(maximum + 1, ToolProfiles.height(profile) + 1),
				new ToolProfilePoint(minimum - 1, ToolProfiles.height(profile) + 1)]);
			cutter = new ToolProfilePrism(rectangle, start, start + horn.reliefLength).geometry();
			var result = part.subtract(cutter);
			cutter.close(); part.close();
			return result;
		} catch (error:Dynamic) {
			if (cutter != null) cutter.close();
			part.close();
			throw error;
		}
	}

	/** Conservative convex tooling volumes, in this segment's CAD frame. */
	public function collisionPrisms(maximumError:Float):Array<ToolProfilePrism> {
		var section = ToolProfiles.collisionSection(profile, maximumError);
		var result:Array<ToolProfilePrism> = [];
		for (piece in section.convexPieces()) {
			if (horn == null) result.push(new ToolProfilePrism(new ToolProfilePolygon(piece), 0, length));
			else {
				var split = horn.end == Left ? horn.reliefLength : length - horn.reliefLength;
				result.push(new ToolProfilePrism(new ToolProfilePolygon(piece), horn.end == Left ? split : 0,
					horn.end == Left ? length : split));
				var clipped = ToolProfileClip.below(piece, horn.remainingHeight);
				if (clipped.length >= 3) result.push(new ToolProfilePrism(new ToolProfilePolygon(clipped),
					horn.end == Left ? 0 : split, horn.end == Left ? split : length));
			}
		}
		return result;
	}
}
