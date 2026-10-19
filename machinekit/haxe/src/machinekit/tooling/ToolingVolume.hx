package machinekit.tooling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.casting.CastingMoldPlan.MoldVolume;
import materia.tooling.casting.MoldVolumeCodec;

/** Rebuildable exact finished tooling volume, in mold coordinates (mm). */
class ToolingVolume extends MachineComponent {
	static var recipe:Null<ComponentType>;

	final volume:MoldVolume;

	static function parameters():Array<machinekit.component.ComponentParameter>
		return [
			P.text("volume", MoldVolumeCodec.encode({
				id: "former",
				material: materia.tooling.casting.CastingMoldPlan.MoldPartMaterial.FormerStock,
				profiles: [],
				pieces: [
					{
						x: 0,
						y: 0,
						z: 0,
						width: 20,
						depth: 20,
						height: 20
					}
				]
			}))
		];

	public static function recipeType():ComponentType {
		if (recipe == null)
			recipe = new ComponentType("machinekit.tooling.volume", parameters(), v -> new ToolingVolume(v));
		return recipe;
	}

	public static function fromVolume(v:MoldVolume):ToolingVolume
		return new ToolingVolume(new ComponentValues().setToken("volume", MoldVolumeCodec.encode(v)));

	public function new(?input:ComponentValues) {
		var v = P.resolve(parameters(), input),
			record = MoldVolumeCodec.decode(v.token("volume"));
		var material = switch (record.material : String) {
			case "eps": "EPS tooling";
			case "steel": "steel";
			default: "coated particleboard";
		};
		super("MOLD-"
			+ record.id
			+ "-"
			+ haxe.crypto.Sha256.encode(MoldVolumeCodec.encode(record)).substr(0, 12), "Tooling "
			+ record.id, material, false, v);
		volume = record;
		addConnector("mount", Mount, F.identity());
	}

	override public function componentType():Null<ComponentType>
		return recipeType();

	override public function hasGeometry():Bool
		return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return geometryOf(volume);

	public static function geometryOf(v:MoldVolume):Part {
		var pieces:Array<Part> = [];
		return Solids.building(pieces, parts -> {
			for (b in v.pieces) {
				parts.push(ToolingBlock.placedBox(b));
			}
			for (profile in v.profiles) {
				var temporary:Array<Part> = [];
				var solid = Solids.building(temporary, owned -> {
					var local = PanelCad.contourPrism(profile.contour, 0, profile.depth);
					owned.push(local);
					var world = machinekit.assembly.PosedParts.moved(local, profile.pose);
					owned.push(world);
					local.close();
					var cuts:Array<Part> = [];
					for (b in profile.subtract) {
						var cut = ToolingBlock.placedBox(b);
						owned.push(cut);
						cuts.push(cut);
					}
					return Solids.cut(world, cuts);
				});
				parts.push(solid);
			}
			return Solids.union(parts);
		});
	}
}
