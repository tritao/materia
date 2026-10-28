package machinekit.tooling;

import cadkit.modeling.Part;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.casting.CastingMoldPlan.MoldVolume;
import materia.tooling.casting.MoldRegions.MoldVolumeRecords;
import materia.tooling.ToolingCodec;

/** Rebuildable exact finished tooling volume, in mold coordinates (mm). */
class ToolingVolume extends MachineComponent {
  static var recipe:Null<ComponentType>;

  final volume:MoldVolume;

  static function parameters():Array<machinekit.component.ComponentParameter> return [
    P.text("stockMaterial", "coated particleboard"),
    P.text(
      "volume",
      ToolingCodec.canonical(MoldVolumeRecords.record({
        id: "former",
        material: materia.tooling.casting.CastingMoldPlan.MoldPartMaterial.FormerStock,
        cuts: [],
        regions: [],
        profiles: [],
        pieces: [{x: 0, y: 0, z: 0, width: 20, depth: 20, height: 20}]
      }))
    )
  ];

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.tooling.volume",
      parameters(), v -> new ToolingVolume(v));
    return recipe;
  }

  public static function fromVolume(v:MoldVolume, ?materialSpec:String):ToolingVolume {
    var material = materialSpec == null ? switch v.material {
      case EPSStock:
        "EPS tooling";
      case SteelStock:
        "steel";
      case FormerStock:
        "coated particleboard";
      case SealantStock:
        throw "Closure material must come from the recipe";
      default:
        throw "Unknown tooling volume material";
    }
    :materialSpec;
    return new ToolingVolume(new ComponentValues().setToken(
      "volume",
      ToolingCodec.canonical(MoldVolumeRecords.record(v))
    ).setToken(
      "stockMaterial",
      material
    ));
  }

  /** Realize finished foam, never the rectangular cutting blank. Placement is
	 * the record's stockToMold frame, supplied by the assembly caller.
	 */
  public static function fromCoreCutting(id:String,
    cutting:materia.tooling.casting.CoreSupport.CoreCutting):ToolingVolume return fromVolume({
      id: id,
      material: EPSStock,
      pieces: [],
      profiles: [],
      cuts: [],
      regions: [materia.casting.CastingRegion.CastingRegionCodec.copy(cutting.solid)]
    });

  public function new(?input:ComponentValues) {
    var v = P.resolve(parameters(), input), record = MoldVolumeRecords.snapshot(haxe.Json.parse(v.token("volume")));
    var material = v.token("stockMaterial");
    super(
      "MOLD-" + record.id + "-" + haxe.crypto.Sha256.encode(ToolingCodec.canonical({
        volume: MoldVolumeRecords.record(record),
        materialSpec: material
      })).substr(
        0,
        12
      ),
      "Tooling " + record.id,
      record.material == SealantStock ? "unspecified" : material,
      false,
      v
    );
    // Seal chemistry is a recipe/shop input, not a material-library key.
    // Keep its full reference in stockMaterial and the identity above. The
    // generic scene material supplies no seal strength or compatibility.

    volume = record;
    addConnector("mount", Mount, F.identity());
  }

  override public function componentType():Null<ComponentType> return recipeType();

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part return geometryOf(volume);

  public static function geometryOf(v:MoldVolume):Part return machinekit
    .structural.CastingRegionCad.part(materia.tooling.casting.MoldRegions.tooling(v));
}
