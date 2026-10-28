package machinekit.pneumatic;

import cadkit.InertiaTensor;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;

/** A maker's suction cup: published dimensions, mass, holding force at 60 kPa and vacuum thread. */
typedef SuctionCupRow = {
  var maker:String;
  var designation:String;
  var label:String;
  var nominalDiameterMm:Float;
  var envelopeDiameterMm:Float;
  var heightMm:Float;
  var massKg:Float;
  var theoreticalForceAt60KpaN:Float;
  var vacuumThread:String;
  var sourceUrl:String;
}

/**
 * A suction cup with a vacuum input and contact frame: a catalogue product (`model`, its maker's row: dimensions,
 * published mass, sealed area inferred from the maker's theoretical force at 60 kPa, not a measured guarantee) or
 * a custom cup from its dimensions.
 */
class SuctionCup extends MachineComponent {
  public static inline var CUSTOM = "custom";
  static var table:Null<Catalog<SuctionCupRow>>;

  public static function catalog():Catalog<SuctionCupRow> {
    if (table == null) table = new Catalog(
      "suction cup",
      row -> row.designation,
      [
        {
          maker: "Schmalz",
          designation: "10.01.01.11400",
          label: "SAF 30 NBR-45 G1/4-IG",
          nominalDiameterMm: 30,
          envelopeDiameterMm: 34,
          heightMm: 20,
          massKg: 0.01188,
          theoreticalForceAt60KpaN: 38,
          vacuumThread: "G1/4-F",
          sourceUrl: "https://www.schmalz.ru/en-ru/products/automation-743270/vacuum-grippers-746238/vacuum-suction-cups-301609/flat-suction-cups-round-301610/flat-suction-cups-saf-303827/10.01.01.11400"
        },
        {
          maker: "Schmalz",
          designation: "10.01.01.11401",
          label: "SAF 40 NBR-45 G1/4-IG",
          nominalDiameterMm: 40,
          envelopeDiameterMm: 46,
          heightMm: 22,
          massKg: 0.0136,
          theoreticalForceAt60KpaN: 69,
          vacuumThread: 'G1/4-F',
          sourceUrl: "https://www.schmalz.co.jp/en-jp/products/vacuum-technology-for-automation-301607/vacuum-components-301608/vacuum-suction-cups-301609/flat-suction-cups-round-301610/flat-suction-cups-saf-302308/10.01.01.11401"
        },
        {
          maker: "Schmalz",
          designation: "10.01.01.11441",
          label: "SAF 50 NBR-45 G1/4-IG",
          nominalDiameterMm: 50,
          envelopeDiameterMm: 56,
          heightMm: 33,
          massKg: 0.0308,
          theoreticalForceAt60KpaN: 100,
          vacuumThread: "G1/4-F",
          sourceUrl: "https://www.schmalz.com/en-it/products/automation-743270/vacuum-grippers-746238/vacuum-suction-cups-301609/flat-suction-cups-round-301610/flat-suction-cups-saf-303827/10.01.01.11441"
        }
      ],
      row -> ({
        source: row.sourceUrl,
        standard: null,
        standardEdition: null,
        dimensionKind: Mixed,
        conformance: NominalEnvelope,
        verifiedFields: [
          "nominalDiameterMm",
          "envelopeDiameterMm",
          "heightMm",
          "massKg",
          "theoreticalForceAt60KpaN",
          "vacuumThread"
        ]
      })
    );
    return table;
  }

  /** The catalogue cup `designation`. */
  public static function model(designation:String):SuctionCup {
    var row = catalog().get(designation);
    return new SuctionCup(
      row.nominalDiameterMm,
      row.heightMm,
      row.theoreticalForceAt60KpaN / 60000 * 1e6,
      null,
      Thread(row.vacuumThread),
      row
    );
  }

  /** The catalogue row it is, or null for a custom cup. */
  public final row:Null<SuctionCupRow>;
  public final diameter:Float;
  public final height:Float;
  /** Measured or vendor-rated sealed area in mm²; null until specified. */
  public final effectiveAreaMm2:Null<Float>;
  /** Rated overturning moment at the chosen vacuum, in N m; null if unknown. */
  public final ratedMomentNm:Null<Float>;

  public function new(
    diameter:Float,
    height:Float,
    ?effectiveAreaMm2:Float,
    ?ratedMomentNm:Float,
    ?vacuumInterface:PortInterface,
    ?row:SuctionCupRow
  ) {
    if (!Math.isFinite(diameter)
      || diameter <= 0 || !Math.isFinite(height) || height <= 0) throw "Suction cup needs positive dimensions";
    var nominalArea = Math.PI * diameter * diameter / 4;
    if (effectiveAreaMm2 != null && (!Math.isFinite(effectiveAreaMm2) || effectiveAreaMm2 <= 0 || effectiveAreaMm2
      > nominalArea + 1e-9)) throw "Suction cup effective area must fit within its nominal diameter";
    if (ratedMomentNm != null && (!Math.isFinite(ratedMomentNm)
      || ratedMomentNm <= 0)) throw "Suction cup moment rating must be positive and finite";
    var ratingId = effectiveAreaMm2 == null ? "" : '-A${Dimension.format(effectiveAreaMm2)}';
    if (ratedMomentNm != null) ratingId += '-M${Dimension.format(ratedMomentNm)}';
    var interfaceId = switch vacuumInterface {
      case null:
        "";
      case PushIn(size):
        size == 6 ? "" : '-PI${Dimension.format(size)}';
      case Thread(name):
        '-TH$name';
      case Plug(name, pins):
        '-PL$name-$pins';
      case Coupling(key, channel):
        '-CO$key-$channel';
      case Unspecified:
        "-UNSPECIFIED";
    };
    super(
      row == null
      ? 'SUCTION-CUP-${Dimension.format(diameter)}-${Dimension.format(height)}$ratingId$interfaceId' : row.designation,
      row == null ? "Generic suction cup" : '${row.maker} ${row.label} suction cup',
      "rubber",
      row == null
    );
    this.row = row;
    this.diameter = diameter;
    this.height = height;
    this.effectiveAreaMm2 = effectiveAreaMm2;
    this.ratedMomentNm = ratedMomentNm;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("contact", Face, Solids.axial(0, 0, height));
    addPort({
      name: "vacuum",
      kind: Vacuum,
      role: Consumer,
      iface: vacuumInterface == null ? PushIn(6) : vacuumInterface,
      required : true
    });
    addFacet(new SuctionFacet(effectiveAreaMm2, ratedMomentNm, "vacuum", "contact"));
    if (row != null) {
      setMaterial("nitrile rubber NBR");
      // The published mass, its centre and inertia from a solid cylinder of the envelope.
      var radius = row.envelopeDiameterMm / 2;
      var transverse = row.massKg * (3 * radius * radius + row.heightMm * row.heightMm) / 12;
      declareMass(
        row.massKg,
        new Vector(0, 0, row.heightMm / 2),
        new InertiaTensor(
          transverse,
          0,
          0,
          transverse,
          0,
          row.massKg * radius * radius / 2
        )
      );
    }
  }

  /** The cup's outside diameter: a catalogue cup's lip may exceed its nominal diameter. */
  function outside():Float return row == null ? diameter : row.envelopeDiameterMm;

  static var recipeTypeCache:Null<ComponentType>;

  public static function recipeType():ComponentType {
    if (recipeTypeCache == null) recipeTypeCache = new ComponentType("machinekit.pneumatic.suction-cup", [
      ComponentRecipeSupport.choice(
        "model",
        [CUSTOM].concat(catalog().designations()),
        CUSTOM
      ),
      ComponentRecipeSupport.length(
        "diameter",
        40
      ),
      ComponentRecipeSupport.length(
        "height",
        18
      ),
      ComponentRecipeSupport.optionalScalar("effectiveAreaMm2"),
      ComponentRecipeSupport.optionalScalar("ratedMomentNm")
    ].concat(ComponentRecipeSupport.interfaceParameters("vacuumInterface")),
    // A catalogue model's row sets every other input.
    v -> v.token("model") != CUSTOM ? model(v.token("model")) : new SuctionCup(
      v.number("diameter"),
      v.number("height"),
      v.optionalNumber("effectiveAreaMm2"),
      v.optionalNumber("ratedMomentNm"),
      ComponentRecipeSupport.interfaceFrom(
        v,
        "vacuumInterface"
      )
    ), true);
    return recipeTypeCache;
  }

  /** Subclasses must declare their own recipe and saved values. */
  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    SuctionCup) ? recipeType() : null;

  override public function values():machinekit.component.ComponentValues {
    var values = new machinekit.component.ComponentValues().setNumber("diameter",
      diameter).setNumber("height", height).set(
        "effectiveAreaMm2",
        effectiveAreaMm2
      == null ? machinekit.component.ComponentValue.Unset : machinekit.component.ComponentValue.Number(effectiveAreaMm2)
    ).set(
      "ratedMomentNm",
      ratedMomentNm
      == null ? machinekit.component.ComponentValue.Unset : machinekit.component.ComponentValue.Number(ratedMomentNm)
    ).setToken(
      "model",
      row == null ? CUSTOM : row.designation
    );
    ComponentRecipeSupport.interfaceValues(values, "vacuumInterface", port("vacuum").iface);
    return values.setToken("material", materialSpec());
  }

  override public function hasGeometry():Bool return true;

  /** Stem at the mount, flaring to the sealing lip at the contact face (`PneumaticBodies.suctionCup`). The envelope box is also the mass model. */
  override public function geometry(detail:ComponentDetail = Preview):Part return detail
    == Envelope ? Part.cylinderSpan(
      outside() / 2,
      0,
      height
    ) : PneumaticBodies.suctionCup(
      outside(),
      height
    );

  override public function massGeometry():Null<Part> return Part.cylinderSpan(outside() / 2, 0, height);
}
