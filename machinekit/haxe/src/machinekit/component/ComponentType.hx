package machinekit.component;

import machinekit.component.ComponentValue.*;
import machinekit.component.ComponentParameterType.*;
import materia.project.MaterialLibrary;

/** Stable recipe for one kind of single part. */
class ComponentType {
  public final id:String;
  public var manufacturingFamily(default, null):Null<ComponentFamily>;

  /** False when a typed mechanical input already determines the material. */
  public final materialEditable:Bool;

  final inputs:Array<ComponentParameter>;
  final build:ComponentValues -> MachineComponent;
  /** The generator before the material input is applied: what material these inputs give by themselves. */
  final naturalBuild:ComponentValues -> MachineComponent;
  final keepDesignation:Bool;
  final materialInDesignation:Bool;
  final defaultMaterial:String;
  final bomExcludedInputs:Array<String>;
  public var generatorVersion(default, null):Int = 1;
  var numberFor:Null<ComponentValues -> String>;
  var purchaseKeyFor:Null<ComponentValues -> String>;
  var procurementFor:Null<ComponentValues -> Procurement>;
  var requirementsFor:Null<ComponentValues -> Array<machinekit.component.Procurement.ProcurementRequirement>>;

  /**
	 * Mass properties already computed for a component of a recipe, by its `MachineComponent.geometryKey` (which names the
	 * recipe). They come from cutting and integrating real solids, and each machine compile builds new component instances,
	 * so a per-instance cache never hit across compiles. `MassProperties` never changes, so one value is shared by every such
	 * component.
	 */
  public static final masses = new machinekit.cache.Memo<MassProperties>("component-masses", 8192);

  public function new(
    id:String,
    inputs:Array<ComponentParameter>,
    build:ComponentValues -> MachineComponent,
    keepDesignation:Bool = false,
    materialInDesignation:Bool = false,
    ?bomExcludedInputs:Array<String>,
    materialEditable:Bool = true,
    ?metadataInputs:ComponentValues
  ) {
    if (id == null || id.length == 0 || inputs == null || build == null) throw "Incomplete component type";
    this.id = id;
    this.materialEditable = materialEditable;
    this.keepDesignation = keepDesignation;
    this.materialInDesignation = materialInDesignation;
    this.bomExcludedInputs = bomExcludedInputs == null ? [] : bomExcludedInputs.copy();
    var seen:Map<String, Bool> = [];
    for (input in inputs) {
      if (seen.exists(input.name)) throw 'Duplicate component input "${input.name}"';
      seen.set(input.name, true);
    }
    var baseline = new ComponentValues();
    // Required inputs can have a representative metadata instance without becoming defaults.
    for (input in inputs) {
      var sample = metadataInputs == null ? null : metadataInputs.get(input.name);
      baseline.set(input.name, sample == null ? input.defaultValue : sample);
    }
    var sourceBuild = build;
    var defaultComponent = sourceBuild(baseline);
    defaultMaterial = defaultComponent.materialSpec();
    this.inputs = inputs.copy();
    if (!materialEditable && seen.exists("material")) throw 'Derived material on "$id" cannot also be an input';
    if (materialEditable && !seen.exists("material")) this.inputs.push(new ComponentParameter(
      "material",
      Choice(MaterialLibrary.specs()),
      Token(defaultMaterial)
    ));
    else if (materialEditable) {
      var declared = switch this.inputs[seenIndex(this.inputs, "material")].defaultValue {
        case Token(spec):
          spec;
        default:
          "";
      };
      if (declared != defaultMaterial) throw 'Component type "$id" material default differs from its generator';
    }
    naturalBuild = sourceBuild;
    this.build = values -> {var component = sourceBuild(values);
    // A material left at the recipe's default keeps the model's own: a catalogue row's product material.
    if (materialEditable
      && values.token("material") != defaultMaterial) component.setMaterial(values.token("material"));
    return component;
    };
  }

  public function parameters():Array<ComponentParameter> return inputs.copy();

  /** Declare the catalogue or drawing identity for this recipe. */
  public function numbered(number:ComponentValues -> String):ComponentType {
    if (number == null || numberFor != null) throw 'Part number already defined for "$id"';
    numberFor = number;
    return this;
  }

  /** Increment when the generator changes geometry without an input change. */
  public function versioned(version:Int):ComponentType {
    if (version < 1) throw 'Invalid geometry generator version for "$id"';
    generatorVersion = version;
    return this;
  }

  /** Declare a purchase configuration when placement inputs differ from the purchased item. */
  public function purchaseKeyed(key:ComponentValues -> String):ComponentType {
    if (key == null || purchaseKeyFor != null) throw 'Purchase key already defined for "$id"';
    purchaseKeyFor = key;
    return this;
  }

  /** Declare the procurement decision next to the recipe's identity. */
  public function procured(intent:ComponentValues -> Procurement):ComponentType {
    if (intent == null || procurementFor != null) throw 'Procurement already defined for "$id"';
    procurementFor = intent;
    return this;
  }

  /** Declare the numeric inputs or ratings used to select an assumed purchase. */
  public function requiring(terms:ComponentValues -> Array<machinekit.component.Procurement.ProcurementRequirement>):ComponentType {
    if (terms == null || requirementsFor != null) throw 'Requirements already defined for "$id"';
    requirementsFor = terms;
    return this;
  }

  /** Select the recipe's explicit numeric requirements and their bounds. */
  public function requiringInputs(nominal:Array<String>, ?minimum:Array<String>, ?maximum:Array<String>):ComponentType {
    var selected:Array<{parameter:ComponentParameter, bound:String}> = [];
    var seen:Map<String, Bool> = [];
    function append(names:Null<Array<String>>, bound:String):Void {
      if (names == null) return;
      for (name in names) {
        if (seen.exists(name)) throw 'Duplicate requirement input "$name" for "$id"';
        seen.set(name, true);
        var found:Null<ComponentParameter> = null;
        for (parameter in inputs) if (parameter.name == name) found = parameter;
        if (found == null) throw 'Unknown requirement input "$name" for "$id"';
        switch found.type {
          case Scalar | Length | Angle | Count:
          case Choice(_) | Bool if (bound == "nominal"):
          default: throw 'Requirement input "$name" has an unsupported type or bound';
        }
        selected.push({parameter:found, bound:bound});
      }
    }
    append(nominal, "nominal"); append(minimum, "min"); append(maximum, "max");
    if (selected.length == 0) throw 'No requirement inputs declared for "$id"';
    return requiring(v -> [for (selection in selected) {
      var parameter = selection.parameter;
      var unit = parameter.unit == null ? "count" : parameter.unit;
      switch v.get(parameter.name) {
        case Token(value): Procurement.ProcurementCodec.choice(parameter.name, value);
        case Boolean(value): Procurement.ProcurementCodec.choice(parameter.name, value ? "true" : "false");
        case Number(value):
          switch selection.bound {
            case "min": Procurement.ProcurementCodec.requirement(parameter.name, unit, value);
            case "max": Procurement.ProcurementCodec.requirement(parameter.name, unit, null, value);
            default: Procurement.ProcurementCodec.requirement(parameter.name, unit, null, null, value);
          }
        case Integer(value):
          switch selection.bound {
            case "min": Procurement.ProcurementCodec.requirement(parameter.name, unit, value);
            case "max": Procurement.ProcurementCodec.requirement(parameter.name, unit, null, value);
            default: Procurement.ProcurementCodec.requirement(parameter.name, unit, null, null, value);
          }
        default: throw 'Missing requirement input "${parameter.name}"';
      }
    }]);
  }

  public function procurement(values:ComponentValues):Null<Procurement> return procurementFor == null
    ? null : procurementFor(resolve(values));
  public function requirements(values:ComponentValues):Array<machinekit.component.Procurement.ProcurementRequirement> return requirementsFor == null
    ? [] : requirementsFor(resolve(values));

  public function label():String {
    var words = id.split(".").pop().split("-").join(" ");
    return words.substr(0, 1).toUpperCase() + words.substr(1);
  }

  public function matches(component:MachineComponent):Bool return component.componentType() == this;

  public function defaults():ComponentValues return resolve(null);

  public function resolve(?overrides:ComponentValues):ComponentValues return ComponentRecipeSupport.resolve(
    inputs,
    overrides,
    false
  );

  public function attachFamily(family:ComponentFamily):Void {
    if (family.recipe != this || manufacturingFamily != null) throw "Recipe already belongs to a manufacturing family";
    manufacturingFamily = family;
  }

  public function create(?values:ComponentValues):MachineComponent {
    var resolved = resolve(values), component = build(resolved);
    if (manufacturingFamily != null) {
      var variant = manufacturingFamily.variant(resolved.token(manufacturingFamily.variantParameter));
      if (component.materialSpec() != variant.material) throw "Component material differs from manufacturing variant";
      component.lockManufacturingMaterial(variant.material);
    }
    return component;
  }

  public function valuesOf(component:MachineComponent):ComponentValues {
    if (!matches(component)) throw 'Component is not type "$id"';
    return component.values();
  }

  public function key(values:ComponentValues):String return keyFor(values, false);

  /** BOM identity omits recipe inputs that only place or route a physical part. */
  public function bomKey(values:ComponentValues):String return purchaseKeyFor == null
    ? keyFor(values, true) : id + "|" + purchaseKeyFor(resolve(values));

  function keyFor(values:ComponentValues, bom:Bool):String {
    var parts:Array<String> = [];
    for (input in inputs) {
      if (bom && bomExcludedInputs.indexOf(input.name) >= 0) continue;
      var value = values == null ? null : values.get(input.name);
      if (value == null) value = input.defaultValue;
      if (input.name == "material" && switch value {
        case Token(spec):
          spec == defaultMaterial;
        default:
          false;
      }
      ) continue;
      var text = switch value {
        case Unset:
          "unset";
        case Number(number):
          ComponentNumberKey.encode(number);
        case Integer(value):
          Std.string(value);
        case Boolean(value):
          value ? "true" : "false";
        case Token(value):
          value;
        case null:
          throw 'Missing component input "${input.name}"';
      };
      parts.push('${input.name}:${text.length}:$text');
    }
    return id + "|" + parts.join("|");
  }

  /** Keep authored numbers stable and distinguish an independently edited material. */
  public function partNumber(component:MachineComponent):String {
    if (numberFor != null) {
      var number = numberFor(resolve(valuesOf(component)));
      if (!materialInDesignation && component.materialSpec() != naturalMaterial(component))
        number += "-" + MaterialLibrary.fromSpec(component.materialSpec()).toUpperCase();
      return number;
    }
    return keepDesignation && (materialInDesignation
      || component.materialSpec() == naturalMaterial(component)) ? component.designation : key(valuesOf(component));
  }

  function naturalMaterial(component:MachineComponent):String {
    if (component.materialSpec() == defaultMaterial) return defaultMaterial;
    var values = valuesOf(component), inputs = new ComponentValues();
    for (name in values.names()) if (name != "material") inputs.set(name, values.get(name));
    return naturalBuild(resolve(inputs)).materialSpec();
  }

  static function seenIndex(inputs:Array<ComponentParameter>, name:String):Int {
    for (i in 0...inputs.length) if (inputs[i].name == name) return i;
    return -1;
  }
}
