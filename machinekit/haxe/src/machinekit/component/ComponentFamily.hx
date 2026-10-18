package machinekit.component;

/** One interface contract, one serializable recipe, explicit manufacturing alternatives.
 * The recipe must carry the variant in its normal values/BOM key. Geometry-level
 * compliance with interfaceId is established by the family's own checks. */
class ComponentFamily {
  public final id:String;
  public final interfaceId:String;
  public final recipe:ComponentType;
  public final variantParameter:String;
  final alternatives:Array<ManufacturingVariant>;
  public function new(id:String,interfaceId:String,recipe:ComponentType,variants:Array<ManufacturingVariant>,variantParameter:String="variant") {
    if(id==null || id.length==0 || interfaceId==null || interfaceId.length==0 || recipe==null || variants==null || variants.length==0 || recipe.materialEditable)
      throw "Manufacturing family needs an interface, variants and a process-locked material recipe";
    var ids:Array<String> = [];
    for(variant in variants) {
      if(ids.indexOf(variant.id)>=0) throw 'Duplicate manufacturing variant ${variant.id}';
      ids.push(variant.id);
    }
    var found=false;
    for(parameter in recipe.parameters()) if(parameter.name==variantParameter) switch parameter.type {
      case Choice(options):
        if(options.length!=ids.length) throw "Family variants differ from recipe choices";
        for(id in ids) if(options.indexOf(id)<0) throw "Family variant is absent from recipe";
        found=true;
      default: throw "Family variant must be a choice parameter";
    }
    if(!found) throw "Recipe has no manufacturing variant choice";
    var keys:Array<String> = [];
    for(variant in variants) {
      materia.project.MaterialLibrary.fromSpec(variant.material);
      var key=recipe.bomKey(recipe.defaults().setToken(variantParameter,variant.id));
      if(keys.indexOf(key)>=0) throw "Manufacturing variants must have distinct BOM keys";
      keys.push(key);
    }
    this.id=id;this.interfaceId=interfaceId;this.recipe=recipe;this.variantParameter=variantParameter;alternatives=variants.copy();
    recipe.attachFamily(this);
  }
  public function variants():Array<ManufacturingVariant> return alternatives.copy();
  public function variant(id:String):ManufacturingVariant {
    for(item in alternatives) if(item.id==id) return item;
    throw 'Unknown manufacturing variant "$id" in "${this.id}"';
  }
  public function create(variantId:String,?values:ComponentValues):MachineComponent {
    variant(variantId);
    var inputs=values==null ? new ComponentValues() : values.copy();
    inputs.setToken(variantParameter,variantId);return recipe.create(inputs);
  }
}
