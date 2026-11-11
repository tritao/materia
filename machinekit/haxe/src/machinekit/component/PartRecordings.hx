package machinekit.component;

import cadkit.modeling.Part;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.Feature;
import cadkit.parametric.ParameterRole;
import cadkit.parametric.recording.DocumentBuilder;

/** A part's saved recording and its top-level steps (history row id, label with its group, faces it made). */
typedef RecordedHistory = {document:String, steps:Array<{id:Int, label:String, faces:Array<Int>}>};

/**
 * What `MachineComponent` does with a part's recording (`MachineComponent.record`): the derived holes as a last step, the
 * check that recipe inputs and editable dimensions are one set, and the solid and saved form taken from it.
 */
class PartRecordings {
  /**
   * `component`'s recording at `detail`, derived holes included, or null when it does not record its geometry. The caller
   * closes it. The holes a joint derives (`MachineComponent.machine`) are one more step, `Fasteners`, at the end, so the
   * solid, the history and the hole table carry the same holes.
   */
  public static function document(component:MachineComponent, detail:ComponentDetail):Null<Document> {
    var builder = new DocumentBuilder();
    var document:Null<Document> = null;
    try {
      var body = component.record(builder, detail);
      if (body == null) {
        builder.close();
        return null;
      }
      var cuts = component.machinedCuts(), result = body;
      if (cuts.length > 0) builder.group("Fasteners", () -> result = builder.label(builder.holes(body, cuts), "Derived holes"));
      builder.output(result);
      document = builder.finish();
      checkParameters(component, document);
      return document;
    } catch (error:Dynamic) {
      if (document != null) document.close(); else builder.close();
      throw error;
    }
  }

  /** A new solid of `document`'s result, which it closes. Its faces keep the recording's face numbering. */
  public static function solid(document:Document):Part {
    try {
      var part = new Part(document.result().cloneShape());
      document.close();
      return part;
    } catch (error:Dynamic) {
      document.close();
      throw error;
    }
  }

  /** `document` saved, as a scene history carries it; closes it. */
  public static function encoded(document:Document):String {
    try {
      var text = DocumentCodec.encode(document);
      document.close();
      return text;
    } catch (error:Dynamic) {
      document.close();
      throw error;
    }
  }

  /**
   * The recipe's numeric inputs are the document's editable named parameters, by name and value, and every editable named
   * parameter is a recipe input: values computed from others are recorded as derived, values from stock or a catalogue as
   * fixed. So an editor override of a recipe input and the dimension it changes are the same thing.
   */
  /**
   * Every numeric input of the component's recipe as an editable named parameter of `builder`, by name: for a part whose shape
   * comes from a plan computed from its inputs rather than from the dimensions one by one.
   */
  public static function recipeInputs(component:MachineComponent, builder:DocumentBuilder):Map<String, cadkit.parametric.NamedParameter> {
    var out = new Map<String, cadkit.parametric.NamedParameter>(), recipe = component.componentType();
    if (recipe != null) for (parameter in recipe.parameters()) switch parameter.type {
      case Scalar | Length | Angle: out.set(parameter.name, builder.length(parameter.name, component.values().number(parameter.name)));
      case Count: out.set(parameter.name, builder.count(parameter.name, component.values().integer(parameter.name)));
      case _:
    }
    return out;
  }

  static function checkParameters(component:MachineComponent, document:Document):Void {
    var recipe = component.componentType(), designation = component.designation;
    var inputs = new Map<String, Float>();
    if (recipe != null) for (parameter in recipe.parameters()) switch parameter.type {
      case Scalar | Length | Angle:
        inputs.set(parameter.name, component.values().number(parameter.name));
      case Count:
        inputs.set(parameter.name, component.values().integer(parameter.name));
      case _:
    }
    for (name => value in inputs) {
      var named = try document.parameter(name) catch (_:Dynamic) null;
      if (named == null) throw 'Recorded part "$designation": recipe input "$name" is not a named parameter of its document';
      if (named.role != ParameterRole.Editable) throw 'Recorded part "$designation": recipe input "$name" is recorded as ${named.role}';
      if (Math.abs(named.value - value) > 1e-9 * Math.max(1, Math.abs(value)))
        throw 'Recorded part "$designation": "$name" is ${named.value} in the document but $value in the recipe';
    }
    for (named in document.namedParameters()) if (named.role == ParameterRole.Editable && !inputs.exists(named.name))
      throw 'Recorded part "$designation": editable parameter "${named.name}" is not a recipe input; record it as derived or fixed';
  }
}
