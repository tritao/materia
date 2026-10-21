package materia.presentation;

import haxeon.wire.JsonWire;
import materia.assembly.AssemblyDefinition;
import materia.presentation.AssemblyPresentation;

/** Only authored entries are persisted; generated entries are regenerated on every load. */
@:wire typedef AuthoredPresentation = {
  @:id(1) var schemaVersion:Int;
  @:id(2) var definition:String;
  @:id(3) var views:Array<PresentationView>;
  @:id(4) var explode:Array<ExplodeOffset>;
  @:id(5) var procedures:Array<Procedure>;
}
class PresentationEdits {
  public static function empty(definition:String):AuthoredPresentation
    return {schemaVersion:1,definition:definition,views:[],explode:[],procedures:[]};
  public static function decode(text:String):AuthoredPresentation {
    var value:AuthoredPresentation=JsonWire.decode(text);
    if(value.schemaVersion!=1||value.views==null||value.explode==null||value.procedures==null)
      throw "Unsupported or incomplete authored presentation";
    for(view in value.views)if(view.source!=Authored)throw "Saved presentation view must be Authored";
    for(offset in value.explode)if(offset.source!=Authored)throw "Saved explode offset must be Authored";
    for(procedure in value.procedures) {
      if(procedure.source!=Authored)throw "Saved procedure must be Authored";
      for(step in procedure.steps)switch step.action {
        case Remove(_,motion)|Install(_,motion):
          if(motion.evidence!=Illustrative)throw "Authored motions must be Illustrative";
        case _:
      }
    }
    return value;
  }
  public static function encode(value:AuthoredPresentation):String {
    var text=JsonWire.encode(value);decode(text);return text;
  }
  public static function merge(definition:AssemblyDefinition,generated:Null<AssemblyPresentation>,
      authored:Null<AuthoredPresentation>):AssemblyPresentation {
    var base=generated==null?Presentations.assembled(definition):generated;
    var result:AssemblyPresentation={schemaVersion:base.schemaVersion,definition:base.definition,defaultView:base.defaultView,
      views:base.views.copy(),explode:base.explode.copy(),procedures:base.procedures.copy()};
    if(authored!=null) {
      if(authored.definition!=definition.id)throw "Authored presentation belongs to another assembly";
      decode(encode(authored));
      result.views=result.views.concat(authored.views);result.explode=result.explode.concat(authored.explode);
      result.procedures=result.procedures.concat(authored.procedures);
    }
    PresentationCodec.validate(definition,result);return result;
  }
}
