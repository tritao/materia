package materia.presentation;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.presentation.AssemblyPresentation;

/** Shared producer defaults; authored and generated records use the same format. */
class Presentations {
  public static function assembled(definition:AssemblyDefinition, yaw:Float = -1.1, pitch:Float = 0.35,
      producer:String = "assembly"):AssemblyPresentation {
    var flat = AssemblyDefinitionFlattener.flattenView(definition);
    return {schemaVersion:PresentationCodec.VERSION,definition:definition.id,defaultView:"assembled",
      views:[{id:"assembled",label:"Assembled",camera:Frame([for (part in flat.occurrences)
        {occurrence:part.id,subtree:false}],yaw,pitch,1.15),visibility:[],sections:[],explode:0,source:Generated(producer)}],
      explode:[],procedures:[]};
  }
}
