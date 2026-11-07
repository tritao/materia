package materia.presentation;

import haxeon.wire.JsonWire;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.presentation.AssemblyPresentation;
import materia.presentation.PresentationEvaluator;

/** Immutable validated presentation snapshot. Publish a new instance when authoring changes. */
class CompiledPresentation {
  /** Read-only after publication, including nested records. */
  public final definition:AssemblyDefinition;
  public final record:AssemblyPresentation;

  public function new(definition:AssemblyDefinition, record:AssemblyPresentation) {
    this.definition = AssemblyDefinitionFlattener.flatten(definition);
    this.record = JsonWire.decode(JsonWire.encode(record));
    PresentationCodec.validate(this.definition, this.record);
  }

  public function evaluate(solved:Map<String, AssemblyFrame>, cursor:PresentationCursor,
    fingerprint:String, playing:Bool = false):PresentationFrame
    return PresentationEvaluator.evaluateValidated(definition, solved, record, cursor, fingerprint, playing);
}
