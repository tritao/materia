package app.editor;

import Observable;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.presentation.AssemblyPresentation;
import materia.presentation.PresentationCodec;
import materia.presentation.PresentationEvaluator;

/** Each viewport owns one cursor; changes derive display state and never edit engineering state. */
class PresentationController implements Observable {
  public final definition:AssemblyDefinition;
  public final record:AssemblyPresentation;
  public var cursor(default, null):PresentationCursor;
  public var animating(default, null):Bool = false;
  static inline var MOTION_SECONDS:Float = 0.6;
  public function new(definition:AssemblyDefinition, record:AssemblyPresentation) {
    PresentationCodec.validate(definition, record);
    this.definition = definition;
    this.record = record;
    cursor = PresentationEvaluator.reset(record);
  }
  public function retainCursor(previous:PresentationController):Void {
    var old = previous.cursor;
    if (record.views.filter(v -> v.id == old.view).length == 0) return;
    if (old.procedure != null
      && record.procedures.filter(p -> p.id == old.procedure && p.steps.length >= old.step).length == 0) return;
    cursor = new PresentationCursor(old.view, old.explode, old.procedure, old.step, old.progress);
    animating = previous.animating;
  }
  public function reset():Void {
    animating = false;
    cursor = PresentationEvaluator.reset(record);
  }
  public function selectView(id:String):Void {
    for (view in record.views) if (view.id == id) {
      animating = false;
      cursor = new PresentationCursor(id, view.explode, null, 0, 0);
      return;
    }
    throw 'Unknown presentation view "$id"';
  }
  public function setExplode(value:Float):Void {
    if (!Math.isFinite(value) || value < 0 || value > 1) throw "Explode must be in 0..1";
    if (cursor.explode != value) {
      cursor = new PresentationCursor(cursor.view, value, cursor.procedure, cursor.step, cursor.progress);
    }
  }
  public function selectedProcedure():Null<Procedure> {
    for (procedure in record.procedures) if (procedure.id == cursor.procedure) return procedure;
    return null;
  }
  public function selectProcedure(id:Null<String>):Void {
    if (id != null && record.procedures.filter(p -> p.id == id).length == 0) throw 'Unknown procedure "$id"';
    animating = false;
    cursor = new PresentationCursor(cursor.view, 0, id, 0, 0);
  }
  public function restart():Void {
    animating = false;
    cursor = new PresentationCursor(cursor.view, cursor.explode, cursor.procedure, 0, 0);
  }
  public function previous():Void {
    animating = false;
    cursor = new PresentationCursor(cursor.view, cursor.explode, cursor.procedure,
      Std.int(Math.max(0, cursor.step - 1)), 0);
  }
  public function failedMovement():Bool {
    var procedure = selectedProcedure();
    if (procedure == null || cursor.step >= procedure.steps.length) return false;
    return materia.presentation.StepMotions.failed(procedure.steps[cursor.step].action);
  }
  public function next(animate:Bool = true):Void {
    var procedure = selectedProcedure();
    if (procedure == null || cursor.step >= procedure.steps.length || animating || failedMovement()) return;
    var moving = materia.presentation.StepMotions.members(procedure.steps[cursor.step].action).length > 0;
    if (animate && moving) {
      animating = true;
      cursor = new PresentationCursor(cursor.view, cursor.explode, cursor.procedure, cursor.step, 0);
    } else {
      cursor = new PresentationCursor(cursor.view, cursor.explode, cursor.procedure, cursor.step + 1, 0);
    }
  }
  /** Host time is explicit so transport and animation are reproducible in tests. */
  public function advance(seconds:Float):Bool {
    if (!animating) return false;
    if (!Math.isFinite(seconds) || seconds < 0) throw "Invalid procedure frame duration";
    var progress = Math.min(1, cursor.progress + seconds / MOTION_SECONDS), step = cursor.step;
    if (progress >= 1) {
      step++;
      progress = 0;
      animating = false;
    }
    cursor = new PresentationCursor(cursor.view, cursor.explode, cursor.procedure, step, progress);
    return true;
  }
  public function evaluate(solved:Map<String,
    AssemblyFrame>, fingerprint:String, playing:Bool = false):PresentationFrame return PresentationEvaluator.evaluate(
      definition,
      solved,
      record,
      cursor,
      fingerprint,
      playing
    );
}
