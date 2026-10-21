package app.editor;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.presentation.AssemblyPresentation;
import materia.presentation.PresentationCodec;
import materia.presentation.PresentationEvaluator;

/** Each viewport owns one cursor; changes derive display state and never edit engineering state. */
class PresentationController {
  public final definition:AssemblyDefinition;
  public final record:AssemblyPresentation;
  public var cursor(default,null):PresentationCursor;
  public var revision(default,null):Int = 0;
  public var animating(default,null):Bool = false;
  static inline var MOTION_SECONDS:Float = 0.6;
  public function new(definition:AssemblyDefinition, record:AssemblyPresentation) {
    PresentationCodec.validate(definition,record);
    this.definition=definition; this.record=record; cursor=PresentationEvaluator.reset(record);
  }
  public function retainCursor(previous:PresentationController):Void {
    var old=previous.cursor;
    if(record.views.filter(v->v.id==old.view).length==0)return;
    if(old.procedure!=null&&record.procedures.filter(p->p.id==old.procedure&&p.steps.length>=old.step).length==0)return;
    cursor={view:old.view,explode:old.explode,procedure:old.procedure,step:old.step,progress:old.progress};animating=previous.animating;revision++;
  }
  public function reset():Void {animating=false;cursor=PresentationEvaluator.reset(record);revision++;}
  public function selectView(id:String):Void {
    for(view in record.views)if(view.id==id) {
      animating=false;cursor.view=id;cursor.explode=view.explode;cursor.procedure=null;cursor.step=0;cursor.progress=0;revision++;return;
    }
    throw 'Unknown presentation view "$id"';
  }
  public function setExplode(value:Float):Void {
    if(!Math.isFinite(value)||value<0||value>1)throw "Explode must be in 0..1";
    if(cursor.explode!=value){cursor.explode=value;revision++;}
  }
  public function selectedProcedure():Null<Procedure> {
    for(procedure in record.procedures)if(procedure.id==cursor.procedure)return procedure;
    return null;
  }
  public function selectProcedure(id:Null<String>):Void {
    if(id!=null&&record.procedures.filter(p->p.id==id).length==0)throw 'Unknown procedure "$id"';
    animating=false;cursor.procedure=id;cursor.step=0;cursor.progress=0;cursor.explode=0;revision++;
  }
  public function restart():Void {
    animating=false;cursor.step=0;cursor.progress=0;revision++;
  }
  public function previous():Void {
    animating=false;cursor.progress=0;cursor.step=Std.int(Math.max(0,cursor.step-1));revision++;
  }
  public function failedMovement():Bool {
    var procedure=selectedProcedure();if(procedure==null||cursor.step>=procedure.steps.length)return false;
    return materia.presentation.StepMotions.failed(procedure.steps[cursor.step].action);
  }
  public function next(animate:Bool=true):Void {
    var procedure=selectedProcedure();if(procedure==null||cursor.step>=procedure.steps.length||animating||failedMovement())return;
    var moving=materia.presentation.StepMotions.members(procedure.steps[cursor.step].action).length>0;
    if(animate&&moving){animating=true;cursor.progress=0;}else{cursor.step++;cursor.progress=0;}
    revision++;
  }
  /** Host time is explicit so transport and animation are reproducible in tests. */
  public function advance(seconds:Float):Bool {
    if(!animating)return false;
    if(!Math.isFinite(seconds)||seconds<0)throw "Invalid procedure frame duration";
    cursor.progress=Math.min(1,cursor.progress+seconds/MOTION_SECONDS);
    if(cursor.progress>=1){cursor.step++;cursor.progress=0;animating=false;}
    revision++;return true;
  }
  public function evaluate(solved:Map<String,AssemblyFrame>, fingerprint:String, playing:Bool=false):PresentationFrame
    return PresentationEvaluator.evaluate(definition,solved,record,cursor,fingerprint,playing);
}
