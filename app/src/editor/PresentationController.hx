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
  public function new(definition:AssemblyDefinition, record:AssemblyPresentation) {
    PresentationCodec.validate(definition,record);
    this.definition=definition; this.record=record; cursor=PresentationEvaluator.reset(record);
  }
  public function retainCursor(previous:PresentationController):Void {
    var old=previous.cursor;
    if(record.views.filter(v->v.id==old.view).length==0)return;
    if(old.procedure!=null&&record.procedures.filter(p->p.id==old.procedure&&p.steps.length>=old.step).length==0)return;
    cursor={view:old.view,explode:old.explode,procedure:old.procedure,step:old.step,progress:old.progress};revision++;
  }
  public function reset():Void {cursor=PresentationEvaluator.reset(record);revision++;}
  public function selectView(id:String):Void {
    for(view in record.views)if(view.id==id) {
      cursor.view=id;cursor.explode=view.explode;cursor.procedure=null;cursor.step=0;cursor.progress=0;revision++;return;
    }
    throw 'Unknown presentation view "$id"';
  }
  public function setExplode(value:Float):Void {
    if(!Math.isFinite(value)||value<0||value>1)throw "Explode must be in 0..1";
    if(cursor.explode!=value){cursor.explode=value;revision++;}
  }
  public function evaluate(solved:Map<String,AssemblyFrame>, fingerprint:String, playing:Bool=false):PresentationFrame
    return PresentationEvaluator.evaluate(definition,solved,record,cursor,fingerprint,playing);
}
