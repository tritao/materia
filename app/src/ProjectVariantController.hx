package app;

import app.ProjectVariants.ProjectVariant;

typedef ProjectVariantState = {
  var variants:Array<ProjectVariant>;
  var selected:Null<String>;
  var summary:String;
  var switchAllowed:Bool;
  var simulationActive:Bool;
  var loading:Bool;
  var status:String;
  var error:Null<String>;
}

/** Prepares a replacement without disturbing the installed scene or its selection. */
class ProjectVariantController {
  final session:ProjectDocumentSession;
  final simulation:ApplicationSimulation;
  final requestChange:(Void->Void)->Void;
  final changed:Void->Void;
  final installed:Void->Void;
  final blocked:Void->Bool;
  var generation = -1;
  var variants:Array<ProjectVariant> = [];
  var failure:Null<String> = null;
  var loading:Null<ProjectLoadJob> = null;
  var reference:Null<String>;
  var loadGeneration = -1;
  var loadingLabel = "";

  public function new(session:ProjectDocumentSession,simulation:ApplicationSimulation,
      requestChange:(Void->Void)->Void,changed:Void->Void,installed:Void->Void,blocked:Void->Bool) {
    this.session=session;this.simulation=simulation;this.requestChange=requestChange;
    this.changed=changed;this.installed=installed;this.blocked=blocked;
  }
  public function isLoading():Bool return loading!=null;
  function sync():Void {
    if(generation==session.generation)return;
    generation=session.generation;variants=[];failure=null;
    var path=session.projectReference;
    if(path!=null&&!ProjectSourceLoader.isPrebuilt(path))
      try variants=ProjectVariants.read(path) catch(error:Dynamic)failure=Std.string(error);
  }
  public function snapshot():ProjectVariantState {
    sync();var selected=session.projectVariant??session.projectJob,summary="";
    for(variant in variants)if(variant.id==selected)summary=variant.summary;
    return {variants:variants.copy(),selected:selected,summary:summary,
      switchAllowed:!simulation.isActive()&&loading==null&&!blocked(),simulationActive:simulation.isActive(),
      loading:loading!=null,status:loading==null?"":"Opening "+loadingLabel+" · "+loading.control.currentPhase()+" · "+Std.int(loading.elapsedSeconds())+" s",error:failure};
  }
  public function select(id:String):Void {
    var state=snapshot();if(!state.switchAllowed||id==state.selected)return;
    if(state.variants.filter(v->v.id==id).length!=1)throw 'Unknown project variant "$id"';
    var path=session.projectReference,expectedGeneration=session.generation;if(path==null)return;
    requestChange(function() {
      if(simulation.isActive()||isLoading()||session.generation!=expectedGeneration)return;
      reference=path;loadGeneration=session.generation;failure=null;
      for(variant in variants)if(variant.id==id)loadingLabel=variant.label;
      loading=new ProjectLoadJob(path,id);changed();
    });
  }
  public function tick():Void {
    var task=loading;if(task==null)return;
    if(session.generation!=loadGeneration){cancel();return;}
    if(!task.isFinished()){changed();return;}
    loading=null;
    if(!task.wasCancelled())try {
      var candidate=task.take();
      if(simulation.isActive())throw "Stop simulation before changing project variants";
      var path=reference;if(path==null)throw "Project source is missing";
      session.openGeneratedProject(candidate,path);sync();installed();
    }catch(error:Dynamic)failure="Could not load project variant: "+Std.string(error);
    changed();
  }
  public function cancel():Void {if(loading!=null)loading.control.cancel();loading=null;changed();}
}
