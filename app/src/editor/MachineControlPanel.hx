package app.editor;

import app.Main.ReferenceEditorApp;
import app.MachineControlModel;
import haxeon.ui.Insets;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.core.View;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import app.ProjectJointEdit;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.scroll.ScrollView;
import haxeon.ui.widgets.scroll.ScrollAxis;
import haxeon.ui.widgets.scroll.ScrollController;
import haxeon.ui.widgets.text.Text;
import haxeon.ui.widgets.text.TextField;

/** Manual travel testing in design mode, independent of the selected component. */
@:access(app.Main.ReferenceEditorApp)
class MachineControlPanel {
  final drafts:Map<String,String> = [];
  final scroll = new ScrollController();
  var generation = -1;
  var error:Null<String>;
  var edit:Null<ProjectJointEdit>;
  var editAxis:Null<String>;
  var editOwner:Null<ReferenceEditorApp>;
  var pointer:Null<{id:Int,x:Float,y:Float}>;
  public function new() {}
  public function dragging():Bool return edit != null;
  public function commitGesture():Void finishGesture(true);
  public function cancelGesture():Void finishGesture(false);
  function finishGesture(commit:Bool):Void {
    var current=edit,owner=editOwner,captured=pointer;
    edit=null;editAxis=null;editOwner=null;pointer=null;
    if (current==null) return;
    if(commit)current.commit();else current.cancel();
    if(owner!=null) {
      if(captured!=null)owner.ui.pointerCancel(captured.id,captured.x,captured.y);
      owner.invalidateView();
    }
  }
  public static function rotaryTitle(id:String):String {
    var path=id.split("/"),leaf=path.pop();
    if(leaf==null)return "Rotation";
    var stem=leaf=="rotor-turn" && path.length>0 ? path[path.length-1] : StringTools.replace(leaf,"-turn","");
    stem=StringTools.replace(StringTools.replace(stem,"-"," "),"_"," ");
    var title=stem.substr(0,1).toUpperCase()+stem.substr(1)+" rotation";
    var scope=leaf=="rotor-turn" ? path.slice(0,path.length-1) : path;
    return scope.length==0 ? title : scope.join(" / ")+" · "+title;
  }
  static function number(value:Float):String return Std.string(Math.fround(value*1000)/1000);

  public function build(app:ReferenceEditorApp):View {
    if (generation != app.session.generation) {cancelGesture();generation=app.session.generation;drafts.clear();error=null;}
    var axes = MachineControlModel.axes(app.session);
    var editable = !app.simulation.isActive() && app.session.scriptOwnership == null && !app.documents.blocked();
    if (!editable) cancelGesture();
    var style = new LayoutStyle();style.width=LayoutAxis.grow();style.childGap=8;
    style.padding=new Insets(12,12,12,12);
    var rows:Array<KeyedView> = [new KeyedView("heading",app.sectionHeading("MACHINE CONTROL"))];
    var guardPanel=app.simulation.isActive()?EnclosureGuardPanel.build(app.simulation):
      EnclosureGuardPanel.preview(app.session,editable,(id,value)->{
        try {
          cancelGesture();
          MachineControlModel.set(app.session,id,value,
            !app.simulation.isActive()&&app.session.scriptOwnership==null&&!app.documents.blocked());
          error=null;app.documentChanged();
        }catch(failure:Dynamic){error=Std.string(failure);app.invalidateView();}
      });
    if(guardPanel!=null)rows.push(new KeyedView("enclosure-guard",guardPanel));
    var viewport=app.perspectiveViewport;
    var cup=app.simulation.hobbingPlayer();
    var machineMotion=app.session.machineMotion;
    var cupId=machineMotion==null||machineMotion.hobbing==null?null:machineMotion.hobbing.stock;
    if(cup!=null&&cup.handling!=null&&(cup.scenario==null||cup.scenario=="loading")){
      var safe=!cup.cutStarted||cup.completed;
      rows.push(new KeyedView("cup-status",new Text(cup.handling.inspection=="sequence"?cup.handling.phase:"Inspection: "+cup.handling.inspection)));
      rows.push(new KeyedView("cup-note",new Text("Guided loading; sleeve expansion illustrative. Reset → Play replays.")));
      var views:Array<KeyedView> = [];
      for(mode in ["blank","clamped","finished"]){
        var name=mode;
        var button=new Button(name.substr(0,1).toUpperCase()+name.substr(1),null,()->{
          app.simulation.stop();cup.inspectCup(name);if(viewport!=null&&viewport.sectionActive()){viewport.clearSection();viewport.fitAll();}app.invalidateView();
        },"cup-view-"+name);
        button.enabled=safe&&(name!="finished"||cup.cutCompleted);
        views.push(new KeyedView(name,button));
      }
      rows.push(new KeyedView("cup-views",new Row("cup-views",views,ReferenceEditorApp.actionRowStyle())));
      var resume=new Button("Continue sequence",null,()->{cup.inspectCup("sequence");if(viewport!=null&&viewport.sectionActive()){viewport.clearSection();viewport.fitAll();}app.simulation.start();app.invalidateView();},"cup-continue");
      resume.enabled=!cup.completed&&cup.scenario==null;
      var section=new Button("Clamped section",null,()->{app.simulation.stop();cup.inspectCup("clamped");if(viewport!=null)viewport.inspectObject("project:"+cupId);app.invalidateView();},"cup-section");
      section.enabled=safe;
      rows.push(new KeyedView("cup-actions",new Row("cup-actions",[new KeyedView("continue",resume),new KeyedView("section",section)],ReferenceEditorApp.actionRowStyle())));
    }else if(app.session.machineMotion!=null&&app.session.machineMotion.hobbing!=null&&app.session.machineMotion.hobbing.handling!=null){
      rows.push(new KeyedView("cup-start",new Text("Press Play to load, clamp, cut and unload the cup.")));
    }
    if(viewport!=null && viewport.sectionActive()) {
      var close=new Button("Close section",null,()->{viewport.clearSection();app.invalidateView();},"machine-section-close");
      var flip=new Button("Flip side",null,()->{viewport.flipSection();app.invalidateView();},"machine-section-flip");
      rows.push(new KeyedView("section",new Row("machine-section",[new KeyedView("close",close),new KeyedView("flip",flip)],ReferenceEditorApp.actionRowStyle())));
    }
    if (axes.length == 0) rows.push(new KeyedView("empty",new Text("Open a machine assembly with movable joints to test its travel.")));
    else {
      rows.push(new KeyedView("hint",new Text(editable ? "Move axes to check travel. Undoable previews do not require saving." :
        app.simulation.isActive() ? "Stop simulation to edit these design positions." : "The project currently controls this machine.")));
      if(editable) {
        var savePose=new Button("Save current pose as initial",null,()->{
          try {commitGesture();app.session.saveCurrentPoseAsInitial();error=null;app.documentChanged();}
          catch(failure:Dynamic){error=Std.string(failure);app.invalidateView();}
        },"machine-save-initial-pose");
        rows.push(new KeyedView("save-initial-pose",savePose));
      }
      var linked = 0;
      for (axis in axes) {
        if (axis.follows != null) {linked++;continue;}
        var id=axis.id;
        var set = function(value:Float) {
          try {cancelGesture();MachineControlModel.set(app.session,id,value,!app.simulation.isActive() && !app.documents.blocked());error=null;drafts.remove(id);app.documentChanged();}
          catch (failure:Dynamic) {error=Std.string(failure);app.invalidateView();}
        };
        var preview = function(value:Float) {
          var current=edit;
          if(current==null || editAxis!=id) {set(value);return;}
          try {
            current.update(MachineControlModel.nativePosition(app.session,id,value,!app.simulation.isActive() && !app.documents.blocked()));
            error=null;drafts.remove(id);app.invalidateView();
          } catch(failure:Dynamic) {error=Std.string(failure);app.invalidateView();}
        };
        var begin = function(event:haxeon.ui.core.UiEvent) {
          cancelGesture();
          try {
            edit=app.session.beginAssemblyJointEdit(id);editAxis=id;editOwner=app;
            pointer={id:event.pointerId,x:event.x,y:event.y};app.commands.refresh();
          } catch(failure:Dynamic) {
            error=Std.string(failure);event.preventDefault();event.stopImmediatePropagation();app.invalidateView();
          }
        };
        var finish = function(commit:Bool) {finishGesture(commit);app.commands.refresh();};

        var rotary=axis.unit=="deg",wrapped=MachineControlModel.wrapsAngle(axis);
        // Round before wrapping so 359.9999 degrees displays as 0 rather than 360.
        var position=number(wrapped ? MachineControlModel.wrapDegrees(Math.fround(axis.value*1000)/1000) : axis.value);
        var title=rotary ? rotaryTitle(id) : id+" · "+number(axis.value)+" "+axis.unit;
        var controls:Array<KeyedView> = [new KeyedView("label",new Text(title))];
        var minimum=axis.minimum,maximum=axis.maximum;
        if(rotary) {
          var dial=new MachineAngleControl("machine-angle-dial:"+id,title+" dial",axis.value,true,preview,begin,finish,minimum,maximum);
          var scrub=new MachineAngleControl("machine-angle-scrub:"+id,title+" scrubber",axis.value,false,preview,begin,finish,minimum,maximum);
          dial.enabled=editable;scrub.enabled=editable;scrub.style.width=LayoutAxis.fixed(120);
          var stripStyle=new LayoutStyle();stripStyle.width=LayoutAxis.fixed(120);stripStyle.childGap=3;
          var strip=new Column("machine-angle-strip:"+id,[new KeyedView("scrub",scrub),
            new KeyedView("hint",new Text("Drag to turn",null,app.appearance.theme.tokens.textSecondary,
              haxeon.ui.core.TextStyleOverride.text(11))),
            new KeyedView("fine",new Text("Shift for fine",null,app.appearance.theme.tokens.textSecondary,
              haxeon.ui.core.TextStyleOverride.text(11)))],stripStyle);
          controls.push(new KeyedView("angle",new Row("machine-angle:"+id,[new KeyedView("dial",dial),new KeyedView("strip",strip)],ReferenceEditorApp.actionRowStyle())));
          if(wrapped)controls.push(new KeyedView("range",new Text("0 … 360° · wraps")));
          else if(minimum!=null && maximum!=null)controls.push(new KeyedView("range",new Text(number(minimum)+" … "+number(maximum)+"°")));
        } else if (minimum != null && maximum != null && maximum > minimum) {
          var slider=new MachineControlSlider("machine-axis-slider:"+id,id+" position ("+axis.unit+")",axis.value,minimum,maximum,
            Math.min(axis.step,(maximum-minimum)/1000),preview,begin,finish);
          slider.enabled=editable;controls.push(new KeyedView("slider",slider));
          controls.push(new KeyedView("range",new Text(number(minimum)+" … "+number(maximum)+" "+axis.unit)));
        }
        var field=new TextField("machine-axis-value:"+id,drafts.exists(id) ? drafts.get(id) : position,
          value -> drafts.set(id,value));
        field.label=rotary ? title+" (deg)" : "Position ("+axis.unit+")";field.enabled=editable;field.style.width=LayoutAxis.grow();
        if(!rotary) controls.push(new KeyedView("value",field));
        var buttons:Array<KeyedView> = [];
        function button(key:String,label:String,value:Float):Void {
          var b=new Button(label,null,()->set(value),"machine-axis:"+id+":"+key);b.enabled=editable;
          buttons.push(new KeyedView(key,b));
        }
        var applyPosition = function() {
          if (!editable) return;
          try {set(MachineControlModel.enteredPosition(axis,MachineControlModel.parsePosition(drafts.exists(id) ? drafts.get(id) : position)));}
          catch (failure:Dynamic) {error=Std.string(failure);app.invalidateView();}
        };
        field.onSubmit = _ -> applyPosition();
        var apply=new Button(rotary ? "Apply" : "Set",null,applyPosition,"machine-axis:"+id+":set");
        apply.enabled=editable;
        if(rotary) {
          var unitStyle=new LayoutStyle();unitStyle.padding=new Insets(0,8,0,0);
          controls.push(new KeyedView("value",new Row("machine-angle-value:"+id,
            [new KeyedView("field",field),new KeyedView("unit",new Text("°",unitStyle)),new KeyedView("apply",apply)],ReferenceEditorApp.actionRowStyle())));
        } else buttons.push(new KeyedView("set",apply));
        if(viewport!=null) {
          var inspect=new Button("Inspect",null,()->{
            try {
              var definition=app.session.projectAssemblyDefinition;
              if(definition!=null)for(joint in definition.joints)if(joint.id==id){
                viewport.inspectObject("project:"+joint.child);error=null;app.invalidateView();break;
              }
            }catch(failure:Dynamic){error=Std.string(failure);app.invalidateView();}
          },"machine-axis:"+id+":inspect");
          inspect.accessibilityLabel="Inspect "+id;
          buttons.push(new KeyedView("inspect",inspect));
        }
        if (minimum != null) button("min","Min",minimum);
        button("initial",rotary ? "Reset" : "Initial",axis.initial);
        if (axis.initial != 0 && (minimum == null || minimum <= 0) && (maximum == null || maximum >= 0)) button("zero","Zero",0);
        if (maximum != null) button("max","Max",maximum);
        controls.push(new KeyedView("buttons",new Row("machine-axis-buttons:"+id,buttons,ReferenceEditorApp.actionRowStyle())));
        rows.push(new KeyedView(id,new Column("machine-axis:"+id,controls,style)));
      }
      if (linked > 0) rows.push(new KeyedView("linked",new Text('$linked linked or closure joints follow automatically.')));
    }
    if (error != null) rows.push(new KeyedView("error",new Text(error)));
    var fill=ReferenceEditorApp.fillStyle();fill.background=app.appearance.theme.tokens.surface;
    return new ScrollView("machine-control-scroll",new Column("machine-control",rows,style),fill,ScrollAxis.Vertical,scroll);
  }
}
