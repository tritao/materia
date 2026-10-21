package app.editor;

import app.EditorPerspectiveViewport;
import haxeon.ui.core.View;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.controls.Select;
import haxeon.ui.widgets.controls.Slider;
import haxeon.ui.widgets.controls.SelectOption;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.text.Text;

/** Shared controls for the main viewport, dock and picture-in-picture. */
class PresentationControls {
  public static function build(viewport:EditorPerspectiveViewport,changed:Void->Void,key:String,
      ?save:Void->Void,?addStep:Void->Void):View {
    var controller=viewport.presentation;
    var cells:Array<KeyedView> = [];
    var rows:Array<KeyedView> = [];
    if(controller!=null) {
      var options=[for(view in controller.record.views)new SelectOption<String>(view.id,view.label,view.id)];
      var selectStyle=new LayoutStyle();selectStyle.width=LayoutAxis.grow();
      var current=viewport.presentationFrame();
      var select=new Select<String>(key+"-views",options,current==null?controller.cursor.view:current.view,
        id->{viewport.selectPresentationView(id);changed();},selectStyle);
      select.accessibilityLabel="Assembly view";
      rows.push(new KeyedView("views",select));
      select.enabled=viewport.presentationControlsEnabled();
      if(controller.record.explode.length>0) {
        rows.push(new KeyedView("explode-label",new Text("Explode · "+Std.int(controller.cursor.explode*100)+"%")));
        var slider=new Slider(key+"-explode","Explode",controller.cursor.explode,0,1,0.01,
          value->{viewport.setPresentationExplode(value);changed();});
        slider.enabled=viewport.presentationControlsEnabled();
        rows.push(new KeyedView("explode",slider));
      }
    }
    if(controller!=null&&controller.record.procedures.length>0) {
      var options=[new SelectOption<String>("none","No procedure","")];
      for(procedure in controller.record.procedures)options.push(new SelectOption<String>(procedure.id,procedure.label,procedure.id));
      var picker=new Select<String>(key+"-procedure",options,controller.cursor.procedure==null?"":controller.cursor.procedure,
        id->{viewport.selectPresentationProcedure(id==""?null:id);changed();});
      picker.accessibilityLabel="Procedure";picker.enabled=viewport.presentationControlsEnabled();
      rows.push(new KeyedView("procedure",picker));
      var procedure=controller.selectedProcedure();
      if(procedure!=null) {
        var transport:Array<KeyedView> = [];
        function action(id:String,label:String,enabled:Bool,run:Void->Void):Void {
          var button=new Button(label,null,()->{run();changed();},key+"-"+id);
          button.enabled=viewport.presentationControlsEnabled()&&enabled;transport.push(new KeyedView(id,button));
        }
        action("previous","Previous",controller.cursor.step>0||controller.animating,viewport.previousPresentationStep);
        action("next","Next",controller.cursor.step<procedure.steps.length&&!controller.animating&&!controller.failedMovement(),viewport.nextPresentationStep);
        action("restart","Restart",controller.cursor.step>0||controller.animating,viewport.restartPresentationProcedure);
        rows.push(new KeyedView("transport",new Row(key+"-transport",transport)));
        var frame=viewport.presentationFrame();
        if(frame!=null&&frame.activeStep!=null) {
          var step=frame.activeStep;
          rows.push(new KeyedView("step-title",new Text((controller.cursor.step+1)+" / "+procedure.steps.length+" · "+step.title)));
          rows.push(new KeyedView("instruction",new Text(step.instruction)));
          if(frame.evidence!=null)rows.push(new KeyedView("evidence",new Text(frame.evidence,null,
            frame.evidence=="Failed"?haxeon.ui.Color.fromBytes(180,35,24):frame.evidence=="Stale"?haxeon.ui.Color.fromBytes(148,98,0):null)));
        } else rows.push(new KeyedView("complete",new Text("Procedure complete")));
      }
    }
    if(addStep!=null) {
      var button=new Button("Add step",null,addStep,key+"-add-step");
      button.enabled=viewport.presentationControlsEnabled();rows.push(new KeyedView("add-step",button));
    }
    var reset=new Button("Reset view",null,()->{viewport.resetPresentation();changed();},key+"-reset");
    reset.enabled=viewport.presentationControlsEnabled();cells.push(new KeyedView("reset",reset));
    if(save!=null) {
      var button=new Button("Save current view",null,save,key+"-save");
      button.enabled=viewport.presentationControlsEnabled();cells.push(new KeyedView("save",button));
    }
    var style=new LayoutStyle();style.width=LayoutAxis.grow();style.childGap=4;
    rows.push(new KeyedView("actions",new Row(key+"-actions",cells,style)));
    return new Column(key,rows,style);
  }
}
