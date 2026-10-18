package app.editor;

import app.Main.ReferenceEditorApp;
import app.MachineControlModel;
import haxeon.ui.Insets;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.core.View;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.controls.Slider;
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
  public function new() {}
  static function number(value:Float):String return Std.string(Math.round(value*1000)/1000);

  public function build(app:ReferenceEditorApp):View {
    if (generation != app.session.generation) {generation=app.session.generation;drafts.clear();error=null;}
    var axes = MachineControlModel.axes(app.session);
    var editable = !app.simulation.isActive() && app.session.scriptOwnership == null && !app.documents.blocked();
    var style = new LayoutStyle();style.width=LayoutAxis.grow();style.childGap=8;
    style.padding=new Insets(12,12,12,12);
    var rows:Array<KeyedView> = [new KeyedView("heading",app.sectionHeading("MACHINE CONTROL"))];
    if (axes.length == 0) rows.push(new KeyedView("empty",new Text("Open a machine assembly with movable joints to test its travel.")));
    else {
      rows.push(new KeyedView("hint",new Text(editable ? "Move axes to check travel. Changes support Undo." :
        app.simulation.isActive() ? "Stop simulation to edit these design positions." : "The project currently controls this machine.")));
      var linked = 0;
      for (axis in axes) {
        if (axis.follows != null) {linked++;continue;}
        var id=axis.id;
        var set = function(value:Float) {
          try {MachineControlModel.set(app.session,id,value,!app.simulation.isActive() && !app.documents.blocked());error=null;drafts.remove(id);app.documentChanged();}
          catch (failure:Dynamic) {error=Std.string(failure);app.invalidateView();}
        };
        var controls:Array<KeyedView> = [new KeyedView("label",new Text(id+" · "+number(axis.value)+" "+axis.unit))];
        var minimum=axis.minimum,maximum=axis.maximum;
        if (minimum != null && maximum != null && maximum > minimum) {
          var slider=new Slider("machine-axis-slider:"+id,id+" position ("+axis.unit+")",axis.value,minimum,maximum,Math.min(axis.step,(maximum-minimum)/1000),set);
          slider.enabled=editable;controls.push(new KeyedView("slider",slider));
          controls.push(new KeyedView("range",new Text(number(minimum)+" … "+number(maximum)+" "+axis.unit)));
        }
        var field=new TextField("machine-axis-value:"+id,drafts.exists(id) ? drafts.get(id) : number(axis.value),
          value -> drafts.set(id,value));
        field.label="Position ("+axis.unit+")";field.enabled=editable;field.style.width=LayoutAxis.grow();
        controls.push(new KeyedView("value",field));
        var buttons:Array<KeyedView> = [];
        function button(key:String,label:String,value:Float):Void {
          var b=new Button(label,null,()->set(value),"machine-axis:"+id+":"+key);b.enabled=editable;
          buttons.push(new KeyedView(key,b));
        }
        var applyPosition = function() {
          if (!editable) return;
          try {set(MachineControlModel.parsePosition(drafts.exists(id) ? drafts.get(id) : number(axis.value)));}
          catch (failure:Dynamic) {error=Std.string(failure);app.invalidateView();}
        };
        field.onSubmit = _ -> applyPosition();
        var apply=new Button("Set",null,applyPosition,"machine-axis:"+id+":set");
        apply.enabled=editable;buttons.push(new KeyedView("set",apply));
        if (minimum != null) button("min","Min",minimum);
        button("initial","Initial",axis.initial);
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
