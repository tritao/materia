package app.editor;

import app.EditorPerspectiveViewport;
import haxeon.ui.core.View;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.controls.Select;
import haxeon.ui.widgets.controls.SelectOption;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.layout.Column;

/** Shared controls for the main viewport, dock and picture-in-picture. */
class PresentationControls {
  public static function build(viewport:EditorPerspectiveViewport,changed:Void->Void,key:String,
      ?save:Void->Void):View {
    var controller=viewport.presentation;
    var cells:Array<KeyedView> = [];
    var rows:Array<KeyedView> = [];
    if(controller!=null) {
      var options=[for(view in controller.record.views)new SelectOption<String>(view.id,view.label,view.id)];
      var selectStyle=new LayoutStyle();selectStyle.width=LayoutAxis.grow();
      var select=new Select<String>(key+"-views",options,controller.cursor.view,
        id->{viewport.selectPresentationView(id);changed();},selectStyle);
      select.accessibilityLabel="Assembly view";
      rows.push(new KeyedView("views",select));
    }
    cells.push(new KeyedView("reset",new Button("Reset view",null,()->{viewport.resetPresentation();changed();},key+"-reset")));
    if(save!=null)cells.push(new KeyedView("save",new Button("Save current view",null,save,key+"-save")));
    var style=new LayoutStyle();style.width=LayoutAxis.grow();style.childGap=4;
    rows.push(new KeyedView("actions",new Row(key+"-actions",cells,style)));
    return new Column(key,rows,style);
  }
}
