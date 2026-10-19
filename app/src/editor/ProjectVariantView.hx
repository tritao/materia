package app.editor;

import app.ProjectVariantController;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutAlignmentY;
import haxeon.ui.LayoutStyle;
import haxeon.ui.TextWrap;
import haxeon.ui.core.TextStyleOverride;
import haxeon.ui.core.View;
import haxeon.ui.theme.ThemeTokens;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.controls.Select;
import haxeon.ui.widgets.controls.Spinner;
import haxeon.ui.widgets.controls.SelectOption;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.text.Text;

/** Project scene selection; this view has no mission progress or playback controls. */
class ProjectVariantView {
  public static function build(controller:ProjectVariantController,tokens:ThemeTokens,key:String):View {
    var state=controller.snapshot(),rows:Array<KeyedView> = [];
    for(group in ["configuration","study","demo","job"]) {
      var variants=state.variants.filter(v->v.kind==group);if(variants.length==0)continue;
      var label=switch(group){case "configuration":"Configuration";case "study":"Study";case "demo":"Demo";default:"Scene preset";};
      var active=variants.filter(v->v.id==state.selected).length==1;
      var options=[new SelectOption<String>("choose", "Choose "+label.toLowerCase()+"…", "")];
      for(variant in variants)options.push(new SelectOption<String>("variant-"+variant.id,variant.label,variant.id));
      var style=new LayoutStyle();style.width=LayoutAxis.grow();
      var select=new Select<String>(key+"-"+group,options,active?state.selected:"",function(id) {
        if(id!="")controller.select(id);
      },style);
      select.accessibilityLabel=label;select.enabled=state.switchAllowed;
      rows.push(new KeyedView(group+"-label",new Text(label,null,tokens.textSecondary)));
      rows.push(new KeyedView(group,select));
    }
    if(state.summary!="")rows.push(new KeyedView("summary",new Text(state.summary,null,tokens.textSecondary,
      TextStyleOverride.paragraph(TextWrap.WordCharacter))));
    if(state.simulationActive)rows.push(new KeyedView("locked",new Text("Stop simulation to change the project selection.",null,tokens.textSecondary)));
    if(state.loading) {
      var progressStyle=new LayoutStyle();progressStyle.width=LayoutAxis.grow();
      progressStyle.childAlignY=LayoutAlignmentY.Center;progressStyle.childGap=8;
      rows.push(new KeyedView("progress",new Row(key+"-progress",[
        new KeyedView("spinner",new Spinner(key+"-spinner","Loading project selection")),
        new KeyedView("status",new Text(state.status,null,tokens.accent,
          TextStyleOverride.paragraph(TextWrap.WordCharacter)))
      ],progressStyle)));
      rows.push(new KeyedView("cancel",new Button("Cancel",null,controller.cancel,key+"-cancel")));
    }
    if(state.error!=null)rows.push(new KeyedView("error",new Text(state.error,null,tokens.danger)));
    if(state.variants.length==0&&state.error==null)rows.push(new KeyedView("empty",new Text("This scene has no source project variants.")));
    var style=new LayoutStyle();style.width=LayoutAxis.grow();style.childGap=6;
    return new Column(key,rows,style);
  }
}
