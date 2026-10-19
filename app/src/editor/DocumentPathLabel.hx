package app.editor;

import haxeon.editor.TextDocument;
import haxeon.ui.Insets;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutPositioning;
import haxeon.ui.LayoutStyle;
import haxeon.ui.LayoutVisualKind;
import haxeon.ui.ParagraphStyle;
import haxeon.ui.TextLayout;
import haxeon.ui.TextWrap;
import haxeon.ui.core.*;
import haxeon.ui.semantics.*;
import haxeon.ui.theme.TextRole;
import haxeon.ui.widgets.text.Text;

/** Uses the remaining file-bar width; only an overflowing path needs an overlay. */
class DocumentPathLabel implements View {
  final value:String;
  public var truncated(default,null)=false;
  public function new(value:String) this.value=value;

  public static function documentPath(session:app.ProjectDocumentSession):String {
    var path=session.path==null ? session.projectReference : session.path;
    return (path==null ? "Untitled" : path)+(session.isDirty() ? " *" : "");
  }

  public function build(context:BuildContext):RenderNode return context.withScope(new Key("document-path"),()->{
    var style=new LayoutStyle();style.width=LayoutAxis.grow();style.height=LayoutAxis.fit();style.clipToParent=false;
    var root=new RenderNode(context.id("anchor"),LayoutVisualKind.Box,style);
    root.focusable=true;root.semantics=new Semantics(AccessibilityRole.Text,"Document path",value);
    var interaction=context.state(context.id("interaction"),{hovered:false,focused:false});
    var displayed=context.state(context.id("displayed"),value);
    var textStyle=new LayoutStyle();textStyle.width=LayoutAxis.grow();textStyle.clipHorizontal=true;
    var textOverride=TextStyleOverride.text(12);
    var inlineText=new Text(displayed.value,textStyle,context.theme.tokens.textSecondary,
      TextStyleOverride.combine(textOverride,TextStyleOverride.paragraph(TextWrap.None))).build(context);
    root.add(inlineText);
    var popupStyle=new LayoutStyle();popupStyle.positioning=LayoutPositioning.Absolute;
    popupStyle.positionY=24;popupStyle.zIndex=30;popupStyle.clipToParent=false;
    popupStyle.visible=false;popupStyle.padding=new Insets(8,6,8,6);
    popupStyle.background=context.theme.tokens.surface;popupStyle.radiusTopLeft=popupStyle.radiusTopRight=4;
    popupStyle.radiusBottomLeft=popupStyle.radiusBottomRight=4;
    var popup=new RenderNode(context.id("overlay"),LayoutVisualKind.Box,popupStyle);
    popup.hitTestBehavior=HitTestBehavior.None;
    popup.semantics=new Semantics(AccessibilityRole.Text,"Full document path",value);
    var popupTextStyle=new LayoutStyle();popupTextStyle.width=LayoutAxis.grow();
    popup.add(context.withScope(new Key("full-path"),()->new Text(value,popupTextStyle,null,
      TextStyleOverride.combine(textOverride,TextStyleOverride.paragraph(TextWrap.WordCharacter))).build(context)));
    root.add(popup);
    root.on(UiEventKind.HoverEnter,event->{if(event.target.equals(root.id))interaction.update({hovered:true,focused:interaction.value.focused});});
    root.on(UiEventKind.HoverLeave,event->{if(event.target.equals(root.id))interaction.update({hovered:false,focused:interaction.value.focused});});
    root.on(UiEventKind.Focus,_->interaction.update({hovered:interaction.value.hovered,focused:true}));
    var blur=function(_:UiEvent):Void {interaction.update({hovered:interaction.value.hovered,focused:false});};
    root.on(UiEventKind.Blur,blur);root.on(UiEventKind.FocusLost,blur);
    root.on(UiEventKind.KeyDown,event->{if(event.key==UiKey.Escape && popup.layout.style.visible){interaction.update({hovered:false,focused:false});event.preventDefault();event.stopPropagation();}});
    var resolved=context.resolveTextRole(TextRole.Body,textOverride);
    var memo=context.resourceState(context.id("measurement"),function() return {value:"",width:-1.0,result:"",fullWidth:0.0,font:resolved.textStyle.font,fontSize:resolved.textStyle.fontSize},_ -> {});
    inlineText.onResolved(geometry->{
      if(context.fonts==null)return;
      var available=Math.max(0.0,geometry.clippedViewportBounds().width),cache=memo.value;
      if(cache.value!=value || cache.width!=available || cache.font!=resolved.textStyle.font || cache.fontSize!=resolved.textStyle.fontSize) {
        var paragraph=new ParagraphStyle(TextWrap.None,resolved.paragraphStyle.alignment,resolved.paragraphStyle.lineHeight,
          resolved.paragraphStyle.direction,resolved.paragraphStyle.tabWidth);
        var layout=TextLayout.createStyled(context.fonts,value,100000,resolved.textStyle,paragraph);
        var fullWidth=layout.measure().width,next=value;
        if(fullWidth>available) {
          layout.setText("…");next="";
          if(layout.measure().width<=available) {
            var document=new TextDocument(value),low=0,high=document.codepointCount;
            while(low<high) {
              var count=(low+high+1)>>1,candidate="…"+document.sliceCodepoints(document.codepointCount-count,document.codepointCount);
              layout.setText(candidate);if(layout.measure().width<=available)low=count;else high=count-1;
            }
            next="…"+document.sliceCodepoints(document.codepointCount-low,document.codepointCount);
          }
        }
        layout.dispose();cache={value:value,width:available,result:next,fullWidth:fullWidth,font:resolved.textStyle.font,fontSize:resolved.textStyle.fontSize};memo.update(cache);
      }
      truncated=cache.result!=value;
      if(displayed.value!=cache.result)displayed.update(cache.result);
      if(inlineText.layout.text!=cache.result){inlineText.layout.text=cache.result;context.requestLayoutFeedback();}
      var bounds=root.globalBounds(),popupWidth=Math.min(cache.fullWidth+16,Math.max(1,context.environment.width-16));
      var popupX=Math.max(8-bounds.x,Math.min(0,context.environment.width-8-bounds.x-popupWidth));
      var show=truncated && (interaction.value.hovered || interaction.value.focused);
      if(popupStyle.visible!=show || popupStyle.width.value!=popupWidth || popupStyle.positionX!=popupX) {
        popupStyle.visible=show;popupStyle.width=LayoutAxis.fixed(popupWidth);popupStyle.positionX=popupX;context.requestLayoutFeedback();
      }
    });
    return root;
  });
}
