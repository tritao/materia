package tests;

import app.editor.DocumentPathLabel;
import haxeon.ui.LayoutFrame;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.UiContext;
import haxeon.ui.core.UiEventKind;
import haxeon.ui.core.UiKey;

@:access(app.Main.ReferenceEditorApp)
@:access(app.ProjectDocumentSession)
class DocumentPathTests {
  static var checks=0;
  static function check(value:Bool,message:String):Void {checks++;if(!value)throw message;}
  static function find(root:RenderNode,label:String):RenderNode {
    var found=search(root,label);if(found==null)throw "Missing "+label;return found;
  }
  static function search(root:RenderNode,label:String):Null<RenderNode> {
    if(root.semantics!=null && root.semantics.label==label)return root;
    for(child in root.children){var found=search(child,label);if(found!=null)return found;}
    return null;
  }
  static function toolbarLayout(fonts:haxeon.ui.FontCollection):Void {
    var editor=new app.Main.ReferenceEditorApp(fonts,null,null,null,null,null,null,null,true);
    try {
      var source="/home/user/work/machines/hobber/no-guards-mechanical-preview.mtrg";
      editor.session.projectReference=source;
      check(DocumentPathLabel.documentPath(editor.session)==source,"generated scenes display their source path");
      var root=editor.ui.submit(editor.fileBar(false),new LayoutFrame(1600,36));
      var path=find(root,"Document path");
      check(path.globalBounds().width>600,"document path receives unused toolbar width");
      check(path.children[0].layout.text==source,"wide toolbar shows the complete path inline");
      root=editor.ui.submit(editor.fileBar(true),new LayoutFrame(500,36));path=find(root,"Document path");
      var text=path.children[0].layout.text;
      check(text!=null && StringTools.startsWith(text,"…"),"compact toolbar truncates by measured width");
      var more=find(root,"More editor actions").globalBounds();
      check(more.x+more.width<=500.1,"long paths do not push toolbar actions outside the window");
      editor.session.path="/home/user/saved/machine.materia";
      check(DocumentPathLabel.documentPath(editor.session)==editor.session.path,"saved document path takes precedence over source path");
      editor.dispose();
    }catch(error:Dynamic){editor.dispose();throw error;}
  }
  public static function main():Int {
    var fonts=haxeon.ui.FontCollection.create(),path="haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf";
    if(!sys.FileSystem.exists(path))path="../../"+path;fonts.add(path);
    var ui=new UiContext(null,fonts);
    var value="/home/user/work/machines/hobber/no-guards-mechanical-preview.mtrg";
    function submit(width:Float):RenderNode return ui.submit(new DocumentPathLabel(value),new LayoutFrame(width,200));
    try {
      var root=submit(1100),anchor=find(root,"Document path"),overlay=find(root,"Full document path");
      check(anchor.children[0].layout.text==value,"full path uses available width without a character limit");
      ui.pointerMove(100,8);root=submit(1100);
      check(!find(root,"Full document path").layout.style.visible,"fitting paths do not show a hover overlay");
      root=submit(230);anchor=find(root,"Document path");
      var shown=anchor.children[0].layout.text;
      check(shown!=null && StringTools.startsWith(shown,"…"),"narrow paths truncate from the left");
      check(shown!=null && StringTools.endsWith(shown,"preview.mtrg"),"truncation preserves the filename end");
      overlay=find(root,"Full document path");
      check(overlay.layout.style.visible,"hover reveals full path only after it overflows");
      var bounds=overlay.globalBounds();
      check(bounds.x>=7.9 && bounds.x+bounds.width<=222.1,"overlay fits within the window");
      ui.pointerMove(-20,-20);root=submit(230);
      check(!find(root,"Full document path").layout.style.visible,"overlay closes when pointer leaves");
      anchor=find(root,"Document path");ui.focusWidget(anchor.id);root=submit(230);
      check(find(root,"Full document path").layout.style.visible,"keyboard focus reveals an overflowing path");
      ui.key(UiEventKind.KeyDown,UiKey.Escape);root=submit(230);
      check(!find(root,"Full document path").layout.style.visible,"Escape dismisses the overlay");
      root=submit(1100);anchor=find(root,"Document path");
      check(anchor.children[0].layout.text==value,"resize restores the complete inline path");
      check(!find(root,"Full document path").layout.style.visible,"wide resize removes the overlay");
      var session=new app.ProjectDocumentSession(null,false);
      check(DocumentPathLabel.documentPath(session)=="Untitled","new documents retain their untitled label");session.dispose();
      toolbarLayout(fonts);
      ui.dispose();fonts.dispose();
    }catch(error:Dynamic){ui.dispose();fonts.dispose();throw error;}
    Sys.println('Document path: $checks checks passed');return 0;
  }
}
