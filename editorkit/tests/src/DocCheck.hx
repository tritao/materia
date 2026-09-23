import nativekit.editorkit.TextDocument;
import nativekit.editorkit.TextOffsetMap;

class DocCheck {
 static function check(ok:Bool, label:String):Void if (!ok) throw label;
 static function validate(doc:TextDocument, expected:String, step:Int):Void {
  var map=new TextOffsetMap(expected);
  check(doc.text==expected,"text at "+step);
  check(doc.codepointCount==map.codepointCount,"code points at "+step);
  check(doc.utf8ByteLength==map.utf8ByteLength,"UTF-8 length at "+step);
  check(doc.utf16Length==map.utf16Length,"UTF-16 length at "+step);
  check(doc.paragraphCount()==map.paragraphCount(),"paragraph count at "+step);
  for (i in 0...doc.paragraphCount()) {
   var a=doc.paragraphRangeAtIndex(i); var b=map.paragraphRangeAtIndex(i);
   check(a.start==b.start&&a.end==b.end,"paragraph "+i+" at "+step);
  }
  var position=0;
  while (position<=doc.codepointCount) {
   var u8=doc.utf8OffsetForCodepoint(position);
   var u16=doc.utf16OffsetForCodepoint(position);
   check(u8==map.utf8OffsetForCodepoint(position)&&u16==map.utf16OffsetForCodepoint(position),"forward coordinate at "+step+":"+position);
   check(doc.codepointOffsetForUtf8(u8)==position&&doc.codepointOffsetForUtf16(u16)==position,"inverse coordinate at "+step+":"+position);
   check(doc.paragraphIndexAtOffset(position)==map.paragraphIndexAtOffset(position),"paragraph lookup at "+step+":"+position);
   position+=137;
  }
  check(doc.sliceCodepoints(0,doc.codepointCount)==expected,"full slice at "+step);
 }
 static function main():Void {
  var content=new StringBuf();
  for (i in 0...260) content.add(StringTools.lpad(Std.string(i),"0",4)+" Café e\u0301 🙂 🧑‍💻 漢字 שלום\n");
  var expected=content.toString(); var doc=new TextDocument(expected);
  validate(doc,expected,0);
  for (step in 1...81) {
   var map=new TextOffsetMap(expected);
   var start=(step*7919)%(doc.codepointCount+1);
   var end=Std.int(Math.min(doc.codepointCount,start+step%5));
   var insert=switch(step%5) {case 0:"\n";case 1:"🧑‍💻";case 2:"e\u0301";case 3:"";case _:"漢🙂";};
   var next=map.replaceCodepoints(start,end,insert);
   doc.replace(start,end,insert);
   expected=next;
   validate(doc,expected,step);
  }
  Sys.println("PASS Unicode document coordinates and 80 edits");
 }
}
