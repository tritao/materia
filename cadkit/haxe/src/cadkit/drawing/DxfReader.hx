package cadkit.drawing;

import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.drawing.DrawingEntity;

typedef DxfCurve = {layer:String, segment:Segment, entity:String};
typedef DxfLabel = {layer:String, position:Point2, height:Float, text:String, angle:Float};
typedef DxfReadOptions = {
  /** Output is millimetres. Overrides $INSUNITS; unitless R12 defaults to 1 with assumedUnits=true. */
  @:optional var millimetresPerUnit:Float;
  @:optional var tolerance:Float;
  @:optional var allowUnsupported:Bool;
}
typedef DxfDocument = {
  curves:Array<DxfCurve>, labels:Array<DxfLabel>, warnings:Array<String>,
  millimetresPerUnit:Float, assumedUnits:Bool, tolerance:Float
}
private typedef Pair = {code:Int,value:String,line:Int};
private typedef Entity = {kind:String, fields:Array<Pair>, vertices:Array<Entity>, line:Int};
private typedef Block = {base:Point2,entities:Array<Entity>};
private typedef Transform = {a:Float,b:Float,c:Float,d:Float,x:Float,y:Float};

/** Planar ASCII DXF import. Unsupported cutting entities fail unless the caller requests diagnostics. */
class DxfReader {
  static inline final TAU=6.283185307179586;
  static function value(e:Entity,code:Int,?fallback:String):String {
    for(p in e.fields)if(p.code==code)return p.value;
    if(fallback==null)throw 'DXF ${e.kind} at line ${e.line} lacks group $code';
    return fallback;
  }
  static function number(e:Entity,code:Int,?fallback:Float):Float {
    var raw=value(e,code,fallback==null?null:Std.string(fallback));
    return numeric(raw,e.line);
  }
  static function numeric(raw:String,line:Int):Float {
    var n=Std.parseFloat(raw);
    if(!~/^[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?$/.match(StringTools.trim(raw))||!Math.isFinite(n))
      throw 'DXF invalid number at line $line';
    return n;
  }
  static function integer(e:Entity,code:Int,fallback:Int):Int {
    var n=number(e,code,fallback);
    if(n!=Math.floor(n)||Math.abs(n)>2147483647)throw 'DXF invalid integer at line ${e.line}';
    return Std.int(n);
  }
  static function point(e:Entity,base:Int):Point2 {
    if(Math.abs(number(e,base+20,0))>1e-9)throw 'DXF nonplanar coordinate at line ${e.line}';
    return new Point2(number(e,base,0),number(e,base+10,0));
  }
  static function apply(t:Transform,p:Point2):Point2 return new Point2(t.a*p.x+t.b*p.y+t.x,t.c*p.x+t.d*p.y+t.y);
  static function compose(t:Transform,s:Transform):Transform return {
    a:t.a*s.a+t.b*s.c,b:t.a*s.b+t.b*s.d,c:t.c*s.a+t.d*s.c,d:t.c*s.b+t.d*s.d,
    x:t.a*s.x+t.b*s.y+t.x,y:t.c*s.x+t.d*s.y+t.y};
  static function extrusion(e:Entity):Transform {
    if(Math.abs(number(e,210,0))>1e-12||Math.abs(number(e,220,0))>1e-12||Math.abs(Math.abs(number(e,230,1))-1)>1e-12)
      throw 'DXF tilted extrusion is not a sheet at line ${e.line}';
    return {a:number(e,230,1)<0?-1:1,b:0,c:0,d:1,x:0,y:0};
  }
  static function emit(out:DxfDocument,t:Transform,segment:Segment,layer:String,source:String):Void {
    function push(s:Segment):Void {
      if(out.curves.length>=1000000)throw "DXF expanded curve limit exceeded";
      out.curves.push({layer:layer,segment:s,entity:source});
    }
    if(segment.isLine()) {push(Segment.line(apply(t,segment.start),apply(t,segment.end)));return;}
    var u=t.a*t.a+t.c*t.c,v=t.b*t.b+t.d*t.d,dot=t.a*t.b+t.c*t.d;
    if(Math.abs(u-v)<=1e-12*Math.max(u,v)&&Math.abs(dot)<=1e-12*Math.max(u,v)) {
      var center=apply(t,segment.arcCenter()),start=apply(t,segment.start);
      push(Segment.arc(center,segment.radius*Math.sqrt(u),Math.atan2(start.y-center.y,start.x-center.x),segment.sweep*(t.a*t.d-t.b*t.c<0?-1:1)));
    } else {
      // ||C''|| <= radius * ||A||F; linear interpolation error <= bound * dt²/8.
      var bound=segment.radius*Math.sqrt(u+v);
      var count=Std.int(Math.ceil(Math.abs(segment.sweep)*Math.sqrt(bound/(8*out.tolerance))));
      count=Std.int(Math.max(2,count));if(count>1000000)throw "DXF ellipse subdivision limit exceeded";
      var previous=apply(t,segment.start);
      for(i in 1...count+1) {var next=apply(t,segment.point(i/count));push(Segment.line(previous,next));previous=next;}
    }
  }
  static function bulge(a:Point2,b:Point2,value:Float):Segment {
    if(Math.abs(value)<1e-14)return Segment.line(a,b);
    var chord=a.distance(b);if(chord<=1e-9)throw "DXF bulge has coincident vertices";
    var offset=(1-value*value)/(4*value);
    var center=new Point2((a.x+b.x)/2-(b.y-a.y)*offset,(a.y+b.y)/2+(b.x-a.x)*offset);
    return Segment.arc(center,chord*(1+value*value)/(4*Math.abs(value)),Math.atan2(a.y-center.y,a.x-center.x),4*Math.atan(value));
  }
  static function polyline(out:DxfDocument,e:Entity,t:Transform,layer:String):Void {
    var vertices:Array<{p:Point2,bulge:Float}> = [];
    var flags=integer(e,70,0);
    if((flags&~129)!=0)throw 'DXF unsupported fitted/3D/polyface polyline at line ${e.line}';
    if(e.kind=="LWPOLYLINE") {
      if(Math.abs(number(e,38,0))>1e-9)throw "DXF nonzero polyline elevation";
      var current:Null<Entity> = null;
      for(p in e.fields) {
        if(p.code==10) {
          if(current!=null)vertices.push({p:point(current,10),bulge:number(current,42,0)});
          current={kind:"VERTEX",fields:[],vertices:[],line:p.line};
        }
        if(current!=null&&[10,20,30,40,41,42].indexOf(p.code)>=0)current.fields.push(p);
      }
      if(current!=null)vertices.push({p:point(current,10),bulge:number(current,42,0)});
      if(integer(e,90,vertices.length)!=vertices.length)throw "DXF polyline vertex count mismatch";
    } else {
      point(e,10);
      for(vertex in e.vertices) {
        if(integer(vertex,70,0)!=0)throw "DXF only ordinary 2D polyline vertices are supported";
        vertices.push({p:point(vertex,10),bulge:number(vertex,42,0)});
      }
    }
    if(vertices.length<2)throw "DXF polyline needs two vertices";
    var closed=(flags&1)!=0;
    for(i in 0...(closed?vertices.length:vertices.length-1)) {
      var a=vertices[i],b=vertices[(i+1)%vertices.length];
      emit(out,t,bulge(a.p,b.p,a.bulge),layer,e.kind+":"+e.line);
    }
  }
  static function expand(out:DxfDocument,entities:Array<Entity>,blocks:Map<String,Block>,t:Transform,inherited:String,stack:Array<String>):Void {
    for(e in entities) {
      var layer=value(e,8,"0");if(layer=="0")layer=inherited;
      var id=e.kind+":"+e.line;
      switch(e.kind) {
        case "LINE":emit(out,t,Segment.line(point(e,10),point(e,11)),layer,id);
        case "ARC","CIRCLE":
          var center=point(e,10),radius=number(e,40),start=e.kind=="CIRCLE"?0:number(e,50)*Math.PI/180;
          var sweep=e.kind=="CIRCLE"?TAU:(number(e,51)*Math.PI/180-start)%TAU;
          if(sweep<=0)sweep+=TAU;
          emit(out,compose(t,extrusion(e)),Segment.arc(center,radius,start,sweep),layer,id);
        case "LWPOLYLINE","POLYLINE":polyline(out,e,compose(t,extrusion(e)),layer);
        case "INSERT":
          var name=value(e,2),block=blocks.get(name);
          if(block==null||stack.indexOf(name)>=0||stack.length>=32)throw 'DXF unresolved or recursive block "$name" at ${e.line}';
          var origin=point(e,10),angle=number(e,50,0)*Math.PI/180,sx=number(e,41,1),sy=number(e,42,1);
          if(sx==0||sy==0)throw "DXF singular INSERT scale";
          var columns=integer(e,70,1),rows=integer(e,71,1);
          if(columns<1||rows<1||columns*rows>100000)throw "DXF INSERT array limit exceeded";
          var c=Math.cos(angle),s=Math.sin(angle),base:Transform={a:c*sx,b:-s*sy,c:s*sx,d:c*sy,x:origin.x,y:origin.y};
          for(row in 0...rows)for(column in 0...columns) {
            // Array spacing is rotated with the insert, independent of its scale.
            var item:Transform={a:base.a,b:base.b,c:base.c,d:base.d,
              x:base.x+c*column*number(e,44,0)-s*row*number(e,45,0)-base.a*block.base.x-base.b*block.base.y,
              y:base.y+s*column*number(e,44,0)+c*row*number(e,45,0)-base.c*block.base.x-base.d*block.base.y};
            expand(out,block.entities,blocks,compose(t,compose(extrusion(e),item)),layer,stack.concat([name]));
          }
        case "SPLINE":
          var points:Array<Point2> = [],knots:Array<Float> = [],weights:Array<Float> = [],current:Null<Entity> = null;
          for(p in e.fields) {
            if(p.code==40)knots.push(numeric(p.value,p.line));
            if(p.code==41)weights.push(numeric(p.value,p.line));
            if(p.code==10) {
              if(current!=null)points.push(point(current,10));
              current={kind:"CONTROL",fields:[],vertices:[],line:p.line};
            }
            if(current!=null&&[10,20,30].indexOf(p.code)>=0)current.fields.push(p);
          }
          if(current!=null)points.push(point(current,10));
          if(integer(e,72,knots.length)!=knots.length||integer(e,73,points.length)!=points.length)throw "DXF spline control count mismatch";
          if(weights.length==0)weights=[for(_ in points)1.0];
          var norm=Math.sqrt(t.a*t.a+t.b*t.b+t.c*t.c+t.d*t.d);
          for(segment in DxfSpline.sample(points,weights,knots,integer(e,71,3),out.tolerance/norm))emit(out,t,segment,layer,id);
        case "TEXT":
          var height=number(e,40),angle=number(e,50,0)*Math.PI/180;
          var textFrame=compose(t,extrusion(e));
          out.labels.push({layer:layer,position:apply(textFrame,point(e,10)),height:height*Math.sqrt(textFrame.a*textFrame.a+textFrame.c*textFrame.c),
            text:value(e,1,""),angle:Math.atan2(textFrame.c,textFrame.a)+angle});
        case "ATTRIB","ATTDEF":out.warnings.push('Ignored block annotation ${e.kind} at line ${e.line}');
        case "SEQEND":
        case _:out.warnings.push('Unsupported DXF ${e.kind} at line ${e.line}');
      }
    }
  }
  public static function read(source:String,?options:DxfReadOptions):DxfDocument {
    if(source==null||source.length>64000000)throw "DXF input exceeds 64 MB";
    var lines=source.split("\n"),pairs:Array<Pair> = [];
    while(lines.length>0&&StringTools.trim(lines[lines.length-1])=="")lines.pop();
    if(lines.length%2!=0)throw "DXF must contain code/value pairs";
    for(i in 0...Std.int(lines.length/2)) {
      var codeText=StringTools.trim(lines[i*2]),code=Std.parseInt(codeText);
      if(code==null||!~/^[0-9]+$/.match(codeText))throw 'Invalid DXF group at line ${i*2+1}';
      pairs.push({code:code,value:StringTools.trim(lines[i*2+1]),line:i*2+1});
    }
    var cursor=0;
    function record():Entity {
      var head=pairs[cursor++];if(head.code!=0)throw 'DXF entity expected at line ${head.line}';
      var fields:Array<Pair> = [];
      while(cursor<pairs.length&&pairs[cursor].code!=0)fields.push(pairs[cursor++]);
      return {kind:head.value,fields:fields,vertices:[],line:head.line};
    }
    var entities:Array<Entity> = [],blocks:Map<String,Block> = new Map(),section="",blockName:Null<String> = null,blockBase=new Point2(0,0),blockEntities:Array<Entity> = [],units=0,eof=false;
    while(cursor<pairs.length) {
      if(pairs[cursor].code==9&&section=="HEADER") {
        var variable=pairs[cursor++].value;
        if(variable=="$INSUNITS"&&cursor<pairs.length) {
          var raw=pairs[cursor++];var n=Std.parseInt(raw.value);
          if(raw.code!=70||n==null||!~/^[0-9]+$/.match(raw.value))throw "Invalid $INSUNITS";
          units=n;
        } else while(cursor<pairs.length&&pairs[cursor].code!=0&&pairs[cursor].code!=9)cursor++;
        continue;
      }
      if(pairs[cursor].code==999){cursor++;continue;}
      var e=record();
      switch(e.kind) {
        case "SECTION":
          section=value(e,2);
          if(section=="HEADER")for(i in 0...e.fields.length-1)if(e.fields[i].code==9&&e.fields[i].value=="$INSUNITS") {
            var raw=e.fields[i+1],n=Std.parseInt(raw.value);
            if(raw.code!=70||n==null||!~/^[0-9]+$/.match(raw.value))throw "Invalid $INSUNITS";
            units=n;
          }
        case "ENDSEC":section="";
        case "EOF":eof=true;
        case "BLOCK":
          if(section!="BLOCKS"||blockName!=null)throw "Malformed DXF BLOCK";
          blockName=value(e,2);blockBase=point(e,10);blockEntities=[];
        case "ENDBLK":
          if(blockName==null||blocks.exists(blockName))throw "Malformed or duplicate DXF BLOCK";
          blocks.set(blockName,{base:blockBase,entities:blockEntities});blockName=null;
        case _:
          if(section!="ENTITIES"&&section!="BLOCKS")continue;
          if(e.kind=="POLYLINE") {
            while(cursor<pairs.length&&pairs[cursor].value=="VERTEX")e.vertices.push(record());
            if(cursor>=pairs.length||pairs[cursor].value!="SEQEND")throw "DXF POLYLINE lacks SEQEND";
            record();
          }
          if(section=="ENTITIES")entities.push(e);else if(blockName!=null)blockEntities.push(e);
      }
    }
    if(!eof||blockName!=null)throw "Incomplete DXF document";
    var scales:Map<Int,Float> = [1=>25.4,2=>304.8,3=>1609344.0,4=>1.0,5=>10.0,6=>1000.0,7=>1000000.0,8=>0.0000254,9=>0.0254,10=>914.4,11=>1e-7,12=>1e-6,13=>0.001,14=>100.0,15=>10000.0,16=>100000.0];
    var unitOverride=options==null?null:options.millimetresPerUnit;
    if(unitOverride==null&&units!=0&&!scales.exists(units))throw 'Unsupported DXF unit code $units; supply millimetresPerUnit';
    var scale:Float=1.0;
    if(unitOverride!=null)scale=unitOverride;else if(units!=0)scale=cast scales.get(units);
    var tolerance:Float=options==null||options.tolerance==null?0.01:options.tolerance;
    if(!Math.isFinite(scale)||scale<=0||!Math.isFinite(tolerance)||tolerance<=0)throw "Invalid DXF unit scale or tolerance";
    var out:DxfDocument={curves:[],labels:[],warnings:[],millimetresPerUnit:scale,assumedUnits:units==0&&unitOverride==null,tolerance:tolerance};
    expand(out,entities,blocks,{a:scale,b:0,c:0,d:scale,x:0,y:0},"0",[]);
    if([for(w in out.warnings)if(StringTools.startsWith(w,"Unsupported DXF"))w].length>0&&(options==null||options.allowUnsupported!=true))throw out.warnings.join("; ");
    return out;
  }
  public static function drawing(document:DxfDocument):Drawing2D {
    function layer(name:String):DrawingLayer return switch(name) {
      case "OUTLINE":Outline;case "ETCH":Etch;case "BEND_UP":BendUp;case "BEND_DOWN":BendDown;case "TEXT":Text;
      case _:Process(name);
    };
    var entities:Array<DrawingEntity> = [for(curve in document.curves)Curve(layer(curve.layer),curve.segment)];
    for(label in document.labels) {
      if(Math.abs(label.angle)>1e-10)throw "Drawing2D cannot represent rotated DXF text";
      entities.push(Label(layer(label.layer),label.position,label.height,label.text));
    }
    return new Drawing2D(entities);
  }
}
