import cadkit.drawing.*;
import cadkit.drawing.DxfReader.DxfCurve;
import cadkit.sheetmetal.*;

class DxfImportChecks {
  static var assertions=0;
  static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
  static function near(a:Float,b:Float,label:String,tol:Float=1e-8):Void check(Math.abs(a-b)<=tol,label);
  static function rejects(action:Void->Void,label:String):Void {var failed=false;try action()catch(_:Dynamic)failed=true;check(failed,label);}
  static function document(entities:String,header:String="",blocks:String=""):String
    return '0\nSECTION\n2\nHEADER\n'+header+'0\nENDSEC\n'+(blocks==""?"":'0\nSECTION\n2\nBLOCKS\n'+blocks+'0\nENDSEC\n')+'0\nSECTION\n2\nENTITIES\n'+entities+'0\nENDSEC\n0\nEOF\n';
  static function rectangle(x:Float,y:Float,w:Float,h:Float,layer:String):Array<DxfCurve> {
    var p=[new Point2(x,y),new Point2(x+w,y),new Point2(x+w,y+h),new Point2(x,y+h)];
    return [for(i in 0...4){layer:layer,entity:'e$i',segment:Segment.line(p[i],p[(i+1)%4])}];
  }
  static function writer():Void {
    var entities:Array<DrawingEntity> = [for(c in rectangle(0,0,100,80,"OUTLINE"))Curve(Outline,c.segment)];
    entities.push(Curve(Outline,Segment.arc(new Point2(40,40),5,0,2*Math.PI)));
    entities.push(Curve(Etch,Segment.arc(new Point2(20,20),3,0.3,-1.7)));
    entities.push(Label(Text,new Point2(4,5),2,"cabinet"));
    var drawing=new Drawing2D(entities),read=Dxf.read(Dxf.write(drawing));
    check(read.curves.length==6&&read.labels.length==1&&read.assumedUnits,"R12 writer round trip and explicit unitless assumption");
    var output=DxfReader.drawing(read).entities();
    near(read.curves[5].segment.sweep,-1.7,"negative-Z OCS preserves clockwise sweep");
    near(read.curves[5].segment.start.distance(new Point2(20+3*Math.cos(0.3),20+3*Math.sin(0.3))),0,"clockwise arc start in WCS");
    check(output.length==entities.length&&read.labels[0].text=="cabinet","annotations survive round trip");
    var assembled=DxfContours.assemble([for(c in read.curves)if(c.layer=="OUTLINE")c]);
    check(assembled.parts.length==1&&assembled.parts[0].holes.length==1&&assembled.gaps.length==0,"circle nested as exact hole");
    near(assembled.parts[0].outline.area(),8000-25*Math.PI,"exact circular hole area");
  }
  static function bulges():Void {
    var poly='0\nLWPOLYLINE\n8\nCUT\n90\n2\n70\n1\n10\n0\n20\n0\n42\n1\n10\n20\n20\n0\n42\n1\n';
    var curves=Dxf.read(document(poly)).curves;
    check(curves.length==2&&!curves[0].segment.isLine(),"bulge remains an exact arc");
    near(curves[0].segment.radius,10,"bulge radius");near(curves[0].segment.sweep,Math.PI,"bulge sweep");
    near(DxfContours.assemble(curves).parts[0].outline.area(),100*Math.PI,"two bulges enclose a circle");
    var old='0\nPOLYLINE\n8\nCUT\n70\n1\n0\nVERTEX\n10\n0\n20\n0\n42\n1\n0\nVERTEX\n10\n20\n20\n0\n42\n1\n0\nSEQEND\n';
    near(DxfContours.assemble(Dxf.read(document(old)).curves).parts[0].outline.area(),100*Math.PI,"R12 POLYLINE bulges");
  }
  static function gaps():Void {
    var edges=rectangle(0,0,100,80,"CUT");
    edges[3].segment=Segment.line(new Point2(0,80),new Point2(0,0.02));
    var shuffled=[edges[2],edges[0],edges[3],edges[1]];
    var result=DxfContours.assemble(shuffled,0.05);
    check(result.parts.length==1&&result.gaps.length==0,"unordered contour closes its 0.02 mm gap");
    near(result.parts[0].outline.area(),8000,"gap bridge preserves boundary");
    result=DxfContours.assemble(shuffled,0.005);
    check(result.parts.length==0&&result.gaps.length==1,"larger gap remains an actionable diagnostic");
    near(result.gaps[0].distance,0.02,"reported gap distance");near(result.gaps[0].position.x,0,"reported gap location");
    var all=rectangle(0,0,100,100,"OUTER").concat(rectangle(10,10,20,20,"HOLES")).concat(rectangle(15,15,5,5,"OUTER"));
    var operations:Map<String,String> = ["OUTER"=>"profile","HOLES"=>"profile"];
    result=DxfContours.assemble(all,0.05,operations);
    check(result.parts.length==2&&result.parts[0].holes.length==1,"layer mapping permits holes and islands");
    check(result.parts[0].outer.operation=="profile"&&result.parts[0].holes[0].layer=="HOLES","source layer and operation both retained");
    rejects(()->DxfContours.assemble(rectangle(0,0,10,10,"CUT").concat(rectangle(5,5,10,10,"CUT"))),"intersecting contours refused");
  }
  static function inserts():Void {
    var block='0\nBLOCK\n2\nunit\n10\n1\n20\n1\n0\nCIRCLE\n8\n0\n10\n2\n20\n1\n40\n1\n0\nENDBLK\n';
    var insert='0\nINSERT\n8\nDRILL\n2\nunit\n10\n10\n20\n20\n41\n2\n42\n2\n50\n90\n70\n2\n44\n5\n';
    var d=Dxf.read(document(insert,"9\n$INSUNITS\n70\n1\n",block));
    check(!d.assumedUnits&&d.curves.length==2&&d.curves[0].layer=="DRILL","INSERT arrays, inherited layer and inch units");
    var arc=d.curves[0].segment;near(arc.radius,50.8,"INSERT scale and units");near(arc.arcCenter().x,254,"block base X");near(arc.arcCenter().y,558.8,"block base, rotation and translation");
    near(d.curves[1].segment.arcCenter().y,685.8,"rotated array spacing independent of scale");
    var ellipse=Dxf.read(document(StringTools.replace(insert,'42\n2\n','42\n1\n'),'',block),{tolerance:0.002});
    check(ellipse.curves.length>20&&ellipse.curves[0].segment.isLine(),"nonuniform insert yields tolerance-bounded ellipse chords");
    var nested=block+'0\nBLOCK\n2\nnested\n0\nINSERT\n2\nunit\n10\n3\n20\n4\n0\nENDBLK\n';
    var nestedRead=Dxf.read(document('0\nINSERT\n8\nCUT\n2\nnested\n10\n10\n20\n20\n','',nested));
    check(nestedRead.curves.length==1&&nestedRead.curves[0].layer=="CUT","nested INSERT layer inheritance");
    near(nestedRead.curves[0].segment.arcCenter().x,14,"nested INSERT transform composition");
    rejects(()->Dxf.read(document('0\nINSERT\n2\nmissing\n')),"missing block refused");
    var recursive='0\nBLOCK\n2\nloop\n0\nINSERT\n2\nloop\n0\nENDBLK\n';
    rejects(()->Dxf.read(document('0\nINSERT\n2\nloop\n','',recursive)),"recursive block refused");
  }
  static function deBoor(points:Array<Point2>,weights:Array<Float>,knots:Array<Float>,degree:Int,t:Float):Point2 {
    var k=degree;while(k<points.length-1&&knots[k+1]<=t)k++;
    var d=[for(i in k-degree...k+1)[points[i].x*weights[i],points[i].y*weights[i],weights[i]]];
    for(r in 1...degree+1)for(step in 0...degree-r+1) {
      var j=degree-step,i=k-degree+j,alpha=(t-knots[i])/(knots[i+degree-r+1]-knots[i]);
      for(axis in 0...3)d[j][axis]=(1-alpha)*d[j-1][axis]+alpha*d[j][axis];
    }
    return new Point2(d[degree][0]/d[degree][2],d[degree][1]/d[degree][2]);
  }
  static function splineOracle(points:Array<Point2>,weights:Array<Float>,knots:Array<Float>,degree:Int):Void {
    var segments=DxfSpline.sample(points,weights,knots,degree,0.001),worst=0.0;
    for(step in 0...501) {
      var t=knots[degree]+(knots[points.length]-knots[degree])*step/500;
      var p=deBoor(points,weights,knots,degree,t),nearest=Math.POSITIVE_INFINITY;
      for(s in segments) {
        var dx=s.end.x-s.start.x,dy=s.end.y-s.start.y;
        var f=Math.max(0,Math.min(1,((p.x-s.start.x)*dx+(p.y-s.start.y)*dy)/(dx*dx+dy*dy)));
        nearest=Math.min(nearest,p.distance(s.point(f)));
      }
      worst=Math.max(worst,nearest);
    }
    check(worst<=0.001+1e-10,"Bezier subdivision agrees with independent de Boor evaluation within tolerance");
  }
  static function splines():Void {
    var spline='0\nSPLINE\n8\nCUT\n70\n12\n71\n2\n72\n6\n73\n3\n40\n0\n40\n0\n40\n0\n40\n1\n40\n1\n40\n1\n41\n1\n41\n0.7071067811865476\n41\n1\n10\n1\n20\n0\n10\n1\n20\n1\n10\n0\n20\n1\n';
    var d=Dxf.read(document(spline),{tolerance:0.0005}),error=0.0;
    for(c in d.curves)for(i in 0...9){var p=c.segment.point(i/8);error=Math.max(error,Math.abs(Math.sqrt(p.x*p.x+p.y*p.y)-1));}
    check(d.curves.length>8&&error<=0.0005,"positive rational spline is within declared tolerance of the exact quarter circle");
    var points=[new Point2(0,0),new Point2(1,1),new Point2(2,0)];
    var samples=DxfSpline.sample(points,[1.0,1.0,1.0],[0.0,0.0,0.0,0.5,1.0,1.0],2,0.001);
    check(samples.length>1,"nonuniform spline knots are refined");
    var open=DxfSpline.sample(points,[1.0,1.0,1.0],[0.0,1.0,2.0,3.0,4.0,5.0],2,0.001);
    near(open[0].start.x,0.5,"unclamped spline domain start");near(open[open.length-1].end.x,1.5,"unclamped spline domain end");
    splineOracle(points,[1.0,1.0,1.0],[0.0,1.0,2.0,3.0,4.0,5.0],2);
    splineOracle(points,[1.0,0.7,1.0],[0.0,0.0,0.0,0.5,1.0,1.0],2);
    splineOracle([new Point2(0,0),new Point2(1,2),new Point2(3,-1),new Point2(5,1)],
      [1.0,0.8,1.3,1.0],[0.0,0.0,0.0,0.4,1.0,1.0,1.0],2);
    splineOracle([new Point2(0,0),new Point2(1,2),new Point2(3,-1),new Point2(5,1)],
      [1.0,0.8,1.3,1.0],[0.0,1.0,2.0,3.0,4.0,5.0,6.0,7.0],3);
    splineOracle([new Point2(0,0),new Point2(1,2),new Point2(3,-1),new Point2(5,1),new Point2(7,0)],
      [1.0,0.8,1.3,1.0,1.0],[0.0,0.0,0.0,0.0,0.35,1.0,1.0,1.0,1.0],3);
    rejects(()->DxfSpline.sample(points,[1.0,-1.0,1.0],[0.0,0.0,0.0,1.0,1.0,1.0],2,0.01),"nonpositive spline weights refused");
  }
  static function main():Void {
    writer();bulges();gaps();inserts();splines();
    rejects(()->Dxf.read(document('0\nELLIPSE\n')),"unhandled cutting geometry is not silently dropped");
    check(Dxf.read(document('0\nELLIPSE\n'),{allowUnsupported:true}).warnings.length==1,"explicit diagnostic mode reports unsupported geometry");
    rejects(()->Dxf.read(document('0\nLINE\n10\n0\n20\n0\n11\n1\n21\n1\n31\n2\n')),"nonplanar sheet geometry refused");
    Sys.println('DXF import checks passed ($assertions assertions)');
  }
}
