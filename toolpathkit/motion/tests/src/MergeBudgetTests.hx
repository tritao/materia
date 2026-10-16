import motionkit.math.QuinticPolynomial;
import motionkit.robot.MergeErrorBudget;
import motionkit.path.PosePositionBound;
import motionkit.path.PathPoint;
import motionkit.path.LineSegment;
import motionkit.path.ArcSegment;
import motionkit.path.CircularSegment;
import motionkit.path.CircularPlane;
import motionkit.path.QuinticBlend;
import toolpathkit.motion.ToolpathPosePrimitive;

class MergeBudgetTests {
  static function check(value:Bool,message:String):Void {if(!value)throw message;}
  static function close(a:Float,b:Float):Bool return Math.abs(a-b)<=1e-12*Math.max(1,Math.abs(b));
  static function geometry(primitive:ToolpathPosePrimitive,from:Float,to:Float):Void {
    var a=primitive.waypointAt(from).pose,b=primitive.waypointAt(to).pose;
    var da=primitive.derivativesAt(from),db=primitive.derivativesAt(to);
    var control=QuinticPolynomial.bernstein(QuinticPolynomial.hermite(a.positionArray(),b.positionArray(),
      da.linear,db.linear,da.linearSecond,db.linearSecond,to-from));
    var bound=primitive.positionErrorBound(control,from,to);
    check(Math.isFinite(bound) && bound>=0,"Geometry has no finite bound");
    for(sample in 0...1001){
      var u=sample/1000.0,work=QuinticPolynomial.slice(control,0,1);
      // Evaluate independently with de Casteljau rather than the power-basis evaluator used by the planner.
      var point=[for(row in work){var values=row.copy();for(level in 1...6)
        values=[for(i in 0...values.length-1)(1-u)*values[i]+u*values[i+1]];values[0];}];
      var expected=primitive.waypointAt(from+(to-from)*u).pose;
      var error=Math.sqrt(Math.pow(point[0]-expected.x,2)+Math.pow(point[1]-expected.y,2)+Math.pow(point[2]-expected.z,2));
      check(error<=bound+1e-14,'Geometry error $error exceeds $bound');
    }
    // Translating the whole candidate must consume the same positional allowance.
    var moved=[for(row in control)row.copy()];for(i in 0...6)moved[1][i]+=0.001;
    check(primitive.positionErrorBound(moved,from,to)>=0.001-bound-1e-12,"Candidate displacement was not counted");
  }
  public static function main():Void {
    for(duration in [0.001,0.01,0.2])for(epsilon in [1e-8,1e-5,0.001]){
      var budget=new MergeErrorBudget([epsilon,0],duration);
      check(budget.position[1]==0,"Held joint gained a budget");
      for(sample in 0...1001){var t=duration*sample/1000;
        check((2*epsilon/duration)*Math.min(t,duration-t)<=epsilon+1e-15,"Velocity integral bound");
        check((8*epsilon/(duration*duration))*t*(duration-t)/2<=epsilon+1e-15,"Acceleration Green-function bound");}
    }
    for(bad in [0.0,-1.0,Math.NaN,Math.POSITIVE_INFINITY]){
      var rejected=false;try new MergeErrorBudget([1e-5],bad) catch(_:Dynamic)rejected=true;
      check(rejected,"Invalid merge window accepted");
    }
    var drive=new motionkit.path.DistanceDrive(3,[{start:0.0,end:1.0,rate:5.0}],1.0,0.01);
    check(close(drive.maximumRate(),7.5),"Distance-drive startup overshoot was omitted");
    var fifth=QuinticPolynomial.hermite([0.0],[1.0],[0.0],[5.0],[0.0],[20.0],1.0);
    var cut=QuinticPolynomial.slice(QuinticPolynomial.bernstein(fifth),0.2,0.8);
    for(sample in 0...101){var u=sample/100.0,split=QuinticPolynomial.split(cut,u);
      check(close(split[0][0][5],Math.pow(0.2+0.6*u,5)),"Quintic restriction changed its polynomial");}
    var line=new LineSegment(new PathPoint(0,0,0),new PathPoint(0.02,0.01,0.005));
    geometry(new ToolpathPosePrimitive(line,0.1,1e-4,0.02),0,line.length());
    var arc=new ArcSegment(new PathPoint(0,0,0),0.01,0,1.0);
    geometry(new ToolpathPosePrimitive(arc,0.1,1e-4,0.02),0,arc.length());
    var helix=new CircularSegment(new PathPoint(0,0,0),0.003,0,1.0,CircularPlane.XY,0.001);
    geometry(new ToolpathPosePrimitive(helix,0.1,1e-4,0.02),0,helix.length());
    var before=new LineSegment(new PathPoint(-0.005,0,0),new PathPoint(0,0,0));
    var after=new LineSegment(new PathPoint(0,0,0),new PathPoint(0.005,0.005,0));
    var blend=new QuinticBlend(before,before.length()-0.0004,after,0.0004,0.0008);
    var primitive=new ToolpathPosePrimitive(blend,0.1,1e-4,0.02);
    geometry(primitive,0,blend.length());geometry(primitive,0.23*blend.length(),0.79*blend.length());
    Sys.println("Merge budgets and continuous geometric bounds passed");
  }
}
