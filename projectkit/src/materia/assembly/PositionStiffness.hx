package materia.assembly;

/** A parallel sum of springs whose series compliances vary affinely with physical joint coordinates.
 * A belt span is L(q)/EA; a supported screw is L(q)/EA + 1/K_nut + 1/K_bearing.
 * Coordinates and compliance use the caller's units, in the coupling's stated coordinate. No samples or callbacks cross the ABI.
 */
typedef ComplianceCoordinate = { var joint:String; var coefficient:Float; }
private typedef StiffnessInterval = { var lower:Float; var upper:Float; }
typedef ComplianceSpring = { var compliance:Float; var coordinates:Array<ComplianceCoordinate>; }

class PositionStiffness {
  public final springs:Array<ComplianceSpring>;
  public function new(springs:Array<ComplianceSpring>) {
    if (springs == null || springs.length == 0) throw "Position stiffness needs spring compliances";
    this.springs = springs;
    for (spring in springs) {
      if (!Math.isFinite(spring.compliance)) throw "Spring compliance must be finite";
      var seen = new Map<String,Bool>();
      for (coordinate in spring.coordinates) {
        if (coordinate.joint == null || coordinate.joint.length == 0 || seen.exists(coordinate.joint) ||
            !Math.isFinite(coordinate.coefficient) || coordinate.coefficient == 0.0)
          throw "Spring compliance needs unique finite coordinate gradients";
        seen.set(coordinate.joint,true);
      }
    }
  }

  public function at(position:String->Float):Float {
    var stiffness = 0.0;
    for (spring in springs) {
      var compliance = spring.compliance;
      for (coordinate in spring.coordinates) compliance += coordinate.coefficient * position(coordinate.joint);
      if (!(compliance > 0.0) || !Math.isFinite(compliance)) throw "Spring compliance is non-positive at this physical pose";
      stiffness += 1.0/compliance;
    }
    return stiffness;
  }

  /** Tight certified bounds on a finite travel box. Reciprocal positive affine forms are convex.
   * Maxima are at vertices. Coordinate minimisation supplies a point; its supporting hyperplane supplies a lower bound,
   * so even early termination never turns a sampled minimum into a claimed worst-case stiffness.
   */
  public function range(bounds:String->{lower:Float,upper:Float}):{minimum:Float,maximum:Float} {
    var ids:Array<String> = [];
    for (spring in springs) for (coordinate in spring.coordinates) if (ids.indexOf(coordinate.joint)<0) ids.push(coordinate.joint);
    if (ids.length>8) throw "Position stiffness travel box has more than eight independent coordinates";
    var lower:Array<Float> = [], upper:Array<Float> = [], q:Array<Float> = [];
    for (id in ids) {
      var bound = bounds(id);
      if (!Math.isFinite(bound.lower) || !Math.isFinite(bound.upper) || bound.lower>bound.upper)
        throw 'Position stiffness needs finite ordered travel for "$id"';
      lower.push(bound.lower); upper.push(bound.upper); q.push((bound.lower+bound.upper)*0.5);
    }
    for (spring in springs) {
      var enclosed = exact(spring.compliance);
      for (coordinate in spring.coordinates) {
        var index = ids.indexOf(coordinate.joint);
        enclosed = add(enclosed,multiply(exact(coordinate.coefficient),exact(coordinate.coefficient>0 ? lower[index] : upper[index])));
      }
      if (!(enclosed.lower>0.0)) throw "A spring span reaches zero compliance within declared travel";
    }
    function value():Float return at(id -> q[ids.indexOf(id)]);
    function gradient(index:Int):Float {
      var result = 0.0;
      for (spring in springs) {
        var compliance = spring.compliance, derivative = 0.0;
        for (coordinate in spring.coordinates) {
          compliance += coordinate.coefficient*q[ids.indexOf(coordinate.joint)];
          if (coordinate.joint == ids[index]) derivative = coordinate.coefficient;
        }
        result -= derivative/(compliance*compliance);
      }
      return result;
    }
    var maximum = enclosedValue(ids,q).upper;
    for (corner in 0...(1<<ids.length)) {
      for (axis in 0...ids.length) q[axis] = (corner & (1<<axis))==0 ? lower[axis] : upper[axis];
      maximum = Math.max(maximum,enclosedValue(ids,q).upper);
    }
    for (axis in 0...ids.length) q[axis] = (lower[axis]+upper[axis])*0.5;
    var minimum = value();
    for (_ in 0...256) {
      for (axis in 0...ids.length) {
        var a = lower[axis], b = upper[axis];
        q[axis] = a; var ga = gradient(axis);
        q[axis] = b; var gb = gradient(axis);
        if (ga >= 0.0) q[axis] = a;
        else if (gb <= 0.0) q[axis] = b;
        else {
          for (_ in 0...60) { q[axis]=(a+b)*0.5; if (gradient(axis)>0.0) b=q[axis]; else a=q[axis]; }
          q[axis]=(a+b)*0.5;
        }
      }
      var here = enclosedValue(ids,q), certificate = here;
      for (axis in 0...ids.length) {
        var enclosedGradient = exact(0.0);
        for (spring in springs) {
          var derivative = 0.0;
          for (coordinate in spring.coordinates) if (coordinate.joint == ids[axis]) derivative = coordinate.coefficient;
          if (derivative == 0.0) continue;
          var c = enclosedCompliance(spring,ids,q);
          var reciprocalSquare = reciprocal(multiply(c,c));
          enclosedGradient = add(enclosedGradient,multiply(exact(-derivative),reciprocalSquare));
        }
        var delta = outward(lower[axis]-q[axis],upper[axis]-q[axis]);
        certificate = add(certificate,multiply(enclosedGradient,delta));
      }
      minimum = certificate.lower;
      if (here.upper-minimum <= here.upper*1e-10) break;
    }
    if (!(minimum>0.0) || !Math.isFinite(maximum)) throw "Could not certify a positive finite position stiffness range";
    return {minimum:minimum,maximum:maximum};
  }

  static function exact(value:Float):StiffnessInterval return {lower:value,upper:value};
  static function outward(lower:Float,upper:Float):StiffnessInterval {
    // Two epsilons cover each rounded arithmetic operation including the outward addition itself.
    // The least subnormal covers underflow. No engineering margin is hidden in the certificate.
    return {lower:lower-(Math.abs(lower)*4.440892098500626e-16+4.9406564584124654e-324),
      upper:upper+(Math.abs(upper)*4.440892098500626e-16+4.9406564584124654e-324)};
  }
  static function add(a:StiffnessInterval,b:StiffnessInterval):StiffnessInterval return outward(a.lower+b.lower,a.upper+b.upper);
  static function multiply(a:StiffnessInterval,b:StiffnessInterval):StiffnessInterval {
    var values = [a.lower*b.lower,a.lower*b.upper,a.upper*b.lower,a.upper*b.upper];
    var lower = values[0], upper = values[0];
    for (value in values) { lower=Math.min(lower,value); upper=Math.max(upper,value); }
    return outward(lower,upper);
  }
  static function reciprocal(a:StiffnessInterval):StiffnessInterval {
    if (!(a.lower>0.0)) throw "Cannot certify positive spring compliance";
    return outward(1.0/a.upper,1.0/a.lower);
  }
  static function enclosedCompliance(spring:ComplianceSpring,ids:Array<String>,q:Array<Float>):StiffnessInterval {
    var result = exact(spring.compliance);
    for (coordinate in spring.coordinates) result=add(result,multiply(exact(coordinate.coefficient),exact(q[ids.indexOf(coordinate.joint)])));
    return result;
  }
  function enclosedValue(ids:Array<String>,q:Array<Float>):StiffnessInterval {
    var result = exact(0.0);
    for (spring in springs) result=add(result,reciprocal(enclosedCompliance(spring,ids,q)));
    return result;
  }

  public function scaled(complianceScale:Float, coordinateScale:String->Float, origin:String->Float):PositionStiffness {
    return new PositionStiffness([for (spring in springs) {
      var constant = spring.compliance;
      for (coordinate in spring.coordinates) constant += coordinate.coefficient*origin(coordinate.joint);
      {compliance:constant*complianceScale,coordinates:[for (coordinate in spring.coordinates)
        {joint:coordinate.joint,coefficient:coordinate.coefficient*complianceScale/coordinateScale(coordinate.joint)}]};
    }]);
  }
}
