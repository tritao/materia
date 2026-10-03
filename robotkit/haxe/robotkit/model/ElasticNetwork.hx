package robotkit.model;

import robotkit.model.EngineeringAssumptions.QuantityAssumption;

/** One free span's axial spring and displacement coordinates, entirely in SI units. */
typedef ElasticSpan = {
  var stiffness:Float;
  var terms:Array<ElasticTerm>;
}

typedef ElasticTerm = {
  var joint:String;
  var coefficient:Float;
}

typedef ElasticClearance = {
  var joint:String;
  var allowance:Float;
}

/** Derived span energies; listed motion couplings supply no additional elastic spring. */
class ElasticNetwork {
  public final id:String;
  public final couplings:Array<String>;
  public final spans:Array<ElasticSpan>;
  public var assumptions:Array<QuantityAssumption> = [];
  public var clearances:Array<ElasticClearance> = [];

  public function new(id:String, couplings:Array<String>, spans:Array<ElasticSpan>) {
    this.id = id; this.couplings = couplings; this.spans = spans;
  }

  public function validate(joints:Array<Joint>, motion:Array<JointCoupling>):Void {
    if (id == null || StringTools.trim(id).length == 0 || spans.length == 0)
      throw "Elastic network needs an identity and spans";
    var owners:Array<String> = [];
    for (owner in couplings) {
      var found = false;
      for (coupling in motion) if (coupling.id == owner) found = true;
      if (!found || owners.indexOf(owner) >= 0) throw 'Elastic network "$id" has an unknown or duplicate motion coupling';
      owners.push(owner);
    }
    var clearanceJoints:Array<String> = [];
    for (clearance in clearances) {
      var found = false;
      for (span in spans) for (term in span.terms) if (term.joint == clearance.joint) found = true;
      if (!found || clearanceJoints.indexOf(clearance.joint) >= 0 || !Math.isFinite(clearance.allowance) || clearance.allowance < 0)
        throw 'Elastic network "$id" has an invalid tooth clearance';
      clearanceJoints.push(clearance.joint);
    }
    for (span in spans) {
      if (!(span.stiffness > 0) || !Math.isFinite(span.stiffness) || span.terms.length == 0)
        throw 'Elastic network "$id" needs positive finite span stiffness';
      var terms:Array<String> = [];
      for (term in span.terms) {
        var found = false;
        for (joint in joints) if (joint.id == term.joint && joint.type != JointType.Fixed) found = true;
        if (!found || terms.indexOf(term.joint) >= 0 || !Math.isFinite(term.coefficient) || term.coefficient == 0)
          throw 'Elastic network "$id" has an invalid span coordinate';
        terms.push(term.joint);
      }
    }
  }
}
