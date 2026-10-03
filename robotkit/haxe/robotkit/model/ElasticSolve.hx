package robotkit.model;

import robotkit.model.DriveLoads.AxisLoad;

/** Static span energy with held motors and physical transmission constraints. */
class ElasticSolve {
  public final compliance:Array<Array<Float>>;
  public final backlash:Array<Float>;

  public function new(model:RobotModel, loads:Array<AxisLoad>) {
    var ids:Array<String> = [];
    for (load in loads) ids.push(load.axis);
    var owned:Array<String> = [];
    for (network in model.elasticNetworks) {
      network.validate(model.joints, model.couplings);
      for (id in network.couplings) {
        if (owned.indexOf(id) >= 0) throw "A motion coupling belongs to several elastic networks";
        owned.push(id);
      }
      for (span in network.spans) for (term in span.terms) if (ids.indexOf(term.joint) < 0) ids.push(term.joint);
    }
    // Include the complete connected transmission graph, preserving intermediate shafts.
    for (_ in 0...model.joints.length) for (coupling in model.couplings)
      if (owned.indexOf(coupling.id) < 0 && (ids.indexOf(coupling.leader) >= 0 || ids.indexOf(coupling.follower) >= 0)) {
        if (ids.indexOf(coupling.leader) < 0) ids.push(coupling.leader);
        if (ids.indexOf(coupling.follower) < 0) ids.push(coupling.follower);
      }
    var count = ids.length;
    var stiffness = [for (_ in 0...count) [for (_ in 0...count) 0.0]];
    var rigid:Array<Array<Float>> = [], basis:Array<Array<Float>> = [];
    var softLosses:Array<{row:Array<Float>, stiffness:Float, bound:Float}> = [];
    var rigidLosses:Array<{index:Int, bound:Float}> = [];
    for (network in model.elasticNetworks) for (span in network.spans) {
      var row = [for (_ in 0...count) 0.0];
      for (term in span.terms) row[ids.indexOf(term.joint)] = term.coefficient;
      spring(stiffness, row, span.stiffness);
    }
    var targets:Array<String> = [];
    for (coupling in model.couplings) if (owned.indexOf(coupling.id) < 0 && targets.indexOf(coupling.follower) < 0)
      targets.push(coupling.follower);
    for (target in targets) {
      if (ids.indexOf(target) < 0) continue;
      var row = [for (_ in 0...count) 0.0];
      row[ids.indexOf(target)] = 1.0;
      var k = 0.0, loss = 0.0;
      for (coupling in model.couplings) if (coupling.follower == target) {
        if (owned.indexOf(coupling.id) >= 0)
          throw "A summed follower cannot mix a network motion term and an independent scalar transmission";
        row[ids.indexOf(coupling.leader)] -= coupling.ratio;
        loss += Math.abs(coupling.ratio) * coupling.backlash;
        if (coupling.stiffness > 0) {
          var reflected = coupling.stiffness / (coupling.ratio * coupling.ratio);
          k = k == 0.0 ? reflected : Math.min(k, reflected);
        }
      }
      if (k > 0) {
        spring(stiffness, row, k);
        if (loss > 0) softLosses.push({row: row, stiffness: k, bound: loss});
      } else {
        var norm = rowNorm(row), at = constraint(rigid, basis, row);
        if (at >= 0 && loss > 0) rigidLosses.push({index: at, bound: loss / norm});
      }
    }
    for (actuator in model.actuators) switch actuator.transmission {
      case SimpleTransmission(joint, _, _):
        var at = ids.indexOf(joint);
        if (at >= 0) {
          var row = [for (_ in 0...count) 0.0]; row[at] = 1.0;
          var held = constraint(rigid, basis, row);
        }
    }
    var size = count + rigid.length;
    var system = [for (i in 0...size) [for (j in 0...size)
      i < count && j < count ? stiffness[i][j] : i < count ? rigid[j - count][i] : j < count ? rigid[i - count][j] : 0.0]];
    var inverse = DriveCompliance.invert(system);
    compliance = [for (i in 0...loads.length) [for (j in 0...loads.length) inverse[i][j]]];
    backlash = [for (_ in loads) 0.0];
    // Contact clearances shift all adjoining spans together, preserving their correlation.
    for (network in model.elasticNetworks) for (clearance in network.clearances) {
      var effort = [for (_ in 0...count) 0.0];
      for (span in network.spans) {
        var coefficient = 0.0;
        for (term in span.terms) if (term.joint == clearance.joint) coefficient = term.coefficient;
        for (term in span.terms)
          effort[ids.indexOf(term.joint)] += span.stiffness * term.coefficient * coefficient;
      }
      for (axis in 0...loads.length) {
        var response = 0.0;
        for (i in 0...count) response += inverse[axis][i] * effort[i];
        backlash[axis] += Math.abs(response) * clearance.allowance;
      }
    }
    for (loss in softLosses) for (axis in 0...loads.length) {
      var response = 0.0;
      for (i in 0...count) response += inverse[axis][i] * loss.row[i];
      backlash[axis] += Math.abs(response) * loss.stiffness * loss.bound;
    }
    for (loss in rigidLosses) for (axis in 0...loads.length)
      backlash[axis] += Math.abs(inverse[axis][count + loss.index]) * loss.bound;
  }

  static function spring(matrix:Array<Array<Float>>, row:Array<Float>, k:Float):Void {
    for (i in 0...row.length) for (j in 0...row.length) matrix[i][j] += k * row[i] * row[j];
  }

  static function rowNorm(row:Array<Float>):Float {
    var value = 0.0;
    for (entry in row) value += entry * entry;
    return Math.sqrt(value);
  }

  static function constraint(rows:Array<Array<Float>>, basis:Array<Array<Float>>, row:Array<Float>):Int {
    var residual = row.copy();
    for (direction in basis) {
      var dot = 0.0;
      for (i in 0...row.length) dot += residual[i] * direction[i];
      for (i in 0...row.length) residual[i] -= dot * direction[i];
    }
    var norm = rowNorm(residual), originalNorm = rowNorm(row);
    if (norm <= 1e-10 * originalNorm) return -1;
    for (i in 0...row.length) residual[i] /= norm;
    basis.push(residual);
    rows.push([for (value in row) value / originalNorm]);
    return rows.length - 1;
  }
}
