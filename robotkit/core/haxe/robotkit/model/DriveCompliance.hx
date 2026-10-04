package robotkit.model;

import robotkit.model.DriveLoads.AxisLoad;

/** Elastic coordinates shared by every driven axis, including rigid motor constraints. */
class DriveCompliance {
  public final compliance:Array<Array<Float>>;

  public function new(loads:Array<AxisLoad>, ?model:RobotModel) {
    if (model != null) {
      var count = loads.length;
      compliance = [for (_ in 0...count) [for (_ in 0...count) 0.0]];
      var touches = [for (network in model.elasticNetworks)
        [for (load in loads) networkTouches(model, load.axis, network)]];
      var assigned = [for (_ in 0...count) false];
      for (start in 0...count) if (!assigned[start]) {
        var group = [start]; assigned[start] = true;
        var changed = true;
        while (changed) {
          changed = false;
          for (candidate in 0...count) if (!assigned[candidate]) {
            var connected = false;
            for (member in group) {
              for (a in loads[member].motors) for (b in loads[candidate].motors)
                if (a.actuator.id == b.actuator.id) connected = true;
              for (network in touches) if (network[member] && network[candidate]) connected = true;
            }
            if (connected) { group.push(candidate); assigned[candidate] = true; changed = true; }
          }
        }
        var block = [for (index in group) loads[index]];
        if (DriveLoads.uncontrolledAxes(block).length > 0) continue;
        var hasNetwork = false;
        for (network in touches) for (index in group) if (network[index]) hasNetwork = true;
        var matrix:Array<Array<Float>>;
        var losses:Array<Float> = [];
        if (hasNetwork) {
          var solved = new ElasticSolve(model, block);
          matrix = solved.compliance;
          losses = solved.backlash;
        } else {
          var scalar = new DriveCompliance(block);
          matrix = scalar.compliance;
          losses = [for (load in block) load.backlash];
        }
        for (row in 0...group.length) {
          var axis = group[row];
          for (column in 0...group.length) compliance[axis][group[column]] = matrix[row][column];
          loads[axis].backlash = losses[row];
          for (networkIndex in 0...model.elasticNetworks.length)
            if (touches[networkIndex][axis])
              EngineeringAssumptions.merge(loads[axis].assumptions, model.elasticNetworks[networkIndex].assumptions);
        }
      }
      for (axis in 0...count) {
        loads[axis].elastic = this;
        loads[axis].stiffness = compliance[axis][axis] > 1e-20 ? 1.0 / compliance[axis][axis] : 0.0;
      }
      return;
    }
    var count = loads.length;
    var ids:Array<String> = [];
    for (load in loads) for (motor in load.motors) if (ids.indexOf(motor.actuator.id) < 0) ids.push(motor.actuator.id);
    var rows:Array<Array<Float>> = [];
    var springs:Array<Float> = [];
    var losses:Array<Float> = [];
    for (id in ids) {
      var row = [for (_ in 0...count) 0.0];
      var spring = 0.0, loss = 0.0;
      for (axis in 0...count) for (motor in loads[axis].motors) if (motor.actuator.id == id) {
        row[axis] = motor.ratio;
        if (motor.stiffness > 0.0) spring = spring == 0.0 ? motor.stiffness : Math.min(spring, motor.stiffness);
        loss = Math.max(loss, motor.backlash);
      }
      rows.push(row); springs.push(spring); losses.push(loss);
    }
    var stiffness = [for (_ in 0...count) [for (_ in 0...count) 0.0]];
    var rigid:Array<Array<Float>> = [];
    for (motor in 0...rows.length) {
      var row = rows[motor];
      if (springs[motor] > 0.0) {
        for (i in 0...count) for (j in 0...count) stiffness[i][j] += springs[motor] * row[i] * row[j];
      } else {
        // Keep only independent constraints, so parallel rigid drives do not make the solve singular.
        var residual = row.copy();
        for (basis in rigid) {
          var dot = 0.0;
          for (i in 0...count) dot += residual[i] * basis[i];
          for (i in 0...count) residual[i] -= dot * basis[i];
        }
        var norm = 0.0;
        for (value in residual) norm += value * value;
        if (norm > 1e-20) {
          norm = Math.sqrt(norm);
          for (i in 0...count) residual[i] /= norm;
          rigid.push(residual);
        }
      }
    }
    var size = count + rigid.length;
    var system = [for (i in 0...size) [for (j in 0...size)
      i < count && j < count ? stiffness[i][j] : i < count ? rigid[j - count][i] : j < count ? rigid[i - count][j] : 0.0]];
    var inverse = invert(system);
    compliance = [for (i in 0...count) [for (j in 0...count) inverse[i][j]]];
    // Project independent motor lost-motion bounds through the full coordinate Jacobian.
    var gram = [for (_ in 0...count) [for (_ in 0...count) 0.0]];
    for (row in rows) for (i in 0...count) for (j in 0...count) gram[i][j] += row[i] * row[j];
    var projection = invert(gram);
    for (axis in 0...count) {
      loads[axis].elastic = this;
      for (other in loads) for (motor in other.motors) if (rows[ids.indexOf(motor.actuator.id)][axis] != 0.0)
        EngineeringAssumptions.merge(loads[axis].assumptions, motor.assumptions);
      loads[axis].stiffness = compliance[axis][axis] > 1e-20 ? 1.0 / compliance[axis][axis] : 0.0;
      loads[axis].backlash = 0.0;
      for (motor in 0...rows.length) {
        var factor = 0.0;
        for (j in 0...count) factor += projection[axis][j] * rows[motor][j];
        loads[axis].backlash += Math.abs(factor) * losses[motor];
      }
    }
  }

  public function deflections(forces:Array<Float>):Array<Float> {
    if (forces.length != compliance.length)
      throw 'Drive compliance needs ${compliance.length} axis forces, got ${forces.length}';
    var result:Array<Float> = [];
    for (row in compliance) {
      var value = 0.0;
      for (i in 0...forces.length) value += row[i] * forces[i];
      result.push(value);
    }
    return result;
  }

  static function networkTouches(model:RobotModel, axis:String, target:ElasticNetwork):Bool {
    var reached = [axis], changed = true;
    while (changed) {
      changed = false;
      for (coupling in model.couplings) {
        if (reached.indexOf(coupling.leader) >= 0 && reached.indexOf(coupling.follower) < 0) {
          reached.push(coupling.follower); changed = true;
        }
        if (reached.indexOf(coupling.follower) >= 0 && reached.indexOf(coupling.leader) < 0) {
          reached.push(coupling.leader); changed = true;
        }
      }
      for (network in model.elasticNetworks) {
        var touches = false;
        for (span in network.spans) for (term in span.terms)
          if (reached.indexOf(term.joint) >= 0) touches = true;
        if (touches) for (span in network.spans) for (term in span.terms)
          if (reached.indexOf(term.joint) < 0) { reached.push(term.joint); changed = true; }
      }
    }
    for (span in target.spans) for (term in span.terms)
      if (reached.indexOf(term.joint) >= 0) return true;
    return false;
  }

  public static function invert(matrix:Array<Array<Float>>):Array<Array<Float>> {
    var count = matrix.length;
    var work = [for (i in 0...count) matrix[i].concat([for (j in 0...count) i == j ? 1.0 : 0.0])];
    for (column in 0...count) {
      var pivot = column;
      for (row in column + 1...count) if (Math.abs(work[row][column]) > Math.abs(work[pivot][column])) pivot = row;
      if (Math.abs(work[pivot][column]) < 1e-20) throw 'Driven axes have a singular coupling Jacobian';
      var swap = work[column]; work[column] = work[pivot]; work[pivot] = swap;
      var scale = work[column][column];
      for (j in 0...2 * count) work[column][j] /= scale;
      for (row in 0...count) if (row != column) {
        var factor = work[row][column];
        for (j in 0...2 * count) work[row][j] -= factor * work[column][j];
      }
    }
    return [for (row in work) row.slice(count)];
  }
}
