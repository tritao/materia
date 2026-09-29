package robotkit.policy;

import RobotKitPolicy;
import RobotKitRuntime;

/** One named tensor of a loaded model. */
typedef PolicyTensor = {name:String, elements:Int, offset:Int};

/**
 * A learned policy: an ONNX model run on the CPU by ONNX Runtime. It is a
 * pure function from named float tensors to named float tensors; state a
 * controller must carry between calls (a recurrent network's, say) is an
 * ordinary input the caller feeds back from an output.
 */
class OnnxPolicy {
  final owner:Ownedrk_policy;
  final io:rk_policy_io;
  public final inputs:Array<PolicyTensor> = [];
  public final outputs:Array<PolicyTensor> = [];
  var inputValues:Int = 0;
  var closed:Bool = false;

  function new(owner:Ownedrk_policy) {
    this.owner = owner;
    io = new rk_policy_io();
    io.set_struct_size(rk_policy_io.size());
    var info = new rk_policy_info();
    info.set_struct_size(rk_policy_info.size());
    check(RobotKitPolicy.rk_policy_get_info(owner.borrow(), info).status, "policy.info");
    inputValues = info.get_input_values();
    for (index in 0...info.get_input_count()) inputs.push(tensor(RobotKitPolicyConstants.RK_POLICY_INPUT, index));
    for (index in 0...info.get_output_count()) outputs.push(tensor(RobotKitPolicyConstants.RK_POLICY_OUTPUT, index));
  }

  /** Loads a model file. Throws when it cannot be read or uses unsupported tensors. */
  public static function load(path:String):OnnxPolicy {
    var result = RobotKitPolicy.rk_policy_create(path);
    check(result.status, 'policy.load($path)');
    return new OnnxPolicy(result.out_policy);
  }

  function tensor(direction:Int, index:Int):PolicyTensor {
    var value = new rk_policy_tensor();
    value.set_struct_size(rk_policy_tensor.size());
    check(RobotKitPolicy.rk_policy_get_tensor(owner.borrow(), direction, index, value).status, "policy.tensor");
    var name = new StringBuf();
    for (i in 0...64) {
      var code = value.get_name(i);
      if (code == 0) break;
      name.addChar(code);
    }
    return {name: name.toString(), elements: value.get_elements(), offset: value.get_offset()};
  }

  /** The input tensor called `name`; throws for an unknown name. */
  public function input(name:String):PolicyTensor return find(inputs, name, "input");

  /** The output tensor called `name`; throws for an unknown name. */
  public function output(name:String):PolicyTensor return find(outputs, name, "output");

  static function find(tensors:Array<PolicyTensor>, name:String, kind:String):PolicyTensor {
    for (tensor in tensors) if (tensor.name == name) return tensor;
    throw 'the policy has no $kind "$name"';
  }

  /**
   * Runs the model once. Every input must be given, with exactly its element
   * count; outputs come back by name.
   */
  public function run(values:Map<String, Array<Float>>):Map<String, Array<Float>> {
    if (closed) throw "policy is disposed";
    for (tensor in inputs) {
      var given = values.get(tensor.name);
      if (given == null) throw 'missing policy input "${tensor.name}"';
      if (given.length != tensor.elements)
        throw 'policy input "${tensor.name}" needs ${tensor.elements} values, got ${given.length}';
      for (i in 0...tensor.elements) io.set_inputs(tensor.offset + i, given[i]);
    }
    check(RobotKitPolicy.rk_policy_run(owner.borrow(), io).status, "policy.run");
    var result = new Map<String, Array<Float>>();
    for (tensor in outputs)
      result.set(tensor.name, [for (i in 0...tensor.elements) io.get_outputs(tensor.offset + i)]);
    return result;
  }

  public function dispose():Void {
    if (closed) return;
    closed = true;
    owner.close();
  }

  static function check(status:Int, operation:String):Void {
    if (status != RobotKitRuntimeConstants.RK_OK) throw '$operation failed with RobotKit status $status';
  }
}
