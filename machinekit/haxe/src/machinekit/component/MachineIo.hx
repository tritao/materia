package machinekit.component;
/** Required controller channels. Hardwired safety contacts are separate component ports. */
typedef MachineIo = {var inputs: Array<String>;
var outputs:Array<String>;
var analogOutputs:Array<String>;
}
