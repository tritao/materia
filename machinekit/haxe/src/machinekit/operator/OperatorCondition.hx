package machinekit.operator;

/**
 * Machine facts the cabinet controller evaluates itself; jog rules name them. The numbering is the
 * bit in the active-condition mask and is fixed by `OperatorCondition` in operator.wire.idl.
 */
enum OperatorCondition { GearboxEngaged; DoorOpen; SpindleRunning; ProgramRunning; }
