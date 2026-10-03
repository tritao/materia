package machinekit.motion;

/**
 * How one end of a lead screw is held, for its bending. A screw driven through a rigid coupling
 * from a motor is held (`Fixed`) at that end. A bearing that carries radial load but lets the
 * end tilt is `Simple`; a pair of bearings, or a bearing with a long fit, is `Fixed`; an end with
 * no bearing is `Free`.
 */
enum ScrewSupport {
	Free;
	Simple;
	Fixed;
}
