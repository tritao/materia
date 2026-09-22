package nativekit.scene;

import nativekit.ffi.NativeKitGpu;

/** Result of polling an asynchronous GPU pick request. */
enum PickPollResult {
	Pending;
	Ready(result:PickResult);
	Stale;
	Failed(error:NativeKitGpu.GpuStatus);
}
