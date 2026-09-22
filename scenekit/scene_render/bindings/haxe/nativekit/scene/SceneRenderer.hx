package nativekit.scene;

import nativekit.ffi.NativeKitGpu;
import NativeKitScene;
import NativeKitSceneRender;
import NativeKitSceneRenderConstants;
import haxe.io.Bytes;
import GraphicsImageRef;

private typedef SceneGpuRenderer = {
	var nativeHandle:Void->nkgpu_renderer;
}

/**
 * Keeps render-plan and NativeKit GPU synchronization behind one explicit
 * render boundary. Scene snapshots and change sets are owned by the caller.
 */
class SceneRenderer {
	final gpuOwner:Null<SceneGpuRenderer>;
	var executor:Ownednkscene_render_executor;
	var planOwner:Null<Ownednkscene_render_plan> = null;
	var lastUpdateValue:Null<nkscene_render_update> = null;
	var disposed:Bool = false;

	private function new(gpuOwner:Null<SceneGpuRenderer>, renderer:nkgpu_renderer,
			borrowedRendererId:Int = 0) {
		this.gpuOwner = gpuOwner;
		if (borrowedRendererId == 0) {
			var made = NativeKitSceneRender.nkscene_render_executor_create(renderer);
			checkScene(made.status, "sceneRenderer.create");
			executor = made.out_executor;
		} else {
			var made = NativeKitSceneRender.nkscene_render_executor_create_from_renderer_id(
				borrowedRendererId);
			checkScene(made.status, "sceneRenderer.create");
			executor = made.out_executor;
		}
	}

	/** Creates a renderer attached to a live NativeKit GPU renderer. */
	public static function create(renderer:SceneGpuRenderer):SceneRenderer
		return new SceneRenderer(renderer, renderer.nativeHandle());

	/** Creates the headless resource/command executor used by tests and tools. */
	public static function createHeadless():SceneRenderer
		return new SceneRenderer(null, new nkgpu_renderer());

	/** Creates an executor around a caller-owned renderer handle. */
	public static function createBorrowed(renderer:nkgpu_renderer):SceneRenderer
		return new SceneRenderer(null, renderer);

	/** Creates an executor around a caller-owned opaque renderer ID. */
	public static function createBorrowedId(rendererId:Int):SceneRenderer
		return new SceneRenderer(null, new nkgpu_renderer(), rendererId);

	/** Compiles, refreshes, or incrementally updates the plan, then executes it. */
	public function render(snapshot:Snapshot, view:SceneView,
			?changes:Null<ChangeSet>):nkscene_render_execution_stats {
		ensureLive();
		prepare(snapshot, view, changes);
		var snapshotValue = snapshot.nativeHandle();

		var executed = NativeKitSceneRender.nkscene_render_executor_execute(
			executor.borrow(), planOwner.borrow(), snapshotValue);
		checkScene(executed.status, "sceneRenderer.execute");
		checkGpu(executed.out_stats.get_result(), "sceneRenderer.execute");
		return executed.out_stats;
	}

	/** Renders through the GPU into tightly packed RGBA8 pixels. */
	public function captureRgba8(snapshot:Snapshot, view:SceneView, width:Int, height:Int,
			clearRed:Float = 0.025, clearGreen:Float = 0.035, clearBlue:Float = 0.055,
			clearAlpha:Float = 1.0):Bytes {
		ensureLive();
		if (width <= 0 || height <= 0 || width > Std.int(536870911 / height))
			throw "Scene capture dimensions are invalid";
		prepare(snapshot, view, null);
		var pixels = Bytes.alloc(width * height * 4);
		var captured = NativeKitSceneRender.nkscene_render_executor_capture_rgba8(
			executor.borrow(), planOwner.borrow(), snapshot.nativeHandle(), width, height,
			clearRed, clearGreen, clearBlue, clearAlpha, pixels, pixels.length);
		if (captured != 0) {
			var last = NativeKitSceneRender.nkscene_render_executor_get_last_result(executor.borrow());
			checkScene(last.status, "sceneRenderer.captureRgba8");
			checkGpu(last.out_result, "sceneRenderer.captureRgba8");
		}
		checkScene(captured, "sceneRenderer.captureRgba8");
		return pixels;
	}

	/** Renders directly into a retained, backend-neutral GPU image. */
	public function renderImage(snapshot:Snapshot, view:SceneView, width:Int, height:Int,
			clearRed:Float = 0.025, clearGreen:Float = 0.035, clearBlue:Float = 0.055,
			clearAlpha:Float = 1.0):GraphicsImageRef {
		ensureLive();
		if (width <= 0 || height <= 0)
			throw "Scene image dimensions are invalid";
		prepare(snapshot, view, null);
		var rendered = NativeKitSceneRender.nkscene_render_executor_render_image_id(
			executor.borrow(), planOwner.borrow(), snapshot.nativeHandle(), width, height,
			clearRed, clearGreen, clearBlue, clearAlpha);
		if (rendered.status != 0) {
			var last = NativeKitSceneRender.nkscene_render_executor_get_last_result(executor.borrow());
			checkScene(last.status, "sceneRenderer.renderImage");
			checkGpu(last.out_result, "sceneRenderer.renderImage");
		}
		checkScene(rendered.status, "sceneRenderer.renderImage");
		return GraphicsImageRef.fromBorrowedId(rendered.out_image_id, width, height);
	}

	function prepare(snapshot:Snapshot, view:SceneView, changes:Null<ChangeSet>):Void {
		var snapshotValue = snapshot.nativeHandle(), viewValue = view.nativeValue();
		if (planOwner == null) {
			var compiled = NativeKitSceneRender.nkscene_render_plan_compile(snapshotValue, viewValue);
			checkScene(compiled.status, "sceneRenderer.compile");
			planOwner = compiled.out_plan;
			lastUpdateValue = null;
		} else if (changes != null) {
			var updated = new nkscene_render_update();
			updated.set_struct_size(nkscene_render_update.size());
			var changeSet = changes.nativeHandle();
			var updateResult = NativeKitSceneRender.nkscene_render_plan_update(
				planOwner.borrow(), snapshotValue, changeSet, viewValue, updated);
			checkScene(updateResult.status, "sceneRenderer.update");
			lastUpdateValue = updateResult.out_update;
		} else {
			var refreshed = new nkscene_render_update();
			refreshed.set_struct_size(nkscene_render_update.size());
			var refreshResult = NativeKitSceneRender.nkscene_render_plan_refresh(
				planOwner.borrow(), snapshotValue, viewValue, refreshed);
			checkScene(refreshResult.status, "sceneRenderer.refresh");
			lastUpdateValue = refreshResult.out_update;
		}

	}

	/** Renders one owned frame, including its optional incremental change set. */
	public function renderFrame(frame:SceneFrame, view:SceneView):nkscene_render_execution_stats
		return render(frame.sceneSnapshot(), view, frame.changeSet());

	/** Returns the last plan update metrics, or null before the first update. */
	public function lastUpdate():Null<nkscene_render_update>
		return lastUpdateValue;

	/** Reports whether this renderer has compiled a render plan yet. */
	public function hasPlan():Bool
		return planOwner != null;

	/** Performs a GPU ID pass and resolves one pixel to scene ownership. */
	public function pickPixel(snapshot:Snapshot, width:Int, height:Int, x:Int, y:Int):PickResult {
		ensureLive();
		if (planOwner == null)
			throw "sceneRenderer.pickPixel requires a compiled render plan";
		var picked = NativeKitSceneRender.nkscene_render_executor_pick_pixel(
			executor.borrow(), planOwner.borrow(), snapshot.nativeHandle(), width, height, x, y);
		if (picked.status != 0) {
			var last = NativeKitSceneRender.nkscene_render_executor_get_last_result(executor.borrow());
			checkScene(last.status, "sceneRenderer.pickPixel");
			checkGpu(last.out_result, "sceneRenderer.pickPixel");
			throw "sceneRenderer.pickPixel failed";
		}
		return new PickResult(picked.out_result);
	}

	/** Starts a non-blocking GPU ID pass and pixel readback. */
	public function pickPixelAsync(snapshot:Snapshot, width:Int, height:Int, x:Int, y:Int):PickRequest {
		ensureLive();
		if (planOwner == null)
			throw "sceneRenderer.pickPixelAsync requires a compiled render plan";
		var started = NativeKitSceneRender.nkscene_render_executor_pick_pixel_begin(
			executor.borrow(), planOwner.borrow(), snapshot.nativeHandle(), width, height, x, y);
		checkScene(started.status, "sceneRenderer.pickPixelAsync");
		return new PickRequest(this, started.out_request);
	}

	@:allow(PickRequest)
	function pollPick(request:PickRequest, snapshot:Snapshot):PickPollResult {
		ensureLive();
		if (planOwner == null)
			throw "sceneRenderer.pollPick requires a compiled render plan";
		var polled = NativeKitSceneRender.nkscene_render_executor_pick_pixel_poll(
			executor.borrow(), request.nativeHandle(), planOwner.borrow(), snapshot.nativeHandle());
		checkScene(polled.status, "sceneRenderer.pollPick");
		return switch (polled.out_state) {
			case NativeKitSceneRenderConstants.NKS_RENDER_PICK_PENDING: Pending;
			case NativeKitSceneRenderConstants.NKS_RENDER_PICK_READY: Ready(new PickResult(polled.out_result));
			case NativeKitSceneRenderConstants.NKS_RENDER_PICK_STALE: Stale;
			default: Failed(polled.out_error);
		};
	}

	@:allow(SceneInteraction)
	function nativeExecutor():nkscene_render_executor {
		ensureLive();
		return executor.borrow();
	}

	@:allow(SceneInteraction)
	function nativePlan():nkscene_render_plan {
		ensureLive();
		if (planOwner == null)
			throw "sceneInteraction requires a compiled render plan";
		return planOwner.borrow();
	}

	public function dispose():Void {
		if (disposed)
			return;
		if (planOwner != null) {
			planOwner.close();
			planOwner = null;
		}
		executor.close();
		disposed = true;
	}

	public function isDisposed():Bool
		return disposed;

	function ensureLive():Void {
		if (disposed)
			throw "sceneRenderer has been disposed";
	}

	static function checkScene(status:Int, operation:String):Void {
		if (status != 0)
			throw '$operation failed with NativeKit scene status $status';
	}

	static function checkGpu(status:Int, operation:String):Void {
		if (status != 0)
			throw '$operation failed with NativeKit GPU status $status: ${NativeKitGpu.nkgpu_last_error()}';
	}
}
