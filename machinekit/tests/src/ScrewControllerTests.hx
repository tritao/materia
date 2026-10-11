import haxe.io.Bytes;
import processkit.screwdriving.openprotocol.*;
import processkit.screwdriving.*;
import processkit.screwdriving.ScrewdriverIo.ScrewdriverIoSample;
import haxe.Int64;
import nativekit.ffi.NativeKitTypes;
import haxeon.platform.NativeKitRuntime;
import haxeon.platform.NativeKitEventValue;
import haxeon.platform.NativeKitEventBytes;
import robotkit.transport.NativeTransport;

private class ScriptedTighteningController implements OpenProtocolTransport {
	public final requests:Array<OpenProtocolFrame> = [];
	public final replies:Array<Bytes> = [];
	public var automatic = true;
	public var closed = false;
	public function new() {}
	public function send(bytes:Bytes):Void {
		if (closed) throw "Scripted controller disconnected";
		var frame = OpenProtocolFrame.decode(bytes); requests.push(frame);
		if (!automatic) return;
		if (frame.mid == 1) replies.push(new OpenProtocolFrame(2,"010001020103" + spaces(25)).bytes);
		else if (frame.mid == 9999) replies.push(frame.bytes);
		else if (frame.mid != 62) replies.push(new OpenProtocolFrame(5,OpenProtocolFrame.digits(frame.mid,4)).bytes);
	}
	public static function spaces(count:Int):String { var result = ""; for (_ in 0...count) result += " "; return result; }
	public function receive():Bytes {
		if (closed) throw "Scripted controller disconnected";
		if (replies.length == 0) return Bytes.alloc(0);
		var reply = replies.shift(); return reply == null ? Bytes.alloc(0) : reply;
	}
	public function close():Void closed = true;
}

private class ScriptedScrewIo implements ScrewdriverIo {
	public final outputs:Array<Bool> = [false,false,false,false,false];
	public var latest:Null<ScrewdriverIoSample> = null;
	public function new() {}
	public function write(output:Int, value:Bool):Void outputs[output] = value;
	public function sample():Null<ScrewdriverIoSample> return latest;
	public function acquire(sequence:Int, timestamp:Float, result:Int = 0):Void {
		latest = {sequence:Int64.ofInt(sequence),acquiredAt:timestamp,clockId:"commissioned-host-clock",
			selectedProgramCode:(outputs[0] ? 1 : 0) + (outputs[1] ? 2 : 0),
			inputs:[true,result == 1,result > 1],values:[0.2,4*Math.PI,2,5,result == 0 ? 2 : 5,result,1]};
	}
}

/** Published revision-1 packets, including stream corruption and owner-clock failures. */
class ScrewControllerTests {
	static var count = 0;
	static function check(ok:Bool, message:String):Void { count++; if (!ok) throw message; }
	static function rejects(f:Void->Void):Void {
		var refused = false; try f() catch (_:Dynamic) refused = true;
		check(refused,"Malformed controller input must be refused");
	}
	static function result(id:String = "4294967295", ok:Bool = true):OpenProtocolFrame {
		var fields = ["0001","01",ScriptedTighteningController.spaces(25),ScriptedTighteningController.spaces(25),
			"00","001","0000","0000",ok ? "1" : "0","1","1","000000","002000","001650","001650",
			"00000","99999","00000","00720","2026-10-11:12:34:56","2026-10-11:12:34:56","2",id];
		var body = "";
		for (i in 0...fields.length) body += OpenProtocolFrame.digits(i + 1,2) + fields[i];
		return new OpenProtocolFrame(61,body);
	}
	static function initialized(server:ScriptedTighteningController):OpenProtocolClient {
		var client = new OpenProtocolClient(server);
		client.safe();
		for (i in 0...4) client.poll(i * 0.01);
		check(client.initialized && client.safeAcknowledged() && !client.ready(),
			'Handshake disables before subscribing: initialized=${client.initialized}, fault=${client.fault}, requests=${[for (request in server.requests) request.mid].join(",")}');
		check([for (request in server.requests) request.mid].join(",") == "1,42,60","Published handshake MIDs");
		return client;
	}
	/** The standalone entry also exercises TCP with the host's NativeKit library. */
	public static function main():Void {
		run();
		ScrewHardwareTests.native();
		var pureCount = count; tcp();
		trace('Open Protocol TCP controller tests passed (${count - pureCount} checks)');
	}
	public static function run():Void {
		count = 0;
		discrete();
		var encoded = new OpenProtocolFrame(18,"007").bytes;
		check(encoded.length == 24 && encoded.getString(0,11) == "00230018001" && encoded.get(23) == 0,"Length excludes NUL");
		for (split in 1...encoded.length) {
			var stream = new OpenProtocolStream();
			stream.feed(Bytes.view(encoded,0,split)); check(stream.next() == null,"Fragment must wait");
			stream.feed(Bytes.view(encoded,split,encoded.length - split));
			var frame = stream.next(); check(frame != null && frame.mid == 18,"Fragment reassembly");
			check(stream.next() == null,"Stream drains exactly one packet");
		}
		var joined = Bytes.alloc(encoded.length * 2);
		joined.blit(0,encoded,0,encoded.length); joined.blit(encoded.length,encoded,0,encoded.length);
		var stream = new OpenProtocolStream(); stream.feed(joined);
		check(stream.next() != null && stream.next() != null && stream.next() == null,"Coalesced packets");
		for (index in [0,4,8,11,12,18,23]) {
			var invalid = Bytes.alloc(encoded.length); invalid.blit(0,encoded,0,encoded.length); invalid.set(index,65);
			rejects(() -> { var frame = OpenProtocolFrame.decode(invalid); });
		}
		rejects(() -> {var frame = new OpenProtocolFrame(0);});
		rejects(() -> {var frame = new OpenProtocolFrame(18,"é");});
		rejects(() -> {var stream = new OpenProtocolStream(); stream.feed(Bytes.ofString("0019")); var frame = stream.next();});
		rejects(() -> {var stream = new OpenProtocolStream(); stream.feed(Bytes.alloc(20001));});
		var reading = new OpenProtocolResult(result());
		check(reading.id == 4294967295.0 && reading.program == 1 && reading.ok && reading.torqueNm == 16.5
			&& Math.abs(reading.angleRad - 4*Math.PI) < 1e-12,"Measured torque and angle units, uint32 ID without Int truncation");
		check(!new OpenProtocolResult(result("0000000001",false)).ok,"NOK remains unclassified");
		rejects(() -> {var value = new OpenProtocolResult(result("4294967296"));});
		var malformed = result().bytes; malformed.set(138,48); malformed.set(139,49);
		rejects(() -> {var value = new OpenProtocolResult(OpenProtocolFrame.decode(malformed));});
		var server = new ScriptedTighteningController(), client = initialized(server);
		rejects(() -> client.selectProgram(1000)); rejects(() -> client.enable());
		client.selectProgram(7); client.poll(0.04);
		check(!client.ready() && client.selectedProgram == 0,"Selection waits for acknowledgement");
		client.poll(0.05); check(client.selectedProgram == 7,"Acknowledged selection");
		client.enable(); client.poll(0.06); check(!client.ready(),"Enable waits for acknowledgement");
		client.poll(0.07); check(client.ready(),"Selected, subscribed and enabled");
		check(server.requests[3].mid == 18 && server.requests[3].bytes.getString(20,3) == "007"
			&& server.requests[4].mid == 43,"Parameter set and enable use published MIDs");
		server.replies.push(result("0000000001").bytes); client.poll(0.08);
		var first = client.nextResult(); check(first != null && first.torqueNm == 16.5,"Subscribed measured result");
		server.replies.push(result("0000000001").bytes); client.poll(0.09);
		check(client.nextResult() == null && server.requests[6].mid == 62,"Duplicate result acknowledged without republishing");
		client.safe(); check(!client.safeAcknowledged() && !client.ready(),"Safe waits for disable acknowledgement");
		client.poll(0.1); client.poll(0.11); check(client.safeAcknowledged(),"Disable acknowledged");
		client.poll(10.12); check(server.requests[server.requests.length - 1].mid == 9999,"Ten-second inactivity keep-alive");
		client.poll(10.13); check(client.fault == null,"Keep-alive echo accepted");
		client.poll(10.12); check(client.fault != null && server.closed,"Backwards owner clock disconnects");
		var lost = new ScriptedTighteningController(); lost.automatic = false;
		var timeout = new OpenProtocolClient(lost); timeout.poll(0); timeout.poll(0.5);
		check(timeout.fault != null && lost.closed,"Missing acknowledgement disconnects");
		var mismatch = new ScriptedTighteningController(); var wrong = initialized(mismatch); mismatch.automatic = false;
		wrong.selectProgram(7); wrong.poll(0.04); mismatch.replies.push(new OpenProtocolFrame(5,"0043").bytes); wrong.poll(0.05);
		check(wrong.fault != null,"Mismatched acknowledgement rejected");
		var rejected = new ScriptedTighteningController(); var error = initialized(rejected); rejected.automatic = false;
		error.selectProgram(7); error.poll(0.04); rejected.replies.push(new OpenProtocolFrame(4,"001802").bytes); error.poll(0.05);
		check(error.fault != null && error.fault.indexOf("18") >= 0 && error.fault.indexOf("2") >= 0,"Error names MID and published error code");
		var changed = new ScriptedTighteningController(); var duplicate = initialized(changed);
		changed.replies.push(result("0000000001").bytes); duplicate.poll(0.04);
		changed.replies.push(result("0000000001",false).bytes); duplicate.poll(0.05);
		check(duplicate.fault != null,"Changed duplicate tightening ID rejected");
		var late = new ScriptedTighteningController(); var stopping = initialized(late);
		stopping.selectProgram(1); stopping.poll(0.04); stopping.poll(0.05); stopping.enable(); stopping.poll(0.06);
		stopping.safe(); stopping.poll(0.07); check(!stopping.ready(),"Late enable acknowledgement cannot defeat queued disable");
		stopping.poll(0.08); check(stopping.safeAcknowledged(),"Late enable is followed by acknowledged disable");
		trace('Screw controller profile tests passed ($count checks)');
		ScrewHardwareTests.run();
	}
	static function profile():ScrewdriverIoProfile return new ScrewdriverIoProfile([0,1],
		[{slot:1,code:2},{slot:2,code:1}],2,3,4,0,1,2,true,"commissioned-host-clock",0.05);
	static function discrete():Void {
		var io = new ScriptedScrewIo(), driver = new DiscreteScrewdriver(io,profile());
		check(!driver.ready() && !io.outputs[2] && !io.outputs[3],"Discrete controller starts stopped");
		driver.selectProgram(1); check(!io.outputs[0] && io.outputs[1],"Commissioned LSB-first program code");
		rejects(() -> driver.start(true)); rejects(() -> driver.selectProgram(3));
		io.acquire(1,0); driver.poll(0); check(driver.ready(),"Fresh measured ready sample");
		var value = driver.reading(); check(value.angle == 4*Math.PI && value.torqueNm == 0.2 && value.strokeMm == 5,"Measured SI values preserved");
		value.torqueNm = 99; check(driver.reading().torqueNm == 0.2,"Feedback returns a copy");
		driver.start(true); check(io.outputs[2],"Forward output asserted");
		rejects(() -> driver.selectProgram(2)); rejects(() -> driver.reverse(true));
		driver.start(false); driver.reverse(true); check(io.outputs[2] && io.outputs[3],"Commissioned reverse also asserts start");
		driver.reverse(false); check(!io.outputs[2] && !io.outputs[3],"Reverse stop drops both outputs");
		driver.vacuum(true); driver.safe(); check(io.outputs[4],"Controlled stop retains vacuum");
		driver.selectProgram(2); check(io.outputs[0] && !io.outputs[1] && !driver.ready(),"New selection requires new acquisition");
		io.acquire(2,0.01); driver.poll(0.01); check(driver.ready(),"New acquisition after selection");
		driver.start(true); driver.poll(0.061);
		check(driver.faultDetail() != null && !driver.ready() && !io.outputs[2] && !io.outputs[3] && io.outputs[4],"Repeating sample goes stale and stops without losing vacuum");
		io.acquire(3,0.07); driver.poll(0.07); check(!driver.ready(),"Acquisition fault cannot silently restart");
		var badIo = new ScriptedScrewIo(), bad = new DiscreteScrewdriver(badIo,profile()); bad.selectProgram(1);
		badIo.acquire(1,0,4); bad.poll(0);
		check(bad.faultDetail() == null && bad.reading().result == 4,"NOK uses measured classification rather than guessing from its bit");
		var process = new ScrewdriverProcessDevice(bad,bad,{start:"start",direction:"reverse",program:"program",vacuum:"vacuum"},1,bad);
		process.prepare(); check(!process.ready() && process.fault() == "screwdriver: cross-thread","Shared process boundary carries measured NOK");
		var bitsIo = new ScriptedScrewIo(), bits = new DiscreteScrewdriver(bitsIo,profile()); bits.selectProgram(1);
		bitsIo.acquire(1,0); var inconsistent = bitsIo.latest;
		if (inconsistent == null) throw "Test acquisition missing";
		inconsistent.inputs[1] = true; bits.poll(0); check(bits.faultDetail() != null,"OK bit cannot fabricate a measured result");
		var clockIo = new ScriptedScrewIo(), clock = new DiscreteScrewdriver(clockIo,profile()); clockIo.acquire(1,0);
		var wrongClock = clockIo.latest; if (wrongClock == null) throw "Test acquisition missing";
		wrongClock.clockId = "other-clock"; clock.poll(0); check(clock.faultDetail() != null,"Clock domains cannot be mixed");
		var futureIo = new ScriptedScrewIo(), future = new DiscreteScrewdriver(futureIo,profile()); futureIo.acquire(1,1);
		future.poll(0); check(future.faultDetail() != null,"Future acquisition rejected");
		rejects(() -> {var p = new ScrewdriverIoProfile([0,0],[{slot:1,code:1}],2,3,4,0,1,2,false,"clock",0.05);});
		rejects(() -> {var p = new ScrewdriverIoProfile([0],[{slot:1,code:2}],2,3,4,0,1,2,false,"clock",0.05);});
		rejects(() -> {var p = new ScrewdriverIoProfile([0],[{slot:1,code:1},{slot:2,code:1}],2,3,4,0,1,2,false,"clock",0.05);});
	}
	static function tcp():Void {
		var runtime = NativeKitRuntime.start();
		var listener = NativeTransport.listen(36279);
		var accepted:Array<TransportHandle> = [];
		var subscription = runtime.events.listen(function(event:NativeKitEventValue) {
			switch event {
				case Raw(kind,source,_,_,_,_,data):
					if (kind == EventKind.TransportAccepted && source.rawValue() == listener.borrow().rawValue())
						accepted.push(new TransportHandle(NativeKitEventBytes.readU32(data,4)));
				case _: return;
			}
		});
		var client = OpenProtocolClient.connect("127.0.0.1",36279);
		var script = new ScriptedTighteningController(), stream = new OpenProtocolStream();
		var failure:Dynamic = null;
		try {
			var deadline = Sys.time() + 3;
			while (accepted.length == 0 && Sys.time() < deadline) {runtime.events.poll(); runtime.events.wait(0.001);}
			check(accepted.length == 1,"TCP scripted controller accepted");
			var stage = 0;
			while (stage < 4 && Sys.time() < deadline) {
				runtime.events.poll();
				var bytes = NativeTransport.receive(accepted[0],2048); if (bytes.length > 0) stream.feed(bytes);
				while (true) {var frame = stream.next(); if (frame == null) break; script.send(frame.bytes);}
				while (script.replies.length > 0) {
					var reply = script.receive();
					NativeTransport.send(accepted[0],Bytes.view(reply,0,5));
					NativeTransport.send(accepted[0],Bytes.view(reply,5,reply.length - 5));
				}
				client.poll(Sys.time());
				if (client.fault != null) throw client.fault;
				if (stage == 0 && client.initialized) {client.selectProgram(7); stage = 1;}
				else if (stage == 1 && client.selectedProgram == 7) {client.enable(); stage = 2;}
				else if (stage == 2 && client.ready()) {
					NativeTransport.send(accepted[0],result("0000000001").bytes); stage = 3;
				} else if (stage == 3) {
					var value = client.nextResult();
					if (value != null) {check(value.ok && value.torqueNm == 16.5,"TCP result decoded in measured Nm"); stage = 4;}
				}
				runtime.events.wait(0.001);
			}
			check(stage == 4,"TCP handshake, selection, enable and result finish before deadline");
			client.safe();
			while (!client.safeAcknowledged() && Sys.time() < deadline) {
				runtime.events.poll();
				var bytes = NativeTransport.receive(accepted[0],2048); if (bytes.length > 0) stream.feed(bytes);
				while (true) {var frame = stream.next(); if (frame == null) break; script.send(frame.bytes);}
				while (script.replies.length > 0) NativeTransport.send(accepted[0],script.receive());
				client.poll(Sys.time()); runtime.events.wait(0.001);
			}
			check(client.safeAcknowledged(),"TCP disable acknowledged after result");
			check([for (request in script.requests) request.mid].join(",") == "1,42,60,18,43,62,42","TCP uses exactly the commissioned protocol commands");
		} catch (error:Dynamic) failure = error;
		client.close();
		for (transport in accepted) NativeTransport.close(transport);
		subscription.dispose(); listener.close(); runtime.dispose();
		if (failure != null) throw failure;
	}
}
