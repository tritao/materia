import haxeon.platform.NativeKitRuntime;
import processkit.moonraker.MoonrakerClient;
import processkit.moonraker.MoonrakerSocket;
class MoonrakerWebCompile {
  static function main() {
    var runtime = NativeKitRuntime.start();
    var http = new MoonrakerClient("127.0.0.1",7125,runtime.events);
    var socket = new MoonrakerSocket("127.0.0.1",7125,runtime.events);
    socket.subscribe();
    http.status(value -> trace(value.state));
    while (runtime.events.poll()) {}
    http.poll(0); socket.poll(0);
    http.close(); socket.close(); runtime.dispose();
  }
}
