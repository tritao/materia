package machinekit.pendant;

import cadkit.modeling.Part;
import machinekit.assembly.MachineAssembly;
import machinekit.component.*;
import machinekit.hmi.*;

/** A control port the pendant board is wired to, and the board's own port for it. */
typedef PendantLink = {
  var control:String;
  /** The body of the expanded control that carries the port, or null when the control is one solid. */
  var body:Null<String>;
  var port:ComponentPort;
  var board:ComponentPort;
}

/**
 * The pendant's circuit board as a component: its envelope, and one port for each wired port of the face's controls, so
 * every control is wired to it. A board port, named `<control>-<port>`, has the control port's interface and the role
 * that serves it: the board supplies the switch contacts, the LEDs, the display and the wheel, and reads the wheel's two
 * phases. The E-stop's NC contacts are not on the board: they are wired into the cable's safety loop at the receptacle,
 * and the board only reads the loop back there.
 */
class PendantBoard extends PanelSolid {
  public function new(face:PanelLayout, build:() -> Part) {
    super("PANEL-BOARD", "pendant board", build, "FR4 PCB");
    for (link in links(face)) addPort(link.board);
  }

  /** Every control port of the face the board is wired to, with the board's port for it. */
  public static function links(face:PanelLayout):Array<PendantLink> {
    var result:Array<PendantLink> = [];
    for (c in face.controls) {
      if (Std.isOfType(c.part, EmergencyStop)) continue;
      var device:Null<HmiPart> = Std.isOfType(c.part, HmiPart) ? cast c.part : null;
      var bodies = device == null ? null : device.bodies();
      for (port in c.part.ports()) {
        var body:Null<String> = null;
        if (bodies != null) for (spec in bodies) for (p in spec.ports) if (p.name == port.name) body = spec.id;
        var role:PortRole = port.role == Supply ? Consumer : Supply;
        result.push({
          control: c.id,
          body: body,
          port: port,
          board: {
            name: c.id + "-" + port.name.toLowerCase(),
            kind: port.kind,
            role: role,
            iface: port.iface,
            required: role == Consumer
          }
        });
      }
    }
    return result;
  }

  /**
   * Wires every link in an assembly that PanelAssemblyBuilder built from `face` and `groups`, `board` being the board's
   * instance path. Each connection is named after the board's port.
   */
  public static function wire(assembly:MachineAssembly, face:PanelLayout, groups:Array<PanelGroup>, board:String):Void {
    for (link in links(face)) {
      var path = link.control;
      for (g in groups) if (g.members.indexOf(link.control) >= 0) path = g.id + "/" + link.control;
      if (link.body != null) path += "/" + link.body;
      assembly.connectPorts(link.board.name, path, link.port.name, board, link.board.name);
    }
  }
}
