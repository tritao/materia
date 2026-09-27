# Project UI extensions

A Materia project may provide an inspector panel and respond to selection through a local Haxeon command. The editor provides widgets and scene colours; the project owns the action logic and text. Projects without a `uiExtension` declaration use the normal inspector.

Declare the extension beside the existing build and entrypoint fields in `materia.project.json`:

```json
"uiExtension": {
  "kind": "haxeon-command",
  "protocol": "materia.project-ui.v1",
  "manifest": "haxeon.json",
  "artifact": "build/host/main.hl",
  "command": "ui-extension"
}
```

`manifest` and `artifact` resolve relative to the project file. When the project opens, Materia builds the Haxeon manifest once and invokes the resulting HashLink program with `<command> <request.json> <response.json>`. The executable must write one JSON response. Materia invokes it again for each button press or project-object selection. Each request contains the full ordered action history, so the project can reconstruct its state deterministically without an editor-owned inventory model.

Request:

```json
{"protocol":"materia.project-ui.v1","actions":[
  {"kind":"action","id":"start-order"},
  {"kind":"select","id":"project:rack-01/shelf-01/bin-01"}
]}
```

Response:

```json
{"protocol":"materia.project-ui.v1",
 "panel":{"title":"VIRTUAL PICKING STATION","actionAfter":2,
   "rows":[{"key":"status","text":"Status: Running"}],
   "actions":[{"id":"confirm-pick","label":"Confirm pick","enabled":true}]},
 "colours":[{"id":"project:rack-01/shelf-01/bin-01","r":1.0,"g":0.7,"b":0.16}],
 "selectScene":null}
```

`actionAfter` inserts the button row before the row at that zero-based index; omitting it places buttons last. Colours are RGB values from 0 to 1 and use scene object IDs. A `selectScene` ID lets an action clear or change the viewport selection. The host discards the extension instance when the document changes or the editor closes. `--project-action=ID` sends an initial action, useful for scripted captures.

The [picking-station example](../../machinekit/examples/picking-station/README.md) is an implementation of this protocol.
