#!/usr/bin/env python3
"""Compile and measure the artifact decoder in headless Chrome, excluding input transfer."""
import argparse
import base64
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('artifact', type=Path)
parser.add_argument('--chrome', default=shutil.which('google-chrome') or shutil.which('chromium'))
parser.add_argument('--output', type=Path, default=Path('/tmp/materia-artifact-browser-timings.json'))
args = parser.parse_args()
if not args.chrome:
    parser.error('Chrome is required; pass --chrome')
root = Path(__file__).resolve().parents[3]
haxeon = root / 'haxeon'
fixture = Path(__file__).resolve().parent
sources = root / 'projectkit/src'
env = os.environ.copy()
env['LD_LIBRARY_PATH'] = ':'.join([str(haxeon / 'out'), str(haxeon / '.tools/hashlink'), env.get('LD_LIBRARY_PATH', '')])
with tempfile.TemporaryDirectory(prefix='materia-artifact-browser-') as directory:
    directory = Path(directory)
    compiler = directory / 'compiler.hl'
    subprocess.run([str(haxeon / '.tools/haxe/haxe'), '-cp', 'src', '-main', 'compiler.tools.HaxeonCompiler', '-hl', str(compiler)], cwd=haxeon, env=env, check=True)
    modules = {}
    for target in ['wasm32', 'wasm-gc']:
        output = directory / (target + '.wasm')
        command = [str(haxeon / '.tools/hashlink/hl'), str(compiler), '--target=' + target, '--output=' + str(output),
                   '--entry=ArtifactDecodingBrowserBenchmark', '--root=' + str(sources), '--root=' + str(fixture)]
        command += ['--export=ArtifactDecodingBrowserBenchmark.' + name for name in ['initialize', 'setWord', 'setByte', 'decode']]
        command += sorted(str(path) for path in sources.rglob('*.hx')) + [str(fixture / 'ArtifactDecodingBrowserBenchmark.hx')]
        compiled = subprocess.run(command, cwd=haxeon, env=env, capture_output=True, text=True)
        if compiled.returncode:
            raise SystemExit(compiled.stdout + compiled.stderr)
        modules[target] = base64.b64encode(output.read_bytes()).decode()
    script = r'''
const data=Uint8Array.from(atob(ARTIFACT),c=>c.charCodeAt(0)), words=new DataView(data.buffer), results={};
for(const [target,encoded] of Object.entries(MODULES)) {
 const module=new WebAssembly.Module(Uint8Array.from(atob(encoded),c=>c.charCodeAt(0)));
 const imports={std:{sys_exit:c=>{throw Error('exit '+c)}},haxeon_runtime:{__math_floor:Math.floor,__math_is_finite:x=>Number.isFinite(x)?1:0}};
 const e=new WebAssembly.Instance(module,imports).exports, prefix='ArtifactDecodingBrowserBenchmark.';
 e[prefix+'initialize'](data.length); let offset=0;
 for(;offset+4<=data.length;offset+=4)e[prefix+'setWord'](offset,words.getInt32(offset,true));
 for(;offset<data.length;offset++)e[prefix+'setByte'](offset,data[offset]);
 const decode=e[prefix+'decode'], parts=decode(1), milliseconds=[];
 for(let i=0;i<5;i++) {const start=performance.now();if(decode(1)!==parts)throw Error('part count changed');milliseconds.push(performance.now()-start);}
 results[target]={sourceBytes:data.length,parts,milliseconds};
}
document.body.textContent=JSON.stringify(results);
'''.replace('ARTIFACT', json.dumps(base64.b64encode(args.artifact.read_bytes()).decode())).replace('MODULES', json.dumps(modules))
    html = directory / 'benchmark.html'
    html.write_text('<body>pending<script>try{' + script + '}catch(e){document.body.textContent="ERROR:"+e.stack}</script>')
    browser = subprocess.run([args.chrome, '--headless', '--no-sandbox', '--disable-gpu', '--dump-dom', html.as_uri()], capture_output=True, text=True, timeout=60)
    match = re.search(r'<body>(.*?)</body>', browser.stdout, re.DOTALL)
    if browser.returncode or not match:
        raise SystemExit(browser.stderr + browser.stdout)
    results = json.loads(match.group(1))
    args.output.write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))
