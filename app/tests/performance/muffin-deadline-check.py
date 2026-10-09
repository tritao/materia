# Diagnostic checks for an isolated Muffin 6.2.0 build only.
# Uses private symbols and verified ClutterStageCogl field offsets.
import ctypes as C,subprocess,json,sys
from pathlib import Path
lib=Path(sys.argv[1]);patched=sys.argv[2]=='patched'
L=C.CDLL(str(lib));O=C.CDLL('libgobject-2.0.so.0');G=C.CDLL('libglib-2.0.so.0')
L._clutter_stage_cogl_get_type.restype=C.c_size_t
O.g_object_new.argtypes=[C.c_size_t,C.c_void_p];O.g_object_new.restype=C.c_void_p
O.g_object_unref.argtypes=[C.c_void_p];G.g_get_monotonic_time.restype=C.c_int64
symbols={}
for line in subprocess.check_output(['nm','-an',str(lib)],text=True).splitlines():
 parts=line.split()
 if len(parts)==3 and parts[2] in ['_clutter_stage_cogl_get_type','clutter_stage_cogl_schedule_update']:symbols[parts[2]]=int(parts[0],16)
base=C.cast(L._clutter_stage_cogl_get_type,C.c_void_p).value-symbols['_clutter_stage_cogl_get_type']
schedule=C.CFUNCTYPE(None,C.c_void_p,C.c_int)(base+symbols['clutter_stage_cogl_schedule_update'])
typeid=L._clutter_stage_cogl_get_type();rows=[]
for name,offset,requested in [('future-urgent',1000000,-1),('due-urgent',-1000000,-1),('future-normal',1000000,2),('empty-urgent',None,-1),('future-other-urgent',1000000,-2)]:
 obj=O.g_object_new(typeid,None);now=G.g_get_monotonic_time();before=-1 if offset is None else now+offset
 update=C.c_int64.from_address(obj+0x38);nexttime=C.c_int64.from_address(obj+0x48);delay=C.c_int.from_address(obj+0x54)
 update.value=before;nexttime.value=-1 if offset is None else now+1200000;delay.value=2
 schedule(obj,requested);after=G.g_get_monotonic_time()
 expect_advance=offset is None or (patched and offset>0 and requested<0)
 assert (now<=update.value<=after) if expect_advance else update.value==before,(name,before,update.value,now,after)
 if expect_advance:assert nexttime.value==-1 and delay.value==requested,(name,nexttime.value,delay.value)
 rows.append(dict(case=name,advanced=expect_advance,passed=True));O.g_object_unref(obj)
print(json.dumps(dict(build=sys.argv[2],checks=rows),indent=2))
