import struct, subprocess, sys, json
import numpy as np

def decompress(b):
    if b[:4]!=b'RSCC': return b
    mode,block,total=struct.unpack_from('<III',b,4)
    n=(total+block-1)//block
    sizes=struct.unpack_from('<%dI'%n,b,16)
    p=16+4*n
    out=bytearray()
    for s in sizes:
        chunk=b[p:p+s]; p+=s
        if mode==2:
            out+=subprocess.run(['zstd','-d','-c'],input=chunk,capture_output=True,check=True).stdout
        else: raise Exception('mode %d'%mode)
    return bytes(out[:total])

class R:
    def __init__(s,b): s.b=b; s.p=0
    def u32(s): v=struct.unpack_from('<I',s.b,s.p)[0]; s.p+=4; return v
    def i32(s): v=struct.unpack_from('<i',s.b,s.p)[0]; s.p+=4; return v
    def u64(s): v=struct.unpack_from('<Q',s.b,s.p)[0]; s.p+=8; return v
    def i64(s): v=struct.unpack_from('<q',s.b,s.p)[0]; s.p+=8; return v
    def f32(s): v=struct.unpack_from('<f',s.b,s.p)[0]; s.p+=4; return v
    def f64(s): v=struct.unpack_from('<d',s.b,s.p)[0]; s.p+=8; return v
    def str(s):
        l=s.u32(); v=s.b[s.p:s.p+l].rstrip(b'\0').decode('utf8','replace'); s.p+=l; return v

def parse(b):
    r=R(b)
    assert b[:4]==b'RSRC'; r.p=4
    big=r.u32(); real64=r.u32(); vmaj=r.u32(); vmin=r.u32(); fmt=r.u32()
    rtype=r.str()
    r.u64(); flags=r.u32(); r.u64()
    if flags&8: r.str()   # script class
    for _ in range(11): r.u32()
    nstr=r.u32(); strings=[r.str() for _ in range(nstr)]
    next_=r.u32()
    ext=[]
    for _ in range(next_):
        t=r.str(); path=r.str()
        if flags&2: r.u64()  # uid
        ext.append((t,path))
    nint=r.u32(); ints=[]
    for _ in range(nint):
        path=r.str(); ofs=r.u64(); ints.append((path,ofs))
    res={}
    def variant():
        t=r.u32()
        if t==1: return None
        if t==2: return bool(r.u32())
        if t==3: return r.i32()
        if t==4: return r.f64() if real64 else r.f32()
        if t==5: return r.str()
        if t==10: return [r.f32(),r.f32()]
        if t==12: return [r.f32() for _ in range(3)]
        if t==14: return [r.f32() for _ in range(4)]
        if t==20: return [r.f32() for _ in range(4)]
        if t==22:
            nc=struct.unpack_from('<H',r.b,r.p)[0]; sc=struct.unpack_from('<H',r.b,r.p+2)[0]; r.p+=4
            sc&=0x7fff
            def sref():
                i=r.u32()
                if i&0x80000000:
                    l=i&0x7fffffff; v=r.b[r.p:r.p+l].rstrip(b'\0').decode(); r.p+=l; return v
                return strings[i]
            names=[sref() for _ in range(nc)]; subs=[sref() for _ in range(sc)]
            return ('/'.join(names))+(':'+':'.join(subs) if subs else '')
        if t==24:
            sub=r.u32()
            if sub==0: return None
            if sub==1:
                r.str(); r.str(); return ('ext',)
            if sub==2:
                idx=r.u32(); return ('int',idx)
            if sub==3:
                idx=r.u32(); return ('extidx',idx)
        if t==26:
            n=r.u32()&0x7fffffff
            d={}
            for _ in range(n):
                k=variant(); v=variant(); d[k if not isinstance(k,list) else tuple(k)]=v
            return d
        if t==30:
            n=r.u32()&0x7fffffff
            return [variant() for _ in range(n)]
        if t==31:
            n=r.u32(); v=r.b[r.p:r.p+n]; r.p+=n+((4-n%4)%4); return v
        if t==32:
            n=r.u32(); v=np.frombuffer(r.b,'<i4',n,r.p); r.p+=4*n; return v
        if t==33:
            n=r.u32(); v=np.frombuffer(r.b,'<f4',n,r.p); r.p+=4*n; return v
        if t==34:
            n=r.u32(); return [r.str() for _ in range(n)]
        if t==35:
            n=r.u32(); v=np.frombuffer(r.b,'<f4',n*3,r.p).reshape(n,3); r.p+=12*n; return v
        if t==40: return r.i64()
        if t==41: return r.f64()
        if t==44: return r.str()
        if t==48:
            n=r.u32(); v=np.frombuffer(r.b,'<i8',n,r.p); r.p+=8*n; return v
        if t==49:
            n=r.u32(); v=np.frombuffer(r.b,'<f8',n,r.p); r.p+=8*n; return v
        raise Exception('variant %d at %d'%(t,r.p))
    return r,strings,ints,variant,rtype

def read_resources(b):
    r,strings,ints,variant,rtype=parse(b)
    out={}
    for path,ofs in ints:
        r.p=ofs
        t=r.str(); n=r.u32(); props={}
        for _ in range(n):
            name=strings[r.u32()]
            props[name]=variant()
        out[path]=(t,props)
    return out


def read_pck(path):
    """The files of a Godot 4 .pck as {path: bytes}."""
    data = open(path, 'rb').read()
    assert data[:4] == b'GDPC', 'not a Godot pack'
    base = struct.unpack_from('<Q', data, 24)[0]
    pos = 32 + 64
    count = struct.unpack_from('<I', data, pos)[0]
    pos += 4
    files = {}
    for _ in range(count):
        length = struct.unpack_from('<I', data, pos)[0]
        pos += 4
        name = data[pos:pos + length].rstrip(b'\0').decode()
        pos += length
        offset, size = struct.unpack_from('<QQ', data, pos)
        pos += 16 + 16 + 4
        files[name] = data[base + offset:base + offset + size]
    return files


def animations(blob):
    """The Animation resources of an AnimationLibrary resource (plain or compressed), by clip name."""
    compressed = blob[:4] == b'RSCC'
    data = decompress(blob)
    if compressed:
        # A compressed resource's stream starts after the magic, and its resources are addressed from there.
        data = b'RSRC' + data[:-4]
    reader, strings, resources, variant, _ = parse(data)
    found = {}
    for _path, offset in resources:
        reader.p = offset + (4 if compressed else 0)
        kind = reader.str()
        count = reader.u32()
        props = {}
        for _ in range(count):
            key = strings[reader.u32()]
            props[key] = variant()
        if kind == 'Animation':
            found[props['resource_name']] = props
    return found
