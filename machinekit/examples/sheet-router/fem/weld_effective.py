"""Continuous-run effective lengths for the nominal 5 mm weld schedule.

Open starts/ends lose one throat each. Connected returns belong to the same
physical run; isolated sub-30 mm edge fragments never become structural welds.
"""
from collections import defaultdict
from copy import deepcopy
import math


def effective_stitches(native,stitches,physical_only=False):
    seams={s['id']:s for s in native['seams']};parts=defaultdict(list)
    for s in stitches:
        seam=seams[s['seam']];pair=tuple(sorted((seam['a']['member'],seam['b']['member'])))
        row=deepcopy(s);row['_physical']=deepcopy(s);parts[s['group'],pair].append(row)
    accepted=[];rejected=[]
    for (group,pair),lines in sorted(parts.items()):
        vertices=defaultdict(list)
        for i,s in enumerate(lines):
            for end in ('a','b'):vertices[tuple(round(v,4) for v in s[end])].append((i,end))
        pending=set(range(len(lines)))
        while pending:
            component=set();todo=[min(pending)]
            while todo:
                i=todo.pop()
                if i in component:continue
                component.add(i);pending.discard(i)
                for end in ('a','b'):todo += [j for j,_ in vertices[tuple(round(v,4) for v in lines[i][end])] if j not in component]
            throat=max(lines[i].get('legMm',5)/math.sqrt(2) for i in component)
            original=sum(lines[i]['lengthMm'] for i in component)
            # Trim along connected short returns before reaching a long edge.
            tips=[(i,end) for i in component for end in ('a','b') if len(vertices[tuple(round(v,4) for v in lines[i][end])])==1]
            for i,end in tips:
                # Credit full physical length only for an explicit run-on/off
                # tab detail, with full-size end sections verified after removal.
                if lines[i].get('fullSizeEnds',False):continue
                remaining=throat;visited=set()
                while remaining>1e-9 and i not in visited:
                    visited.add(i);s=lines[i];other='b' if end=='a' else 'a';length=math.dist(s['a'],s['b']);take=min(remaining,length)
                    vertex=tuple(round(v,4) for v in s[other]);s[end]=[s[end][k]+(s[other][k]-s[end][k])*take/length for k in range(3)] if length else list(s[other]);s['lengthMm']=max(0,length-take);remaining-=take
                    if remaining>1e-9:
                        following=[(j,e) for j,e in vertices[vertex] if j not in visited]
                        if len(following)!=1:break
                        i,end=following[0]
            length=sum(lines[i]['lengthMm'] for i in component)
            if length<max(30,6*throat)-1e-6:
                rejected.append(dict(group=group,pieces=pair,physicalRunMm=original,effectiveRunMm=length,reason='effective run shorter than max(30 mm, 6 throats); no structural weld credited'))
            else:
                for i in sorted(component):
                    if lines[i]['lengthMm']<=1e-7 and not physical_only:continue
                    row=lines[i]['_physical'] if physical_only else {k:v for k,v in lines[i].items() if k!='_physical'}
                    accepted.append(row)
    return accepted,rejected
