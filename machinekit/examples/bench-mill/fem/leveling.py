"""Level the four-spring model from its measured linear foot flexibility.

Rigid-reference RF output is not used to determine preload. The five-case deck
keeps the actual spring-supported boundary conditions, then measures each foot's
vertical response to unit forces at all four feet.
"""
from frame_solver import read_results, solve_small
from packet import digest


def add_influence_steps(path, model):
    stations = list(model['supportConstraints'])
    lines = [path.read_text().rstrip()]
    for station in stations:
        node = model['references']['foot-' + station]['translation']
        lines += ['*STEP', '** foot flexibility ' + station, '*STATIC', '*DLOAD,OP=NEW',
                  '*CLOAD,OP=NEW', f'{node}, 3, 1', '*NODE PRINT,NSET=OBS', 'U', '*NODE PRINT,NSET=SUPPORTS', 'RF', '*NODE PRINT,NSET=SUPPORTS', 'U', '*END STEP']
    path.write_text('\n'.join(lines) + '\n')
    model['inputSha256'] = digest(path)
    return model


def offsets_from_tables(model, tables):
    stations = list(model['supportConstraints'])
    nodes = [model['references']['foot-' + s]['translation'] for s in stations]
    heights = [tables[1, 'displacements', 'SUPPORTS'][n][2] for n in nodes]
    flexibility = [[tables[j+2, 'displacements', 'SUPPORTS'][n][2]
                    for j in range(len(nodes))] for n in nodes]
    scale = max(abs(v) for row in flexibility for v in row)
    symmetry = max(abs(flexibility[i][j]-flexibility[j][i]) for i in range(4) for j in range(4))/scale
    if symmetry > 1e-4:
        raise ValueError(f'Foot flexibility reciprocity failed: {symmetry}')
    preloads = solve_small(flexibility, [-v for v in heights])
    if any(v <= 0 for v in preloads):
        raise ValueError('Four-point leveling requires foot tension')
    offsets = {s: f/model['springNPerMm'] for s, f in zip(stations, preloads)}
    return offsets, {'method': 'Five-case spring-model displacement influence solve; no rigid-reference RF calibration',
        'unleveledFootHeightsMm': dict(zip(stations, heights)), 'footFlexibilityMmPerN': flexibility,
        'symmetryRelativeError': symmetry, 'preloadsN': dict(zip(stations, preloads))}
