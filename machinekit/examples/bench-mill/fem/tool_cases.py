"""Reduce verified FE reference-point studies into decision and contextual tool cases."""
import argparse
import json
from pathlib import Path
from compliance_budget import budget,tool_case_packet
from compliance import reduce
from packet import digest


def collect(directory,inputs_path,config_path):
    inputs=json.loads(inputs_path.read_text());study=json.loads(config_path.read_text());rows=[];sources={};frame_packet=None
    for case in inputs['toolCases']:
        source_case=case['id'].replace('-carbide','-steel')
        path=directory/source_case/'report.json';report=json.loads(path.read_text())
        if frame_packet is None:frame_packet=report['manifestSha256']
        if frame_packet!=report['manifestSha256']:raise ValueError('Tool cases must use the same frozen CAD packet')
        if not report['converged']:raise ValueError('Unconverged FE case: '+source_case)
        if report['budgetConfigurationSha256']!=digest(inputs_path) or report['configurationSha256']!=digest(config_path):
            raise ValueError('Budget/config input differs from FE report')
        manifest=Path(report['frozenManifestPath']);packet=json.loads(manifest.read_text())
        if digest(manifest)!=report['manifestSha256']:raise ValueError('Changed frozen packet')
        packet=tool_case_packet(packet,inputs,case['id'])
        if packet['datums']['toolTipMm']!=report['analysisContactMm']:raise ValueError('Tool case requires a different FE contact point')
        supports={}
        for name,result in report['meshes'][-1]['supports'].items():
            result=budget(packet,study,inputs,result['solution'])
            groups=result['componentComplianceUmPerN'];total=result['complianceUmPerN']
            machine=[[total[i][j]-groups['tool'][i][j] for j in range(3)] for i in range(3)]
            # Tool holder is absent at spindle nose; test bar retains only taper contact.
            casting=result['frameEnergyDiagonalUmPerN']
            shares={a:sum(casting[a][g] for g in ('BASE_CASTING','COLUMN_CASTING'))/machine[i][i] for i,a in enumerate('xyz')}
            result.update(machineLoopExcludingTool=reduce(machine,study['cuttingForceN']),castingMachineLoopShares=shares,
                geometryEligible=(max(shares.values())>=study['frameShareTarget']['maximumFraction']) if case['decisionCase'] else None)
            supports[name]=result
        rows.append({'toolCase':case,'supports':supports,'sourceReport':str(path),'sourceReportSha256':digest(path)})
        sources[source_case]=digest(path)
    decision=[r for r in rows if r['toolCase']['decisionCase']]
    above=any(s['geometryEligible'] for r in decision for s in r['supports'].values())
    return {'status':'Tool-case budget gate passed; travel and modal gates remain separate',
        'decisionBasis':'Machine loop excluding tool; spindle-nose and short stiff test bar under both supports',
        'castingShareReachesTarget':above,'frameShareTarget':study['frameShareTarget'],'cases':rows,
        'inputSha256':digest(inputs_path),'configurationSha256':digest(config_path),'sourceReports':sources}


def markdown(report):
    lines=['# Machine-loop and tool-case compliance budget','',report['decisionBasis'],'',
        '| Tool case | Support | Casting X/Y/Z, excluding tool | Total worst ±500 N box | Machine-loop worst box |',
        '| --- | --- | --- | --- | --- |']
    for row in report['cases']:
        for name,result in row['supports'].items():
            shares='/'.join(f'{100*result["castingMachineLoopShares"][a]:.2f}%' for a in 'xyz')
            lines.append(f'| {row["toolCase"]["id"]} | {name} | {shares} | {result["worstLoadBoxCorner"]["normUm"]:.3f} µm | {result["machineLoopExcludingTool"]["worstLoadBoxCorner"]["normUm"]:.3f} µm |')
    for row in report['cases']:
        lines+=['',f'## {row["toolCase"]["id"]}','',json.dumps(row['toolCase']), '']
        for name,result in row['supports'].items():
            lines += [f'### {name}','','| Component | X share | Y share | Z share |','| --- | --- | --- | --- |']
            for group in result['componentComplianceUmPerN']:
                lines.append('| '+group+' | '+' | '.join(f'{100*result["directionalShares"][a][group]:.3f}%' for a in 'xyz')+' |')
            lines+=['','FE energy groups (directional diagonals; µm/N):','',
                '| Group | X | Y | Z |','| --- | --- | --- | --- |']
            for group in result['frameEnergyDiagonalUmPerN']['x']:
                values=[result['frameEnergyDiagonalUmPerN'][a][group] for a in 'xyz']
                lines.append('| '+group+' | '+' | '.join(f'{v:.6g}' for v in values)+' |')
            lines+=['',f'FE report: {row["sourceReport"]}; SHA256 {row["sourceReportSha256"]}','']
    lines+=['','Catalogue values remain hardware proxies. Carbide E=600 GPa and cutter stick-outs are explicit study assumptions. No mode/excitation or geometry acceptance follows from this budget alone.']
    return '\n'.join(lines)+'\n'

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    parser.add_argument('--inputs',type=Path,default=Path(__file__).with_name('compliance-budget.json'))
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('frame-study.json'));args=parser.parse_args()
    result=collect(args.directory,args.inputs,args.config)
    (args.directory/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    (args.directory/'report.md').write_text(markdown(result))
