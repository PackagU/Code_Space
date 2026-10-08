#!/usr/bin/env python3
"""Export small reviewable summaries/plots; bags and dense traces remain local."""
import csv
import json
import math
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
LOG=ROOT/'logs/real_map_sim'
FIELDS=['name','scenario','floor','params','action_results','truth_samples','min_wall_m',
        'corner_min_wall_m','collision_ahead','lethal_start','rpm_blocks','ready_drops',
        'elapsed_sim_s','elapsed_wall_s','controller_missed','pose_capture_exit',
        'pose_capture_raw_exit','analysis_exit','bag_contract_exit','error']


def main():
    destination=HERE/'results'
    destination.mkdir(exist_ok=True)
    results=[]
    for path in sorted(LOG.glob('S*/result.json')):
        data=json.loads(path.read_text())
        # Interrupted boot attempts do not count as completed scenario repetitions.
        if 'truth_samples' in data:
            results.append(data)
    with open(destination/'scenario_summary.csv','w',newline='') as handle:
        writer=csv.DictWriter(handle,fieldnames=FIELDS,extrasaction='ignore',lineterminator='\n')
        writer.writeheader()
        for data in results:
            row={k:data.get(k) for k in FIELDS}
            row['action_results']=';'.join(data['action_results'])
            writer.writerow(row)
    (destination/'scenario_summary.json').write_text(json.dumps(results,indent=2)+'\n')
    geometry=json.loads((HERE/'generated/geometry_validation.json').read_text())
    (destination/'geometry_validation.json').write_text(json.dumps(geometry,indent=2)+'\n')
    for label, pattern in [('mission_summary', 'M*/result.json'),
                           ('operational_summary', 'sim_s6*/result.json'),
                           ('localization_alignment', 'localization_alignment/S*.json')]:
        summaries=[]
        for path in sorted(LOG.glob(pattern)):
            data=json.loads(path.read_text())
            data.setdefault('name',path.stem if label=='localization_alignment' else path.parent.name)
            summaries.append(data)
        (destination/(label+'.json')).write_text(json.dumps(summaries,indent=2)+'\n')
    print('Completed scenario conditions:',len(results))
    for data in results:
        print(data['name'],data['action_results'],'wall=',data.get('min_wall_m'),
              'corner=',data.get('corner_min_wall_m'),'collision=',data.get('collision_ahead'),
              'lethal=',data.get('lethal_start'))
    if results:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from PIL import Image
        import yaml
        images=ROOT/'docs/sim/images'
        images.mkdir(parents=True,exist_ok=True)
        for floor in ('F1','F2'):
            if not any(data['floor']==floor for data in results):
                continue
            fig,ax=plt.subplots(figsize=(9,9))
            map_path=ROOT/('src/slam_pkg/maps/field/f1/f1_manual_clean_v3.yaml' if floor=='F1'
                          else 'src/slam_pkg/maps/field/f2/f2_nav_unknown_v1.yaml')
            cfg=yaml.safe_load(map_path.read_text())
            raster=np.asarray(Image.open(map_path.parent/cfg['image']))
            ox,oy,_=cfg['origin'];res=cfg['resolution']
            ax.imshow(raster,cmap='gray',vmin=0,vmax=255,extent=[ox,ox+res*raster.shape[1],oy,oy+res*raster.shape[0]],origin='upper')
            for data in results:
                if data['floor']!=floor:
                    continue
                csv_path=LOG/data['name']/'clearance.csv'
                if not csv_path.exists():
                    continue
                trace=np.genfromtxt(csv_path,delimiter=',',names=True)
                if not len(trace):
                    continue
                ax.plot(trace['true_x'],trace['true_y'],label=data['name'],linewidth=1)
            if floor=='F2':
                ax.add_patch(plt.Circle((-12.95,-5.25),3,fill=False,color='orange'))
            ax.set_aspect('equal');ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)')
            ax.set_title(f'{floor}: simulation ground truth trajectories')
            handles,labels=ax.get_legend_handles_labels()
            if handles:ax.legend(fontsize=7)
            fig.tight_layout();fig.savefig(images/f'{floor.lower()}_true_paths.png',dpi=150);plt.close(fig)


if __name__=='__main__':
    main()
