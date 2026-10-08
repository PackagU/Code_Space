// Deterministic occupancy-grid reconstruction, not a recovered SLAM posegraph.
// The user's editable vector geometry is the sole source for the new walls.
const fs=require('fs'), path=require('path'),crypto=require('crypto');
const sharp=require('C:/Users/k/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/sharp');
const {readPGM}=require('./inspect_map.cjs');
const root=__dirname,name='f1_manual_clean_v2';
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const srcPath=path.join(root,'source/f1_raw_20260914.pgm');
const expected='be58abad3f25fb597993021f694422ecb26b2204e5bc6ee1afa1181feecbfe73';
const source=readPGM(srcPath),rawHash=hash(fs.readFileSync(srcPath));
if(rawHash!==expected)throw Error('Raw map hash differs; review registration');
const reg=JSON.parse(fs.readFileSync(path.join(root,'registration.json')));
const g=JSON.parse(fs.readFileSync(path.join(root,'geometry.json')));
const toRaw=([x,y])=>[(x-reg.tx)/reg.sx,(y-reg.ty)/reg.sy];
const outer=g.outer_outline.map(toRaw),inner=g.overlapping_wall_polyline.map(toRaw),wedge=g.ambiguous_wedge.map(toRaw);
function inside(x,y,poly){let inPoly=false;for(let i=0,j=poly.length-1;i<poly.length;j=i++){
 const [xi,yi]=poly[i],[xj,yj]=poly[j];
 if((yi>y)!==(yj>y)&&x<(xj-xi)*(y-yi)/(yj-yi)+xi)inPoly=!inPoly;
}return inPoly;}
function segmentDistance(x,y,a,b){const dx=b[0]-a[0],dy=b[1]-a[1],den=dx*dx+dy*dy;
 const t=den?Math.max(0,Math.min(1,((x-a[0])*dx+(y-a[1])*dy)/den)):0;
 return Math.hypot(x-a[0]-t*dx,y-a[1]-t*dy);
}
const outerWalls=outer.flatMap((p,i)=>g.open_outer_edge_indices.includes(i)?[]:[[p,outer[(i+1)%outer.length]]]);
const walls=outerWalls.concat(inner.slice(1).map((p,i)=>[inner[i],p]));
const {width,height}=source,grid=Buffer.alloc(width*height,127);
for(let y=0;y<height;y++)for(let x=0;x<width;x++){
 let v=inside(x,y,outer)&&!inside(x,y,wedge)?254:127;
 if(walls.some(([a,b])=>segmentDistance(x,y,a,b)<=g.wall_width_raw_pixels/2))v=0;
 grid[y*width+x]=v;
}
const origYaml=fs.readFileSync(path.join(root,'source/f1_raw_20260914.yaml'),'utf8');
const yaml=origYaml.replace(/^image:.*$/m,`image: ${name}.pgm`);
if(yaml===origYaml)throw Error('Image YAML field unchanged');
// Validate trinary semantics against the actual source thresholds, not display gray.
const free=Number(origYaml.match(/^free_thresh:\s*(.+)$/m)[1]);
const occupied=Number(origYaml.match(/^occupied_thresh:\s*(.+)$/m)[1]);
const classify=v=>(1-v/255)>occupied?'occupied':(1-v/255)<free?'free':'unknown';
if(classify(0)!=='occupied'||classify(127)!=='unknown'||classify(254)!=='free')throw Error('Palette semantics');
const counts={occupied:0,free:0,unknown:0},changes={};
grid.forEach((v,i)=>{counts[classify(v)]++;if(v!==source.data[i]){
 const key=`${source.data[i]}->${v}`;changes[key]=(changes[key]||0)+1;
}});
// Flood-fill one-cell connectivity only; this does not establish robot clearance.
const pts=Object.fromEntries(Object.entries(g.checkpoints_annotation).map(([k,p])=>[k,toRaw(p).map(Math.round)]));
const visited=new Uint8Array(grid.length),queue=new Int32Array(grid.length);
let head=0,tail=0,start=pts.main_hall[1]*width+pts.main_hall[0];
if(grid[start]!==254)throw Error('Main hall not free');visited[start]=1;queue[tail++]=start;
while(head<tail){const i=queue[head++],x=i%width,y=Math.floor(i/width);
 for(const n of [x>0?i-1:-1,x+1<width?i+1:-1,y>0?i-width:-1,y+1<height?i+width:-1]){
  if(n>=0&&!visited[n]&&grid[n]===254){visited[n]=1;queue[tail++]=n;}
 }
}
const connectivity=Object.fromEntries(Object.entries(pts).map(([k,[x,y]])=>[k,!!visited[y*width+x]]));
if(Object.values(connectivity).some(v=>!v))throw Error('Disconnected intended regions');
const pgm=Buffer.concat([Buffer.from(`P5\n# Manual user-outline reconstruction. NOT field validated.\n${width} ${height}\n255\n`),grid]);
fs.writeFileSync(path.join(root,`${name}.pgm`),pgm);
fs.writeFileSync(path.join(root,`${name}.yaml`),yaml);
const svgPath=poly=>poly.map((p,i)=>`${i?'L':'M'} ${p[0].toFixed(3)} ${p[1].toFixed(3)}`).join(' ');
fs.writeFileSync(path.join(root,`${name}.svg`),`<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${height}" viewBox="0 0 ${width} ${height}"><rect width="100%" height="100%" fill="#cdcdcd"/><path d="${svgPath(outer)} Z" fill="#fefefe"/>${wedge.length?`<path d="${svgPath(wedge)} Z" fill="#cdcdcd"/>`:''}<path d="${walls.map(segment=>svgPath(segment)).join(' ')}" fill="none" stroke="black" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/></svg>\n`);
(async()=>{
 const visual=Buffer.from(grid.map(v=>v===127?205:v));
 await sharp(visual,{raw:{width,height,channels:1}}).png().toFile(path.join(root,`${name}.png`));
 const aw=422,ah=434,annotated=await sharp(path.join(root,'source/annotated_f1.png')).removeAlpha().raw().toBuffer();
 const aligned=Buffer.alloc(aw*ah*3),overlay=Buffer.alloc(aw*ah*3);
 for(let y=0;y<ah;y++)for(let x=0;x<aw;x++){
  const [rx,ry]=toRaw([x,y]).map(Math.round),i=(y*aw+x)*3;
  const v=rx>=0&&rx<width&&ry>=0&&ry<height?visual[ry*width+rx]:205;
  const red=annotated[i]>160&&annotated[i]>annotated[i+1]+60&&annotated[i]>annotated[i+2]+60;
  for(let c=0;c<3;c++){aligned[i+c]=v;overlay[i+c]=red?annotated[i+c]:v;}
 }
 await sharp(aligned,{raw:{width:aw,height:ah,channels:3}}).png().toFile(path.join(root,'clean_preview.png'));
 await sharp(overlay,{raw:{width:aw,height:ah,channels:3}}).png().toFile(path.join(root,'red_outline_check.png'));
 const report={status:'offline data/coordinate validation only; not activated; physical geometry unverified',
  source_pgm_sha256:rawHash,source_yaml_sha256:hash(Buffer.from(origYaml)),annotation_sha256:hash(fs.readFileSync(path.join(root,'source/annotated_f1.png'))),
  output_pgm_sha256:hash(pgm),output_yaml_sha256:hash(Buffer.from(yaml)),dimensions:[width,height],resolution_m:0.05,origin:[-18.5,-11.6,0],
  yaml_change:'image filename only; all map metadata preserved',registration:reg,wall_width_cells_proposed:g.wall_width_raw_pixels,
  classes:counts,pixel_transitions:changes,raw_205_classification:classify(205),new_127_classification:classify(127),
  connectivity_cells_only:connectivity,free_component_size:tail,
  polygon_raw_pixels:outer,wall_raw_pixels:inner,
  unknowns:['Exact real-world wall positions and dimensions','Actual exterior entrance door width','Obstacle omissions inside outline','AMCL scan alignment and robot clearance'],
  posegraph:'Not reconstructed or modified. Original posegraph does not match manual grid.',
  policies:{glass_door:g.glass_door_policy,overlap:g.overlap_policy,entrance:g.entrance_policy,elevator:g.elevator_policy}};
 fs.writeFileSync(path.join(root,'manifest.json'),JSON.stringify(report,null,2)+'\n');
 const files=[`${name}.pgm`,`${name}.yaml`,`${name}.png`,`${name}.svg`,'geometry.json','registration.json','manifest.json'];
 fs.writeFileSync(path.join(root,'SHA256SUMS.txt'),files.map(f=>`${hash(fs.readFileSync(path.join(root,f)))}  ${f}`).join('\n')+'\n');
 if(hash(fs.readFileSync(srcPath))!==rawHash)throw Error('Source mutated');
 console.log(JSON.stringify({dimensions:report.dimensions,counts,connectivity,raw_205:report.raw_205_classification,clean_127:report.new_127_classification,changed_cells:Object.values(changes).reduce((a,b)=>a+b,0)},null,2));
})();
