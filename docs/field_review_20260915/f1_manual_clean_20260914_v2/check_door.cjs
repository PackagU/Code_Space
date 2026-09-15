const fs=require('fs'),path=require('path');
const sharp=require('C:/Users/k/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/sharp');
const {readPGM}=require('./inspect_map.cjs');
const a=readPGM('../f1_manual_clean_20260914_v1/f1_manual_clean_v1.pgm'),b=readPGM('f1_manual_clean_v2.pgm');
const reg=JSON.parse(fs.readFileSync('registration.json'));
const toRaw=([x,y])=>[(x-reg.tx)/reg.sx,(y-reg.ty)/reg.sy];
const W=a.width,H=a.height,res=0.05,ox=-18.5,oy=-11.6;
const toWorld=([px,py])=>[ox+(px+0.5)*res, oy+(H-py-0.5)*res];
let diff={};for(let i=0;i<W*H;i++)if(a.data[i]!==b.data[i]){const k=a.data[i]+'->'+b.data[i];diff[k]=(diff[k]||0)+1;}
const L=toRaw([157,332]),M=toRaw([183,340.5]),R=toRaw([209,349]);
const d=(p,q)=>Math.hypot(p[0]-q[0],p[1]-q[1])*res;
// locker goal clearance to occupied cells in v2
const gx=-5.125,gy=-8.625,gp=[(gx-ox)/res-0.5,H-(gy-oy)/res-0.5];
let best=1e9;for(let y=0;y<H;y++)for(let x=0;x<W;x++)if(b.data[y*W+x]===0)best=Math.min(best,Math.hypot(x-gp[0],y-gp[1]));
console.log(JSON.stringify({diff,door_world:{wall_end:toWorld(L),mid:toWorld(M),open_end:toWorld(R)},door_total_m:d(L,R).toFixed(2),open_m:d(M,R).toFixed(2),locker_px:gp.map(v=>v.toFixed(1)),locker_to_wall_m:(best*res).toFixed(3)}));
// zoomed crop preview x3 with v2 map, changed cells red, locker blue
const x0=215,y0=300,cw=100,ch=123,S=4,out=Buffer.alloc(cw*S*ch*S*3);
for(let y=0;y<ch*S;y++)for(let x=0;x<cw*S;x++){const px=x0+Math.floor(x/S),py=y0+Math.floor(y/S),i=py*W+px,o=(y*cw*S+x)*3;
 let c=b.data[i]===127?[205,205,205]:[b.data[i],b.data[i],b.data[i]];
 if(a.data[i]!==b.data[i])c=[220,30,30];
 if(Math.hypot(px-gp[0],py-gp[1])<2.5)c=[30,90,230];
 out[o]=c[0];out[o+1]=c[1];out[o+2]=c[2];}
sharp(out,{raw:{width:cw*S,height:ch*S,channels:3}}).png().toFile('door_change_zoom.png');
