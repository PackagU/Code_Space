// Read-only image registration diagnostics and lossless previews for occupancy data.
const fs = require('fs');
const path = require('path');
const sharp = require('C:/Users/k/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/sharp');
const root = __dirname;
function readPGM(file) {
  const b = fs.readFileSync(file); let i = 0;
  function token() {
    while (i < b.length) {
      if (b[i] === 35) { while (b[i] !== 10 && i < b.length) i++; }
      else if (b[i] <= 32) i++; else break;
    }
    const begin = i; while (b[i] > 32 && i < b.length) i++;
    return b.subarray(begin, i).toString('ascii');
  }
  if (token() !== 'P5') throw Error('P5 required');
  const width = Number(token()), height = Number(token()), max = Number(token());
  if(max !== 255) throw Error('8-bit expected');
  if(b[i] === 13 && b[i+1] === 10) i += 2; else i++;
  const data=b.subarray(i);
  if(data.length !== width*height) throw Error('raster size');
  return {data,width,height};
}
module.exports={readPGM};
if(require.main===module) (async()=>{
  const raw=readPGM(path.join(root,'source/f1_raw_20260914.pgm'));
  await sharp(raw.data,{raw:{width:raw.width,height:raw.height,channels:1}}).png().toFile(path.join(root,'source/raw_preview.png'));
  const meta=await sharp(path.join(root,'source/annotated_f1.png')).metadata();
  console.log(JSON.stringify({raw:[raw.width,raw.height],annotation:[meta.width,meta.height]}));
})();
