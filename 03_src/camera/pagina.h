#pragma once
static const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="es"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WRO · Camara</title>
<style>
body{font:16px system-ui;background:#111923;color:#eaf0f6;max-width:1050px;margin:24px auto;padding:0 16px}
h1{font-size:26px} .grid{display:flex;gap:20px;flex-wrap:wrap} section{flex:1;min-width:280px}
canvas{width:100%;image-rendering:pixelated;background:#000;border:1px solid #526274}
pre,#info{background:#202d3b;padding:16px;border-radius:8px;white-space:pre-wrap}
.hint{color:#b7c7d9} #warning{color:#ffd783} table{border-collapse:collapse;width:100%;margin:20px 0}
td,th{text-align:left;padding:8px;border-bottom:1px solid #526274}
</style>
<h1>WRO · Detector rojo / verde</h1>
<p>Envia <b>1</b> en el monitor serial para tomar una foto. Esta pagina muestra la ultima captura, sin internet.</p>
<p id="state">Esperando conexion...</p><p id="warning"></p>
<div class="grid"><section><h2>Foto y objetos</h2><canvas id="photo" width="160" height="120"></canvas></section>
<section><h2>Mascara de color</h2><canvas id="mask" width="160" height="120"></canvas></section></div>
<p class="hint">Solo se detecta dentro de las lineas amarillas. Las franjas grises se ignoran. La mascara muestra los pixeles de color de la zona activa; solo las regiones que pasan los filtros tienen rectangulo. Toca la foto para consultar HSV aproximado.</p>
<div id="pixel">HSV: selecciona un punto en la foto.</div>
<details><summary>Ajustar deteccion (sin volver a cargar el programa)</summary>
<p>Los ajustes se aplican a la proxima foto enviada con 1. Se pierden al reiniciar; anota los valores que funcionen.</p>
<form id="settings"><div id="fields"></div><button>Aplicar ajustes</button></form><p id="saved"></p>
</details>
<pre id="result">Sin captura.</pre><div id="info"></div><div id="objects"></div>
<script>
const el=id=>document.getElementById(id), photo=el('photo'), mask=el('mask');
const original=document.createElement('canvas'); let seen=-1, config='';
const name=c=>c===1?'ROJO':'VERDE';
async function poll(){
 try {
  const status=await fetch('/status',{cache:'no-store'}).then(r=>r.json());
  if(status.error){el('state').textContent='Error: '+status.error+' · La imagen anterior, si existe, no es una captura nueva.';}
  else el('state').textContent=status.id?'Captura #'+status.id+' · Lista. Envia 1 para actualizar.':'Listo. Envia 1 por serial.';
  if(status.id && status.id!==seen){
   const response=await fetch('/snapshot',{cache:'no-store'}); if(!response.ok) throw Error('No se pudo leer la captura');
   const buffer=await response.arrayBuffer(), bytes=new Uint8Array(buffer);
   const length=new DataView(buffer).getUint32(0,true);
   const d=JSON.parse(new TextDecoder().decode(bytes.subarray(4,4+length)));
   const start=4+length, jpg=bytes.subarray(start,start+d.jpg);
   const bitmap=await createImageBitmap(new Blob([jpg],{type:'image/jpeg'}));
   for(const c of [photo,mask,original]){c.width=d.w;c.height=d.h;}
   original.getContext('2d').drawImage(bitmap,0,0); bitmap.close();
   const ctx=photo.getContext('2d');ctx.drawImage(original,0,0);
   const mctx=mask.getContext('2d'), pixels=mctx.createImageData(d.w,d.h);
   const labels=bytes.subarray(start+d.jpg);
   for(let i=0;i<d.w*d.h;i++){const c=labels[i]&3;pixels.data[i*4]=c===1?255:0;pixels.data[i*4+1]=c===2?235:0;pixels.data[i*4+3]=255;}
   mctx.putImageData(pixels,0,0);
   const roiTop=Math.max(0,Math.min(d.h,Number(d.roiTop)||0));
   const roiLeft=Math.max(0,Math.min(d.w,Number(d.roiLeft)||0));
   const roiRight=Math.max(roiLeft,Math.min(d.w,d.roiRight==null?d.w:Number(d.roiRight)));
   for(const c of [ctx,mctx]){
    c.save();c.fillStyle='rgba(90,90,90,0.55)';
    c.fillRect(0,0,d.w,roiTop);
    c.fillRect(0,roiTop,roiLeft,d.h-roiTop);
    c.fillRect(roiRight,roiTop,d.w-roiRight,d.h-roiTop);
    c.strokeStyle='#ffd54a';c.lineWidth=1;c.beginPath();
    c.moveTo(roiLeft+.5,d.h);c.lineTo(roiLeft+.5,roiTop+.5);
    c.lineTo(roiRight-.5,roiTop+.5);c.lineTo(roiRight-.5,d.h);c.stroke();
    c.restore();
   }
   d.objects.forEach((o,i)=>{
    for(const c of [ctx,mctx]){c.strokeStyle=o.c===1?'#ff5959':'#40ff88';c.lineWidth=1;c.strokeRect(o.x+.5,o.y+.5,o.w-1,o.h-1);}
    const label=name(o.c)+' '+o.w+'x'+o.h;
    ctx.font='7px monospace';const x=Math.min(o.x,Math.max(0,d.w-ctx.measureText(label).width-2)), y=Math.max(8,o.y-2);
    ctx.fillStyle='#000';ctx.fillRect(x,y-7,ctx.measureText(label).width+2,9);ctx.fillStyle='#fff';ctx.fillText(label,x+1,y);
   });
   let result='Detectados: '+d.objects.length;
   if(d.objects.length)result+='\nMas cercano: '+name(d.objects[0].c);
   if(d.objects.length>1)result+='\nMas lejano: '+name(d.objects[d.objects.length-1].c);
   el('result').textContent=result;
   el('warning').textContent=d.uncertain?'Orden de distancia incierto: alturas similares o un objeto toca el borde de la imagen o de la zona de deteccion. En empate se ordena de izquierda a derecha.':'';
   el('info').textContent='Captura #'+d.id+' | '+d.w+' x '+d.h+' px | Captura y analisis: '+d.ms+' ms\n'+d.config;
   el('objects').innerHTML='<table><tr><th>Objeto</th><th>Color</th><th>Ancho x alto</th><th>Area de color</th></tr>'+d.objects.map((o,i)=>'<tr><td>'+(i===0?'Mas cercano':i===d.objects.length-1?'Mas lejano':'Intermedio')+'</td><td>'+name(o.c)+'</td><td>'+o.w+' x '+o.h+' px</td><td>'+o.area+' px</td></tr>').join('')+'</table>';
   seen=d.id;
  }
 }catch(e){el('state').textContent='Sin respuesta. Revisa la conexion a WRO-CAM. '+e.message;}
 setTimeout(poll,1000);
}
photo.onclick=e=>{
 if(seen<0)return;const rect=photo.getBoundingClientRect();
 const x=Math.min(photo.width-1,Math.max(0,Math.floor((e.clientX-rect.left)*photo.width/rect.width)));
 const y=Math.min(photo.height-1,Math.max(0,Math.floor((e.clientY-rect.top)*photo.height/rect.height)));
 const [r,g,b]=original.getContext('2d').getImageData(x,y,1,1).data;
 const hi=Math.max(r,g,b),lo=Math.min(r,g,b),delta=hi-lo;let h=0;
 if(delta){h=hi===r?60*(g-b)/delta:hi===g?120+60*(b-r)/delta:240+60*(r-g)/delta;if(h<0)h+=360;}
 el('pixel').textContent=`Punto (${x},${y}): H=${Math.round(h)} grados, S=${hi?Math.round(delta*255/hi):0}, V=${hi}. Lectura aproximada del navegador.`;
};
const fields=[['rs','S minima ROJO',0,255],['gs','S minima VERDE',0,255],['v','Brillo minimo',0,255],['area','Area minima (px)',1,19200],['height','Altura minima (px)',1,120],['rhmax','Rojo: extremo bajo H (0 a este valor)',0,60],['rhmin','Rojo: extremo alto H (este valor a 359)',270,359],['ghmin','Verde: H minimo',40,200],['ghmax','Verde: H maximo',40,200]];
el('fields').innerHTML=fields.map(([id,label,min,max])=>`<label style="display:block;margin:10px 0">${label} <input style="width:85px" required type="number" step="1" name="${id}" id="${id}" min="${min}" max="${max}"></label>`).join('');
async function loadSettings(){try{const r=await fetch('/settings',{cache:'no-store'});if(!r.ok)throw Error();const d=await r.json();fields.forEach(([id])=>el(id).value=d[id]);}catch(e){el('saved').textContent='No se pudieron leer los ajustes. Recarga la pagina.';}}
el('settings').onsubmit=async e=>{e.preventDefault();try{const r=await fetch('/settings',{method:'POST',body:new URLSearchParams(new FormData(e.target))});if(!r.ok)throw Error(await r.text());el('saved').textContent='Aplicados en RAM. Envia 1 para ver el efecto en una nueva foto.';}catch(e){el('saved').textContent='No se aplicaron: '+e.message;}};
loadSettings();poll();
</script></html>
)HTML";
