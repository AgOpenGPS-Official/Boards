// Web UI page, kept in a header so the .ino prototype generator does not parse the JavaScript.
#pragma once

const char PageRoot[] PROGMEM = R"rawliteral(<!doctype html>
<html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>AOG Machine</title>
<style>
body{font-family:system-ui,sans-serif;margin:0 auto;max-width:640px;padding:12px;background:#f4f1ea;color:#222}
h1{font-size:1.25em}h2{font-size:1em;margin:18px 0 6px}
.card{background:#fff;border-radius:8px;padding:10px 12px;box-shadow:0 1px 3px #0002}
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}
.io{border-radius:6px;padding:8px 4px;text-align:center;background:#ddd;font:inherit;font-size:.85em;border:0}
.on{background:#2a2;color:#fff}.in{background:#26c;color:#fff}
td{padding:2px 12px 2px 0}input[type=number]{width:4em}
</style></head><body>
<h1>AgOpenGPS Machine &ndash; Waveshare 8DI-8RO</h1>
<div class=card><table id=st></table></div>
<h2>Relays</h2>
<div class=card><div class=grid id=rel></div>
<p><label><input type=checkbox id=tm onchange="setTest()"> Test mode &ndash; click a relay to toggle it. Only possible while AgOpenGPS is not sending; ends as soon as it does.</label></p></div>
<h2>Digital inputs</h2>
<div class=card><div class=grid id=din></div></div>
<h2>Network</h2>
<div class=card><form method=post action=/network>
<input type=number name=ip0 id=ip0 min=0 max=255> . <input type=number name=ip1 id=ip1 min=0 max=255> . <input type=number name=ip2 id=ip2 min=0 max=255> . <span id=ip3></span>
<button>Save &amp; reboot</button></form></div>
<h2>Firmware update</h2>
<div class=card><form method=post action=/update enctype=multipart/form-data>
<input type=file name=firmware accept=.bin> <button>Upload</button></form>
<p>Relays switch off during the update.</p></div>
<script>
const F=['-'];for(let i=1;i<=16;i++)F.push('Section '+i);F.push('Hyd lower','Hyd raise','Tram right','Tram left','Geo stop');
let S={};
const fn=n=>n<F.length?F[n]:'#'+n;
async function load(){
 try{S=await(await fetch('/status')).json()}catch(e){return}
 st.innerHTML=`<tr><td>Firmware<td>${S.fw}<tr><td>IP<td>${S.ip}, link ${S.link?'up':'down'}<tr><td>AgOpenGPS<td>${S.aog?'receiving machine data':'no data'}<tr><td>Relay expander<td>${S.relayChip?'ok':'NOT FOUND'}<tr><td>Uptime<td>${S.uptime} s`;
 rel.innerHTML=S.pins.map((p,i)=>`<button class="io ${S.out>>i&1?'on':''}" onclick="toggle(${i})">R${i+1}<br>${fn(p)}</button>`).join('');
 din.innerHTML=[0,1,2,3,4,5,6,7].map(i=>`<div class="io ${S.in>>i&1?'in':''}">DI${i+1}</div>`).join('');
 tm.checked=S.test;tm.disabled=S.aog;
 if(ip0.value===''){ip0.value=S.net[0];ip1.value=S.net[1];ip2.value=S.net[2]}
 ip3.textContent=S.net[3];
}
async function setTest(){await fetch('/test?on='+(tm.checked?1:0));load()}
async function toggle(i){if(!S.test)return;await fetch('/test?ch='+i);load()}
load();setInterval(load,1000);
</script></body></html>)rawliteral";
