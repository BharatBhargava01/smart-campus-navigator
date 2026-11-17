"""Generate an interactive Leaflet map (outputs/campus_map.html).

The page embeds the campus graph and re-implements the same state-space
search (BFS / UCS / Greedy / A*, modes walk / 2-wheeler / car with parking)
in JavaScript so it can be demoed live: pick start, destination, vehicle and
algorithm; click any path segment to close / reopen it.

The browser version uses base speeds only (no congestion model); the Python
package is the reference implementation.
"""
from __future__ import annotations

import json
from pathlib import Path

from .problem import PARK_TIME, PARKING_RULES, SPEEDS


def export_graph(g):
    return {
        "nodes": [[n.id, round(n.lat, 7), round(n.lon, 7), n.kind, n.name or ""] for n in g.nodes.values()],
        "edges": [[e.id, e.u, e.v, round(e.length, 2), e.layer, sorted(e.modes),
                   round(e.grade, 4), e.stairs] for e in g.edges.values()],
        "places": {k: {"label": p["label"], "nodes": p["nodes"]} for k, p in g.places.items()},
        "speeds": SPEEDS, "park_time": PARK_TIME, "parking_rules": PARKING_RULES,
    }


HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SAU Campus Navigator</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
 :root{--bg:#f6f5f2;--panel:#fff;--ink:#1d1d1f;--muted:#6b6b70;--line:#e2e0da;--accent:#2f5d9e}
 @media (prefers-color-scheme:dark){:root{--bg:#141416;--panel:#1e1e21;--ink:#ececef;--muted:#9a9aa2;--line:#2e2e33;--accent:#7aa7e6}}
 *{box-sizing:border-box} body{margin:0;font:14px/1.45 system-ui,-apple-system,Segoe UI,sans-serif;background:var(--bg);color:var(--ink)}
 #app{display:grid;grid-template-columns:330px 1fr;height:100vh}
 #side{background:var(--panel);border-right:1px solid var(--line);padding:16px;overflow:auto}
 #map{height:100%}
 h1{font-size:17px;margin:0 0 2px} .sub{color:var(--muted);font-size:12px;margin-bottom:14px}
 label{display:block;font-size:12px;color:var(--muted);margin:10px 0 3px}
 select,button{width:100%;padding:7px 8px;border:1px solid var(--line);border-radius:7px;background:var(--bg);color:var(--ink);font:inherit}
 .row{display:flex;gap:8px}.row>*{flex:1}
 .chk{display:flex;align-items:center;gap:6px;margin-top:10px;font-size:13px;color:var(--ink)} .chk input{width:auto}
 button{background:var(--accent);color:#fff;border:none;margin-top:14px;cursor:pointer;font-weight:600}
 button.ghost{background:transparent;color:var(--accent);border:1px solid var(--line);font-weight:500;margin-top:8px}
 #out{margin-top:14px;font-size:13px} .stat{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin:8px 0}
 .stat div{background:var(--bg);border-radius:7px;padding:6px 8px} .stat b{display:block;font-size:15px}
 ol{padding-left:18px;margin:6px 0} .note{font-size:11.5px;color:var(--muted);margin-top:12px}
 .legend span{display:inline-block;width:12px;height:4px;border-radius:2px;margin-right:4px;vertical-align:middle}
 @media (max-width:760px){#app{grid-template-columns:1fr;grid-template-rows:auto 60vh;height:auto}#side{border-right:none}}
</style></head><body>
<div id="app"><aside id="side">
 <h1>SAU Campus Navigator</h1><div class="sub">Search over (location, mode) on the surveyed campus graph</div>
 <label>From</label><select id="from"></select>
 <label>To</label><select id="to"></select>
 <div class="row"><div><label>Vehicle</label><select id="veh"><option value="walk">On foot</option><option value="bike">2-wheeler</option><option value="car">4-wheeler</option></select></div>
 <div><label>Role</label><select id="role"><option>student</option><option>faculty</option><option>resident</option><option>visitor</option></select></div></div>
 <div class="row"><div><label>Algorithm</label><select id="alg"><option value="astar">A*</option><option value="ucs">UCS (Dijkstra)</option><option value="greedy">Greedy best-first</option><option value="bfs">BFS</option></select></div>
 <div><label>Minimise</label><select id="metric"><option value="time">time</option><option value="distance">distance</option></select></div></div>
 <label class="chk"><input type="checkbox" id="wc"> Wheelchair (avoid grade &gt; 1:12 and stairs)</label>
 <label class="chk"><input type="checkbox" id="expl" checked> Show expanded nodes</label>
 <button id="go">Find route</button>
 <button class="ghost" id="clear">Reopen all closures</button>
 <div id="out"></div>
 <div class="note legend"><span style="background:#1b9e77"></span>walk <span style="background:#d95f02"></span>2-wheeler <span style="background:#c2185b"></span>car <span style="background:#d32f2f;height:6px"></span>closed<br>
 Click any path segment to close or reopen it, then search again. If you start at a building with a vehicle, it is assumed parked at the nearest parking spot.</div>
</aside><div id="map"></div></div>
<script>
const D = __DATA__;
const N = {}, E = {}, ADJ = {};
D.nodes.forEach(([id,lat,lon,kind,name]) => { N[id] = {id,lat,lon,kind,name}; ADJ[id] = []; });
D.edges.forEach(([id,u,v,len,layer,modes,grade,stairs]) => { E[id] = {id,u,v,len,layer,modes:new Set(modes),grade,stairs}; ADJ[u].push(id); ADJ[v].push(id); });
const closed = new Set();
const R = 6371008.8, rad = x => x*Math.PI/180;
function hav(a,b){const dp=rad(b.lat-a.lat),dl=rad(b.lon-a.lon);const h=Math.sin(dp/2)**2+Math.cos(rad(a.lat))*Math.cos(rad(b.lat))*Math.sin(dl/2)**2;return 2*R*Math.asin(Math.sqrt(h));}

const map = L.map('map',{zoomControl:true}).setView([28.4850,77.2000],17);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',{maxZoom:20,attribution:'Imagery © Esri'}).addTo(map);
const STY = {lane:{color:'#9fc3ff',weight:3},road:{color:'#f2f2f2',weight:5},connector:{color:'#cfe0ff',weight:1.5},entrance:{color:'#ffd38a',weight:1.5,dashArray:'3'},parking:{color:'#c6a9ff',weight:1.5,dashArray:'3'},gate:{color:'#9be29f',weight:2}};
const edgeLayer = {};
Object.values(E).forEach(e => {
  const s = Object.assign({opacity:0.85}, STY[e.layer]);
  const pl = L.polyline([[N[e.u].lat,N[e.u].lon],[N[e.v].lat,N[e.v].lon]], s).addTo(map);
  pl.on('click', () => { if (closed.has(e.id)) { closed.delete(e.id); pl.setStyle(s); } else { closed.add(e.id); pl.setStyle({color:'#d32f2f',weight:7,opacity:1}); } });
  pl.bindTooltip(`${e.layer} · ${e.len.toFixed(0)} m${e.grade>0.0833?' · steep':''}`, {sticky:true});
  edgeLayer[e.id] = pl;
});
Object.values(N).forEach(n => {
  if (n.kind==='parking') L.circleMarker([n.lat,n.lon],{radius:6,color:'#fff',weight:1.5,fillColor:'#7a5fb0',fillOpacity:1}).bindTooltip('P · '+n.name).addTo(map);
  if (n.kind==='gate') L.circleMarker([n.lat,n.lon],{radius:6,color:'#fff',weight:1.5,fillColor:'#2e7d32',fillOpacity:1}).bindTooltip(n.name).addTo(map);
  if (n.kind==='entrance') L.circleMarker([n.lat,n.lon],{radius:3.5,color:'#ffd38a',weight:1,fillOpacity:1}).bindTooltip(n.name).addTo(map);
});

const opts = Object.entries(D.places).map(([k,p]) => `<option value="${k}">${p.label}</option>`).join('');
document.getElementById('from').innerHTML = opts; document.getElementById('to').innerHTML = opts;
document.getElementById('from').value = 'GATE1'; document.getElementById('to').value = 'LIBRARY';

// ---------------- state-space search: state = "node|mode"
function makeProblem(cfg){
  const starts = D.places[cfg.from].nodes, goals = new Set(D.places[cfg.to].nodes);
  const goalNodes = [...goals].map(i=>N[i]);
  const speed = m => (m==='walk'||m==='fetch') ? (cfg.wc ? D.speeds.wheelchair : D.speeds.walk) : D.speeds[m];
  const canPark = name => { const r = D.parking_rules[name]; return !r || r.includes(cfg.role); };
  let vehicleNode = null, init;
  if (cfg.veh==='walk' || starts.every(s => ['gate','parking'].includes(N[s].kind))) init = starts.map(s => s+'|'+cfg.veh);
  else { // nearest parking on foot holds the vehicle
    const dist = {}; let q = starts.map(s => [0,s]); starts.forEach(s=>dist[s]=0);
    while (q.length){ q.sort((a,b)=>a[0]-b[0]); const [d,n]=q.shift(); if(d>dist[n])continue;
      if (N[n].kind==='parking' && ADJ[n].some(eid=>E[eid].modes.has(cfg.veh))) { vehicleNode=n; break; }
      for (const eid of ADJ[n]) { const e=E[eid], m=e.u==n?e.v:e.u; if(e.modes.has('walk') && d+e.len<(dist[m]??1e18)){dist[m]=d+e.len;q.push([dist[m],m]);} } }
    init = starts.map(s => s+'|fetch');
  }
  return {
    init,
    goal: s => { const [n,m]=s.split('|'); return (m==='walk'||m==='fetch') && goals.has(+n); },
    succ: s => { const [ns,m]=s.split('|'), n=+ns, out=[], em = m==='fetch'?'walk':m;
      for (const eid of ADJ[n]) { const e=E[eid]; if(!e.modes.has(em)||closed.has(eid))continue;
        if (em==='walk' && cfg.wc && (e.stairs||e.grade>0.0833)) continue;
        const other = e.u==n?e.v:e.u; const c = cfg.metric==='distance'? e.len : e.len/speed(m);
        out.push([other+'|'+m, c, ['move',eid]]); }
      if (m==='fetch' && n===vehicleNode) out.push([n+'|'+cfg.veh, cfg.metric==='distance'?0:D.park_time[cfg.veh], ['unpark',N[n].name]]);
      if ((m==='car'||m==='bike') && N[n].kind==='parking' && canPark(N[n].name)) out.push([n+'|walk', cfg.metric==='distance'?0:D.park_time[m], ['park',N[n].name]]);
      return out; },
    h: s => { const [ns,m]=s.split('|'); const d=Math.min(...goalNodes.map(g=>hav(N[+ns],g)));
      if (cfg.metric==='distance') return d; const v = m==='fetch'? Math.max(speed('walk'),D.speeds[cfg.veh]) : speed(m); return d/v; }
  };
}
class PQ{constructor(){this.a=[]}push(k,v){const a=this.a;a.push([k,v]);let i=a.length-1;while(i>0){const p=(i-1)>>1;if(a[p][0]<=a[i][0])break;[a[p],a[i]]=[a[i],a[p]];i=p;}}
 pop(){const a=this.a,top=a[0],last=a.pop();if(a.length){a[0]=last;let i=0;for(;;){let l=2*i+1,r=l+1,m=i;if(l<a.length&&a[l][0]<a[m][0])m=l;if(r<a.length&&a[r][0]<a[m][0])m=r;if(m===i)break;[a[m],a[i]]=[a[i],a[m]];i=m;}}return top;}get size(){return this.a.length}}
function search(P, alg){
  const parent = {}, g = {}, explored = []; let expanded = 0;
  P.init.forEach(s => { parent[s]=null; g[s]=0; });
  const done = s => { const st=[s],ac=[]; while(parent[st[st.length-1]]){const [p,a]=parent[st[st.length-1]];ac.push(a);st.push(p);} return {found:true,states:st.reverse(),actions:ac.reverse(),cost:g[s],expanded,explored}; };
  if (alg==='bfs'){ const q=[...P.init], seen=new Set(P.init); let qi=0;
    while(qi<q.length){ const s=q[qi++]; expanded++; explored.push(+s.split('|')[0]); if(P.goal(s)) return done(s);
      for(const [t,c,a] of P.succ(s)) if(!seen.has(t)){seen.add(t);parent[t]=[s,a];g[t]=g[s]+c;q.push(t);} }
    return {found:false,expanded,explored}; }
  const pr = (gv,hv) => alg==='ucs'?gv : alg==='greedy'?hv : gv+hv;
  const pq = new PQ(); P.init.forEach(s=>pq.push(pr(0,P.h(s)),s)); const closedS = new Set();
  while(pq.size){ const [,s]=pq.pop(); if(closedS.has(s))continue; if(P.goal(s)) return done(s);
    closedS.add(s); expanded++; explored.push(+s.split('|')[0]);
    for(const [t,c,a] of P.succ(s)){ const g2=g[s]+c; if(g2 < (g[t]??Infinity) && !(alg==='greedy'&&closedS.has(t))){ g[t]=g2; parent[t]=[s,a]; closedS.delete(t); pq.push(pr(g2,P.h(t)),t);} } }
  return {found:false,expanded,explored};
}
const MC = {walk:'#1b9e77',fetch:'#66c2a5',bike:'#d95f02',car:'#c2185b'};
let drawn = L.layerGroup().addTo(map);
function run(){
  drawn.clearLayers();
  const cfg = {from:from.value,to:to.value,veh:veh.value,role:role.value,metric:metric.value,wc:wc.checked};
  const P = makeProblem(cfg); const t0 = performance.now(); const r = search(P, alg.value); const ms = performance.now()-t0;
  const out = document.getElementById('out');
  if (expl.checked) r.explored.forEach(n => L.circleMarker([N[n].lat,N[n].lon],{radius:3,stroke:false,fillColor:'#ff4d4d',fillOpacity:0.55}).addTo(drawn));
  if (!r.found){ out.innerHTML = `<b>No route.</b> Every path is blocked for this traveller. Expanded ${r.expanded} states.`; return; }
  let dist=0, steps=[], cur=null;
  r.actions.forEach((a,i) => {
    if (a[0]!=='move'){ steps.push(a[0]==='park'?`Park at <b>${a[1]}</b>`:`Collect your vehicle at <b>${a[1]}</b>`); cur=null;
      const n=N[+r.states[i].split('|')[0]]; L.marker([n.lat,n.lon]).bindTooltip(a[1]).addTo(drawn); return; }
    const e=E[a[1]], m=r.states[i+1].split('|')[1]; dist+=e.len;
    L.polyline([[N[e.u].lat,N[e.u].lon],[N[e.v].lat,N[e.v].lon]],{color:MC[m],weight:7,opacity:0.95}).addTo(drawn);
    if(!cur||cur.m!==m){cur={m,len:0};steps.push(cur);} cur.len+=e.len; });
  const verb={walk:'Walk',fetch:'Walk',bike:'Ride',car:'Drive'};
  const tstr = cfg.metric==='time' ? (r.cost/60).toFixed(1)+' min' : (r.cost).toFixed(0)+' m';
  out.innerHTML = `<div class="stat"><div>Cost<b>${tstr}</b></div><div>Distance<b>${dist.toFixed(0)} m</b></div><div>Expanded<b>${r.expanded}</b></div><div>Runtime<b>${ms.toFixed(1)} ms</b></div></div>
   <ol>${steps.map(s=>typeof s==='string'?`<li>${s}</li>`:`<li>${verb[s.m]} ${s.len.toFixed(0)} m</li>`).join('')}</ol>`;
}
go.onclick = run; clear.onclick = () => { closed.forEach(id => edgeLayer[id].setStyle(Object.assign({opacity:0.85},STY[E[id].layer]))); closed.clear(); run(); };
run();
</script></body></html>
"""


def write_map(g, path):
    Path(path).write_text(HTML.replace("__DATA__", json.dumps(export_graph(g), separators=(",", ":"))),
                          encoding="utf-8")
    return path
