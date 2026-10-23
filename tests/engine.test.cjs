const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
let L; try {L=require('../dist/engine.js')}catch{}
const modes=['nav-dijkstra','nav-astar','theta','smac','hybrid','lattice'];
function map(){return {w:20,h:15,walls:Array(300).fill(false)}}
test('引擎提供可运行的规划接口',()=>assert.equal(typeof L?.plan,'function'));
for(const mode of modes){
 test(mode+'：空地图路径包含起终点并输出搜索事件',()=>{
  assert.ok(L,'引擎尚未实现');const m=map(); const r=L.plan(m,{x:3,y:7,theta:0},{x:15,y:7,theta:0},mode);
  assert.equal(r.success,true,r.message);assert.ok(r.events.length>2);assert.ok(Math.hypot(r.path[0].x-3,r.path[0].y-7)<.1);
  assert.ok(Math.hypot(r.path.at(-1).x-15,r.path.at(-1).y-7)<.8);
  for(const p of r.path)assert.ok(L.free(r.map,p.x,p.y),'路径不可碰撞');
 });
 test(mode+'：完全隔离时返回不可达',()=>{
  assert.ok(L);const m=map();for(let y=0;y<m.h;y++)m.walls[y*m.w+9]=true;
  const r=L.plan(m,{x:3,y:7,theta:0},{x:15,y:7,theta:0},mode);assert.equal(r.success,false);assert.equal(r.events.at(-1).phase,'failed');
 });
 test(mode+'：同位姿返回单点路径',()=>{
  assert.ok(L);const p={x:3,y:7,theta:0};const r=L.plan(map(),p,p,mode);assert.equal(r.success,true);assert.equal(r.path.length,1);
 });
}
test('NavFn 保存零源点势值，末点势值为有限正数',()=>{
 assert.ok(L);const r=L.plan(map(),{x:2,y:3},{x:16,y:11},'nav-astar');assert.equal(r.potential[3*20+2],0);assert.ok(Number.isFinite(r.potential[11*20+16]));assert.ok(r.events.some(e=>e.phase==='gradient'));
});
test('LOS 不允许穿障碍，也不允许斜穿两个障碍的夹角',()=>{
 assert.ok(L);const m=map();m.walls[5*20+5]=true;let a=L.prepare(m);assert.equal(L.line(a,{x:3,y:5},{x:7,y:5}).ok,false);
 const n=map();n.walls[3*20+4]=true;n.walls[4*20+3]=true;assert.equal(L.line(L.prepare(n),{x:3,y:3},{x:4,y:4}).ok,false);
});
test('非法端点被明确拒绝',()=>{
 assert.ok(L);const m=map();m.walls[3*20+3]=true;assert.equal(L.plan(m,{x:3,y:3},{x:10,y:3},'smac').success,false);
});
test('Smac 短路径保留原始几何，单独报告朝向计算',()=>{
 assert.ok(L);const r=L.plan(map(),{x:3,y:7},{x:7,y:7},'smac');assert.equal(r.success,true);assert.ok(r.events.some(e=>e.phase==='orientation'));assert.ok(!r.events.some(e=>e.phase==='smooth'));
});
test('Theta 空地可以祖先重连，原始路径端点完整',()=>{
 assert.ok(L);const r=L.plan(map(),{x:2,y:3},{x:16,y:11},'theta');assert.ok(r.events.some(e=>e.phase==='los'&&e.accepted));assert.deepEqual(r.raw.at(-1),{x:16,y:11});
});
test('NavFn 回放事件包含零势值源',()=>{
 const r=L.plan(map(),{x:2,y:3},{x:16,y:11},'nav-astar');const values=Array(300).fill(Infinity);for(const e of r.events)for(const u of e.updates||[])values[u.id]=u.v;assert.equal(values[3*20+2],0);
});
test('Lattice 明确拒绝非基元档位朝向，不静默更改车头',()=>{
 const r=L.plan(map(),{x:3,y:7,theta:Math.PI/12},{x:15,y:7,theta:0},'lattice');assert.equal(r.success,false);assert.match(r.message,/45/);
});
