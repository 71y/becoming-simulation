/* Browser translation of Maya's interaction_points_gray_consumed simulation.
   Same force law, proximity removal, semi-implicit Euler integration and
   per-step damping. DT=.1, K=0. Weight also defines encounter radius. */
(function(root){
'use strict';
const DT=.1;
function initial(points){return {x:50,y:50,vx:0,vy:0,t:0,step:0,points:points.map(p=>({...p,consumed:false})),trail:[[50,50]],order:[]};}
function advance(s,{c=10,p=.3,damping=.9,consume=true}={}){
 for(const site of s.points) if(consume&&!site.consumed&&Math.hypot(site.x-s.x,site.y-s.y)<site.weight){site.consumed=true;s.order.push(site.id);}
 let ax=0,ay=0;
 for(const site of s.points){if(site.consumed)continue;const dx=site.x-s.x,dy=site.y-s.y,d2=Math.max(.01,dx*dx+dy*dy);const pull=c*site.weight/Math.pow(d2,p),r=Math.sqrt(d2);ax+=pull*dx/r;ay+=pull*dy/r;}
 s.vx=(s.vx+ax*DT)*damping;s.vy=(s.vy+ay*DT)*damping;s.x+=s.vx*DT;s.y+=s.vy*DT;s.step++;s.t=s.step*DT;s.trail.push([s.x,s.y]);return s;
}
const api={DT,initial,advance};if(typeof module!=='undefined')module.exports=api;else root.Ecology=api;
})(typeof globalThis!=='undefined'?globalThis:this);
