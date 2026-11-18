#include <airseafly_real400/core.hpp>
#include <iostream>
#include <functional>
using namespace airseafly_real400;
int checks=0;
void check(bool ok,const char* message) { ++checks; if(!ok) throw std::runtime_error(message); }
void near(const V3& a,const V3& b,double e=1e-8) { check((a-b).norm()<e,"vector mismatch"); }
void rejects(std::function<void()> f) { bool caught=false; try { f(); } catch(const std::exception&) { caught=true; } check(caught,"expected rejection"); }
State anchor(double t=1.) { State s; s.t=t; return s; }
int main() {
 try {
  Frames f; State s=anchor(); s.p=V3(2,-3,4); f.initialize(s);
  near(f.transform(s,V3::Zero()).p,V3::Zero());
  for(int i=0;i<3;i++) for(int sign:{-1,1}) { State moved=s; moved.p[i]+=sign*.3; V3 expected=V3::Zero(); expected[i]=sign*.3; near(f.transform(moved,V3::Zero()).p,expected); }
  rejects([&]{f.initialize(s);});
  Frames tilted; State tilt=anchor(); tilt.q=Q(Eigen::AngleAxisd(.25,V3::UnitX()));
  tilt.gravity=tilt.q*V3(0,0,-9.81); tilted.initialize(tilt);
  near(tilted.q_ow*tilt.gravity,V3(0,0,-9.81));
  Frames tilted_start;State physical_tilt=anchor();physical_tilt.gravity=expQ(V3(0,.2,0))*V3(0,0,-9.81);
  tilted_start.initialize(physical_tilt);
  near(tilted_start.q_ow*physical_tilt.gravity,V3(0,0,-9.81));
  check(angle(tilted_start.transform(physical_tilt,V3::Zero()).q,Q::Identity())>.1,"tilted startup erased roll/pitch");
  Frames lever; lever.t_bi=V3(.1,.02,.05); State initial=anchor(); initial.p=lever.t_bi; lever.initialize(initial);
  State turn=initial; turn.q=Q(Eigen::AngleAxisd(1.5707963267948966,V3::UnitZ()));
  const V3 omega(0,0,1.); turn.p=turn.q*lever.t_bi; turn.v=omega.cross(turn.p);
  auto b=lever.transform(turn,omega); near(b.p,V3::Zero()); near(b.v_world,V3::Zero());
  turn.v+=V3(0,1,0); b=lever.transform(turn,omega); near(b.v_body,V3(1,0,0)); near(b.q*b.v_body,b.v_world);
  near(lever.scanPoint(turn,V3(.3,0,0)),lever.q_ow*(turn.q*V3(.3,0,0)+turn.p)+lever.t_ow);
  Frames mount; mount.q_bi=Q(Eigen::AngleAxisd(.2,V3::UnitZ())); mount.t_bi=V3(.05,.03,.08);
  State mounted=anchor(); mounted.q=mount.q_bi; mounted.p=mount.t_bi; mount.initialize(mounted);
  check(angle(mount.transform(mounted,V3::Zero()).q,Q::Identity())<1e-7,"mount rotation");
  Predictor predictor; for(int i=0;i<=20;i++) predictor.add({1.+i*.005,V3(0,0,1),V3::Zero()});
  State posterior=anchor(); posterior.accel_scale=9.81; predictor.correct(posterior);
  near(predictor.predict(1.1).state.p,V3::Zero());
  posterior.t=1.05; predictor.correct(posterior); near(predictor.predict(1.1).state.p,V3::Zero());
  rejects([&]{predictor.predict(1.4);});
  rejects([&]{predictor.add({1.1,V3::Zero(),V3::Zero()});});
  check(!predictor.fault.empty(),"duplicate IMU not latched");
  Predictor acceleration; for(int i=0;i<=40;i++) acceleration.add({1.+i*.005,V3(1,0,9.81),V3::Zero()});
  acceleration.correct(anchor()); auto predicted=acceleration.predict(1.2);
  near(predicted.state.p,V3(.02,0,0)); near(predicted.state.v,V3(.2,0,0));
  State delayed=anchor(1.1); delayed.p=V3(.005,0,0); delayed.v=V3(.1,0,0);
  acceleration.correct(delayed); near(acceleration.predict(1.2).state.p,V3(.02,0,0));
  rejects([&]{acceleration.correct(anchor(.9));});
  Predictor gap; gap.add({1.,V3(0,0,9.81),V3::Zero()}); gap.add({1.1,V3(0,0,9.81),V3::Zero()}); gap.correct(anchor());
  rejects([&]{gap.predict(1.1);});
  Predictor jump;jump.add({1.,V3(0,0,9.81),V3::Zero()});jump.add({1.01,V3(0,0,9.81),V3::Zero()});jump.correct(anchor());
  State discontinuous=anchor(1.01);discontinuous.p=V3(2,0,0);rejects([&]{jump.correct(discontinuous);});
  check(!jump.fault.empty(),"posterior discontinuity not latched");
  Predictor units;State scaled=anchor();scaled.accel_scale=9.81;scaled.bg=V3(.01,0,0);scaled.ba=V3(.02,0,0);
  for(int i=0;i<=20;++i) units.add({1.+i*.005,V3(.02/9.81,0,1),V3(.01,0,0)});
  units.correct(scaled);near(units.predict(1.1).state.p,V3::Zero());
  check(angle(units.predict(1.1).state.q,Q::Identity())<1e-7,"bias/acceleration unit mismatch");
  Predictor rate; rate.add({1.,V3(0,0,9.81),V3::Zero()}); rate.add({1.005,V3(0,0,9.81),V3::Zero()}); rate.correct(anchor());
  int unique=0; double previous=1.;
  for(int i=2;i<=12000;i++) { double t=1.+i*.005; rate.add({t,V3(0,0,9.81),V3::Zero()}); if(i%20==0) rate.correct(anchor(t-.005)); auto p=rate.predict(t); check(p.state.t>previous,"duplicate predicted stamp"); previous=p.state.t; unique++; }
  check(unique>=11990,"200Hz unique samples");
  AttitudeHistory hist; hist.add({1,Q::Identity(),V3(0,0,9.81)}); hist.add({1.01,expQ(V3(0,0,.02)),V3(0,0,9.81)});
  check(angle(hist.sample(1.005).q,expQ(V3(0,0,.01)))<1e-7,"attitude interpolation");
  rejects([&]{hist.sample(1.02);}); hist.add({1.02,expQ(V3(0,0,1)),V3(0,0,9.81)});
  rejects([&]{hist.sample(1.015);});
  AttitudeHistory sparse;sparse.add({1,Q::Identity(),V3::Zero()});sparse.add({1.1,Q::Identity(),V3::Zero()});
  rejects([&]{sparse.sample(1.05);});rejects([&]{sparse.add({.5,Q::Identity(),V3::Zero()});});
  BodyHistory bh; BodyState bs; bs.t=1.01; bh.add(bs);
  check(std::abs(pairStates(bh,hist,1.03).body.t-1.01)<1e-8,"common time pairing");
  rejects([&]{pairStates(bh,hist,2.);});
  // Pure reference-yaw reset must preserve the requested body attitude error.
  Q lio=expQ(V3(.1,-.2,.4)), desired=expQ(V3(-.05,.15,.6));
  for(double reset:{0.,.5,-1.}) { Q px4=expQ(V3(0,0,reset))*lio; Q output=px4*lio.conjugate()*desired; check(angle(px4.conjugate()*output,lio.conjugate()*desired)<1e-7,"reference-yaw invariant"); }
  RcGate rc; std::vector<uint16_t> channels(8,1500); channels[4]=channels[5]=2000;
  check(!rc.update(channels).hover,"startup high authorized"); channels[4]=channels[5]=1000; rc.update(channels);
  channels[4]=2000; auto d=rc.update(channels); check(d.hover && d.hover_rising && !d.command,"RC5 hover edge");
  channels[5]=2000; d=rc.update(channels); check(d.command && d.command_rising,"RC6 command edge");
  channels[7]=2000; d=rc.update(channels); check(d.command && !d.command_rising,"RC8 command side effect");
  channels[2]=1700; check(!rc.update(channels).centered,"third stick center check");
  rejects([&]{rc.update(std::vector<uint16_t>(4,1500));});
  check(!fresh(2.,1.) && !fresh(1.,1.1) && fresh(1.01,1.),"freshness bounds");
  std::cout << "PASS " << checks << " assertions (includes 60s / 200Hz deterministic replay)\n";
  return 0;
 } catch(const std::exception& e) { std::cerr << "FAIL after " << checks << ": " << e.what() << '\n'; return 1; }
}
