param NE;
param NumObs;
param libai_x{ii in {1..NumObs}, jj in {1..NE}};
param libai_y{ii in {1..NumObs}, jj in {1..NE}};
param Radius_obs{ii in {1..(NumObs+1)}};

param tf = 40;
param hi = tf / NE;
set I0 = {0..NE};
set I = {1..NE};
set I1 = {2..NE};
param Rego = Radius_obs[NumObs+1];

param amax = 1.0;
param wmax = 0.3;
param vmax = 1;
param vmin = 0;
param phymax = 0.35;

var x{i in I0};
var y{i in I0};
var theta{i in I0};
var v{i in I0};
var phy{i in I0};

################## Optimization Objective
minimize cost_function:
(x[NE] - 40)^2 + (y[NE] - 5)^2;

################## ODEs
s.t. DIFF_dxdt {i in I}:
x[i] = v[i] * cos(theta[i]) * hi + x[i-1];

s.t. DIFF_dydt {i in I}:
y[i] = v[i] * sin(theta[i]) * hi + y[i-1];

s.t. DIFF_dthetadt {i in I}:
theta[i] = v[i] * tan(phy[i]) / 1.414 * hi + theta[i-1];

################### Two-point bounds
s.t. EQ_ending_theta :
theta[NE] = 0;
s.t. EQ_ending_phy :
phy[NE] = 0;
s.t. EQ_ending_v :
v[NE] = 0;

################
s.t. Bonds_v_ub {i in I}:
v[i] <= vmax;
s.t. Bonds_v_lb {i in I}:
v[i] >= vmin;

s.t. Bonds_phy_ub {i in I}:
phy[i] <= phymax;
s.t. Bonds_phy_lb {i in I}:
phy[i] >= -phymax;

s.t. Bonds_w_ub {i in I1}:
(phy[i] - phy[i-1]) / hi <= wmax;
s.t. Bonds_w_lb {i in I1}:
(phy[i] - phy[i-1]) / hi >= -wmax;

s.t. Bonds_a_ub {i in I1}:
(v[i] - v[i-1]) / hi <= amax;
s.t. Bonds_a_lb {i in I1}:
(v[i] - v[i-1]) / hi >= -amax;

s.t. Collision_Avoidance {i in {1..NE}, kk in {1..NumObs}}:
(libai_x[kk,i] - (x[i] + 0.707 * cos(theta[i])))^2 + (libai_y[kk,i] - (y[i] + 0.707 * sin(theta[i])))^2 >= (Rego + Radius_obs[kk])^2;

s.t. Bonds_y_ub {i in I}:
y[i] <= 10;
s.t. Bonds_y_lb {i in I}:
y[i] >= 0;
s.t. Bonds_x_ub {i in I}:
x[i] <= 40;
s.t. Bonds_x_lb {i in I}:
x[i] >= -5;


data;
param libai_x:= include libai_x;
param libai_y:= include libai_y;
param Radius_obs:= include Radius_obs;
param NumObs:= include NumObs;
param NE:= include NE;