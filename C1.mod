param NLPinfo{ii in {1..12}};
param Nfe == NLPinfo[1];
param Nobs == NLPinfo[2];
param x0 == NLPinfo[3];
param y0 == NLPinfo[4];
param xf == NLPinfo[5];
param yf == NLPinfo[6];
param radius == NLPinfo[7];
param tf == NLPinfo[8];
param x_min == NLPinfo[9];
param x_max == NLPinfo[10];
param y_min == NLPinfo[11];
param y_max == NLPinfo[12];

param path{ii in {1..(Nfe/3)}, jj in {1..2}};
param obs{ii in {1..Nobs}, jj in {1..Nfe}, kk in {1..3}};

param hi = tf / (Nfe - 1);
set I = {1..Nfe};
set SI = {1..(Nfe/3)};

var x{i in I};
var y{i in I};
var dx{i in I};
var dy{i in I};
var ddx{i in I};
var ddy{i in I};

minimize cost_function:
sum{i in I}((dx[i]^2 + dy[i]^2) + 100 * (ddx[i]^2 + ddy[i]^2)) + 0.0001 * sum{i in SI}((x[(i-1)*3+1] - path[i,1])^2 + (y[(i-1)*3+1] - path[i,2])^2);

################## ODEs
s.t. DIFF_1 {i in {2..Nfe}}:
x[i] = x[i-1] + dx[i-1] * hi;
s.t. DIFF_2 {i in {2..Nfe}}:
y[i] = y[i-1] + dy[i-1] * hi;
s.t. DIFF_3 {i in {2..Nfe}}:
dx[i] = dx[i-1] + ddx[i-1] * hi;
s.t. DIFF_4 {i in {2..Nfe}}:
dy[i] = dy[i-1] + ddy[i-1] * hi;

s.t. EQ_1:
x[1] = x0;
s.t. EQ_2:
y[1] = y0;
s.t. EQ_3:
x[Nfe] = xf;
s.t. EQ_4:
y[Nfe] = yf;

s.t. Bonds_x {i in I}:
x_min + radius <= x[i] <= x_max - radius;
s.t. Bonds_y {i in I}:
y_min + radius <= y[i] <= y_max - radius;

s.t. Collision_avoidance {i in I, j in {1..Nobs}}:
(x[i] - obs[j,i,1])^2 + (y[i] - obs[j,i,2])^2 >= 1.21 * (obs[j,i,3] + radius)^2;

data;
param NLPinfo:= include NLPinfo;
param path:= include path;
param obs:= include obs;