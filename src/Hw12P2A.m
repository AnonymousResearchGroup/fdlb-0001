% ===========================================================================================
%  Copyright(c) 2020 Professor Filip C. Filippou (filippou@berkeley.edu)
%  Department of Civil and Environmental Engineering, UC Berkeley
% ===========================================================================================

%% CE 221 Nonlinear Structural Analysis, Homework Set 12, Problem 1

%% Cyclic Load Analysis of Cantilever Column with Concentrated Plasticity Element

%% Clear memory, close any open windows and insert units
CleanStart

Units

%% Specify length of cantilever column
L = 6*ft;
% specify node coordinates
XYZ(1,:) = [ 0   0];  % first node
XYZ(2,:) = [ 0   L];  % second node

% connectivity array
CON(1,:) = [ 1  2];

% boundary conditions
BOUN(1,:) = [1 1 1];
BOUN(2,:) = [1 0 0];

%% Element name: 2d nonlinear frame element with concentrated inelasticity
ElemName{1} = 'Inel2dFrm_wOneComp';
% ElemName{1} = 'Inel2dFrm_wLHNMYS';

% generate Model data structure
Model = Create_Model (XYZ,CON,BOUN,ElemName);

%% Element properties
E  = 29000*ksi;
fy = 60*ksi;
SecProp = AISC_Section('W14x426');
A  = SecProp.A *in^2;
I  = SecProp.Ix*in^4;
Z  = SecProp.Zx*in^3;

d  = SecProp.d *in;
bf = SecProp.bf*in;
tf = SecProp.tf*in;
tw = SecProp.tw*in;

Np = A*fy;
Mp = Z*fy;

% Hr = 0.05;     % hardening ratio for multi-component models
Hr = 1e-9;       % hardening ratio for multi-component models
ElemData{1}.E   = E;
ElemData{1}.A   = A;
ElemData{1}.I   = I;
ElemData{1}.Np  = Np;
ElemData{1}.Mp  = Mp;
ElemData{1}.Hkr = Hr;

%% Default values for missing element properties
ElemData = Structure ('chec',Model,ElemData);

%% 1. vertical force (constant)
% specify nodal forces
Pe(2,2) = -0.20*Np;   
Loading = Create_Loading (Model,Pe);

% initial solution strategy parameters
SolStrat = Initialize_SolStrat;
% single load step for the application of gravity load with Dlam0 = 1
SolStrat.IncrStrat.Dlam0  = 1;
SolStrat.IncrStrat.LFCtrl = 'no';
S_InitialStep

clear Pe

%% 2. Cyclic axial force and horizontal displacement
% specify nodal forces and displacements
Pe(2,2) = -0.30*Np;
Ue(2,1) = 0.005*L;   
Loading = Create_Loading (Model,Pe,Ue);

% specify time step and time at each reversal (no_step per reversal = T_Rev/Dt)
Deltat = 0.01;
T_Rev  = 1; 
% specify force/displacement reversal values 
RevVal = [ 1 -1 1 -1 ];
no_Rev = length(RevVal);          % no of load reversals
Loading.FrcHst.Value = [0 RevVal];
Loading.FrcHst.Time  = [0 (1:no_Rev)*T_Rev];
RevVal = [ 1 -2 3 -4 ];
Loading.DspHst.Value = [0 RevVal];
Loading.DspHst.Time = [0 (1:no_Rev)*T_Rev];
% time at end of analysis
Tmax = no_Rev*T_Rev;

%% Cyclic analysis for imposed force/displacement with time history
tic
SolStrat.IncrStrat.Deltat = Deltat;
S_MultiStep_wLoadHist
toc

%% post-processing
Post_Hw12P2