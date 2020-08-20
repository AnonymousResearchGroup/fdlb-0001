function output = Post_Hw12P2(Model, SecProp, Post)
output = containers.Map;
%% post-processing of results for Problem 2 of Homework 12
Units
L = 6*ft;
A  = SecProp.A *in^2;
I  = SecProp.Ix*in^4;
Z  = SecProp.Zx*in^3;

d  = SecProp.d *in;
bf = SecProp.bf*in;
tf = SecProp.tf*in;
tw = SecProp.tw*in;

fy = SecProp.fy; 
E  = SecProp.E;

Np = A*fy;
Mp = Z*fy;
% plot parameters
% -------------------------------------------------------------------------
WinXr = 0.40;  % X-ratio of plot window to screen size 
WinYr = 0.80;  % Y-ratio of plot window to screen size
% -------------------------------------------------------------------------

%% Post-processing
% extract displacements from Post
np = length(Post);
Uh = zeros(np,1);
LatF = zeros(np,1);
Momt = zeros(np,1);
Naxl = zeros(np,1);
pltDOF = Model.DOF(2,1);
supDOF = Model.DOF(1,1);
for k=1:np
   Uh(k) = Post(k).U(pltDOF);
   LatF(k) = -sum(Post(k).Pr(supDOF));
   Naxl(k) = Post(k).Elem{1}.q(1);
   Momt(k) = Post(k).Elem{1}.q(2);
end

%% plot force displacement relation
% Create_Window (WinXr,WinYr);

Xp = Uh./L;
Yp = LatF*L/Mp;

output('force-displ') = containers.Map({'x','y'},{Xp,Yp});

% PlotOpt.XLbl = 'Horizontal drift ratio';
% PlotOpt.YLbl = '$$(P_h \, L)/M_p$$';

% AxHndlA = Plot_XYData (Xp,Yp,PlotOpt);

% AxHndlA.XLim  = [-0.02 0.02 ];
% AxHndlA.XTick = -0.02:0.01:0.02;
% AxHndlA.YLim  = [ -1.25 1.25 ];
% AxHndlA.YTick = -1.25:0.5:1.25;

%% plot axial force N-bending moment M history
% Create_Window (WinXr,WinYr);

Xp = Momt./Mp;
Yp = Naxl./Np;

output('axial-moment') = containers.Map({'x','y'},{Xp,Yp});

% PlotOpt.XLbl = '$$ M / M_p$$';
% PlotOpt.YLbl = '$$ N / N_p$$';

% AxHndlB = Plot_XYData (Xp,Yp,PlotOpt);

Df = Uh./L;
%-------------------------------------------------------------
%% superimpose analytical solution for N-M interaction diagram of wide-flange section
n   = -1:0.002:1;

eta = (abs(n).*(1-(1-tw/bf)*(1-2*tf/d)))./(tw/bf);

fl = find(eta>1-2*tf/d);
eta(fl) = (abs(n(fl)).*(1-(1-tw/bf)*(1-2*tf/d)))+(1-tw/bf)*(1-2*tf/d);

m = (4*tf/d*(1-tf/d) + tw/bf.*((1-2*tf/d)^2-eta.^2))./(1-(1-tw/bf)*(1-2*tf/d)^2);
m(fl) = (1-eta(fl).^2)./(1-(1-tw/bf)*(1-2*tf/d)^2);
% plot( m,n,'-','LineWidth',3,'Color','r');
% plot(-m,n,'-','LineWidth',3,'Color','r');

% AxHndlB.XLim  = [ -1.25 1.25 ];
% AxHndlB.XTick = -1.25:0.5:1.25;
% AxHndlB.YLim  = [-0.6 0.2 ];
% AxHndlB.YTick = -0.6:0.2:0.2;
output('analytic-NM') = containers.Map({'x','y'},{m,n});
