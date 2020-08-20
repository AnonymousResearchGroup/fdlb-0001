function output = Hw12P2A(Elem,Hkr)
    % Elem = 'LHNMYS';
    % Elem = 'OneComp';
    % Hr = 1e-9;
    % Hkr = Hr;
    %% CE 221 Nonlinear Structural Analysis, Homework Set 12, Problem 1
    
    %% Cyclic Load Analysis of Cantilever Column with Concentrated Plasticity Element
    
    %% Clear memory, close any open windows and insert units
    if exist ('XYZ','var');  clear XYZ; end
    if exist ('BOUN','var'); clear BOUN; end
    if exist ('CON','var');  clear CON; end
    if exist ('ElemData','var'); clear ElemData; end
    if exist ('Model','var'); clear Model; end
    if exist ('State','var'); clear State; end
    if exist ('Post','var');  clear Post; end
    if exist ('Loading','var'); clear Loading; end
    
    if exist ('SecData','var'); clear SecData; end
    if exist ('MatData','var'); clear MatData; end
    if exist ('Shape','var');   clear Shape; end
    
    Units
    E  = 29000*ksi;
    fy = 60*ksi;
    
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
    ElemName{1} = ['Inel2dFrm_w', Elem];
    % ElemName{1} = 'Inel2dFrm_wLHNMYS';
    
    % generate Model data structure
    Model = Create_Model (XYZ,CON,BOUN,ElemName);
    
    %% Element properties
    SecGeom = 'W14x426';
    SecProp = AISC_Section(SecGeom);
    SecProp.fy = fy;
    SecProp.E  = E ;
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
    % Hr = 1e-9;       % hardening ratio for multi-component models
    ElemData{1}.E   = E;
    ElemData{1}.A   = A;
    ElemData{1}.I   = I;
    ElemData{1}.Np  = Np;
    ElemData{1}.Mp  = Mp;
    ElemData{1}.Hkr = Hkr;
    
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
    % Tmax = no_Rev*T_Rev;
    SolStrat.Tmax = no_Rev*T_Rev;
    State = Initialize_State(Model,ElemData);
    %% Cyclic analysis for imposed force/displacement with time history
    time_start = tic;
    SolStrat.IncrStrat.Deltat = Deltat;
    % S_MultiStep_wLoadHist
    [State,Post] = MultiStep_wLoadHist(Model,ElemData,Loading,SolStrat,State,{},{'Kf', 'KL'});
    time_elapsed = toc(time_start);
    
    %% post-processing
    output = Post_Hw12P2(Model,SecProp,Post);
    output('time') = time_elapsed;
    output('Hkr') = Hkr;
    output('Elem') = Elem;
    for i = 1:length(Post), Kf{i} = full(Post(i).Kf); end
    % for i = 1:length(Post), Kf22(i) = Post(i).Kf(2); end
    % 
    output('Kf') = Kf
    % output('Kf22') = Kf22
    
    output = jsonencode(output);
end    
    