function [resu,sysP,sysC,sysD] = prsatag(sysP,sysACT,sysC,sysD,optsat)
%PRSATAG  Stability analysis 
%         of a position and rate saturated closed-loop system - GLOBAL case
%         Strategy through Sector nonlinearities
%
%[resu,sysP,sysC,sysD] = prsatag(sysP,sysACT,sysC,sysD,optsat) 
%
%   The structure sysACT is actually not used in the function but let in
%   the input arguments to keep the same input and output for all the 
%   function, as much as possible
%
%   returns 
%   - just the fact that the system is globally stable or not (feasible
%     or unfeasible problem) in the case without disturbance or energy
%     bounded disturbance (the L2-gain gamma from w(t) to z(t) may be 
%     computed in this later case);
%   - an ellipsoidal set E(inv(W),eta) = {x / x'inv(W) x <= 1/eta) in 
%     which the trajectory (initiated anywhere) converges and is confined 
%     in the case of amplitude-bounded disturbance. This set and/or the 
%     size of the admissible delta may be optimized.
%     => resu.W, resu.eta, resu.delta, resu.beta, resu.gamma
%   ---------------------------------------------------------
%   CLOSED-LOOP SYSTEM definition (sysP, sysACT (optional), sysC, 
%                                                          sysD (optional))
%    
%   system:      dx/dt = sysP.A  x + sysP.Bu  u + sysP.Bw  w     
%                    y = sysP.Cy x + sysP.Dyu u + sysP.Dyw w 
%                    z = sysP.Cz x + sysP.Dzu u + sysP.Dzw w
%   => sysP.A, .Bu, .Cy, .Dyu, .Bw, .Dyw, .Cz, .Dzu, .Dzw
%
%   actuator:    dxa/dt = sat_u1(T0 sat_u0 (yc) - T0 xa),    
%                     u = xa
%   => sysACT.T0 (The bounds u0 and u1 may be let undefined)
%
%   controller: dxc/dt = sysC.A  xc + sysC.Bu  y + sysC.Bw  w
%                   yc = sysC.Cy xc + sysC.Dyu y + sysC.Dyw w
%   => sysC.A, .Bu, .Cy, .Dyu, .Bw, .Dyw
%
%   All unspecified matrices of the problem are set equal to 0, except
%   sysP.Cy set equal to the identity matrix. 
%   In particular, for the state-feedback case (only sysC.Dyu given), 
%   sysP.Cy and sysP.Dyu may be undefined.
%    
%   disturbance (optional):
%      sysD.opt = 'none' (Default), 'amplitude', 'energy' 
%            E(R,delta) = {w / w' R w <= 1/delta} (amplitude) 
%                      or {w / \integrale ( w' R w) <= 1/delta (energy)
%      sysD.R: if unspecified, R = I
%      sysD.delta: if unspecified, delta = 1
%      sysD.tau1 and sysD.tau2 (parameter of the S-procedure for the
%           amplitude-bounded disturbance). If unspecified, set = 1 
%      Remark: Note that for the case of amplitude bounded disturbance,
%              tau1/2 > | l_max(A+BK) |
%      Remark: when not used, sysD.opt = 'none' AND the dimension of the
%              disturbance if forced to 0 (with appropriate associated  
%              null matrices), independently of what has been set in SysP
%              and sysC !!!
%  
%   optsat (optional): structure element for problem optimization
%       optsat.W: MINIMIZATION of the size of E(inv(W),eta), i.e. on W +
%                 eta. Used only for the case of amplitude-bounded 
%                 disturbance
%                 Take value between 0 and 3
%                 1 : max trace(W)       
%                 2 : min trace(inv(W))
%                 3 : min mu = 1/sqrt(beta), scaling of Xi0. Xi0 has to
%                     be specified. Otherwise, Default is used.                                   
%                 0 : No optimisation on the set      ----- Default -------
%        optsat.XI0: matrix of dimension (np+m+nc,nr) which columns are
%                 vertices define a set of initial conditions or directions
%                 in which increasing E(inv(W),eta). Used only with 
%                 optsat.W = 3.  
%        optsat.cond: conditioning of W. Specify the limit value. 
%                 0 = no conditioning   ----- Default -------
%        optsat.D: optimization of the disturbance. Used only in case of 
%                  amplitude-bounded disturbance. 
%                 0 : No optimization   ----- Default -------
%                 1 : Optimization of delta (recommanded to set R=I).
%                     Used in case of amplitude-bounded disturbance.
%                 2 : Computation of the L2-gain gamma from w(t) to z(t).
%                     Used in case of energy-bounded disturbance. R must 
%                     be equal to I to perserve the L2-significance of
%                     the problem.
%        optsat.solver: choice of solver in sdpsettings. It may be
%                 'lmilab' (Default), 'sedumi'...
%
%See also PsatAL, PsatSG, PsatSL
%

%   [resu,sysP,sysC,sysD] = prsatag(sysP,sysACT,sysC,sysD,optsat)
%
%   Ref.: S. Tarbouriech, G. Garcia, J.M. Gomes da Silva Jr and I. Queinnec, 
%         Stability and Stabilization of Linear Systems with Saturating 
%         Actuators. Springer, London (UK), 2011
%
%   This file is part of SATAW-Tool
%   Last Update 13-November-2019
%   Copyright (C) 2019 Isabelle Queinnec and Sophie Tarbouriech
%   queinnec@laas.fr
%   LAAS-CNRS, Toulouse, France
%

fprintf('  \n');
% ----------------------------------------------------
% Part I: test of input arguments
% ----------------------------------------------------
if nargin < 3,                error('There must be at least 3 input arguments.');   end
% test of sysP
% ------------
sysP = testsysp(sysP);
[n,m] = size(sysP.Bu);  p = size(sysP.Cy,1);  l = size(sysP.Cz,1);
% test of sysACT
% --------------
if isempty(sysACT)
    error('sysACT.T0 must be defined');
elseif isfield(sysACT,'T0')
    T0 = sysACT.T0;
    if size(T0,1)~=m || size(T0,2)~=m,error('sysACT.T0 must be a diagonal square matrix of dimension m.');                             end
else
    error('sysACT.T0 must be defined');
end
% test of sysC
% ------------
sysC = testsysc(sysC,sysP);
nc = size(sysC.A,1);
% test of sysD... if used
% -----------------------
if nargin == 3, sysD = []; end
[sysD,sysP,sysC] = testsysd(sysD,sysP,sysC);

% Read optsat... if used
% ----------------------
if nargin > 4 && ~isempty(optsat) % optsat is at least partially defined
    if isfield(optsat,'D') && ~isequal(sysD.opt,'none') 
        switch optsat.D
            case 0, %ok no optimization
            case 1,
                if isequal(sysD.opt,'energy')
                    fprintf('Warning: optsat.D=1 not allowed with energy-bounded disturbance. The Default value is forced.\n');
                    fprintf(' \n');
                    optsat.D = 0;
                end
            case 2, 
                if isequal(sysD.opt,'amplitude')
                    fprintf('Warning: Optimization of the L2-gain not allowed with amplitude-bounded disturbance. The default value is forced.\n');
                    optsat.D = 0;            
                elseif isequal(l,0)  % l is the dimension of the z output
                    fprintf('Warning: sysP.Cz must be defined to optimize the L2-gain. The default value is forced.\n');
                    optsat.D = 0;
                end
            otherwise,
                fprintf('Warning: optsat.D should be an integer between 0 and 2. The Default value is forced.\n');
                fprintf(' \n');
                optsat.D = 0;
        end
    else
        optsat.D = 0;
    end
     % optsat.W is used only for amplitude-bounded disturbance
    if isfield(optsat,'W') && isequal(sysD.opt,'amplitude')
        wchoice = 0:3;
        if find(wchoice-optsat.W == 0)>0
            % ok
        else
            fprintf('Warning: optsat.W should be an integer between 0 and 3. The Default value is forced.\n');
            fprintf(' \n');            
            optsat.W = 0;
        end
        if isfield(optsat,'Xi0') == 0 && (optsat.W == 3)
            error('optsat.Xi0 must be set to be able to consider the case optsat.W = 3.');
        end
    else
        optsat.W = 0;
    end
    if isfield(optsat,'Xi0')
        if optsat.W == 3
            Xi0 = optsat.Xi0;
            if size(Xi0,1) ~= n+m+nc, error('The columns of optsat.Xi0 should be of length n+m+nc.'); end
            if size(Xi0,2) < 1,     error('There must be at least one vector to defined optsat.Xi0.'); end
        else
            fprintf('Warning: optsat.Xi0 is not used with your selection for optsat.W\n');
            fprintf(' \n');
        end
    end    
    if isfield(optsat,'cond')
        if ~isnumeric(optsat.cond) || optsat.cond < 0
            fprintf('Warning: optsat.cond should be a positif value. The Default value (no conditionning) is forced.\n');
            fprintf(' \n');
            optsat.cond = 0;
        end
    else
        optsat.cond = 0;
    end
    if isfield(optsat,'solver') == 0
        optsat.solver = 'lmilab';
    end
else  % Default values for optsat when not used
    optsat.W = 0; % No need then to define optsat.Xi0
    optsat.cond = 0;
    optsat.D = 0;
    optsat.solver = 'lmilab';
end
% last security
% -------------
if nargin > 5, error('There must be 3, 4 or 5 input arguments.'); end

T0 = sysACT.T0;
Ap = sysP.A;
Bp = sysP.Bu;
Bpw = sysP.Bw;
Cp = sysP.Cy;
Dp = sysP.Dyu;
Dpw = sysP.Dyw;
Cz = sysP.Cz;
Dz = sysP.Dzu;
Dzw = sysP.Dzw;
Ac = sysC.A;
Bc = sysC.Bu;
Bcw = sysC.Bw;
Cc = sysC.Cy;
Dc = sysC.Dyu;
Dcw = sysC.Dyw;
tau1 = sysD.tau1;
tau2 = sysD.tau2;
R = sysD.R;

Im = eye(m);

Abar = [Ap           Bp                 zeros(n,nc);...
        T0*Dc*Cp     T0*(Dc*Dp - Im)    T0*Cc;...
        Bc*Cp        Bc*Dp              Ac];
Bphi0 = [zeros(n,m);T0;zeros(nc,m)];
Bphi1 = [zeros(n,m);eye(m);zeros(nc,m)];
C0 = [Dc*Cp  Dc*Dp Cc];                           
C1 = T0*[Dc*Cp  Dc*Dp-eye(m)  Cc];
C2 = [Cz  Dz zeros(l,nc)];
D1 = T0;
Bwbar = [Bpw;T0*(Dc*Dpw +Dcw);Bc*Dpw + Bcw];  %B2
D0w = Dcw+Dc*Dpw;
D1w = T0*(Dcw+Dc*Dpw);

if isequal(sysD.opt,'amplitude')
     [XX,DD] = reig(Abar);
     if tau1 >= 2*abs(max(diag(DD)))
         DD
         error('tau1 has to be smaller than 2*abs(l_max("A+B*K"))');
     end
end

% ----------------------------------------------------
% Part II: Programming of the OPTIMIZATION PROBLEM
% ----------------------------------------------------
% Define the sdp variables
% ------------------------
W = sdpvar(n+m+nc,n+m+nc,'symmetric');
S0 = [];
for i=1:m,  S0 = [S0;Im(i,:)*sdpvar(1,1,'symmetric')]; end;
S1 = [];
for i=1:m,  S1 = [S1;Im(i,:)*sdpvar(1,1,'symmetric')]; end;
if ~isequal(sysD.opt,'none')
    eta = sdpvar(1,1,'symmetric');
else
    eta = 1;
end
% additionnal variables related to the optimization of E(inv(W),eta)
if optsat.W == 2
    Mmu = 1; % 1 or n+nc to build either Mmu*I or the matrix Mmu
    mu = sdpvar(Mmu,Mmu,'symmetric');    
elseif optsat.W == 3
    mu = sdpvar(1,1,'symmetric');        
end
% additionnal variables related to the optimization of the admissible
% disturbance (delta)
switch optsat.D 
    case 1, delta = sdpvar(1,1,'symmetric'); %run over the value of delta if given
    case 2, gamma = sdpvar(1,1,'symmetric');
    otherwise,
            if isequal(sysD.opt,'amplitude'), delta = sysD.delta; end
end
% additionnal variables related to the conditionning on W
if optsat.cond > 0 
    lambdaW = sdpvar(1,1,'symmetric'); 
end

% Define the lmi conditions
% -------------------------
pblmi = [];

% Lyapunov condition  
if ~isequal(optsat.D,2)
    Mlyap = [Abar*W+W*Abar'+tau1*W  Bphi0*S0-W*C0'  Bphi1*S1-W*C1'   Bwbar    ;...
             S0*Bphi0'-C0*W        -2*S0           -S0*D1'          -D0w   ;...
             S1*Bphi1'-C1*W        -D1*S0          -2*S1            -D1w   ;...
             Bwbar'                -D0w'           -D1w'            -tau2*R  ];
else
    Mlyap = [Abar*W+W*Abar'+tau1*W  Bphi0*S0-W*C0'  Bphi1*S1-W*C1'   Bwbar    W*C2';...
             S0*Bphi0'-C0*W        -2*S0           -S0*D1'          -D0w      zeros(m,l);...
             S1*Bphi1'-C1*W        -D1*S0          -2*S1            -D1w      zeros(m,l);...
             Bwbar'                -D0w'           -D1w'            -tau2*R   Dzw';...
             C2*W                   zeros(l,m)      zeros(l,m)       Dzw     -gamma*eye(l)];
end
pblmi = [pblmi, Mlyap <= 0];

% Scalar condition
if isequal(sysD.opt,'amplitude')
     pblmi = [pblmi, -tau1*delta+tau2*eta < 0];
end

% Conditionning of W
if optsat.cond > 0
    McondW1 = W - lambdaW*eye(n+m+nc);
    McondW2 = W - optsat.cond*lambdaW*eye(n+m+nc);
    pblmi = [pblmi, McondW1 >= 0, McondW2 <= 0];
end    
    
% Conditions related to the optimization on W
switch optsat.W
    case 2,    McondOpt = [mu*eye(n+m+nc) eye(n+m+nc);...
                           eye(n+m+nc)    W];
               pblmi = [pblmi, McondOpt > 0];                  
    case 3,    for i=1:size(optsat.Xi0,2)
                   McondOpt = [mu optsat.Xi0(:,i)';optsat.Xi0(:,i) W];
                   pblmi = [pblmi, McondOpt > 0];
               end 
end

% Define the optimization criterion
% ---------------------------------
switch optsat.W   % minimization of the set
    case 1, crit = trace(W);
    case 2, crit = -mu;
    case 3, crit = -mu;
    %case 4, crit = geomean(W);
    case 0, crit = 0;
end
if ~isequal(optsat.W,0) && ~isequal(sysD.opt,'none')
    crit = crit + eta;
end
switch optsat.D
    case 1, crit = crit + delta;
    case 2, crit = crit + gamma;
end
if isequal(crit,0), crit= []; end

% Solve the problem
% -----------------
options_sdp = sdpsettings('solver',optsat.solver,'verbose',0);
options_sdp.lmilab.maxiter = 500;    % default = 100
options_sdp.lmilab.reltol = 0.01;    % default = 0.01 in lmilab
options_sdp.lmilab.feasradius = 1e9; % R<0 signifie "no bound", default = 1e9

solpb = solvesdp(pblmi,crit,options_sdp);
resu.problem = solpb.problem; % see its significance in yalmiperror

[pres,dres] = checkset(pblmi);
if isequal(sum(pres>0),length(pres))
    resu.W = double(W);
    if isequal(sysD.opt,'amplitude'), resu.eta = double(eta);
    else                              resu.eta = [];
    end
    if isequal(optsat.D,1), resu.delta = double(delta);
    else                    resu.delta = sysD.delta;
    end
    if optsat.W == 3, resu.beta = 1/sqrt(double(mu));
    else              resu.beta = [];
    end
    if isequal(optsat.D,2), resu.gamma = double(gamma);
    else                    resu.gamma = [];
    end
    message_feas = 'The problem has been found feasible\n';
else
    message_feas = 'The problem has been found unfeasible\n';
    resu.W = NaN;
    resu.eta = NaN;
    resu.beta = NaN;
    resu.delta = NaN;
    resu.gamma = NaN;
end

% Display the results
% -------------------
if nc == 0, message = 'GLOBAL ANALYSIS (state feedback)';
else        message = 'GLOBAL ANALYSIS (dynamic feedback)';
end
switch optsat.W, 
    case 1, message = strcat(message,', max(trace(W))');
    case 2, message = strcat(message,', min(trace(inv(W)))');
    case 3, message = strcat(message,', max beta (scaling of Xi0)');
    case 4, message = strcat(message,', max geomean(W)');
    case 0, message = strcat(message,', no optimization of W');
end
if optsat.cond > 0
   valcond = num2str(optsat.cond);
   message = strcat(message,', conditionning on W= ',valcond,'\n');
else
   message = strcat(message,', no conditioning on W\n');
end
switch sysD.opt
   case 'none', message2 = 'No disturbance\n';
   case 'amplitude', message2 = 'Amplitude bounded disturbance';
   case 'energy', message2 = 'Energy bounded disturbance';
end
if ~isequal(sysD.opt,'none')
   if isequal(optsat.D,1), message2 = strcat(message2,', optim delta\n');
   else                    message2 = strcat(message2,' \n');
   end
end
fprintf('You have been solving the following problem:\n');
fprintf('    -------------\n');
fprintf('Position+rate saturation - use of Sector nonlinearity model\n');
fprintf(message);
fprintf(message2);
fprintf(' \n');
fprintf(message_feas);
fprintf('    -------------\n');
