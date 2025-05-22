function [resu,sysP,sysC,sysD,sysAW] = pawal(sysP,sysACT,sysC,sysD,sysAW,optsat)
%PAWAL   Anti-windup - Stability analysis
%        Position saturated closed-loop system with AW - LOCAL case
%        Strategy through Sector nonlinearities
%
%[resu,sysP,sysC,sysD,sysAW] = pawal(sysP,sysACT,sysC,sysD,sysAW,optsat)
%
%   returns: 
%   * an ellipsoidal set E(inv(W),1) = {x / x'inv(W) x <= 1/delta) 
%   This set is a region of asymptotic stability (RAS) in the case  
%   without disturbance (delta=1) or a region where the trajectory is 
%   confined in the case with additive disturbance. 
%   * The H2 norm gamma between the energy-bounded disturbance w and 
%   the controlled output z is also minimized.
%   => resu.W, resu.eta, resu.delta, resu.beta, resu.gamma
%   ---------------------------------------------------------
%   CLOSED-LOOP SYSTEM definition (sysP, sysACT, sysC, sysD (optional), sysAW)
%    
%   system:      dx/dt = sysP.A  x + sysP.Bu  u + sysP.Bw  w     
%                    y = sysP.Cy x + sysP.Dyu u + sysP.Dyw w 
%                    z = sysP.Cz x + sysP.Dzu u + sysP.Dzw w
%   => sysP.A, .Bu, .Cy, .Dyu, .Bw, .Dyw, .Cz, .Dzu, .Dzw
%
%   actuator:        u = sat_u0 (yc),    |u| <= sysACT.u0
%   => sysACT.u0
%
%   controller:  dxc/dt = sysC.A  xc + sysC.Bu  y + sysC.Bw  w + nux
%                    yc = sysC.Cy xc + sysC.Dyu y + sysC.Dyw w + nuy
%   => sysC.A, .Bu, .Cy, .Dyu, .Bw, .Dyw
%
%   Anti-windup: dxa/dt = sysAW.A  xa + sysAW.Bu  (sat_u0(yc)-yc)
%             [nux;nuy] = sysAW.Cy xa + sysAW.Dyu (sat_u0(yc)-yc)
%   => sysAW.A, .Bu, .Cy, .Dyu
%   Remark: sysAW is optional. If not given, one solves the analysis 
%           problem without anti-windp = PsatAL
%           sysAW.Dyu only = static anti-windup
%           sysAW.A, sysAW.Bu and sysAW.Cy (at least) = dynamic anti-windup 
%
%   All unspecified matrices of the problem are set equal to 0, except
%   sysP.Cy set equal to the identity matrix. 
%    
%   disturbance (optional):
%      sysD.opt = 'none' (Default), 'energy' 
%            E(R,delta) = {w / \integrale ( w' w) <= 1/delta (energy)
%      sysD.delta: if unspecified, delta = 1
%      sysD.R is not used in this function, even if specified. R=I!!!
%      Remark: when not used, sysD.opt = 'none' AND the dimension of the
%              disturbance if forced to 0 (with appropriate associated  
%              null matrices), independently of what has been set in SysP
%              and sysC !!!
%
%   optsat (optional): structure element for problem optimization
%       optsat.W: MAXIMIZATION of the size of E(inv(W),eta)
%                 optimization on W (+ min eta if additive disturbance)
%                 Take value between 0 and 4
%                 1 : max trace(W)       ----- Default -------
%                 2 : min trace(inv(W))
%                 3 : min mu = 1/sqrt(beta), scaling of Xi0. Xi0 has to
%                     be specified. Otherwise, Default is used.                                   
%                 4 : max geomean(W) 
%                 0 : No optimisation on the set.
%        optsat.XI0: matrix of dimension (np+nc+na,nr) which columns are
%                 vertices define a set of initial conditions or directions
%                 in which increasing E(inv(W),eta). Used only with 
%                 optsat.W = 3.  
%        optsat.cond: conditioning of W. Specify the limit value. 
%                 0 = no conditioning   ----- Default -------
%        optsat.D: optimization of the disturbance. Used only if 
%                 disturbance input matrices are specified. 
%                 0 : No optimization   ----- Default -------
%                 1 : optimization of delta
%                 2 : optimization of gamma
%        optsat.solver: choice of solver in sdpsettings. It may be
%                 'lmilab' (Default), 'sedumi'... 
%
%See also psatal, pawag, pawsl
%

%   [resu,sysP,sysC,sysD,sysAW] = pawal(sysP,sysACT,sysC,sysD,sysAW,optsat)
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



fprintf('  \n');
% ----------------------------------------------------
% Part I: test of input arguments
% ----------------------------------------------------
if nargin < 3, error('There must be at least 3 input arguments.');   end
% test of sysP
% ------------
sysP = testsysp(sysP);
[np,m] = size(sysP.Bu);  p = size(sysP.Dyw,1);  
% test of sysACT
% --------------
sysACT = testsysact(sysACT,sysP,'position');
% test of sysC
% ------------
sysC = testsysc(sysC,sysP);
nc = size(sysC.A,1);
% test of sysD... if used
% -----------------------
if nargin == 3, sysD = []; end
[sysD,sysP,sysC] = testsysd(sysD,sysP,sysC);
% test of sysAW
% -------------
if nargin < 5 || isempty(sysAW)
    sysAW.Dyu = zeros(nc+m,m);
end
sysAW = testsysaw(sysAW,sysC);
na = size(sysAW.A,1);

% Read optsat... if used
% ----------------------
if nargin > 5  && ~isempty(optsat) % optsat is at least partially defined
    if isfield(optsat,'D') && ~isequal(sysD.opt,'none') 
        dchoice = 0:2;
        if find(dchoice-optsat.D == 0)>0
            % ok
        else
            fprintf('Warning: optsat.D should be an integer between 0 and 2. The Default value is forced.\n');
            fprintf(' \n');
            optsat.D = 0;
        end
    else
        optsat.D = 0;
    end
    if isfield(optsat,'W') 
        wchoice = 0:4;
        if find(wchoice-optsat.W == 0)>0
            % ok
        else
            fprintf('Warning: optsat.W should be an integer between 0 and 4. The Default value is forced.\n');
            fprintf(' \n');            
            optsat.W = 1;
        end
        if isfield(optsat,'Xi0') == 0 && (optsat.W == 3)
            error('optsat.Xi0 must be set to be able to consider the case optsat.W = 3.');
        end
    else
        optsat.W = 1;
    end
    if isfield(optsat,'Xi0') 
        if optsat.W == 3
            Xi0 = optsat.Xi0;
            if size(Xi0,1) ~= np+nc+na, error('The columns of optsat.Xi0 should be of length np+nc+na.'); end
            if size(Xi0,2) < 1,         error('There must be at least one vector to defined optsat.Xi0.'); end
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
    if isfield(optsat,'solver')==0 %no solver defined
        optsat.solver = 'lmilab';
    end
else  % Default values for optsat when not used
    optsat.W = 1; % No need then to define optsat.Xi0
    optsat.cond = 0;
    optsat.D = 0;
    optsat.solver = 'lmilab';
end
% last security
% -------------
if nargin > 6, error('There must be 3 to 6 input arguments.'); end

u0 = sysACT.u0;

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
Aaw = sysAW.A;
Baw = sysAW.Bu;
Caw = sysAW.Cy;
Daw = sysAW.Dyu;

Im = eye(m);
Ip = eye(p);
Inc = eye(nc);
q = size(sysP.Bw,2);
l = size(sysP.Cz,1);

Delta = Im - Dc*Dp;
% System well-posed ==> Delta non singular!!!
if (isequal(Dc*Dp,Im) || isequal(Dp*Dc,Ip)), error('system is not well-posed');     end
Abar = [Ap+Bp/Delta*Dc*Cp              Bp/Delta*Cc;...
        Bc*(Ip+Dp/Delta*Dc)*Cp     Ac+Bc*Dp/Delta*Cc];
Bphi = [Bp*(Im+Im/Delta*Dc*Dp);...
        Bc*Dp*(Im+Im/Delta*Dc*Dp)];
B2 = [Bp/Delta*(Dcw+Dc*Dpw)+ Bpw;...              % Bwbar in PsatAL
      Bc*Dp/Delta*(Dcw+Dc*Dpw) + Bcw + Bc*Dpw];
C1 = [Im/Delta*Dc*Cp  Im/Delta*Cc];               % Kbar
D1 = Im/Delta*Dc*Dp;                              % Dphi
D12g = Im/Delta*(Dcw+Dc*Dpw);                     % Dwbar

    
Bnu = [Bp/Delta*[zeros(m,nc) Im];...
       Bc*Dp/Delta*[zeros(m,nc) Im] + [Inc zeros(nc,m)]];
Cnu1 = Im/Delta*[zeros(m,nc) Im];
Cnu2 = Dz/Delta*[zeros(m,nc) Im];
C2 = [Cz+Dz/Delta*Dc*Cp   Dz/Delta*Cc];
D2 = Dz*(Im+Im/Delta*Dc*Dp);

% The full pb including the AW elements (see equation (7.13), p.288)
Ag = [Abar Bnu*Caw;zeros(na,np+nc) Aaw];
B1g = [Bphi+Bnu*Daw;Baw];
B2g = [B2;zeros(na,q)];
C1g = [C1 Cnu1*Caw];
C2g = [C2 Cnu2*Caw];
D11g = D1 + Cnu1*Daw;
D21g = D2 + Cnu2*Daw;
D22g = Dzw + Dz/Delta*(Dcw+Dc*Dpw);

% ----------------------------------------------------
% Part II: Programming of the OPTIMIZATION PROBLEM
% ----------------------------------------------------
% Define the sdp variables
% ------------------------
S = [];
for i=1:m,  S = [S;Im(i,:)*sdpvar(1,1,'symmetric')]; end
Z = sdpvar(m,np+nc+na,'full');  
Q = sdpvar(np+nc+na,np+nc+na,'symmetric');
% additionnal variables related to the optimization of E(inv(W),1)
if optsat.W == 2
    Mmu = 1; % 1 or n+nc to build either Mmu*I or the matrix Mmu
    mu = sdpvar(Mmu,Mmu,'symmetric');    
elseif optsat.W == 3
    mu = sdpvar(1,1,'symmetric');        
end
% additionnal variables related to the optimization of the admissible
% disturbance (delta)
if optsat.D == 1
    delta = sdpvar(1,1,'symmetric'); %run over the value of delta if given
    gamma = [];
elseif optsat.D == 2 % optimization of gamma 
    gamma = sdpvar(1,1,'symmetric');
    delta = sysD.delta;    
else  % no optimization on the disturbance
    gamma = [];
    delta = sysD.delta;
    if isequal(delta,0)
        delta = 1;   % if no disturbance, it is forced to 1 for the inclusion condition
    end
end
% additionnal variables related to the conditionning on Q
if optsat.cond > 0 
    lambdaQ = sdpvar(1,1,'symmetric'); 
end

% Define the lmi conditions
% -------------------------
pblmi = [];

% Lyapunov condition
B1gS = [Bphi+Bnu*Daw;Baw]*S;
D11gS = (D1 + Cnu1*Daw)*S;
D21gS = (D2 + Cnu2*Daw)*S;
Mlyap = [Ag*Q+Q*Ag'        B1gS-Q*C1g'-Z'   B2g;...
         B1gS'-C1g*Q-Z  -2*S-D11gS-D11gS'  -D12g ;...
         B2g'               -D12g'         -eye(q)];
if isequal(optsat.D,2)
    Mlyap = [      Mlyap       [Q*C2g';D21gS';D22g'];...
             C2g*Q D21gS D22g    -gamma*eye(l)];
end
pblmi = [pblmi, Mlyap <= 0];

% Inclusion condition
for i = 1:m
   MInc = [Q Z'*Im(:,i);Im(i,:)*Z delta*u0(i)^2];
   pblmi = [pblmi, MInc >= 0];
end

% Conditionning of W
if optsat.cond > 0
    McondQ1 = Q - lambdaQ*eye(np+nc+na);
    McondQ2 = Q - optsat.cond*lambdaQ*eye(np+nc+na);
    pblmi = [pblmi, McondQ1 >= 0, McondQ2 <= 0];
end    

% Conditions related to the optimization on W
switch optsat.W
    case 2,    McondOpt = [mu*eye(np+nc+na) eye(np+nc+na);...
                           eye(np+nc+na)    Q];
               pblmi = [pblmi, McondOpt >= 0];                  
    case 3,    lmicondOpt = set('');
               for i=1:size(optsat.Xi0,2)
                   McondOpt = [mu optsat.Xi0(:,i)';optsat.Xi0(:,i) Q];
                   pblmi = [pblmi, McondOpt >= 0];
               end 
end

% To ensure that S>0
if ~isequal(D11gS,zeros(m,m))
    pblmi = [pblmi, S >= 0];
end


% Define the optimization criterion
% ---------------------------------
switch optsat.W
    case 1, critW = -trace(Q);
    case 2, critW = mu;
    case 3, critW = mu;
    case 4, critW = -geomean(Q);
    case 0, critW = 0;
end
switch optsat.D
    case 1, critD = delta;
    case 2, critD = gamma;
    otherwise, critD = 0;
end
crit = critD + critW;
if isequal(crit,0), crit= []; end

% Solve the problem
% -----------------
options_sdp = sdpsettings('solver',optsat.solver,'verbose',0);
options_sdp.lmilab.maxiter = 500;   % default = 100
options_sdp.lmilab.reltol = 0.01;    % default = 0.01 dans lmilab
options_sdp.lmilab.feasradius = 1e9; % R<0 signifies "no bound", default = 1e9

solpb = solvesdp(pblmi,crit,options_sdp);
resu.problem = solpb.problem; % see its significance in yalmiperror

[pres,dres] = checkset(pblmi);
if isequal(sum(pres>0),length(pres))
    resu.W = double(Q);
    resu.S = double(S);
    if optsat.D == 1, resu.delta = double (delta);
    else,             resu.delta = sysD.delta;
    end
    if optsat.D == 2, resu.gamma = double (gamma);
    else,             resu.gamma = [];
    end
    if optsat.W == 3, resu.beta = 1/sqrt(double(mu));
    else,             resu.beta = [];
    end
    message_feas = 'The problem has been found feasible\n';
else
    message_feas = 'The problem has been found unfeasible\n';
    resu.W = NaN;
    resu.beta = NaN;
    resu.delta = NaN;
    resu.gamma = NaN;
end


% Display the results
% -------------------
if na == 0, message = 'LOCAL ANALYSIS (static AW)';
else        message = 'LOCAL ANALYSIS (dynamic AW)';
end
switch optsat.W
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
       %%%%%%%%%%%%%%%%%%%
       %%%%%%%%%%%%%%%%%%%%
      % Il va falloit mettre quelque part que c'est pas prevu le cas
      % amplitude!!!!!!
   case 'amplitude', message2 = 'Amplitude bounded disturbance';
   case 'energy', message2 = 'Energy bounded disturbance';
end
if ~isequal(sysD.opt,'none')
   if isequal(optsat.D,1),     message2 = strcat(message2,', optim delta\n');
   elseif isequal(optsat.D,2), message2 = strcat(message2,', optim gamma\n');
   else,                       message2 = strcat(message2,' \n');
   end
end
fprintf('You have been solving the following problem:\n');
fprintf('    -------------\n');
fprintf('Position saturation - use of Sector nonlinearity model\n');
fprintf(message);
fprintf(message2);
fprintf(' \n');
fprintf(message_feas);
fprintf('    -------------\n');

