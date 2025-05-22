function [resu,sysP,sysC,sysD,sysAW] = pawag(sysP,sysACT,sysC,sysD,sysAW,optsat)
%PAWAG   Anti-windup - Stability analysis
%        Position saturated closed-loop system with AW - GLOBAL case
%        Strategy through Sector nonlinearities
%
%[resu,sysP,sysC,sysD,sysAW] = pawag(sysP,sysACT,sysC,sysD,sysAW,optsat)
%
%   The structure sysACT is actually not used in the function but let in
%   the input arguments to keep the same input and output for all the 
%   function, as much as possible
%
%   returns:
%   * just the fact that the system is globally stable or not (feasible
%     or unfeasible problem) in the case without disturbance 
%   * and also the L2-gain gamma from w(t) to z(t) in the case with energy
%     bounded disturbance
%   => resu.W, resu.gamma
%   ---------------------------------------------------------
%   CLOSED-LOOP SYSTEM definition (sysP, sysACT (optional), sysC, 
%                                                  sysD (optional), sysAW)
%    
%   system:      dx/dt = sysP.A  x + sysP.Bu  u + sysP.Bw  w     
%                    y = sysP.Cy x + sysP.Dyu u + sysP.Dyw w 
%                    z = sysP.Cz x + sysP.Dzu u + sysP.Dzw w
%   => sysP.A, .Bu, .Cy, .Dyu, .Bw, .Dyw, .Cz, .Dzu, .Dzw
%
%   actuator:        u = sat_u0 (yc),    |u| <= sysACT.u0
%   => sysACT.u0 (but may be let undefined)
%
%   controller:  dxc/dt = sysC.A  xc + sysC.Bu  y + sysC.Bw  w + nux
%                    yc = sysC.Cy xc + sysC.Dyu y + sysC.Dyw w + nuy
%   => sysC.A, .Bu, .Cy, .Dyu, .Bw, .Dyw
%
%   Anti-windup: dxa/dt = sysAW.A xa + sysAW.Bu  (sat_u0(yc)-yc)
%             [nux;nuy] = sysAW.C xa + sysAW.Dyu (sat_u0(yc)-yc)
%   => sysAW.A, .Bu, .Cy, .Dyu
%   Remark: sysAW is optional. If not given, one solves the analysis 
%           problem without anti-windp = PsatAG
%           sysAW.Dyu only = static anti-windup
%           sysAW.A, sysAW.Bu and sysAW.Cy (at least) = dynamic anti-windup 
%
%   All unspecified matrices of the problem are set equal to 0, except
%   sysP.Cy set equal to the identity matrix. 
%    
%   disturbance (optional):
%      sysD.opt = 'none' (Default) or 'energy' 
%            E(R,delta) = {w / \integrale ( w' w) <= 1/delta (energy)
%      Remark: when not used, sysD.opt = 'none' AND the dimension of the
%              disturbance if forced to 0 (with appropriate associated  
%              null matrices), independently of what has been set in SysP
%              and sysC !!!
%
%   optsat (optional): structure element for problem optimization
%        optsat.cond: conditioning of W. Specify the limit value. 
%                 0 = no conditioning   ----- Default -------
%        optsat.D: optimization of the disturbance. Used only if 
%                 disturbance input matrices are specified. 
%                 0 : No optimization   ----- Default -------
%                 1 : not used!!!!!!
%                 2 : optimization of gamma
%        optsat.solver: choice of solver in sdpsettings. It may be
%                 'lmilab' (Default), 'sedumi'... 
%
%See also psatag, pawal, pawsg
%

%   [resu,sysP,sysC,sysD,sysAW] = pawag(sysP,sysACT,sysC,sysD,sysAW,optsat)
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
if ~isempty(sysACT)
    fprintf('Note that the actuator limit is not used in the global case.\n');
end
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
    sysAW.Dyu = zeros(nc+m,1);
end
sysAW = testsysaw(sysAW,sysC);
na = size(sysAW.A,1);

% Read optsat... if used
% ----------------------
if nargin > 5  && ~isempty(optsat) % optsat is at least partially defined
    if isfield(optsat,'D') && ~isequal(sysD.opt,'none') 
        dchoice = [0 2];
        if find(dchoice-optsat.D == 0)>0
            % ok
        else
            fprintf('Warning: optsat.D should be 0 or 2. The Default value is forced.\n');
            fprintf(' \n');
            optsat.D = 0;
        end
    else
        optsat.D = 0;
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
    optsat.cond = 0;
    optsat.D = 0;
    optsat.solver = 'lmilab';
end
% last security
% -------------
if nargin > 6, error('There must be 3 to 6 input arguments.'); end

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
Q = sdpvar(np+nc+na,np+nc+na,'symmetric');
S = [];
for i=1:m,  S = [S;Im(i,:)*sdpvar(1,1,'symmetric')]; end;
% additionnal variables related to the optimization of the admissible
% disturbance (delta)
if optsat.D == 2 % optimization of gamma 
    gamma = sdpvar(1,1,'symmetric');
else  % no optimization on the disturbance
    gamma = [];
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
Mlyap = [Ag*Q+Q*Ag'        B1gS-Q*C1g'    B2g;...
         B1gS'-C1g*Q  -2*S-D11gS-D11gS'  -D12g ;...
         B2g'               -D12g'       -eye(q)];
if isequal(optsat.D,2)
    Mlyap = [      Mlyap       [Q*C2g';D21gS';D22g'];...
             C2g*Q D21gS D22g    -gamma*eye(l)];
end
pblmi = [pblmi, Mlyap <= 0];

% Conditionning of Q
if optsat.cond > 0
    McondQ1 = Q - lambdaQ*eye(np+nc+na);
    McondQ2 = Q - optsat.cond*lambdaQ*eye(np+nc+na);
    pblmi = [pblmi, McondQ1 >= 0, McondQ2 <= 0];
end    

% To ensure that S>0
pblmi = [pblmi, S >= 0];


% Define the optimization criterion
% ---------------------------------
if optsat.D == 2
    crit = gamma;
else
    crit= [];
end

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
    if optsat.D == 2, resu.gamma = double (gamma);
    else              resu.gamma = [];
    end
    message_feas = 'The problem has been found feasible\n';
else
    message_feas = 'The problem has been found unfeasible\n';
    resu.W = NaN;
    resu.gamma = NaN;
end


% Display the results
% -------------------
if na == 0, message = 'GLOBAL ANALYSIS (static AW)';
else        message = 'GLOBAL ANALYSIS (dynamic AW)';
end
%message = strcat(message,', no optimization of W');
if optsat.cond > 0
   valcond = num2str(optsat.cond);
   message = strcat(message,', conditionning on W= ',valcond,'\n');
else
   message = strcat(message,', no conditioning on W\n');
end
switch sysD.opt
   case 'none',   message2 = 'No disturbance\n';
   case 'energy', message2 = 'Energy bounded disturbance';
end
if ~isequal(sysD.opt,'none')
   if isequal(optsat.D,2), message2 = strcat(message2,', optim gamma\n');
   else                    message2 = strcat(message2,' \n');
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

