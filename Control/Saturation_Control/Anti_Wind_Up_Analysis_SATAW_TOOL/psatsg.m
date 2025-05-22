function [resu,sysP,sysC,sysD] = psatsg(sysP,sysACT,sysC,sysD,optsat)
%PSATSG  State-feedback design 
%        for a position saturated closed-loop system - GLOBAL case
%        Strategy through Sector nonlinearities
%
%   [resu,sysP,sysC,sysD] = psatsg(sysP,sysACT,sysC,sysD,optsat) 
%
%   The structure sysACT is actually not used in the function but let to
%   keep the same input and output for all the function, as much as
%   possible
%
%   returns 
%   - a state-feedback control gain sysC.Dyu (no optimization in case 
%     without disturbance) and
%   - an ellipsoidal set E(inv(W),eta) = {x / x'inv(W) x <= 1/eta) in 
%     which the trajectory (initiated anywhere) converges and is confined 
%     in the case of amplitude-bounded disturbance. This set and/or the 
%     size of the admissible delta may be optimized.
%   - optionally, the L2-gain fro w to z which may be optimized in the
%     case of energy-bounded disturbance
%     => resu.W, resu.eta, resu.delta, resu.beta, resu.gamma
%   ---------------------------------------------------------
%   SYSTEM definition (sysP, sysACT (optional), sysC (optional), 
%                                                          sysD (optional))
%    
%   system:      dx/dt = sysP.A  x + sysP.Bu  u + sysP.Bw  w     
%                    z = sysP.Cz x + sysP.Dzu u + sysP.Dzw w
%   => sysP.A, .Bu, .Bw, .Cz, .Dzu, .Dzw
%
%   actuator:        u = sat_u0 (yc),    |u| <= sysACT.u0
%   => sysACT.u0 (but may be let undefined)
%
%   controller (optional: sysC.Dyw):
%                   yc =             sysC.Dyu  x + sysC.Dyw  w
%   => sysC.Dyw
%
%   All unspecified matrices of the problem are set equal to 0 except
%   sysP.Cy set equal to the identity matrix (to solve a state-feedback 
%   problem).
%
%   disturbance (optional):
%      sysD.opt = 'none' (Default), 'amplitude', 'energy' 
%            E(R,delta) = {w / w' R w <= 1/delta} (amplitude) 
%                      or {w / \integrale ( w' R w) <= 1/delta (energy)
%      sysD.R: if unspecified, R = I
%      sysD.delta: if unspecified, delta = 1
%      sysD.tau1 and sysD.tau2 (parameter of the S-procedure for the
%           amplitude-bounded disturbance). If unspecified, set = 1 
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
%                 1 : max trace(W)       ----- Default -------
%                 2 : min trace(inv(W))
%                 3 : min mu = 1/sqrt(beta), scaling of Xi0. Xi0 has to
%                     be specified. Otherwise, Default is used.                                   
%                 0 : No optimisation on the set. 
%        optsat.Xi0: matrix of dimension (np,nr) which columns are
%                 vertices define a set of initial conditions or directions
%                 in which increasing E(inv(W),eta). Used only with 
%                 optsat.W = 3.  
%        optsat.cond: conditioning of W. Specify the limit value. 
%                 0 = no conditioning   ----- Default -------
%        optsat.D: optimization of the disturbance. Used only if 
%                 disturbance input matrices are specified. 
%                 0 : No optimization   ----- Default -------
%                 1 : Optimization of delta (recommanded to set R=I).
%                     Used in case of amplitude-bounded disturbance.
%                 2 : Optimization of L2-gain gamma from w(t) to z(t).
%                     Used in case of energy-bounded disturbance. R must 
%                     be equal to I to perserve the L2-significance of
%                     the problem.
%        optsat.solver: choice of solver in sdpsettings. It may be
%                 'lmilab' (Default), 'sedumi'...
%        optsat.PP: additional constraint for pole placement of the linear
%                 closed-loop system:
%                 - at left of optsat.PP if it is a positive scalar
%                 - in a circle of center optsat.PP(1) with radius 
%                   optsat.PP(2) if it is a positive vector of dimension 2.
%                   Note: optsat.PP(2) < optsat.PP(1)
%
%See also psatag, psatsl, psatal
%

%   [resu,sysP,sysC,sysD] = psatsg(sysP,sysACT,sysC,sysD,optsat)
%
%   Ref.: S. Tarbouriech, G. Garcia, J.M. Gomes da Silva Jr and I. Queinnec, 
%         Stability and Stabilization of Linear Systems with Saturating 
%         Actuators. Springer, London (UK), 2011
%
%   This file is part of SATAW-Tool
%   Last Update 12-Sept-2012
%   Copyright (C) 2012 Isabelle Queinnec and Sophie Tarbouriech
%   queinnec@laas.fr
%   LAAS-CNRS, Toulouse, France
%

fprintf('  \n');
% ----------------------------------------------------
% Part I: test of input arguments
% ----------------------------------------------------
if nargin < 1,                error('There must be at least 1 input arguments.');   end
% test of sysP
% ------------
sysP = testsysp(sysP);
[n,m] = size(sysP.Bu);  q = size(sysP.Bw,2);   l = size(sysP.Cz,1);  %p=m in SF
% test of sysACT
% --------------
if ~isempty(sysACT)
    fprintf('Note that the actuator limit is not used in the global case.\n');
end
% test of sysC... if used (only sysC.Dw may be defined, other are unused)
% -----------------------
nc = 0;
if isempty(sysC)  
    sysC.Dyw = zeros(m,q);
else
    if isfield(sysC,'Dyw')
        if size(sysC.Dyw,1) ~= m,   error('sysC.Dw matrix must have as many rows as columns in sysP.B.');end
        if size(sysC.Dyw,2) ~= q,   error('sysC.Dw matrix must have as many columns as sysP.Bw.');end
    else
        sysC.Dyw = zeros(m,q);
    end
end
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
            if size(Xi0,1) ~= n+nc, error('The columns of optsat.Xi0 should be of length np+nc.'); end
            if size(Xi0,2) < 1,     error('There must be at least one vector to defined optsat.Xi0.'); end
        else
            fprintf('Warning: optsat.Xi0 is not used with your selection for optsat.W\n');
            fprintf(' \n');
        end
    end    
    if isfield(optsat,'cond')
        if ~isnumeric(optsat.cond) || optsat.cond < 0
            fprintf('Warning: optsat.cond should be a positif value. The Default value (no conditionning) is considered.\n');
            fprintf(' \n');
            optsat.cond = 0;
        end
    else
        optsat.cond = 0;
    end
    if isfield(optsat,'solver') == 0
        optsat.solver = 'lmilab';
    end
    if isfield(optsat,'PP') 
        switch max(size(optsat.PP))
            case 1, perfo = 'half-plane';           
            case 2,  
                if (optsat.PP(1)>=optsat.PP(2)) && (optsat.PP(2)>0)
                     perfo = 'disk';
                else fprintf('Warning: pole placement in a disk not properly defined. Not used.\n'); 
                     fprintf(' \n');
                     perfo = 'none';
                end                    
            otherwise, perfo = 'none';
        end
    else
        perfo = 'none';
    end
else  % Default values for optsat when not used
    optsat.W = 0; % No need then to define optsat.Xi0
    optsat.cond = 0;
    optsat.D = 0;
    optsat.solver = 'lmilab';
    perfo = 'none';
end
% last security
% -------------
if nargin > 5, error('There must be 1 to 5 input arguments.'); end

Ap = sysP.A;
Bp = sysP.Bu;
Bpw = sysP.Bw;
Dcw = sysC.Dyw;

Cz = sysP.Cz;
Dz = sysP.Dzu;
Dzw = sysP.Dzw;
tau1 = sysD.tau1;
tau2 = sysD.tau2;
R = sysD.R;

Im = eye(m);

Bwbar = Bp*Dcw + Bpw;
Dzwbar = Dz*Dcw+Dzw;

% ----------------------------------------------------
% Part II: Programming of the OPTIMIZATION PROBLEM
% ----------------------------------------------------
% Define the sdp variables
% ------------------------
W = sdpvar(n,n,'symmetric');
Y = sdpvar(m,n,'full');
S = [];
for i=1:m,  S = [S;Im(i,:)*sdpvar(1,1,'symmetric')]; end;
%Z = sdpvar(m,n,'full');  
if isequal(sysD.opt,'amplitude')
    eta = sdpvar(1,1,'symmetric');
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
    Mlyap = [Ap*W+W*Ap'+Bp*Y+Y'*Bp'+tau1*W   Bp*S-Y'    Bwbar;...
             S*Bp'-Y                        -2*S       -Dcw;...
             Bwbar'                         -Dcw'      -tau2*R];
else
    Mlyap = [Ap*W+W*Ap'+Bp*Y+Y'*Bp'+tau1*W   Bp*S-Y'    Bwbar       W*Cz'+Y'*Dz';...
             S*Bp'-Y                        -2*S       -Dcw         S*Dz';...
             Bwbar'                         -Dcw'      -tau2*R      Dzwbar';...
             Cz*W+Dz*Y                       Dz*S       Dzwbar     -gamma*eye(l)];
end
pblmi = [pblmi, Mlyap <= 0];

% Scalar condition
if isequal(sysD.opt,'amplitude')
     pblmi = [pblmi, -tau1*delta+tau2*eta <= 0];
end

% Conditionning of W
if optsat.cond > 0
    McondW1 = W - lambdaW*eye(n);
    McondW2 = W - optsat.cond*lambdaW*eye(n);
    pblmi = [pblmi, McondW1 >= 0, McondW2 <= 0];
end    
    
% Conditions related to the optimization on W
switch optsat.W
    case 2,    McondOpt = [mu*eye(n) eye(n);...
                           eye(n)    W];
               pblmi = [pblmi, McondOpt >= 0];                  
    case 3,    for i=1:size(optsat.Xi0,2)
                   McondOpt = [mu optsat.Xi0(:,i)';optsat.Xi0(:,i) W];
                   pblmi = [pblmi, McondOpt >= 0];
               end 
end

% pole-placement requirements
switch perfo
    case 'half-plane'
        Mperfo = Ap*W+W*Ap'+Bp*Y+Y'*Bp'+2*optsat.PP*W;
        pblmi = [pblmi, Mperfo < 0];
    case 'disk'
        MPerfo =...
            [-optsat.PP(2)*W*optsat.PP(2)     W*(Ap + optsat.PP(1)*eye(n))'+Y'*Bp';... 
              (Ap + optsat.PP(1)*eye(n))*W+Bp*Y   -W];
        pblmi = [pblmi, MPerfo < 0];
end

% Define the optimization criterion
% ---------------------------------
switch optsat.W  % minimization of the set
    case 1, crit = trace(W);
    case 2, crit = -mu;
    case 3, crit = -mu;
    %case 4, crit = -geomean(W);
    case 0, crit = 0;
end
if ~isequal(optsat.W,0) && isequal(sysD.opt,'amplitude')
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
    clear sysC
    if isa(sysP,'ssmodel');
        sysC = ssmodel('controller designed');
    end
    sysC.Dyu = double(Y)/double(W);
    sysC.Dyw = Dcw;
    resu.W = double(W);
    if isequal(sysD.opt,'amplitude'), resu.eta = double(eta);
    else                              resu.eta = [];
    end
    if isequal(optsat.D,1), resu.delta = double (delta);
    else                    resu.delta = [];
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
    clear sysC
    sysC.Dw = Dcw;
    sysC.D = NaN;
    resu.W = NaN;
    resu.eta = NaN;
    resu.beta = NaN;
    resu.delta = NaN;
    resu.gamma = NaN;
end


% Display the results
% -------------------
message = 'GLOBAL SYNTHESIS (state feedback)';
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
switch perfo
    case 'half-plane', 
        valcond1 = num2str(optsat.PP(1));
        message3 = strcat('pole-placement in the left half plane delimited by -',valcond1,'\n');
    case 'disk', 
        valcond1 = num2str(optsat.PP(1));
        valcond2 = num2str(optsat.PP(2));
        message3 = strcat('pole-placement in a disk of center -',valcond1,' and radius ',valcond2,'\n');
    case 'none', 
        message3 = 'no pole placement constraint\n';
end
switch sysD.opt
   case 'none', message2 = 'No disturbance\n';
   case 'amplitude', message2 = 'Amplitude bounded disturbance';
   case 'energy', message2 = 'Energy bounded disturbance';
end
if ~isequal(sysD.opt,'none')
   if isequal(optsat.D,1), message2 = strcat(message2,', optim delta\n');
   elseif isequal(optsat.D,2), message2 = strcat(message2,', optim gamma\n');
   else                    message2 = strcat(message2,' \n');
   end
end
fprintf('You have been solving the following problem:\n');
fprintf('    -------------\n');
fprintf('Position saturation - use of Sector nonlinearity model\n');
fprintf(message);
fprintf(message2);
fprintf(message3);
fprintf(' \n');
fprintf(message_feas);
fprintf('    -------------\n');
