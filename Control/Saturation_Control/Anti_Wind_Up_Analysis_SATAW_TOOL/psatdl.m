function [resu,sysP,sysC,sysD] =  psatdl(sysP,sysACT,sysC,sysD,optsat)
%PSATDL  Dynamic output feedback synthesis 
%        for a position saturated closed-loop system - LOCAL case 
%        Strategy through Sector nonlinearities 
%
%[resu,sysP,sysC,sysD] =  psatdl(sysP,sysACT,sysC,sysD,optsat)
%
%   returns 
%   - a dynamic output-feedback controller sysC and 
%   - an ellipsoidal set E(P,eta) = {x / x'P x <= 1/eta). This 
%     set is a region of asymptotic stability (RAS, eta=1) in the case 
%     without disturbance or a region where the trajectory is confined in 
%     the case with additive disturbance. The size of the admissible 
%     disturbance (delta) or the L2-norm between w and z may also be 
%     optimized.
%     => resu.P, resu.eta, resu.delta, resu.beta, resu.gamma
%   ---------------------------------------------------------
%   SYSTEM definition (sysP, sysACT, sysC (not used), sysD (optional))
%
%   system:      dx/dt = sysP.A  x + sysP.Bu  u + sysP.Bw  w     
%                    y = sysP.Cy x  
%                    z = sysP.Cz x 
%   => sysP.A, .Bu, .Bw, .Cy, .Cz
%
%   actuator:        u = sat_u0 (yc),    |u| <= sysACT.u0
%   => sysACT.u0
%
%   controller (to be designed):
%               dxc/dt = sysC.A  xc + sysC.Bu  y + sysC.Ec (u-yc)
%                   yc = sysC.Cy xc + sysC.Dyu y 
%        Note that the controller involves an anti-windup term sysC.Ec
%        (see chapter 3 of the referenced book below)
%
%   Unspecified matrices of the problem are set equal to 0. sysP.Dyu
%   sysP.Dyw have to be null matrices.
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
%
%   optsat (optional): structure element for problem optimization
%       optsat.P: MAXIMIZATION of the size of E(P,eta) where 
%                 P = [X U;U' Xh] -> optimization on X, rho (bound on Xh)
%                 and U (+ min eta if additive disturbance)
%                 Take value between 0 and 4
%                 1 : min trace(X) + rho (U is a variable) --- Default ---
%                 2 : min trace(X) + rho (U is a variable) + { U+U'>0 }
%                 3 : min trace(X) + SVD to compute U
%                 0 : No optimisation on the set. 
%        optsat.cond: conditioning of X and Y. Specify the value. 
%                     [X optsat.COND;optsat.COND Y] >0, optsat.COND > 1 
%                     (cf Scherer, Gahinet et Chilali, IEEETAC,97)
%                 0 = no conditioning   ----- Default -------
%        optsat.D: optimization of the disturbance. Used only if 
%                 disturbance input matrices are specified. 
%                 0 : No optimization   ----- Default -------
%                 1 : optimization of delta (recommanded to set R=I) both
%                     disturbance cases
%                 2 : optimization of L2-gain gamma from w(t) to z(t). In
%                     in case of energy-bounded disturbance only. R must
%                     be equal to I to perserve the L2-significance of the
%                     problem.
%        optsat.GAIN: constraint on sysC.Dyu. Specify the value 
%                     [optsat.GAIN^2 I sysC.Dyu';sysC.Dyu I] > 0
%                 0 = no constraint     ----- Default -------
%        optsat.solver: choice of solver in sdpsettings. It may be
%                 'lmilab' (Default), 'sedumi'...
%        optsat.SDPSETTINGS: possibility to fix the sdpsettings for lmilab
%                 or sedumi (see sdpsettings for Yalmip)
%        optsat.PP: additional constraint for pole placement of the linear
%                 closed-loop system in a strip between -optsat.PP(1) and
%                 -optsat.PP(2), with 0 < optsat.PP(1) < optsat.PP(2).
%                 optsat.PP is a positive vector of dimension 2.
%
%See also psatdg, psatsl, psatsg
%

%   [resu,sysP,sysC,sysD] = psatdl(sysP,sysACT,sysC,sysD,optsat)
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
if nargin < 2,                error('There must be at least 2 input arguments.');   end
% test of sysP
% ------------
sysP = testsysp(sysP);
[n,m] = size(sysP.Bu);  q = size(sysP.Bw,2); 
p = size(sysP.Cy,1);    l = size(sysP.Cz,1); 
if ~isequal(sysP.Dyu,zeros(p,m)) warning('sysP.Dyu is not used in this function'); end
if ~isequal(sysP.Dyw,zeros(p,q)) warning('sysP.Dyw is not used in this function'); end
if ~isequal(sysP.Dzu,zeros(l,m)) warning('sysP.Dzu is not used in this function'); end
if ~isequal(sysP.Dzw,zeros(l,q)) warning('sysP.Dzw is not used in this function'); end
    
% test of sysACT
% --------------
sysACT = testsysact(sysACT,sysP,'position');
% test of sysC... unused
% -----------------------
if ~isempty(sysC)  
    fprintf('Note that sysC is not used.\n');
else 
    sysC.Dyw = zeros(m,q);
end
% test of sysD... if used
% -----------------------
if nargin == 3, sysD = []; end
[sysD,sysP,sysC] = testsysd(sysD,sysP,sysC);
q = size(sysP.Bw,2);  % For the case where q is forced to 0

% Read optsat... if used
% ----------------------
if nargin > 4 && ~isempty(optsat) % optsat is at least partially defined
    if isfield(optsat,'D') && ~isequal(sysD.opt,'none') 
        dchoice = 0:2;
        if find(dchoice-optsat.D == 0)>0                
            if isequal(optsat.D,2) 
                if isequal(sysD.opt,'amplitude')
                    fprintf('Warning: Optimization of the L2-gain not allowed with amplitude-bounded disturbance. The default value is forced.\n');
                    optsat.D = 0;                    
                elseif isequal(l,0)  % l is the dimension of the z output
                    fprintf('Warning: sysP.Cz must be defined to optimize the L2-gain. The default value is forced.\n');
                    optsat.D = 0;
                end
            end
        else
            fprintf('Warning: optsat.D should be an integer between 0 and 2. The Default value is forced.\n');
            fprintf(' \n');
            optsat.D = 0;
        end
    else
        optsat.D = 0;
    end
    if isfield(optsat,'P')
        wchoice = 0:3;
        if find(wchoice-optsat.P == 0)>0
            % ok
        else
            fprintf('Warning: optsat.P should be an integer between 0 and 3. The Default value is forced.\n');
            fprintf(' \n');            
            optsat.P = 1;
        end
    else
        optsat.P = 1;
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
    if isfield(optsat,'GAIN')
        if ~isnumeric(optsat.GAIN) || optsat.GAIN < 0
            fprintf('Warning: optsat.GAIN should be a positif value. The Default value (no conditionning) is forced.\n');
            fprintf(' \n');
            optsat.GAIN = 0;
        end
    else
        optsat.GAIN = 0;
    end
    if isfield(optsat,'solver') == 0
        optsat.solver = 'lmilab';
    end
    if isfield(optsat,'SDPSETTINGS') == 0
        optsat.SDPSETTINGS = '';
    end    
    if isfield(optsat,'PP') && isequal(max(size(optsat.PP)),2)
        if (optsat.PP(2)>=optsat.PP(1)) && (optsat.PP(1)>0)
             perfo = 'strip';
        else fprintf('Warning: pole placement in a strip not properly defined. Not used.\n'); 
             fprintf(' \n');
             perfo = 'none';
        end                    
    else
        perfo = 'none';
    end
else  % Default values for optsat when not used
    optsat.P = 1; 
    optsat.cond = 0;
    optsat.D = 0;
    optsat.solver = 'lmilab';
    optsat.SDPSETTINGS = '';
    optsat.GAIN = 0;
    perfo = 'none';
end


% last security
% -------------
if nargin > 5, error('There must be from 2 to 5 input arguments.'); end

u0 = sysACT.u0;

A = sysP.A;
B = sysP.Bu;
C = sysP.Cy;
Bw = sysP.Bw;
Cz = sysP.Cz;
tau1 = sysD.tau1;
tau2 = sysD.tau2;
R = sysD.R;

Im = eye(m);
In = eye(n);
Il = eye(l);
On = zeros(n,n);
Omq = zeros(m,q);
Oml = zeros(m,l);
Onl = zeros(n,l);
Oql = zeros(q,l);

% if optsat(5) ~= 0
%     options(1) = optsat(5);
% end;
%options(2) = 500;
%options(3) = -1; % < 0 par de rayon d'arret
%options(4) = 100;
%options(5) = 1;  % 0 pour afficher les iterations, 1 au contraire

% ----------------------------------------------------
% Part II: Programming of the OPTIMIZATION PROBLEM
% ----------------------------------------------------
% Define the sdp variables
% ------------------------
X = sdpvar(n,n,'symmetric');
Y = sdpvar(n,n,'symmetric');
S = [];
for i=1:m,  S = [S;Im(i,:)*sdpvar(1,1,'symmetric')]; end;
Q = sdpvar(n,m,'full');  % The term XBS is replaced by Q in the Lyap cond
L = sdpvar(m,n,'full');
F = sdpvar(n,p,'full');
W = sdpvar(n,n,'full');
Z = sdpvar(m,n,'full');  
Z1 = sdpvar(m,n,'full');
Dc = sdpvar(m,p,'full');  % directly sysC.Dyu
if ~isequal(sysD.opt,'none')
    eta = sdpvar(1,1,'symmetric');
else
    eta = 1;
end
% additionnal variables related to the optimization of the admissible
% disturbance (delta)
if optsat.D == 1
    delta = sdpvar(1,1,'symmetric'); %run over the value of delta if given
else
    delta = sysD.delta;
end
if optsat.D == 2
    gamma = sdpvar(1,1,'symmetric');
end
% additionnal variables related to the optimization of E(inv(W),eta)
if optsat.P == 1 || optsat.P == 2
    rho = sdpvar(1,1,'symmetric');    
    U = sdpvar(n,n,'full');
end
% additionnal variables related to the conditionning on X and Y
if optsat.cond > 0 
    lambdaXY = sdpvar(1,1,'symmetric'); 
end

% Define the lmi conditions
% -------------------------
pblmi = [];

% Lyapunov condition    
if ~isequal(optsat.D,2)
    Mlyap = [A*Y+Y*A'+B*L+L'*B'+tau1*Y,   W+tau1*In,       B*S-Z',   Bw;...
             W'+tau1*In,     A'*X+X*A+C'*F'+F*C+tau1*X,    Q-Z1',    X*Bw;...          
             S*B'-Z,                      Q'-Z1,          -2*S,      Omq;...
             Bw',                         Bw'*X,           Omq',    -tau2*R];
else
    Mlyap = [A*Y+Y*A'+B*L+L'*B'+tau1*Y   W+tau1*In       B*S-Z'   Bw      Y*Cz';...
             W'+tau1*In     A'*X+X*A+C'*F'+F*C+tau1*X    Q-Z1'    X*Bw    Cz';...          
             S*B'-Z                      Q'-Z1          -2*S      Omq     Oml;...
             Bw'                         Bw'*X           Omq'    -tau2*R  Oql;...
             Cz*Y                        Cz            Oml'     Oql'   -gamma*Il];
end
pblmi = [pblmi, Mlyap <= 0];

% Inclusion condition
for i = 1:m
   MInc = [Y              In                 (L'-Z')*Im(:,i);...
           In             X                  (C'*Dc'-Z1')*Im(:,i);...
           Im(i,:)*(L-Z)  Im(i,:)*(Dc*C-Z1)  eta*u0(i)^2];
   pblmi = [pblmi, MInc >= 0];
end

% Scalar condition
switch sysD.opt
    case 'amplitude', pblmi = [pblmi, -tau1*delta+tau2*eta < 0];
    case 'energy',    pblmi = [pblmi, -delta+eta < 0];
end

% conditions definissant le probleme d'optimisation sur P
switch optsat.P 
    case 1,    McondOpt = [rho*In U'  On;...
                           U      X   In;...
                           On     In  Y];
               pblmi = [pblmi, McondOpt >= 0];
    case 2,    McondOpt = [rho*In U'  On;...
                           U      X   In;...
                           On     In  Y];
               Mcondsup = U+U';
               pblmi = [pblmi, McondOpt >= 0, Mcondsup >= 0];
end

% Conditionning of X and Y ??????
if optsat.cond > 0
    McondW1 = [X             lambdaXY*In;...
               lambdaXY*In   Y];
    McondW2 = lambdaXY;
    pblmi = [pblmi, McondW1 >= 0, McondW2 <= 0];
end    
    
% pole-placement requirements
if isequal(perfo,'strip')
    a1 = min(optsat.PP);
    a2 = max(optsat.PP);
    % Upper bound close to the origin
    Mperfo1 = [A*Y+Y*A'+B*L+L'*B' W;W'        A'*X+X*A+F*C+C'*F']...
            + 2*a1*[Y In;In X];
    % lower bound fA*ar to the origin
    Mperfo2 = [A*Y+Y*A'+B*L+L'*B' W;W'        A'*X+X*A+F*C+C'*F']...
            + 2*a2*[Y In;In X];
    pblmi = [pblmi, Mperfo1 <= 0, Mperfo2 >= 0];
end
    
    
% limitation on the size of Dc
if optsat.GAIN >0
    MDc = [optsat.GAIN^2*eye(p) Dc';Dc Im];
    pblmi = [pblmi, MDc >= 0];
end

% Define the optimization criterion
% ---------------------------------
switch optsat.P
    case 1, crit = trace(X) + rho;
    case 2, crit = trace(X) + rho;
    case 3, crit = trace(X);
    case 0, crit = 0;
end
if ~isequal(optsat.P,0) && ~isequal(sysD.opt,'none')
    crit = crit + eta;
end
switch optsat.D
    case 1, crit = crit + delta;
    case 2, crit = crit + gamma;
end
if isequal(crit,0), crit= []; end


% Solve the problem
% -----------------
% options_sdp = sdpsettings('solver',optsat.SOLVER,'verbose',0);
% if sum(strcmp('SDPSETTINGS',fieldnames(optsat))) > 0  
%     optsat.SDPSETTINGS = mysdpsettings(optsat.SDPSETTINGS,optsat.SOLVER);
%     switch optsat.SOLVER
%         case 'lmilab', options_sdp.lmilab = optsat.SDPSETTINGS;
%         case 'sedumi', options_sdp.sedumi = optsat.SDPSETTINGS;        
%     end
% end
options_sdp = mysdpsettings(optsat.SDPSETTINGS,optsat.solver);

solpb = solvesdp(pblmi,crit,options_sdp);
resu.problem = solpb.problem; % see its significance in yalmiperror
resu.crit = double(crit);
[pres,dres] = checkset(pblmi);
if isequal(sum(pres>0),length(pres))
    clear sysC
    if isa(sysP,'ssmodel');
        sysC = ssmodel('controller designed');
    end
    X = double(X);
    Y = double(Y);
    Q = double(Q);
    S = double(S);
    Dc = double(Dc);
    W = double(W);
    L = double(L);
    F = double(F);
    if ~isequal(optsat.P,1) && ~isequal(optsat.P,2) 
        % si pas d'optimisation sur X,rho, alors U n'est pas une variable
        % calcul de U et V / UV' = M'N = eye(n+nc) - X*Y
        % Merci a Jean-Marc Biannic pour le calcul de Q issu de X et Y!!!!! 
%         [UU,Sigma,VV]=svd(eye(n)-X*Y);
%         U=sqrt(Sigma)*UU';                                   % U
%         Vpp=sqrt(Sigma)*VV';                                 % V'
        U = In;
        V = (In-X*Y)'*inv(U');
        Vpp = V';
    else
        U = double(U);
        Vpp = inv(U)*(In - X*Y);
    end
    sysC.Ec = inv(U)*(Q - X*B*S)*inv(S);
    sysC.Dyu = Dc;
    sysC.Cy = (L-Dc*C*Y)*inv(Vpp);
    sysC.Bu = inv(U)*(F-X*B*Dc);
    sysC.A = inv(U)*(W' - (A+B*Dc*C)' - X*A*Y -X*B*L - U*sysC.Bu*C*Y)*inv(Vpp);
    
    % calcul de P
%     Xchap1 = -U'*Y*inv(Vpp);
%     Xchap2 = -U'*Y*inv(eye(n)-X*Y)*U;
%     P = [X U;U' Xchap1];
    resu.P =[X U;U' U'*inv(X-inv(Y))*U];
    resu.eta = double(eta);
    if optsat.D == 1, resu.delta = double (delta);
    else              resu.delta = [];
    end
    if optsat.D == 2, resu.gamma = double(gamma);
    else              resu.gamma = [];
    end
    message_feas = 'The problem has been found feasible\n';
else
    message_feas = 'The problem has been found unfeasible\n';
    resu.W = NaN;
    resu.eta = NaN;
    resu.delta = NaN;
    resu.gamma = NaN;
end

% Display the results
% -------------------
message = 'LOCAL SYNTHESIS (output feedback)';
switch optsat.P, 
   case 1, message = strcat(message,', min(trace(X)+rho)');
   case 2, message = strcat(message,', min(trace(X)+rho) + constraint U+U''>0');
   case 3, message = strcat(message,', min(trace(X), U is not a variable');
   case 0, message = strcat(message,', no set optimization');
end
if optsat.cond > 0
   valcond = num2str(optsat.cond);
   message = strcat(message,', conditionning on X, Y= ',valcond,'\n');
else
   message = strcat(message,', no conditionning on X, Y\n');
end
if isequal(perfo,'strip')
        valcond1 = num2str(optsat.PP(1));
        valcond2 = num2str(optsat.PP(2));
        message3 = strcat('pole-placement in a strip between -',valcond1,' and -',valcond2,'\n');
else
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
   else                    message2 = strcat(message2,', optim on P only\n');
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



