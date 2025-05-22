function [ sysD, sysP, sysC ] = testsysd( sysD, sysP, sysC )  %NEW
%TESTSYSD   Test if the disturbance is correctly defined 
%
%[sysD,sysP,sysC] = testsysd(sysD,sysP,sysC) tests if the disturbance 
%     sysD has been properly defined, especially with repect also to 
%     the plant dimensions. 
%     If true, the function returns sysD with undefined elements replaced 
%     by default values. The disturbance input matrices of sysP and sysC 
%     are also forced to empty matrices if sysD.opt ='none'.
%     Otherwise, it displays a message and aborts.
%
%See also testsysp, testsysact, testsysc, testsysaw
%

%   This file is part of SATAW-Tool
%   Last Update 13-November-2019
%   Copyright (C) 2019 Isabelle Queinnec and Sophie Tarbouriech
%   queinnec@laas.fr
%   LAAS-CNRS, Toulouse, France

q     = size(sysP.Bw,2);
if q == 0 || isempty(sysD)  % NO DISTURBANCE
    sysD.opt = 'none';
    R = zeros(q,q);
    tau1 = 0;
    tau2 = 1;
    delta = 0;
else
    if ~isequal(sysD.opt,'none') && ~isequal(sysD.opt,'energy') && ~isequal(sysD.opt,'amplitude')
        error('sysD.opt must be either ''none'', ''energy'' or ''amplitude'''); 
    end   
    if ~isequal(sysD.opt,'none')
        if isfield(sysD,'R')
            R = sysD.R;
            if size(R,1) ~= size(R,2), error('sysD.R matrix must be square.');               end 
            if size(R,1) ~= q,         error('sysD.R matrix must a square matrix with as many columns as sysP.Bw.');  end
        else
            R = eye(q); 
        end
        if isfield(sysD,'delta')
            delta = sysD.delta;
            if size(delta,1) ~= 1,     error('sysD.delta must be a scalar');  end   
        else
            delta = 1;
        end
    else
        R = zeros(q,q);
        delta = 1;
    end
    if isequal(sysD.opt,'amplitude')
        if isfield(sysD,'tau1')
             tau1 = sysD.tau1;
             if size(tau1,1) ~= 1 || tau1 < 0, error('sysD.tau1 must be a positive scalar');  end
        else tau1 = 1;
        end
        if isfield(sysD,'tau2') 
             tau2 = sysD.tau2;
             if size(tau2,1) ~= 1 || tau2 < 0, error('sysD.tau2 must be a positive scalar');  end
        else tau2 = 1;
        end
    else
        tau1 = 0;
        tau2 = 1;
    end
end

sysD.R = R;
sysD.tau1 = tau1;
sysD.tau2 = tau2;
sysD.delta = delta;

if isequal(sysD.opt,'none') && q>0
    fprintf('Warning: disturbance input matrices have been forced to empty matrices.\n');
    q=0;
    switch class(sysP)
    case 'struct',
        sysP.Bw = zeros(size(sysP.A,1),q);
        sysP.Dyw = zeros(size(sysP.Cy,1),q);
        sysP.Dzw = zeros(size(sysP.Cz,1),q);
    case 'ssmodel',
        [M,ixr,izd,iz,iy,ixc,iwd,iw,iu,Ts,name]=data(sysP);
        sysP = ssmodel(M, ixr,[],[],iy,ixc,[],[],iu, Ts, name);
    end
    switch class(sysC)
    case 'struct',    
        if sum(strcmp('A',fieldnames(sysC)))>0, 
            sysC.Bw = zeros(size(sysC.A,1),q);
        else
            sysC.Bw = zeros(0,q);
        end
        %sysC.Dyw = zeros(size(sysC.Cy,1),q);
        sysC.Dyw = zeros(size(sysP.Bu,1),q);
    case 'ssmodel',
        [M,ixr,izd,iz,iy,ixc,iwd,iw,iu,Ts,name]=data(sysC);
        sysC = ssmodel(M, ixr,[],[],iy,ixc,[],[],iu, Ts, name);
    end
    sysD.R = zeros(q,q);
end

end

