function [ sysC ] = testsysc( sysC, sysP )
%TESTSYSC   Test if the controller sysC is correctly defined 
%   
%sysC = testsysc(sysC,sysP) tests if the system sysC has been
%     properly defined, especially with respect also to the process model
%     dimensions. If true, the function returns sysC with undefined 
%     elements replaced by null matrices of appropriate dimensions. 
%     Otherwise, it displays a message and aborts.
%
%See also testsysp, testsysact, testsysd, testsysaw
%

%   This file is part of SATAW-Tool
%   Last Update 12-Sept-2012
%   Copyright (C) 2012 Isabelle Queinnec and Sophie Tarbouriech
%   queinnec@laas.fr
%   LAAS-CNRS, Toulouse, France

[p,m] = size(sysP.Dyu);
q     = size(sysP.Bw,2);

switch class(sysC)
case 'struct',
    if isfield(sysC,'A')   % dynamic output feedback case 
        Ac = sysC.A;
        nc = size(Ac,1);
        if isfield(sysC,'Bu')
            Bc = sysC.Bu;
            if size(Bc,1) ~= nc,  error('sysC.Bu matrix must have as many rows as sysC.A.');end
            if size(Bc,2) ~= p,   error('sysC.Bu matrix must have as many columns as rows in sysP.Cy.');end
        else
            error('sysC.Bu must be defined');
        end
        if isfield(sysC,'Cy')
            Cc = sysC.Cy;
            if size(Cc,2) ~= nc,  error('sysC.Cy matrix must have as many columns as sysC.A.');end
            if size(Cc,1) ~= m,   error('sysC.Cy matrix must have as many rows as columns in sysP.Bu.');end
        else
            error('sysC.Cy must be defined');
        end
        if isfield(sysC,'Dyu')
            Dc = sysC.Dyu;
            if size(Dc,2) ~= p,  error('sysC.Dyu matrix must have as many columns as rows in sysP.Cy.');end
            if size(Dc,1) ~= m,  error('sysC.Dyu matrix must have as many rows as columns in sysP.Bu.');end
        else
            Dc = zeros(m,p);
        end
        if isfield(sysC,'Bw')
            Bcw = sysC.Bw;
            if size(Bcw,1) ~= nc, error('sysC.Bw matrix must have as many rows as sysC.A.');end
            if size(Bcw,2) ~= q,  error('sysC.Bw matrix must have as many columns as sysP.Bw.');end
        else
            Bcw = zeros(nc,q);
        end    
        if isfield(sysC,'Dyw')
            Dcw = sysC.Dyw;
            if size(Dcw,1) ~= m,   error('sysC.Dyw matrix must have as many rows as sysC.Cy.');end
            if size(Dcw,2) ~= q,   error('sysC.Dyw matrix must have as many columns as sysC.Bw.');end
        else
            Dcw = zeros(m,q);
        end
    else
        if isfield(sysC,'Dyu')
            Dc = sysC.Dyu;
            if size(Dc,2) ~= p,  error('sysC.Dyu matrix must have as many columns as rows in sysP.Cy.');end
            if size(Dc,1) ~= m,  error('sysC.Dyu matrix must have as many rows as columns in sysP.Bu.');end
            Ac = [];
            Bc = zeros(size(Ac,1),p);
            Cc = zeros(m,size(Ac,2));
            Bcw = zeros(size(Ac,1),q);
            Dcw = zeros(m,q);
        else
            error('sysC.Dyu must be defined in the static feedback case');
        end
    end   
    sysC.A = Ac;
    sysC.Bu = Bc;
    sysC.Bw = Bcw;
    sysC.Cy = Cc;
    sysC.Dyu = Dc;
    sysC.Dyw = Dcw;
case 'ssmodel',
    % verif of the sizes of input/output of sysC with respect to the size
    % of sysP
    if size(sysC.Dyu,2) ~= p,  error('The input of the controller must have as many columns as rows in sysP.Cy.');end
    if size(sysC.Dyu,1) ~= m,  error('The output of the controller must have as many rows as columns in sysP.Bu.');end
    if size(sysC.Bw,2) ~= q 
        if size(sysC.Bw,2) == 0, 
            sysC.Bw = zeros(size(sysC.A,1),q); 
            sysC.Dyw = zeros(m,q);
        else error('w must be of same dimension both for the model and the controller');
        end
    end
end

