function [ sysP ] = testsysp( sysP )
%TESTSYSP   Test if the process model is correctly defined
%   
%sysP = testsysp(sysP) tests if the (partial) system sysP has been
%     properly defined. If true, it returns the full defined system. 
%     Undefined matrices are set as either null or identity matrices of 
%     appropriate dimensions.
%     Otherwise, it displays a message and aborts.
%
%See also testsysact, testsysc, testsysd, testsysaw
%

%   This file is part of SATAW-Tool
%   Last Update 12-Sept-2012
%   Copyright (C) 2012 Isabelle Queinnec and Sophie Tarbouriech
%   queinnec@laas.fr
%   LAAS-CNRS, Toulouse, France

switch class(sysP)
case 'struct',
    if isfield(sysP,'A') %=sum(strcmp('A',fieldnames(sysP)))>0
        A = sysP.A;
        if size(A,1) ~= size(A,2),error('sysP.A matrix must be square.');               end
    else
        error('sysP.A must be defined');
    end
    n = size(A,1);
    if isfield(sysP,'Bu')
        Bu = sysP.Bu;
        if size(Bu,1) ~= n,       error('sysP.Bu matrix must have as many rows as sysP.A.');end
    else
        error('sysP.Bu must be defined');
    end
    m = size(Bu,2);
    if isfield(sysP,'Cy')
        Cy = sysP.Cy;
        if size(Cy,2) ~= n,       error('sysP.Cy matrix must have as many columns as sysP.A.');end
    else
        Cy = eye(n);
    end
    p = size(Cy,1);
    if isfield(sysP,'Dyu')
        Dyu = sysP.Dyu;
        if size(Dyu,1) ~= p,      error('sysP.Dyu matrix must have as many rows as sysP.Cy.');end
        if size(Dyu,2) ~= m,      error('sysP.Dyu matrix must have as many columns as sysP.Bu.');end
    else
        Dyu = zeros(p,m);
    end
    if isfield(sysP,'Bw')
        Bw = sysP.Bw;
        if size(Bw,1) ~= n,       error('sysP.Bw matrix must have as many rows as sysP.A.');end
        q = size(Bw,2);
        if isfield(sysP,'Dyw')
            Dyw = sysP.Dyw;
            if size(Dyw,1) ~= p,   error('sysP.Dyw matrix must have as many rows as sysP.Cy.');end
            if size(Dyw,2) ~= q,   error('sysP.Dyw matrix must have as many columns as sysP.Byw.');end
        else
            Dyw = zeros(p,q);
        end
    else
        if isfield(sysP,'Dyw')
            Dyw = sysP.Dyw;
            if size(Dyw,1) ~= p,   error('sysP.Dyw matrix must have as many rows as sysP.Cy.');end
            q = size(Dyw,2);
            Bw = zeros(n,q);
        else %neither Bw or Dyw are defined
            q = 0;
            Bw = zeros(n,q);
            Dyw = zeros(p,q);
        end 
    end
    if isfield(sysP,'Cz')
        Cz = sysP.Cz;
        if size(Cz,2) ~= n,       error('sysP.Cz matrix must have as many columns as sysP.A.');end
        l = size(Cz,1);
        if l>0 && q == 0,         
            fprintf('Warning: the output z is not used when there is no disturbance.\n');
            fprintf(' \n');
        end
        if isfield(sysP,'Dzu')
            Dzu = sysP.Dzu;
            if size(Dzu,1) ~= l,   error('sysP.Dzu matrix must have as many rows as sysP.Cz.');end
            if size(Dzu,2) ~= m,   error('sysP.Dzu matrix must have as many columns as sysP.Bu.');end
        else
            Dzu = zeros(l,m);
        end
        if isfield(sysP,'Dzw')
            Dzw = sysP.Dzw;
            if size(Dzw,1) ~= l,   error('sysP.Dzw matrix must have as many rows as sysP.Cz.');end
            if size(Dzw,2) ~= q,   error('sysP.Dz matrix must have as many columns as sysP.Bw.');end
        else
            Dzw = zeros(l,q);
        end        
    else
        Cz = zeros(0,n);    
        Dzu = zeros(0,m);
        Dzw = zeros(0,q);
        if isfield(sysP,'Dzu') || isfield(sysP,'Dzw')
            error('the case with sysP.Cz=0 and sysP.Dzu or sysP.Dzw non null matrices is not allowed');
        end
    end
    sysP.A = A;
    sysP.Bu = Bu;
    sysP.Bw = Bw;
    sysP.Cy = Cy;
    sysP.Dyu = Dyu;
    sysP.Dyw = Dyw;
    sysP.Cz = Cz;
    sysP.Dzu = Dzu;
    sysP.Dzw = Dzw;
case 'ssmodel',
    % ssmodel already induces that defined matrices are all of appropriate
    % dimensions, including that undefined matrices are null matrices of
    % appropriate dimensions. The exception is Cy which must be forced, if
    % not defined to the identity matrix!!!
    if size(sysP.Cy,1)==0
        sysP.Cy = eye(size(sysP.A,1));
        sysP.Dyu = zeros(size(sysP.A,1),size(sysP.Bu,2));
        sysP.Dyw = zeros(size(sysP.A,1),size(sysP.Bw,2));
    end
end
    
    
    
end

