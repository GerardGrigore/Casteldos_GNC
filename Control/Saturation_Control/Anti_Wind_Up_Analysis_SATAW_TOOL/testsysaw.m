function [ sysAW ] = testsysaw( sysAW, sysC )
%TESTSYSAW   Test if the anti-windup compensator sysAW is correctly defined 
%   
%sysAW = testsysaw(sysAW,sysC) tests if the system sysAW has been
%     properly defined, especially with respect also to the controller
%     dimensions. If true, the function returns sysAW with undefined 
%     elements replaced by null matrices of appropriate dimensions. 
%     Otherwise, it displays a message and aborts.
%
%     Remark: if sysAW is empty, it is set as an empty anti-windup and
%     defined with null matrices of appropriate dimension (na=0)
%
%See also testsysp, testsysc, testsysact, testsysd
%

%   This file is part of SATAW-Tool
%   Last Update 6-March-2013
%   Copyright (C) 2012 Isabelle Queinnec and Sophie Tarbouriech
%   queinnec@laas.fr
%   LAAS-CNRS, Toulouse, France

[m,nc] = size(sysC.Cy);
q = size(sysC.Bw,2);

switch class(sysAW)
case 'struct',
    if isfield(sysAW,'A')   % dynamic AW case --> A,B,C at least have to be defined
        Aaw = sysAW.A;
        na = size(Aaw,1);
        if isfield(sysAW,'Bu')
            Baw = sysAW.Bu;
            if size(Baw,1) ~= na,  error('sysAW.Bu matrix must have as many rows as sysAW.A.');end
            if size(Baw,2) ~= m,   error('sysAW.Bu matrix must have as many columns as rows in sysC.Cy.');end
        else
            error('sysAW.Bu must be defined');
        end
        if isfield(sysAW,'Cy')
            Caw = sysAW.Cy;
            if size(Caw,1) ~= nc+m, error('sysAW.Cy matrix must have as many rows as state+output of sysC.');end
            if size(Caw,2) ~= na,   error('sysAW.Cy matrix must have as many columns as sysAW.A.');end
        else
            error('sysAW.Cy must be defined');
        end
        if isfield(sysAW,'Dyu')
            Daw = sysAW.Dyu;
            if size(Daw,1) ~= nc+m, error('sysAW.Dyu matrix must have as many rows as state+output of sysC.');end
            if size(Daw,2) ~= m,    error('sysAW.Dyu matrix must have as many columns as rows in sysC.Cy.');end
        else
            Daw = zeros(nc+m,m);
        end
        if isfield(sysAW,'Bw')
            fprintf('Warning: sysAW.Bw is not used. It is set to the null matrix of appropriate dimension\n');
            Bw = zeros(na,q);
        else
            Bw = zeros(na,q);
        end    
        if isfield(sysAW,'Dyw')
            fprintf('Warning: sysAW.Dyw is not used. It is set to the null matrix of appropriate dimension\n');
            Dw = zeros(nc+m,q);
        else
            Dw = zeros(nc+m,q);
        end
    elseif isfield(sysAW,'Dyu')   %static AW case -> D is defined 
        Daw = sysAW.Dyu;
        if size(Daw,1) ~= nc+m, error('sysAW.Dyu matrix must have as many rows as state+output of sysC.');end
        if size(Daw,2) ~= m,    error('sysAW.Dyu matrix must have as many columns as rows in sysC.Cy.');end
        Aaw = [];
        Baw = zeros(size(Aaw,1),m);
        Caw = zeros(nc+m,size(Aaw,2));
        Bw = zeros(size(Aaw,1),q);
        Dw = zeros(nc+m,q);
    else
        error('At least (sysAW.A, .Bu, . Cy) or (sysAW.Dyu) must be defined in the dynamic or static anti-windup cases');
    end   
    sysAW.A = Aaw;
    sysAW.Bu = Baw;
    sysAW.Bw = Bw;
    sysAW.Cy = Caw;
    sysAW.Dyu = Daw;
    sysAW.Dyw = Dw;
case 'ssmodel',
    % verif of the sizes of input/output of sysAW with respect to the size of sysC
    if size(sysAW.Dyu,1) ~= nc+m,  error('The output of the controller must have as many rows as state+output of sysC.');end
    if size(sysAW.Dyu,2) ~= m,  error('The input of the anti-windup must have as many columns as rows in sysC.Cy.');end
    if size(sysAW.Bw,2) ~= 0,
        fprintf('Warning: w is not used. It is set to the null matrix of appropriate dimension');
        sysAW.Bw = zeros(size(sysAW.A,1),q); 
        sysAW.Dyw = zeros(nc+m,q);
    end
end

