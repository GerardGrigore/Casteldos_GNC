function [ sysACT ] = testsysact( sysACT , sysP, actuator )
%TESTSYSACT   Test if the actuator is correctly defined
%   
%sysACT = testsysact(sysACT, sysP, actuator) tests if the actuator 
%     sysACT, has been properly defined, with respect also to the plant
%     dimensions. The actuator may be set "position" or "posrate".
%     If true, it returns the actuator elements unchanged.
%     Otherwise, it displays a message and aborts.
%
%See also testsysp, testsysc, testsysd, testsysaw
%

%   This file is part of SATAW-Tool
%   Last Update 12-Sept-2012
%   Copyright (C) 2012 Isabelle Queinnec and Sophie Tarbouriech
%   queinnec@laas.fr
%   LAAS-CNRS, Toulouse, France

m = size(sysP.Bu,2);

if isfield(sysACT,'u0') %=sum(strcmp('u0',fieldnames(sysACT)))>0
    u0 = sysACT.u0;
    if (size(u0,1)|size(u0,2))~=1,error('sysACT.u0 must be a vector.');                             end
    if max(size(u0)) ~= m,        error('sysACT.u0 must have the same number of input as sysP.Bu.'); end
else
    error('sysACT.u0 must be defined');
end
if isequal(actuator,'posrate')
    if isfield(sysACT,'u1')
        u1 = sysACT.u1;
        if (size(u1,1)|size(u1,2))~=1,error('sysACT.u1 must be a vector.');                             end
        if max(size(u1)) ~= m,        error('sysACT.u1 must have the same number of input as sysP.Bu.'); end 
    else
        error('sysACT.u1 must be defined');
    end
    if isfield(sysACT,'T0')
        T0 = sysACT.T0;
        if size(T0,1)~=m || size(T0,2)~=m,error('sysACT.T0 must be a diagonal square matrix of dimension m.');                             end
    else
        error('sysACT.T0 must be defined');
    end
end


end

