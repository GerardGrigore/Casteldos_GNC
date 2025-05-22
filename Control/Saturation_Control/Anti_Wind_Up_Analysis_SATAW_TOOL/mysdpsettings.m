function [ sdpset ] = mysdpsettings( myset, solver )
%mysdpsettings - redefine sdpsettings such as to force the parameters 
%                provided by myset. Only the settings relative to solvers
%                lmilab and sedumi may be redefined. 
%
%   See sdpsettings for Yalmip
%
%   Example: solver = 'lmilab';
%            myset.maxiter = 500;

%   This file is part of SATAW-Tool
%   Last Update 2-Oct-2012
%   Copyright (C) 2012 Isabelle Queinnec and Sophie Tarbouriech
%   queinnec@laas.fr
%   LAAS-CNRS, Toulouse, France

sdpset = sdpsettings('solver',solver,'verbose',0);

switch solver
case 'lmilab',       
    if isfield(myset,'maxiter'),      sdpset.lmilab.maxiter = myset.maxiter; end
    if isfield(myset,'reltol'),        sdpset.lmilab.reltol = myset.reltol; end
    if isfield(myset,'feasradius'),sdpset.lmilab.feasradius = myset.feasradius; end
    if isfield(myset,'L'),                  sdpset.lmilab.L = myset.L; end
case 'sedumi',
    if isfield(myset,'alg'),            sdpset.sedumi.alg = myset.alg; end
    if isfield(myset,'beta'),          sdpset.sedumi.beta = myset.beta; end
    if isfield(myset,'theta'),        sdpset.sedumi.theta = myset.theta; end
    if isfield(myset,'free'),          sdpset.sedumi.free = myset.free; end
    if isfield(myset,'sdp'),            sdpset.sedumi.sdp = myset.sdp; end
    if isfield(myset,'stepdif'),    sdpset.sedumi.stepdif = myset.stepdif; end
    if isfield(myset,'w'),                sdpset.sedumi.w = myset.w; end
    if isfield(myset,'mu'),              sdpset.sedumi.mu = myset.mu; end
    if isfield(myset,'eps'),            sdpset.sedumi.eps = myset.eps; end
    if isfield(myset,'bigeps'),      sdpset.sedumi.bigeps = myset.bigeps; end
    if isfield(myset,'maxiter'),    sdpset.sedumi.maxiter = myset.maxiter; end
    if isfield(myset,'vplot'),        sdpset.sedumi.vplot = myset.vplot; end
    if isfield(myset,'stopat'),      sdpset.sedumi.stopat = myset.stopat; end
    if isfield(myset,'denq'),          sdpset.sedumi.denq = myset.denq; end
    if isfield(myset,'denf'),          sdpset.sedumi.denf = myset.denf; end
    if isfield(myset,'numtol'),      sdpset.sedumi.numtol = myset.numtol; end
    if isfield(myset,'bignumtol'),sdpset.sedumi.bignumtol = myset.bignumtol; end
    if isfield(myset,'numlvlv'),    sdpset.sedumi.numlvlv = myset.numlvlv; end
    if isfield(myset,'chol'), 
        if isfield(myset.chol,'skip'),          sdpset.sedumi.chol.skip = myset.chol.skip; end
        if isfield(myset.chol,'canceltol'),sdpset.sedumi.chol.canceltol = myset.chol.canceltol; end
        if isfield(myset.chol,'maxu'),          sdpset.sedumi.chol.maxu = myset.chol.maxu; end
        if isfield(myset.chol,'abstol'),      sdpset.sedumi.chol.abstol = myset.chol.abstol; end
        if isfield(myset.chol,'maxuden'),    sdpset.sedumi.chol.maxuden = myset.chol.maxuden; end
    end
    if isfield(myset,'cg') 
        if isfield(myset.cg,'maxiter'),sdpset.sedumi.cg.maxiter = myset.cg.maxiter; end
        if isfield(myset.cg,'restol'),  sdpset.sedumi.cg.restol = myset.cg.restol; end
        if isfield(myset.cg,'refine'),  sdpset.sedumi.cg.refine = myset.cg.refine; end
        if isfield(myset.cg,'stagtol'),sdpset.sedumi.cg.stagtol = myset.cg.stagtol; end
        if isfield(myset.cg,'qprec'),  sdpset.sedumi.cg.maxiter = myset.cg.qprec; end
    end
    if isfield(myset,'maxradius'),sdpset.sedumi.maxradius = myset.maxradius; end
otherwise, warning('OPTSAT.SDPSETTINGS is not taken into account for this solver');
end
    
    
    
end

