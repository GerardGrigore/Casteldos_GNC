function [ output_args ] = sataw( choice )
% SATAW - Demo examples for SATAW-Tool
%   
%  sataw   : Introduction to SATAW-Tool and demonstration
%  sataw(1): Analysis of a closed-loop system with position saturation
%  sataw(2): State-feedback design
%  sataw(3): Anti-windup design

%   This file is part of SATAW-Tool
%   Last Update 27-July-2012
%   Copyright (C) 2012 Isabelle Queinnec and Sophie Tarbouriech
%   queinnec@laas.fr
%   LAAS-CNRS, Toulouse, France

warning off
if nargin==0
  clc;
  fprintf(' \n')
  fprintf('Welcome to SATAW-Tool, the SATuration AWare Toolbox\n\n');
  fprintf('For detailed information please\n');
  fprintf(' - refer to the users guide \n');
  fprintf(' - contact Isabelle Queinnec at queinnec@laas.fr\n');
  fprintf('\n%%%% -- press key --'),pause;
  clc;
  fprintf(' \n')
  fprintf('LICENCE AGREEMENT\n') 
  fprintf('SATAW-Tool and YALMIP (that you should have installed)\n')
  fprintf('are currently free of charge and openly distributed,\n');
  fprintf('but note that they are distributed in the hope that\n');
  fprintf('it will be useful, but WITHOUT ANY WARRANTY;\n')
  fprintf('without even the implied warranty of MERCHANTABILITY\n');
  fprintf('or FITNESS FOR A PARTICULAR PURPOSE\n')
  fprintf('SATAW-Tool and YALMIP may not be re-distributed as \n')
  fprintf('a part of a commercial product\n\n')
  fprintf('SATAW-Tool and YALMIP must be referenced when used in a published\n')
  fprintf('work. See the users guide for references\n')
  fprintf('\n%%%% -- press key --'),pause;
  clc;
  fprintf(' \n')
  fprintf('Launch a SATAW-Tool demonstration example\n\n')
  fprintf('choose one among the following problems with\n')
  fprintf('(1) Analysis of a closed-loop system with position saturation.\n');
  fprintf('(2) State-feedback design.\n')
  fprintf('(3) Anti-windup design.\n');
  fprintf('(type (0) to quit)\n\n')
  choice = input('Your choice: ');      
  if isempty(choice)
    choice=0;
  end
end
fprintf(' \n')

switch choice
    case 1, afirstexampletostart;
    case 2, fprintf('not done yet\n\n');
    case 3, fprintf('not done yet\n\n');       
end

fprintf('-- END --\n\n')
end

