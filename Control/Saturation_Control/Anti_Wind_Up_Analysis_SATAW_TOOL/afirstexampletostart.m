% AFIRSTEXAMPLETOSTART - A first example to start
%
%   Analysis of a closed-loop system with position saturation
%
%See also PsatAL

%   This file is part of SATAW-Tool
%   Last Update 12-Sept-2012
%   Copyright (C) 2012 Isabelle Queinnec and Sophie Tarbouriech
%   queinnec@laas.fr
%   LAAS-CNRS, Toulouse, France
clc
clear
text1=['%%%% --------------------------------------\n',...
       '%%%% Stability analysis of the closed-loop\n'...
       '%%%% saturated system described by:\n',...
       '%%%%\n',...	     
       '%%%%    dx/dt = A x + B u\n'...
       '%%%%     u = sat_u0(K x)\n',...
       '%%%%\n'];
clc;fprintf(text1);
fprintf('%%%% ---------------------------------------\n');
fprintf('%%%% -- press key --'),pause,
fprintf(' \n\n');
fprintf('%%%% ------------------\n');
fprintf('%%%% Define the system:\n');
fprintf('%%%% ------------------\n');
fprintf('%%%% >> sysP = ssmodel(''A short example'');\n');   % This line is optional 
fprintf('%%%% >> sysP.A = [0.1 -0.1;0.1 -3];\n');
fprintf('%%%% >> sysP.Bu = [5 0;0 1];\n');
fprintf('%%%% >> sysC = ssmodel(''Its state-feedback control'');\n');   % This line is also optional 
fprintf('%%%% >> sysC.Dyu = [-0.7283 -0.0338;-0.0135 -1.3583];\n');
fprintf('%%%% >> sysACT.u0 = [5;2];\n');
%sysP = ssmodel('A short example');
sysP.A = [0.1 -0.1;0.1 -3];
sysP.Bu = [5 0;0 1];
%sysC = ssmodel('Its state-feedback control');
sysC.Dyu = [-0.7283 -0.0338;-0.0135 -1.3583];
sysACT.u0 = [5;2];
fprintf('%%%% -- press key --'),pause,

fprintf(' \n\n');
fprintf('%%%% --------------------------------\n');
fprintf('%%%% Solve the problem - default case\n');
fprintf('%%%% --------------------------------\n');
fprintf('%%%% >> resu1 = psatal(sysP,sysACT,sysC);\n');
optsat.solver = 'lmilab';
resu1 = psatal(sysP,sysACT,sysC);
resu1.W
fprintf('%%%% -- press key --'),pause,

fprintf(' \n\n');
fprintf('%%%% -------------------------------------\n');
fprintf('%%%% Solve the problem - with optimization\n');
fprintf('%%%% of directions which form a square box\n');
fprintf('%%%% -------------------------------------\n');
fprintf('%%%% >> OPTSAT.W = 3;\n');
fprintf('%%%% >> OPTSAT.Xi0 = [1 1 -1 -1;1 -1 1 -1];\n');
fprintf('%%%% >> resu2 = psatal(sysP,sysACT,sysC,'''',optsat);\n');
optsat.W = 3;
optsat.Xi0 = [1 1 -1 -1;1 -1 1 -1];
resu2 = psatal(sysP,sysACT,sysC,'',optsat);
resu2.beta
fprintf('%%%% -- press key --'),pause,

fprintf(' \n\n');
fprintf('%%%% -------------------------------------------------\n');
fprintf('%%%% Consider now a dynamic output-feedback controller\n');
fprintf('%%%% with unchanged optimization criterion\n');
fprintf('%%%% --------------------------------------------------\n');
fprintf('%%%% >> clear sysC\n');
fprintf('%%%% >> sysC = ssmodel(''A dynamic-feedback control'');\n');   % This line is optional 
fprintf('%%%% >> sysC.A = [-171.2 27.2;-68 -626.8];\n');
fprintf('%%%% >> sysC.Bu = [-592.2 5.539;-4.567 149.8];\n');
fprintf('%%%% >> sysC.Cy = [0.146 0.088;-6.821 -5.67]; \n');
fprintf('%%%% >> optsat.Xi0 = [1 1 -1 -1;1 -1 1 -1;0 0 0 0;0 0 0 0]; \n');
fprintf('%%%% >> resu3 = psatal(sysP,sysACT,sysC,'''',optsat);\n');
clear sysC
%sysC = ssmodel('dynamic output-feedback');
sysC.A = [-171.2 27.2;-68 -626.8];
sysC.Bu = [-592.2 5.539;-4.567 149.8]; 
sysC.Cy = [0.146 0.088;-6.821 -5.67]; 
optsat.Xi0 = [1 1 -1 -1;1 -1 1 -1;0 0 0 0;0 0 0 0]; 
resu3 = psatal(sysP,sysACT,sysC,'',optsat);
resu3.beta
fprintf('%%%% -- press key --'),pause,

fprintf(' \n\n');
fprintf('%%%% -------------------------------------------------\n');
fprintf('%%%% Then add an energy-bounded disturbance which acts\n');
fprintf('%%%% on the dynamics of the plant. Solve the problem\n');
fprintf('%%%% with the same dynamic controller and optimization\n');
fprintf('%%%% criterion\n');
fprintf('%%%% -------------------------------------------------\n');
fprintf('%%%% >> sysP.Bw = [1;1]; \n');
fprintf('%%%% >> sysD.opt = ''energy'';\n'); 
fprintf('%%%% >> sysD.R = 1;\n'); 
fprintf('%%%% >> sysD.delta = 1.2;\n'); 
fprintf('%%%% >> resu4 = psatal(sysP,sysACT,sysC,sysD,optsat);\n');
sysP.Bw = [2;1]; 
sysD.opt = 'energy'; 
sysD.R = 1; 
sysD.delta = 1.2; 
resu4 = psatal(sysP,sysACT,sysC,sysD,optsat);
resu4.beta
fprintf('%%%% -- press key --'),pause,

fprintf(' \n\n');
fprintf('%%%% ------------------------------------------------------\n');
fprintf('%%%% It may be also interesting to evaluate the maximal\n');
fprintf('%%%% admissible disturbance (min delta) without restriction\n'); 
fprintf('%%%% on the size of the admissible region of initial states\n');
fprintf('%%%% ------------------------------------------------------\n');
fprintf('%%%% >> optsat.D = 1;\n');
fprintf('%%%% >> optsat.W = 0;\n'); 
fprintf('%%%% >> resu5 = psatal(sysP,sysACT,sysC,sysD,optsat);\n');
optsat.D = 1;
optsat.W = 0;
resu5 = psatal(sysP,sysACT,sysC,sysD,optsat);
resu5.delta





