clear all
clc

IS_BOOL = 0;

if IS_BOOL
    VAR_TEST = FUNC_VARARGIN(1,2,[1 0;0 1]);
else
    VAR_TEST = FUNC_VARARGIN(1,2);
end