function VAL = FUNC_VARARGIN(a,b,varargin)

if ~isempty(varargin)
    varargin_in = cell2mat(varargin);
else
    varargin_in = 0;
end

VAL = a + b + varargin_in(1);

% VAL = a + b;

end