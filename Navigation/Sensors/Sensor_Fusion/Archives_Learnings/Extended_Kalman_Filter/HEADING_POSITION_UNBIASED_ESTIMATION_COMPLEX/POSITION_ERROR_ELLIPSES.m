function [POSITION_EAST_ELLIPSE,POSITION_NORTH_ELLIPSE] = POSITION_ERROR_ELLIPSES(EAST_ORIGIN_ELLIPSE,...
                                                                                  NORTH_ORIGIN_ELLIPSE,...
                                                                                  POSITION_COVARIANCE_MATRIX,...
                                                                                  CONFIDENCE_INTERVAL)

% This code was entirely retaken from the sensor fusion thesis. The theory
% behind shall be understood and proved. Algorithm used to assess, by first
% glance, if results are compliant and logical.

if nargin < 3
    error('Too few inputs');
end
if nargin < 4
    CONFIDENCE_INTERVAL = 50;
end
if CONFIDENCE_INTERVAL == 50
    PARAMETER = 1.1572;
elseif CONFIDENCE_INTERVAL == 95;
    PARAMETER = 2.449;
else
    error('Input must be either 50 or 95 percentage as confidence interval.');
end

DELTA_TIME = 0.1*pi/180; % Angular resolution of the plot (0.1° precision).
TIME = [(0:DELTA_TIME:2*pi)';0];
[EVAL,EVEC] = eig(POSITION_COVARIANCE_MATRIX);
RADIUS_LENGTH_AXES = PARAMETER*sqrt(diag(EVEC));
EAST_POSITION = RADIUS_LENGTH_AXES(1)*cos(TIME);
NORTH_POSITION = RADIUS_LENGTH_AXES(2)*sin(TIME);
POSITION_EAST_ELLIPSE = EAST_ORIGIN_ELLIPSE + [EAST_POSITION NORTH_POSITION]*EVAL(1,:)';
POSITION_NORTH_ELLIPSE = NORTH_ORIGIN_ELLIPSE + [EAST_POSITION NORTH_POSITION]*EVAL(2,:)';

end

