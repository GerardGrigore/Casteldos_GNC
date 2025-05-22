% ----------------------------------------------------------------
% Heading estimation based on GNSS, IMU, GYROCOMPASS measurements.
% ----------------------------------------------------------------

Is_Plot_Option_Activated = 0;

% ----------------------------------------------------------------
% 1) Definition of the process model:
% ----------------------------------------------------------------
% We define the following process model state vector:
% States_Initial = [Position_X_Current;...
%                   Position_Y_Current;...
%                   Psi_Heading_Current;...
%                   Velocity_Current];

% Initiate the variables:
Position_X_Current = 0;
Position_Y_Current = 0;
Psi_Heading_Current = -1.13; % To be aligned with the first measurements.
Velocity_Current = 5/3.6;
States_Initial = [Position_X_Current;Position_Y_Current;Psi_Heading_Current;Velocity_Current];

% We then define the process noise covariance matrix refered as the 'Q' matrix in estimation:
Variance_Position_X = 5;
Variance_Position_Y = 5;
Variance_Heading = (15*pi/180);
Variance_Velocity = 10e-4;
Process_Model_Covariance_Matrix = [Variance_Position_X^2 0 0 0;
                                   0 Variance_Position_Y^2 0 0;
                                   0 0 Variance_Heading^2 0;
                                   0 0 0 Variance_Velocity^2]; % Search for the influence of this term.

% We define the covariance matrix of the states at the initial step refered as the 'P0|0':
% We just take Covariance_Matrix_States_Initial =
% Process_Model_Covariance_Matrix:
Covariance_States_Initial = Process_Model_Covariance_Matrix; % Search for the influence of this term.

% Define the control input matrix refered as the 'B' matrix in the usual
% description of linear state-space system reprezentation:
Control_Inputs_Initial = 0; % Process model considered as without input influence.

% Chose the step time, it shall be aligned with HW specification:
Time_Step_HW = 1;

% NOTA:
% When calling the EKF function such that Estimates = EKF(), to get the
% proper evaluated functions used inside (Jacobians for example), then use
% the notation Estimates = EKF(Jacobian(Variable1,Variable2)).
% If it's not working, then delete the function as an input of the EKF,
% call it directly inside the EKF function. But pay attention to get the
% good inputs inside the EKF function.
% /!\ Same thing for the Function_Process_Model inside the EKF /!\.

% ----------------------------------------------------------------
% 2) Measurement & Innovation model:
% ----------------------------------------------------------------
% We define the following measurement model state vector:
% Measurement_Vector = [Position_X_Current;...
%                       Position_Y_Current;...
%                       Psi_Heading_GNSS_Current;...
%                       Psi_Heading_Mag_Current;...
%                       Psi_Heading_IMU_Current
%                       Velocity_Current];

% Definition of the measurement noise covariance matrix refered as 'R' matrix:
% Retake the Variance_Position_X, Variance_Position_Y, Variance_Velocity
% and Variance_Heading for Variance_Heading_GNSS used in the process model section:
Variance_Heading_GNSS = Variance_Heading;
Variance_Heading_Magnetic = 5*(pi/180);
Variance_Heading_IMU = 1*(pi/180);
Measurement_Covariance_Matrix = [Variance_Position_X 0 0 0 0 0;
                                 0 Variance_Position_Y 0 0 0 0;
                                 0 0 Variance_Heading_GNSS 0 0 0;
                                 0 0 0 Variance_Heading_Magnetic 0 0;
                                 0 0 0 0 Variance_Heading_IMU 0;
                                 0 0 0 0 0 Variance_Velocity];

% Define the measurement model matrix: (ofter refered as 'H' or 'Gx'
% matrices):
Measurement_Model_Matrix = [1 0 0 0;
                            0 1 0 0;
                            0 0 1 0;
                            0 0 1 0;
                            0 0 1 0;
                            0 0 0 1];

% Define some measurements along time to simulate the sensor fusion done by
% the EKF. Then the Measurement_Vector will has to be called in the loop
% calculation:
% Re-use the constant velocity declared in the process model section.
% Definition of step times:
Position_X = [0,1,2,3,4,5,6,7,8,9,10];
Position_Y = -Position_X;
% Position_X = [0,200,310,500,850,1000,1600,1750,2200,2290,2320];
% Position_Y = [0,-150,-200,-190,-90,-20,-10,-1300,-2000,-2050,-2090];
Psi_Heading_GNSS = [-65.98,-64.58,-100,-125,-250,-130.88,-180.69,-131.3,-125,-127,-128]*(pi/180);
Psi_Heading_Magnetic = [-65.8,-64.3,-93,-130,-125.9,-135,-182,-131.2,-127,-127.5,-128.1]*(pi/180);
Psi_Heading_IMU = [-65.54,-63.3,-95,-140,-126.9,-135.2,-182.96,-133.2,-128,-125.5,-128.8]*(pi/180);
Velocity = [Velocity_Current,Velocity_Current,Velocity_Current,Velocity_Current,Velocity_Current,...
            Velocity_Current,Velocity_Current,Velocity_Current,Velocity_Current,Velocity_Current,...
            Velocity_Current];
% Adding some random noise to the signals of headings: in the intervall of
% -10*(pi/180) and +10*(pi/180).
Heading_Noise = -10*(pi/180) + (+10*(pi/180) - (-10*(pi/180)))*rand(1,length(Psi_Heading_GNSS));
Psi_Heading_GNSS = Psi_Heading_GNSS + Heading_Noise;
Psi_Heading_Magnetic = Psi_Heading_Magnetic + Heading_Noise;
Psi_Heading_IMU = Psi_Heading_IMU + Heading_Noise;

% ----------------------------------------------------------------
% Extended Kalman Filter loop calculations:
% ----------------------------------------------------------------
Number_Measurements = length(Position_Y);
% Store the results:
States_Estimated_EKF = [];
Covariance_Estimated_EKF = [];

for time_indexes = 1:Number_Measurements

    % Initialization:
    States_Previous = States_Initial; % The 'X' state vector.
    Covariance_State_Previous = Covariance_States_Initial; % The 'P0|0' equal to 'Q' in this case.
    Control_Inputs_Previous = Control_Inputs_Initial; % The usual 'B' matrix but unused here.
    Process_Model_Covariance_Matrix_Previous = Process_Model_Covariance_Matrix; % The 'Q' process model matrix.
    Measurement_Covariance_Matrix_Previous = Measurement_Covariance_Matrix; % The 'R' measurement matrix.
    Measurment_Current = [Position_X(time_indexes),Position_Y(time_indexes),Psi_Heading_GNSS(time_indexes),Psi_Heading_Magnetic(time_indexes),...
                          Psi_Heading_IMU(time_indexes),Velocity(time_indexes)];
    
    % EKF loop calculation:
    [State_Aposteriori_Current,Covariance_Aposteriori_Current] = EKF_Heading(States_Previous,Covariance_State_Previous,Control_Inputs_Previous,...
                                                                             Jacobian_State_Process_Model(States_Previous(4),States_Previous(3),Time_Step_HW),...
                                                                             Process_Model_Covariance_Matrix_Previous,Measurement_Model_Matrix,...
                                                                             Measurement_Covariance_Matrix_Previous,...
                                                                             Function_Process_Model(States_Previous(1),States_Previous(2),States_Previous(3),States_Previous(4),Time_Step_HW),...
                                                                             Measurment_Current');


    % Inputs update:
    States_Initial = State_Aposteriori_Current;
    Covariance_States_Initial = Covariance_Aposteriori_Current;

    % Store the results:
    States_Estimated_EKF = [States_Estimated_EKF,State_Aposteriori_Current];
    Covariance_Estimated_EKF = [Covariance_Estimated_EKF,Covariance_Aposteriori_Current];

    % Store the interesting components of the covariance matrix (diagonal
    % terms):
    Covariance_Position_X(time_indexes) = Covariance_Aposteriori_Current(1,1);
    Covariance_Position_Y(time_indexes) = Covariance_Aposteriori_Current(2,2);
    Covariance_Psi_Heading(time_indexes) = Covariance_Aposteriori_Current(3,3);
    Covariance_Velocity(time_indexes) = Covariance_Aposteriori_Current(4,4);

end


% ----------------------------------------------------------------
% Plot simulation results to conclude on the efficiency:
% ----------------------------------------------------------------

if Is_Plot_Option_Activated

    % Plot all the states on several respective figure:
    % Plot the Position_X estimation:
    figure;
    plot([1:Number_Measurements],Position_X,'red');
    hold on
    plot([1:Number_Measurements],States_Estimated_EKF(1,:),'blue');
    legend('Raw sensor position along X-axis','Estimated position along X-axis with heading measurements');
    xlabel('Time simulation (number of measurements)');
    ylabel('Position along X-axis (in meters)');
    title('Position along X-axis in function of time (simulation number)');

    % Plot the Position_Y estimation:
    figure;
    plot([1:Number_Measurements],Position_Y,'red');
    hold on
    plot([1:Number_Measurements],States_Estimated_EKF(2,:),'blue');
    legend('Raw sensor position along Y-axis','Estimated position along Y-axis with heading measurements');
    xlabel('Time simulation (number of measurements)');
    ylabel('Position along Y-axis (in meters)');
    title('Position along Y-axis in function of time (simulation number)');

    % Plot the fused heading angle:
    figure;
    plot([1:Number_Measurements],Psi_Heading_GNSS,'red');
    hold on
    plot([1:Number_Measurements],Psi_Heading_Magnetic,'blue');
    hold on
    plot([1:Number_Measurements],Psi_Heading_IMU,'magenta');
    hold on
    Heading_Fused = plot([1:Number_Measurements],States_Estimated_EKF(3,:),'green');
    Heading_Fused.LineWidth = 3;
    legend('Raw sensor heading GNSS','Raw sensor heading Gyrocompass','Raw sensor heading IMU','Fused heading');
    xlabel('Time simulation (number of measurements)');
    ylabel('Raw sensors measurements (in rad)');
    title('Sensor headings and fused headings in function of time (simulation number)');

    % No need to plot the velocity as it is considered being time invariant.

    % ----------------------------------------------------------------
    % Plot the covariance matrices evolution:
    % ----------------------------------------------------------------

    figure;
    plot([1:Number_Measurements],Covariance_Position_X,'red');
    legend('Covariance of the position along X-axis');
    xlabel('Time simulation (number of measurements)');
    title('Covariance along X-axis in function of time (simulation number)');

    figure;
    plot([1:Number_Measurements],Covariance_Position_Y,'red');
    legend('Covariance of the position along Y-axis');
    xlabel('Time simulation (number of measurements)');
    title('Covariance along Y-axis in function of time (simulation number)');

    figure;
    plot([1:Number_Measurements],Covariance_Psi_Heading,'red');
    legend('Covariance of the PSI angle heading');
    xlabel('Time simulation (number of measurements)');
    title('Covariance of the heading in function of time (simulation number)');

end




