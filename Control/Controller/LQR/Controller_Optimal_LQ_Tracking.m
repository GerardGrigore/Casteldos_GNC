function Heading_Error_LQ = Controller_Optimal_LQ_Tracking(Heading_Aimed,...
                                                           Heading_Estimated,...
                                                           Heading_Rate_Estimated,...
                                                           Optimal_Gain_Heading,...
                                                           Controller_Continuous,...
                                                           Wind_Torque_Estimator,...
                                                           Is_Wind_Compensation,...
                                                           Yaw_Hydrodynamic_Derivative)

% The coefficients tuning for the LQ Controller can be found in the
% function LQR_Tuning_Heading_Control.m

% Ellaboration of the optimal LQ commanded rudder angle:
Input_Optimal_Control = [Heading_Estimated;Heading_Rate_Estimated];

% Update the output:
if Is_Wind_Compensation == 0
    Heading_Error_LQ = Optimal_Gain_Heading*Heading_Aimed - (-Controller_Continuous'*Input_Optimal_Control);
else
    Heading_Wind_Contribution = Wind_Torque_Estimator/Yaw_Hydrodynamic_Derivative;
    if abs(Heading_Wind_Contribution) > 2*(pi/180)
        Heading_Wind_Contribution = 2*(pi/180);
    end
    Heading_Error_LQ = Optimal_Gain_Heading*Heading_Aimed - (-Controller_Continuous'*Input_Optimal_Control) - ...
                       Heading_Wind_Contribution;

end
end

