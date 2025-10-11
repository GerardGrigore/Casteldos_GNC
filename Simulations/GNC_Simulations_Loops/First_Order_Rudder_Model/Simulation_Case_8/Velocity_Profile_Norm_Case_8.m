% This script serves as a mean to generate the velocity norm of the ship
% for the simulator case number 8. This reprezents a realistic profile of
% velocity that could be used for a certain mission.

% Declaration of realistic velocity steps for certain mission points:
Velocity_Norm_Control_1 = 3/3.6;
Velocity_Norm_Control_2 = 5/3.6;
Velocity_Norm_Control_3 = 10/3.6;
Velocity_Norm_Control_4 = 12/3.6;

% Time profile creation:
Time_Step_Profile_Creation = 1e-3;
Velocity_Norm_Control_Vector = 0;
Time_Mission_Duration_Arbitrary = 2*3600; % Arbitrary duration of 2h for a mission.

% Time reference for velocity norm switching:
Time_Switch_Velocity_1 = 900; % Around 15 minutes at 3 km/h to exit the port.
Time_Switch_Velocity_2 = Time_Switch_Velocity_1 + 600; % Around 10 minutes at 5 km/h to take some velocity.
Time_Switch_Velocity_3 = Time_Switch_Velocity_2 + 1800; % Around 30 minutes at 10 km/h.
Time_Switch_Velocity_4 = Time_Switch_Velocity_3 + 300; % Around 5 minutes at maximal velocity.

% Profile creation:
Time_Current = 0;
Index_Loop = 1;
while Time_Current <= Time_Mission_Duration_Arbitrary
    % Vector concatenation:
    if Time_Current < Time_Switch_Velocity_1
        Velocity_Norm_Control_Vector(Index_Loop) = Velocity_Norm_Control_1;
    elseif Time_Current >= Time_Switch_Velocity_1 && ...
            Time_Current < Time_Switch_Velocity_2
        Velocity_Norm_Control_Vector(Index_Loop) = Velocity_Norm_Control_2;
    elseif Time_Current >= Time_Switch_Velocity_2 && ...
            Time_Current < Time_Switch_Velocity_3
        Velocity_Norm_Control_Vector(Index_Loop) = Velocity_Norm_Control_3;
    elseif Time_Current >= Time_Switch_Velocity_3 && ...
            Time_Current < Time_Switch_Velocity_4
        Velocity_Norm_Control_Vector(Index_Loop) = Velocity_Norm_Control_4;
    elseif Time_Current == Time_Mission_Duration_Arbitrary
        Velocity_Norm_Control_Vector(Index_Loop) = 0;
    else
        Velocity_Norm_Control_Vector(Index_Loop) = Velocity_Norm_Control_2;
    end
    % Incrementation of the various indexes:
    Time_Current = Time_Current + Time_Step_Profile_Creation;
    Time_Stored(Index_Loop) = Time_Current;
    Index_Loop = Index_Loop + 1;
end

% Plot the profile:
figure;
Velocity_Profile = plot(Time_Stored,Velocity_Norm_Control_Vector,'r');
Velocity_Profile.LineWidth = 2;
xlabel('Time of the mission duration (s)');
ylabel('Velocity profile creation (m/s)');
legend('Velocity norm controlled by the user');
title('Profile velocity created');




















