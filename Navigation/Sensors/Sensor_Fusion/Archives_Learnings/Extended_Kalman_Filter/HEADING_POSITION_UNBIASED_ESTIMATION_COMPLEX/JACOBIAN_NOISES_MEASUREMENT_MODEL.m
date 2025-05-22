function JACOBIAN_NOISES_MEASUREMENT_MODEL_OUTPUT = JACOBIAN_NOISES_MEASUREMENT_MODEL(SIZE_MEASUREMENT_VECTOR)

% This function computes in output the jacobian of the noise of the process model
% evaluated on a specific point specified in input.
% Only specify the output directly:
JACOBIAN_NOISES_MEASUREMENT_MODEL_OUTPUT = eye(SIZE_MEASUREMENT_VECTOR);

end

