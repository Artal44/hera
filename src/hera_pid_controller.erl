-module(hera_pid_controller).

-export([pid_init/4, pid_init/6]).
-export([saturation/2, sign/1]).

% Controller initialization without limits on the command and on the integral error
% Kp, Ki, Kd: PID gains
% Set_Point: Desired setpoint for the controller
pid_init(Kp, Ki, Kd, Set_Point) ->
  % Set the process priority to maximum
  process_flag(priority, max),

  % Get the current system time in seconds
  T0 = erlang:system_time() * 1.0e-9,

  % Start the PID interface process with default limits (-1 indicates no limit)
  pid_interface({Kp, Ki, Kd, -1, -1}, {Set_Point, Set_Point}, {0, T0, 0}).

% Complete controller initialization with limits
% Kp, Ki, Kd: PID gains
% Limit: Maximum allowable command value
% Int_limit: Maximum allowable integral error
% Set_Point: Desired setpoint for the controller
pid_init(Kp, Ki, Kd, Limit, Int_limit, Set_Point) ->
  % Get the current system time in seconds
  T0 = erlang:system_time() * 1.0e-9,

  % Start the PID interface process with specified limits
  pid_interface({Kp, Ki, Kd, Limit, Int_limit}, {Set_Point, Set_Point}, {0, T0, 0}).

% Main PID interface process
% Handles incoming messages to update parameters, setpoints, or compute control commands
pid_interface({Kp, Ki, Kd, Limit, Int_limit}, {Set_point, Current_input}, {Prev_error, T0, Integral_error}) ->
	receive
		%Exit process
		{_, {exit}} ->
      hera:logg("[HERA_PID] Exiting~n", []);

    % Update proportional gain (Kp)
    {_, {kp, New_Kp}} ->
      pid_interface({New_Kp, Ki, Kd, Limit, Int_limit}, {Set_point, Current_input}, {Prev_error, T0, Integral_error});

    % Update integral gain (Ki)
    {_, {ki, New_Ki}} ->
      pid_interface({Kp, New_Ki, Kd, Limit, Int_limit}, {Set_point, Current_input}, {Prev_error, T0, Integral_error});

    % Update derivative gain (Kd)
    {_, {kd, New_Kd}} ->
      pid_interface({Kp, Ki, New_Kd, Limit, Int_limit}, {Set_point, Current_input}, {Prev_error, T0, Integral_error});

    % Update command limit
    {_, {limit, New_Limit}} ->
      pid_interface({Kp, Ki, Kd, New_Limit, Int_limit}, {Set_point, Current_input}, {Prev_error, T0, Integral_error});

    % Update integral error limit
    {_, {int_limit, New_Int_limit}} ->
      pid_interface({Kp, Ki, Kd, Limit, New_Int_limit}, {Set_point, Current_input}, {Prev_error, T0, Integral_error});

    % Update the setpoint
    {_, {set_point, New_Set_point}} ->
      pid_interface({Kp, Ki, Kd, Limit, Int_limit}, {New_Set_point, Current_input}, {Prev_error, T0, Integral_error});

    % Compute the next control command
    {PID, {input, New_input}} ->
      pid_controller_iteration({Kp, Ki, Kd, Limit, Int_limit}, {Set_point, New_input}, {Prev_error, T0, Integral_error}, PID);

    % Reset the integral error
    {_, {reset, _}} ->
      pid_interface({Kp, Ki, Kd, Limit, Int_limit}, {Set_point, Current_input}, {Prev_error, T0, 0})
  end.

% Perform a single iteration of the PID controller
% Computes the control command and sends it to the specified process
pid_controller_iteration({Kp, Ki, Kd, Limit, Int_limit}, {Set_point, Input}, {Prev_error, T0, Integral_error}, Output_PID) ->
  % Get the current system time in seconds
  T1 = erlang:system_time() * 1.0e-9,

  % Compute the time difference since the last iteration
  Dt = T1 - T0,

  % Compute the error between the setpoint and the current input
  Error = Set_point - Input,

  % Update the integral error with saturation
  New_Integral_error = saturation(Integral_error + Error * Dt, Int_limit),

  % Compute the derivative of the error
  Derivative_error = (Error - Prev_error) / Dt,

  % Compute the control command with saturation
  Command = saturation(Kp * Error + Ki * New_Integral_error + Kd * Derivative_error, Limit),

  % Send the control command to the specified process
  Output_PID ! {self(), {control, Command}},

  % Continue the PID interface process with updated state
  pid_interface({Kp, Ki, Kd, Limit, Int_limit}, {Set_point, Input}, {Error, T1, New_Integral_error}).

% Saturation function to limit a value within a specified range
% Value: The value to be limited
% Limit: The maximum allowable absolute value
saturation(Value, Limit) ->
  if
    Limit =< 0 -> Value; % No limit if Limit <= 0
    Value > Limit -> Limit; % Limit the value to the upper bound
    Value < -Limit -> -Limit; % Limit the value to the lower bound
    true -> Value % Return the value if within bounds
  end.

% Sign function to determine the sign of a value
% Returns -1 for negative values, 1 for non-negative values
sign(Value) ->
  if
    Value < 0 ->
      -1;
    true ->
      1
  end.