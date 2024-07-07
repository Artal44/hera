-module(pid_controller).

-export([pid_init/4, pid_init/6]).

%Controller initialisation without limits on the command and on the integral error
pid_init(Kp, Ki, Kd, Set_Point) ->
  T0 = erlang:system_time() * 1.0e-9,
  pid_interface({Kp, Ki, Kd, -1, -1}, {Set_Point, Set_Point}, {0, T0, 0}).

%Complete controller initialisation
pid_init(Kp, Ki, Kd, Limit, Int_limit, Set_Point) ->
  T0 = erlang:system_time() * 1.0e-9,
  pid_interface({Kp, Ki, Kd, Limit, Int_limit}, {Set_Point, Set_Point}, {0, T0, 0}).

%General case of the controller
pid_interface({Kp, Ki, Kd, Limit, Int_limit}, {Set_point, Current_input}, {Prev_error, T0, Integral_error}) ->
	receive
		%Exit process
		{_, {exit}} ->
			io:format("closed~n");

		%Parameter modification 
		{_, {kp, New_Kp}} ->
			pid_controller({New_Kp, Ki, Kd, Limit, Int_limit}, {Set_point, Current_input}, {Prev_error, T0, Integral_error});
		{_, {ki, New_Ki}} ->
			pid_controller({Kp, New_Ki, Kd, Limit, Int_limit}, {Set_point, Current_input}, {Prev_error, T0, Integral_error});
		{_, {kd, New_Kd}} ->
			pid_controller({Kp, Ki, New_Kd, Limit, Int_limit}, {Set_point, Current_input}, {Prev_error, T0, Integral_error});
		{_, {limit, New_Limit}} ->
			pid_controller({Kp, Ki, Kd, New_Limit, Int_limit}, {Set_point, Current_input}, {Prev_error, T0, Integral_error});
		{_, {int_limit, New_Int_limit}} ->
			pid_controller({Kp, Ki, Kd, Limit, New_Int_limit}, {Set_point, Current_input}, {Prev_error, T0, Integral_error});

		%Setpoint modification
		{_, {set_point, New_Set_point}} ->
			pid_controller({Kp, Ki, Kd, Limit, Int_limit}, {New_Set_point, Current_input}, {Prev_error, T0, Integral_error});
		
		%Get next value
		{PID, {input, New_input}} ->
			pid_controller_iteration({Kp, Ki, Kd, Limit, Int_limit}, {Set_point, New_input}, {Prev_error, T0, Integral_error}, PID);

		%Reset integral error
		{_, {reset, _}} ->
			pid_controller({Kp, Ki, Kd, Limit, Int_limit}, {Set_point, Current_input}, {Prev_error, T0, 0})
  end.

%Sends the next value for the command to the process with Pid = Output_PID
pid_controller_iteration({Kp, Ki, Kd, Limit, Int_limit}, {Set_point, Input}, {Prev_error, T0, Integral_error}, Output_PID) ->
  T1 = erlang:system_time() * 1.0e-9,
  Dt = T1 - T0,

  Error = Set_point - Input,
  New_Integral_error = saturation(Integral_error + Error * Dt, Int_limit),
  Derivative_error = (Error - Prev_error) / Dt,

  Command = saturation(Kp * Error + Ki * New_Integral_error + Kd * Derivative_error, Limit),

  %Send control to the process with Pid = Output_PID
  Output_PID ! {self(), {control, Command}},

  pid_controller({Kp, Ki, Kd, Limit, Int_limit}, {Set_point, Input}, {Error, T1, New_Integral_error}).


saturation(Value, Limit) ->
  if
    Limit =< 0 -> Value;
    Value > Limit -> Limit;
    Value < -Limit -> -Limit;
    true -> Value
  end.

sign(Value) ->
    if
        Value < 0 ->
            -1;
        true ->
            1
    end.




% test() ->
%   io:format("test~n"),

%   %io:format("time: ~p~n", [erlang:monotonic_time()*1.0e-7]),
%   %timer:sleep(1000),
%   %io:format("time: ~p~n", [erlang:monotonic_time()*1.0e-7]),

%   PID = spawn(pid_controller, pid_controller, [0, 1, 0, 0, 0, 1]),


%   PID_printer = spawn(pid_controller, printer, [PID, 0]),
%   PID_printer ! {self(), {test}},
%   timer:sleep(1000),
%   PID ! {self(), {kp, 10}},
%   timer:sleep(1000),
%   %exit
%   PID ! {self(), {exit}}.