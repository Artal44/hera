-module(kalman).

-export([kf_predict/3, kf_update/4]).
-export([kf/6, ekf/6, ekf_control/7]).

%% see https://en.wikipedia.org/wiki/Kalman_filter


%% A kalman filter without control input
%% A standard Kalman filter implementation
%% @param {X0, P0} - Initial state estimate and covariance matrix
%% @param F - State transition matrix
%% @param H - Observation matrix
%% @param Q - Process noise covariance
%% @param R - Measurement noise covariance
%% @param Z - Measurement vector
%% @return {X1, P1} - Updated state estimate and covariance matrix
kf({X0, P0}, F, H, Q, R, Z) ->  
    % Prediction step
    {Xp, Pp} = kf_predict({X0, P0}, F, Q),
    % Update step
    kf_update({Xp, Pp}, H, R, Z).


%% Prediction step of the Kalman filter
%% @param {X0, P0} - Current state estimate and covariance matrix
%% @param F - State transition matrix
%% @param Q - Process noise covariance
%% @return {Xp, Pp} - Predicted state estimate and covariance matrix
kf_predict({X0, P0}, F, Q) ->
    % Predict the next state estimate
    Xp = mat:'*'(F, X0), 
    % Predict the next covariance matrix
    Pp = mat:eval([F, '*', P0, '*´', F, '+', Q]),
    {Xp, Pp}.


%% Update step of the Kalman filter
%% @param {Xp, Pp} - Predicted state estimate and covariance matrix
%% @param H - Observation matrix
%% @param R - Measurement noise covariance
%% @param Z - Measurement vector
%% @return {X1, P1} - Updated state estimate and covariance matrix
kf_update({Xp, Pp}, H, R, Z) ->
    % Compute the innovation covariance
    S = mat:eval([H, '*', Pp, '*´', H, '+', R]), 
    % Compute the inverse of the innovation covariance
    Sinv = mat:inv(S), 
    % Compute the Kalman gain
    K = mat:eval([Pp, '*´', H, '*', Sinv]),
    % Compute the innovation (measurement residual)
    Y = mat:'-'(Z, mat:'*'(H, Xp)),
    % Update the state estimate
    X1 = mat:eval([K, '*', Y, '+', Xp]),  
    % Update the covariance matrix
    P1 = mat:'-'(Pp, mat:eval([K, '*', H, '*', Pp])), 
    {X1, P1}.


%% An extended Kalman filter without control input
%% This function implements the EKF algorithm for systems without control inputs.
%% @param {X0, P0} - Initial state estimate and covariance matrix
%% @param {F, Jf} - State transition function and its Jacobian
%% @param {H, Jh} - Observation function and its Jacobian
%% @param Q - Process noise covariance
%% @param R - Measurement noise covariance
%% @param Z - Measurement vector
%% @return {X1, P1} - Updated state estimate and covariance matrix
ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z) ->
    % Prediction step
    % Compute the predicted state estimate using the state transition function
    Xp = F(X0),
    % Compute the Jacobian of the state transition function at the current state
    Jfx = Jf(X0),
    % Compute the predicted covariance matrix
    Pp = mat:eval([Jfx, '*', P0, '*´', Jfx, '+', Q]),

    % Update step
    % Compute the Jacobian of the observation function at the predicted state
    Jhx = Jh(Xp),
    % Compute the innovation covariance
    S = mat:eval([Jhx, '*', Pp, '*´', Jhx, '+', R]),
    % Compute the inverse of the innovation covariance
    Sinv = mat:inv(S),
    % Compute the Kalman gain
    K = mat:eval([Pp, '*´', Jhx, '*', Sinv]),
    % Compute the innovation (measurement residual)
    Y = mat:'-'(Z, H(Xp)),
    % Update the state estimate
    X1 = mat:eval([K, '*', Y, '+', Xp]),
    % Update the covariance matrix
    P1 = mat:'-'(Pp, mat:eval([K, '*', Jhx, '*', Pp])),
    {X1, P1}.

%% Same function as ekf/ with command input
%% This function implements the EKF algorithm for systems with control inputs.
%% @param {X0, P0} - Initial state estimate and covariance matrix
%% @param {F, Jf} - State transition function and its Jacobian
%% @param {H, Jh} - Observation function and its Jacobian
%% @param Q - Process noise covariance
%% @param R - Measurement noise covariance
%% @param Z - Measurement vector
%% @param U - Control input
%% @return {X1, P1} - Updated state estimate and covariance matrix
ekf_control({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z, U) -> 
    % Prediction step
    % Compute the predicted state estimate using the state transition function with control input
    Xp = F(X0, U),
    % Compute the Jacobian of the state transition function at the current state
    Jfx = Jf(X0),
    % Compute the predicted covariance matrix
    Pp = mat:eval([Jfx, '*', P0, '*´', Jfx, '+', Q]),

    % Update step
    % Compute the Jacobian of the observation function at the predicted state
    Jhx = Jh(Xp),
    % Compute the innovation covariance
    S = mat:eval([Jhx, '*', Pp, '*´', Jhx, '+', R]),
    % Compute the inverse of the innovation covariance
    Sinv = mat:inv(S),
    % Compute the Kalman gain
    K = mat:eval([Pp, '*´', Jhx, '*', Sinv]),
    % Compute the innovation (measurement residual)
    Y = mat:'-'(Z, H(Xp)),
    % Update the state estimate
    X1 = mat:eval([K, '*', Y, '+', Xp]),
    % Update the covariance matrix
    P1 = mat:'-'(Pp, mat:eval([K, '*', Jhx, '*', Pp])),
    {X1, P1}.