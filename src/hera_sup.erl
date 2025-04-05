-module(hera_sup).

-behaviour(supervisor).

-export([start_link/0]).
-export([init/1]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start_link() ->
    supervisor:start_link(?MODULE, []).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init([]) ->
    SupFlags = #{
        strategy => one_for_one,
        intensity => 6,
        period => 3600
    },
    HeraData = #{
        id => hera_data,
        start => {hera_data, start_link, []}
    },
    HeraCom = #{
        id => hera_com,
        start => {hera_com, start_link, []}
    },
    HeraMeasureSup = #{
        id => hera_measure_sup,
        start => {hera_measure_sup, start_link, []},
        type => supervisor
    },
    HeraSub = #{
    id => hera_sub,
    start => {hera_sub, start_link, []}
    },
    ChildSpecs = [HeraData, HeraCom, HeraMeasureSup, HeraSub],
    {ok, {SupFlags, ChildSpecs}}.