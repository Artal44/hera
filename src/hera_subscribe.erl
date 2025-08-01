-module(hera_subscribe).

-behaviour(gen_server).

-export([start_link/0, subscribe/1, notify/1]).
-export([init/1, handle_call/3, handle_cast/2]).

start_link() ->
    gen_server:start_link({local, ?MODULE}, ?MODULE, [], []).

subscribe(Pid) ->
    hera:logg("[HERA_SUBSCRIBE] New subscriber ~p~n",[Pid]),
    gen_server:call(?MODULE, {subscribe, Pid}).

notify(Msg) ->
    hera:logg("[HERA_SUBSCRIBE] Notifying ~p~n",[Msg]),
    gen_server:cast(?MODULE, {notify, Msg}).

init([]) ->
    {ok, []}.

handle_call({subscribe, Pid}, _From, Subscribers) ->
    {reply, ok, [Pid|Subscribers]}.

handle_cast({notify, Msg}, Subscribers) ->
    [Pid ! {hera_notify, Msg} || Pid <- Subscribers],
    {noreply, Subscribers}.
