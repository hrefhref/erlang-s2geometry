-module(s2_mutable_index).
-behaviour(gen_server).

-export([start_link/0,
         add_point/4, add_point/5,
         add_polyline/3, add_polyline/4,
         add_polygon/3, add_polygon/4,
         nearby/3, nearby/4,
         contains/3, contains/4,
         space_used/1, space_used/2,
         is_fresh/1, is_fresh/2,
         force_build/1, force_build/2,
         minimize/1, minimize/2]).

-define(TIMEOUT, 5000).

% genserver
-export([init/1, handle_call/3, handle_cast/2, handle_info/2, terminate/2]).

start_link() ->
  Ref = s2:new_mutable_shape_index(),
  {ok, Pid} = gen_server:start_link(?MODULE, [Ref], []),
  {ok, Pid}.

add_polyline(Pid, Ref, Coords) -> add_polyline(Pid, Ref, Coords, []).

add_polyline(Pid, Ref, Coords, Opts) ->
  Timeout = proplists:get_value(timeout, Opts, ?TIMEOUT),
  gen_server:call(Pid, {add_polyline, Ref, Coords}, Timeout).

add_polygon(Pid, Ref, Coords) -> add_polygon(Pid, Ref, Coords, []).

add_polygon(Pid, Ref, Coords, Opts) ->
  Timeout = proplists:get_value(timeout, Opts, ?TIMEOUT),
  gen_server:call(Pid, {add_polygon, Ref, Coords}, Timeout).

add_point(Pid, Ref, Lng, Lat) -> add_point(Pid, Ref, Lng, Lat, []).

add_point(Pid, Ref, Lng, Lat, Opts) ->
  Timeout = proplists:get_value(timeout, Opts, ?TIMEOUT),
  gen_server:call(Pid, {add_point, Ref, Lng, Lat}, Timeout).

nearby(Pid, Lng, Lat) -> nearby(Pid, Lng, Lat, []).

nearby(Pid, Lng, Lat, Opts) ->
  Timeout = proplists:get_value(timeout, Opts, ?TIMEOUT),
  gen_server:call(Pid, {nearby, Lng, Lat, Opts}, Timeout).

contains(Pid, Lng, Lat) -> contains(Pid, Lng, Lat, []).

contains(Pid, Lng, Lat, Opts) ->
  Timeout = proplists:get_value(timeout, Opts, ?TIMEOUT),
  gen_server:call(Pid, {contains, Lng, Lat}, Timeout).

space_used(Pid) -> space_used(Pid, []).

space_used(Pid, Opts) ->
  Timeout = proplists:get_value(timeout, Opts, ?TIMEOUT),
  gen_server:call(Pid, space_used, Timeout).

is_fresh(Pid) -> is_fresh(Pid, []).

is_fresh(Pid, Opts) ->
  Timeout = proplists:get_value(timeout, Opts, ?TIMEOUT),
  gen_server:call(Pid, is_fresh, Timeout).

force_build(Pid) -> force_build(Pid, []).

force_build(Pid, Opts) ->
  Timeout = proplists:get_value(timeout, Opts, ?TIMEOUT),
  gen_server:call(Pid, force_build, Timeout).

minimize(Pid) -> minimize(Pid, []).

minimize(Pid, Opts) ->
  Timeout = proplists:get_value(timeout, Opts, ?TIMEOUT),
  gen_server:call(Pid, minimize, Timeout).


%
% --- GenServer API
%

init([Index]) ->
  {ok, Index}.

handle_call({add_polyline, Ref, Coords}, _From, Index) ->
  Rep = s2:index_add(Index, polyline, Ref, Coords),
  {reply, Rep, Index};

handle_call({add_polygon, Ref, Coords}, _From, Index) ->
  Rep = s2:index_add(Index, polygon, Ref, Coords),
  {reply, Rep, Index};

handle_call({add_point, Ref, Lng, Lat}, _From, Index) ->
  Rep = s2:index_add(Index, point, Ref, {Lng, Lat}),
  {reply, Rep, Index};

handle_call({contains, Lng, Lat}, _From, Index) ->
  Rep = case s2:index_containing_point(Index, Lng, Lat) of
    {ok, Results} -> [Id || {Id, _IndexId} <- Results];
    Error -> Error
  end,
  {reply, Rep, Index};

handle_call({nearby, Lng, Lat, Opts}, _From, Index) ->
  Rep = case s2:index_nearby_point(Index, Lng, Lat, Opts) of
    {ok, Results} -> {ok, [{Id, Distance} || {Id, _IndexId, Distance} <- Results]};
    Error -> Error
  end,
  {reply, Rep, Index};


handle_call(space_used, _From, Index) ->
  {reply, s2:index_space_used(Index), Index};

handle_call(is_fresh, _From, Index) ->
  {reply, s2:index_is_fresh(Index), Index};

handle_call(force_build, _From, Index) ->
  {reply, s2:index_force_build(Index), Index};

handle_call(minimize, _From, Index) ->
  {reply, s2:index_minimize(Index), Index}.


handle_cast(_, Index) ->
  {noreply, Index}.

handle_info(_, Index) ->
  {noreply, Index}.

terminate(_, _Index) ->
  ok.


