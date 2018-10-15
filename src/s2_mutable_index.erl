-module(s2_mutable_index).
-behaviour(gen_server).

-export([start_link/0, add_polyline/3, add_polygon/3, contains/3]).

% genserver
-export([init/1, handle_call/3, handle_cast/2, handle_info/2, terminate/2]).

start_link() ->
  Ref = s2:new_mutable_shape_index(),
  {ok, Pid} = gen_server:start_link(?MODULE, [Ref], []),
  {ok, Pid}.

add_polyline(Pid, Ref, Coords) ->
  gen_server:call(Pid, {add_polyline, Ref, Coords}).

add_polygon(Pid, Ref, Coords) ->
  gen_server:call(Pid, {add_polygon, Ref, Coords}).

contains(Pid, Lng, Lat) ->
  gen_server:call(Pid, {contains, Lng, Lat}).

%
% --- GenServer API
%

init([IndexRef]) ->
  {ok, {IndexRef, maps:new()}}.

handle_call({add_polyline, Ref, Coords}, _From, State) ->
  call_add(State, polyline, Ref, Coords);
handle_call({add_polygon, Ref, Coords}, _From, State) ->
  call_add(State, polygon, Ref, Coords);

handle_call({contains, Lng, Lat}, _From, {IndexRef, Refs}) ->
  case s2:index_containing_point(IndexRef, Lng, Lat) of
    true -> {reply, true, {IndexRef, Refs}};
    false -> {reply, false, {IndexRef, Refs}}
  end.

handle_cast(_, State) ->
  {noreply, State}.

handle_info(_, State) ->
  {noreply, State}.

terminate(_, _State) ->
  ok.


call_add({IndexRef, Refs}, Type, Ref, Coords) ->
  {ok, Position} = s2:index_add(IndexRef, Type, Coords),
  Refs2 = maps:put(Position, Ref, Refs),
  {reply, ok, {IndexRef, Refs2}}.

