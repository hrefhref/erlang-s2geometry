-module(s2).

%% API exports
-export([]).

%%====================================================================
%% API functions
%%====================================================================

-export([new_mutable_shape_index/0, index_add/4, index_containing_point/3]).
-export([new_polyline/1]).
-on_load(init/0).

-define(APPNAME, s2).
-define(LIBNAME, s2).

new_mutable_shape_index() ->
  not_loaded(?LINE).

-opaque index() :: reference().
-type shape() :: polygon | polyline.
-type shape_ref() :: binary().
-type shape_index_ref() :: non_neg_integer().
-type lng() :: boolean.
-type lat() :: boolean.
-type coords() :: [{lng(), lat()}, ...].

-spec index_add(index(), shape(), shape_ref(), coords()) -> ok | {error, any()}.

index_add(_Index, _ShapeKind, _ShapeRef, _Coords) ->
  not_loaded(?LINE).

-spec index_containing_point(index(), lng(), lat()) -> {ok, [{shape_ref(), shape_index_ref()}]} | no_match.
index_containing_point(_Index, _Lng, _Lat) ->
  not_loaded(?LINE).

new_polyline(_) ->
    not_loaded(?LINE).

init() ->
    SoName = case code:priv_dir(?APPNAME) of
        {error, bad_name} ->
            case filelib:is_dir(filename:join(["..", priv])) of
                true ->
                    filename:join(["..", priv, ?LIBNAME]);
                _ ->
                    filename:join([priv, ?LIBNAME])
            end;
        Dir ->
            filename:join(Dir, ?LIBNAME)
    end,
    erlang:load_nif(SoName, 0).

not_loaded(Line) ->
    exit({not_loaded, [{module, ?MODULE}, {line, Line}]}).
