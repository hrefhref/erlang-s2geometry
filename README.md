# erls2: Erlang NIF to S2 Geometry library.

An Erlang NIF to [S2 Geometry][s2]. This is a work in progress.

The API is not expected to match entierly s2's: the goal is to make it easily usable for most of the common gis uses
cases. Most of the real interaction with s2 is done in the NIF.

## Shape Index

It provides an abstraction to work with `MutableS2ShapeIndex`: `s2_mutable_index`.

```
{ok, Pid} = s2_mutable_index:start_link(),
ok = s2_mutable_index:add_polygon(Pid, "some-id", [[{lng, lat}, ...]]),
s2_mutable_index:contains(Pid, lng, lat).
% ["some-id"] | []
```

Lower level methods to use the MutableS2ShapeIndex are provided in `s2` module, but be warned that some operations on
the Index are not thread safe and should be executed at once (e.g. in always the same genserver).


[s2]: https://github.com/google/s2geometry

