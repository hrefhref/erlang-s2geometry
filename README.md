# erls2: Erlang NIF to S2 Geometry library.

An Erlang NIF to [S2 Geometry][s2]. This is a work in progress.

## Shape Index

It provides an abstraction to work with `MutableS2ShapeIndex`:

```
{ok, Pid} = s2_mutable_index:start_link(),
ok = s2_mutable_index:add_polygon(Pid, "some-id", [{lng, lat}, ...]),
s2_mutable_index:contains(Pid, lng, lat).
% {ok, ["some-id"}} | no_match
```


[s2]: https://github.com/google/s2geometry

