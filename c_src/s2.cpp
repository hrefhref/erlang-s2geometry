// Copyright 2018 AYUDO SAS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS-IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

// Author: href@random.sh (Jordan Bracco)

#include "nifpp.h"

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <set>
#include <unordered_map>
#include <vector>

#include "s2/base/commandlineflags.h"
#include "s2/base/port.h"
#include "s2/s2earth.h"
#include "s2/s2cap.h"
#include "s2/s2latlng.h"
#include "s2/s2point_index.h"
#include "s2/s2region_term_indexer.h"
#include "s2/s2point.h"
#include "s2/s2loop.h"
#include "s2/s2builder.h"
#include "s2/s2builder_layer.h"
#include "s2/s2builderutil_s2point_vector_layer.h"
#include "s2/s2builderutil_s2polygon_layer.h"
#include "s2/s2builderutil_s2polyline_layer.h"
#include "s2/s2builderutil_snap_functions.h"
#include "s2/s2polyline.h"
#include "s2/s2polygon.h"
#include "s2/s2lax_polygon_shape.h"
#include "s2/s1chord_angle.h"
#include "s2/s2shape_index.h"
#include "s2/s2contains_point_query.h"
#include "s2/mutable_s2shape_index.h"
#include "s2/s2closest_point_query.h"
#include "s2/s2point_index.h"

#include "s2.h"

using std::vector;
using std::make_tuple;
using std::ref;
using std::cout;
bool FLAGS_s2debug = false;
using namespace nifpp;

// from mongodb s2 integration when parsing geojson
bool fixOrientationTo(vector<S2Point>* points, const bool wantClockwise) {
    const vector<S2Point>& pointsRef = *points;
    if (points->size() <= 4) {
      return false;
    }
    //massert(0, "Don't have enough points in S2 orientation fixing to work with", 4 <= points->size());
    double sum = 0;
    // Sum the area under the curve...well really, it's twice the area.
    for (size_t i = 0; i < pointsRef.size(); ++i) {
        S2Point a = pointsRef[i];
        S2Point b = pointsRef[(i + 1) % pointsRef.size()];
        sum += (b[1] - a[1]) * (b[0] - a[0]);
    }
    // If sum > 0, clockwise
    // If sum < 0, counter-clockwise
    bool isClockwise = sum > 0;
    if (isClockwise != wantClockwise) {
        vector<S2Point> reversed(pointsRef.rbegin(), pointsRef.rend());
        *points = reversed;
    }
    return true;
    //uassert(0, "Couldn't fix the orientation of a loop!", 0);
}
static void eraseDuplicatePoints(vector<S2Point>* vertices) {
    for (size_t i = 1; i < vertices->size(); ++i) {
        if ((*vertices)[i - 1] == (*vertices)[i]) {
            vertices->erase(vertices->begin() + i);
            // We could have > 2 adjacent identical vertices, and must examine i again.
            --i;
        }
    }
}

static bool isLoopClosed(const vector<S2Point>& loop) {
  cout << "\n isLoopClosed\n";
    if (loop.empty()) {
        return false;
    }

    if (loop[0] != loop[loop.size() - 1]) {
        return false;
    }

  cout << "\n isLoopClosed ok\n";
    return true;
}


extern "C" {

  static int
  load(ErlNifEnv *env, void **priv, ERL_NIF_TERM load_info)
  {
      nifpp::register_resource<MutableS2ShapeIndex>(env, nullptr, "MutableS2ShapeIndex");
      nifpp::register_resource<MutableIndexReference>(env, nullptr, "MutableIndexReference");
      return 0;
  }

  static ERL_NIF_TERM new_mutable_shape_index(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
  {
    nifpp::resource_ptr<MutableIndexReference> ptr = nifpp::construct_resource<MutableIndexReference>();
    return make(env, ptr);
  }

  static ERL_NIF_TERM index_space_used(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
  {
    MutableIndexReference* index;
    nifpp::get_throws(env, argv[0], index);
    size_t size = index->index.SpaceUsed();
    return make(env, make_tuple(str_atom("ok"), size));
  }

  static ERL_NIF_TERM index_is_fresh(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
  {
    MutableIndexReference* index;
    nifpp::get_throws(env, argv[0], index);
    bool fresh = index->index.is_fresh();
    return make(env, make_tuple(str_atom("ok"), fresh));
  }

  static ERL_NIF_TERM index_force_build(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
  {
    MutableIndexReference* index;
    nifpp::get_throws(env, argv[0], index);
    index->index.ForceBuild();
    return make(env, str_atom("ok"));
  }

  static ERL_NIF_TERM index_minimize(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
  {
    MutableIndexReference* index;
    nifpp::get_throws(env, argv[0], index);
    index->index.Minimize();
    return make(env, str_atom("ok"));
  }


  /* s2:index_contains_latlng(IndexReference, lng,lat) */
  static ERL_NIF_TERM index_containing_point(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
  {
      try
      {
        MutableIndexReference* index;
        nifpp::get_throws(env, argv[0], index);

        double lng;
        double lat;
        get_throws(env, argv[1], lng);
        get_throws(env, argv[2], lat);
        S2LatLng latlng = S2LatLng::FromDegrees(lat, lng);
        S2Point point = latlng.Normalized().ToPoint();

        S2ContainsPointQueryOptions options(S2VertexModel::OPEN);
        auto query = MakeS2ContainsPointQuery(&index->index, options);

        if (query.Contains(point)) {
          std::vector<std::tuple<std::string, int>> shapes;
          for (S2Shape* shape : query.GetContainingShapes(point)) {
              int id = shape->id();
              if(index->references.find(id) != index->references.end()) {
                std::string refid = index->references.at(id);
                shapes.push_back (std::make_tuple(refid, id));
              }
          }

          return make(env, make_tuple(str_atom("ok"), shapes));
        } else {
          return make(env, str_atom("no_match"));
        }

      }
      catch(nifpp::badarg) {}
      return enif_make_badarg(env);
  }

  // index_add_shape(IndexRef :: reference, ShapeType :: polygon | polyline, CoordsList :: [{lng,lat}, ...])
  static ERL_NIF_TERM index_add(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
  {
      try
      {
        MutableIndexReference *index;
        nifpp::get(env, argv[0], index);

        nifpp::str_atom type;
        nifpp::get_throws(env, argv[1], type);

        std::string ref;
        nifpp::get_throws(env, argv[2], ref);

        if (type == "polyline") {
          vector<std::tuple<double,double>> rawlist;
          nifpp::get_throws(env, argv[3], rawlist);

          vector<S2LatLng> latlngs;

          auto f_each = [&](std::tuple<double,double> tuple) {
            double lng = std::get<0>(tuple);
            double lat = std::get<1>(tuple);
            S2LatLng latlng = S2LatLng::FromDegrees(lat, lng);
            latlngs.push_back (latlng);
          };

          std::for_each(rawlist.begin(), rawlist.end(), f_each);

          S2Polyline polyline = S2Polyline();
          polyline.Init(latlngs);

          if (polyline.IsValid()) {
            int res = index->index.Add(absl::make_unique<S2Polyline::Shape>(&polyline));
            return make(env, std::make_tuple(str_atom("ok"), res));
          } else {
            return make(env,str_atom("invalid_polyline"));
          }
        }

        if (type == "polygon") {
          vector<vector<std::tuple<double,double>>> coords;
          nifpp::get_throws(env, argv[3], coords);

        vector<S2Loop *> loops;

          using s2builderutil::IntLatLngSnapFunction;
          S2Builder builder(S2Builder::Options(IntLatLngSnapFunction(7)));
          S2Polygon polygon;

          builder.StartLayer(absl::make_unique<IndexedS2PolygonLayerWithRef>(index, ref));

        auto f_each = [&](vector<std::tuple<double,double>> list) {
          vector<S2Point> points;
          auto load_points = [&](std::tuple<double,double> tuple) {
            double lng = std::get<0>(tuple);
            double lat = std::get<1>(tuple);
            S2LatLng latlng = S2LatLng::FromDegrees(lat, lng);
            S2Point point = latlng.Normalized().ToPoint();
            points.push_back (point);
          };

          std::for_each(list.begin(), list.end(), load_points);
          // Verify loop is closed todo
          eraseDuplicatePoints(&points);
          // Drop the duplicated last point.
          points.resize(points.size() - 1);
          S2Loop loop(points);
          loop.Init(points);
          if(!loop.IsValid()) {
            cout << "!! loop is not valid\n";
            //return make(env,std::make_tuple(str_atom("error"), str_atom("invalid_polygon_loop")));
          };
          loop.Normalize();
          builder.AddLoop(loop);
        };

          std::for_each(coords.begin(), coords.end(), f_each);

          S2Error error;

          if (!builder.Build(&error)) {
            S2_LOG(ERROR) << error;
            cout << "invalid polygon\n";
            return make(env,std::make_tuple(str_atom("error"), str_atom("invalid_polygon")));
          }

          return make(env, str_atom("ok"));
        } // if polygon

        return enif_make_badarg(env);

      }
      catch(nifpp::badarg) {}
      return enif_make_badarg(env);
  }

ErlNifFunc nif_funcs[] =
{
    {"new_mutable_shape_index", 0, new_mutable_shape_index},
    {"index_force_build", 1, index_force_build, ERL_NIF_DIRTY_JOB_CPU_BOUND},
    {"index_minimize", 1, index_minimize, ERL_NIF_DIRTY_JOB_CPU_BOUND},
    {"index_containing_point", 3, index_containing_point, ERL_NIF_DIRTY_JOB_CPU_BOUND},
    {"index_space_used", 1, index_space_used, ERL_NIF_DIRTY_JOB_CPU_BOUND},
    {"index_add", 4, index_add},
    {"index_is_fresh", 1, index_is_fresh},
};

ERL_NIF_INIT(s2, nif_funcs, load,
    NULL, NULL, NULL);

}// extern C

