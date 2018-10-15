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

#include "s2/mutable_s2shape_index.h"
#include "s2/s2builder.h"
#include "s2/s2builder_graph.h"
#include "s2/s2builder_layer.h"
#include "s2/s2error.h"
#include "s2/s2polygon.h"

struct MutableIndexReference {
  MutableS2ShapeIndex index;
  std::map<int, std::string> references = {};
};

class IndexedS2PolygonLayerWithRef : public S2Builder::Layer {
 public:
  using Options = s2builderutil::S2PolygonLayer::Options;
  explicit IndexedS2PolygonLayerWithRef(MutableIndexReference* index,
                                 std::string ref,
                                 const Options& options = Options())
      : index_(index), ref_(ref), polygon_(new S2Polygon),
        layer_(polygon_.get(), options) {}

  GraphOptions graph_options() const override {
    return layer_.graph_options();
  }

  void Build(const Graph& g, S2Error* error) override {
    layer_.Build(g, error);
    if (error->ok() && !polygon_->is_empty()) {
      int id = index_->index.Add(absl::make_unique<S2Polygon::OwningShape>(std::move(polygon_)));
      index_->references.insert (std::pair<int,std::string>(id, ref_)); //:make_tuple(id, ref_));
    }
  }

 private:
  MutableIndexReference* index_;
  std::string ref_;
  std::unique_ptr<S2Polygon> polygon_;
  s2builderutil::S2PolygonLayer layer_;
};

