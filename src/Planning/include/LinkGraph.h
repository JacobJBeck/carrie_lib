// Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_LINKGRAPH_H_
#define PLANNING_LINKGRAPH_H_

// Boost includes
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>

// STL includes
#include <vector>
#include <utility>
#include <string>
#include <map>

// library includes
#include "Math/include/easymath.h"
#include "FileIO/include/FileOut.h"
#include "Planning.h"

typedef boost::adjacency_list
<boost::listS,      // edge container
    boost::vecS,        // vertex container
    boost::directedS,   // edge (u,v) can have a different weight than (v,u)
    boost::no_property,
    boost::property<boost::edge_weight_t, double> > mygraph_t;

typedef boost::hash<mygraph_t::vertex_descriptor> node_hash;

typedef IBoostGraph<mygraph_t, size_t, node_hash, std::equal_to<size_t> > LinkBase; // requires equal_to passed in for linux compatibility

class LinkGraph : public LinkBase {
 public:

     // More convenient types
     typedef std::pair<size_t, size_t> edge;

     
    //! For compliance with base type
    typedef typename LinkBase::vertex_descriptor vertex_descriptor;
    typedef typename LinkBase::dist_map dist_map;
    typedef typename LinkBase::pred_map pred_map;
    vertex_descriptor get_descriptor(size_t v) { return v; }
    size_t get_vertex_base(vertex_descriptor v) { return v; }
    double get_x(vertex_descriptor v) {
            return locations[v].x;
    }

    double get_path_cost(std::list<vertex_descriptor> p) {
        double c = 0;
        for (auto v = p.begin(); std::next(v) != p.end(); v++) {
            auto e = boost::edge(*v, *std::next(v),g).first;
            c += get(boost::edge_weight, g, e);
        }
        return c;
    }
    double get_y(vertex_descriptor v) { return locations[v].y; }

    typedef boost::graph_traits<mygraph_t>::edge_iterator edge_iter;

    matrix1d saved_weights;  // for blocking and unblocking sectors
    std::vector<easymath::XY> locations;
    std::map<easymath::XY, size_t> loc2mem;  // maps location to membership

    void blockVertex(int vertexID);
    void unblockVertex();
    bool fully_connected();  // Tests whether the graph is fully connected
    bool intersects_existing_edge(edge candidate);


    mygraph_t g;
    LinkGraph(const LinkGraph& other) :
        LinkGraph(other.get_locations(), other.get_edges()) {}


    LinkGraph(std::vector<easymath::XY> locations_set,
        const std::vector<edge> &edge_array);
    LinkGraph(size_t n_vertices, size_t xdim, size_t ydim);
    virtual ~LinkGraph(void) {}
    LinkGraph();

    //! Accessor functions
    size_t get_n_vertices() const { return locations.size(); }
    size_t get_n_edges() const { return num_edges(g); }
    easymath::XY get_vertex_loc(size_t vID) const { return locations.at(vID); }
    size_t get_membership(easymath::XY pt) const { return loc2mem.at(pt); }
    matrix1d get_weights() const;
    std::vector<edge> get_edges() const;
    std::vector<easymath::XY> get_locations() const { return locations; }
    void set_weights(matrix1d weights);

    size_t get_direction(size_t m1, size_t m2) const;

    //! Printout
    void print_graph(std::string file_path);
};
#endif  // PLANNING_LINKGRAPH_H_
