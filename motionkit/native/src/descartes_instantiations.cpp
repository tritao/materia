#include <descartes_light/solvers/ladder_graph/impl/ladder_graph.hpp>
#include <descartes_light/solvers/ladder_graph/impl/ladder_graph_dag_search.hpp>
#include <descartes_light/solvers/ladder_graph/impl/ladder_graph_solver.hpp>

namespace descartes_light {
template class LadderGraph<double>;
template class DAGSearch<double>;
template class LadderGraphSolver<double>;
}  // namespace descartes_light
