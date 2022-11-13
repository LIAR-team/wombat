#include "wombat_srrg/srrg_solver/solver_core/factor_graph.h"
#include <wombat_srrg/srrg_boss/deserializer.h>
#include <wombat_srrg/srrg_boss/serializer.h>

// included here since I instantiate all methods
namespace srrg2_solver
{

using namespace std;
using namespace srrg2_core;


FactorGraph::~FactorGraph()
{
  clear();
}

IdVariablePtrContainer& FactorGraph::variables()
{
  return _variables;
}

IdFactorPtrContainer& FactorGraph::factors()
{
  return _factors;
}

void FactorGraph::serialize(ObjectData & odata, IdContext & context)
{
  (void)context;
  using namespace std;
  std::set<VariableBase::Id> selected_variables;
  std::vector<FactorBase*> selected_factors;
  for (auto it = factors().begin(); it != factors().end(); ++it) {
    FactorBase* f = it.value();
    if (f->level() != _level_serialization) {
      continue;
    }
    selected_factors.push_back(f);
    int num_var = f->numVariables();
    for (int v = 0; v < num_var; ++v) {
      selected_variables.insert(f->variableId(v));
    }
  }
  ArrayData* var_data = new ArrayData;
  for (auto it = selected_variables.begin(); it != selected_variables.end(); ++it) {
    VariableBase* v = this->variable(*it);
    var_data->addPointer(v);
  }
  odata.setField("variables", var_data);
  ArrayData* fact_data = new ArrayData;
  for (auto it = selected_factors.begin(); it != selected_factors.end(); ++it) {
    FactorBase* f = *it;
    fact_data->addPointer(f);
  }
  odata.setField("factors", fact_data);
}

bool FactorGraph::addVariable(VariableBasePtr var)
{
  if (var->graphId() == -1) {
    var->setGraphId(_last_graph_id++);
  }
  VariableBase const* v = variable(var->graphId());
  if (v) {
    assert(0 && "variable already in graph");
    return false;
  }
  _variables.insert(std::make_pair(var->graphId(), var));
  _v2f(var->graphId(), true);
  return true;
}

bool FactorGraph::addFactor(FactorBasePtr fac)
{
  if (fac->graphId() == -1) {
    fac->setGraphId(_last_graph_id++);
  }
  FactorBase const * f = factor(fac->graphId());
  if (f) {
    assert("factor already in graph");
    return false;
  }
  bindFactor(fac.get());
  _factors.insert(std::make_pair(fac->graphId(), fac));
  return true;
}

void FactorGraph::addVariable(VariableBase * v)
{
  (void)v;
  throw std::runtime_error (std::string(__PRETTY_FUNCTION__) + "| only accepts managed pointers");
}

void FactorGraph::addFactor(FactorBase * f)
{
  (void)f;
  throw std::runtime_error (std::string(__PRETTY_FUNCTION__) + "| only accepts managed pointers");
}

bool FactorGraph::removeFactor(FactorBasePtr f)
{
  return removeFactor(f.get());
}

bool FactorGraph::removeFactor(FactorBase* f)
{
  //cerr << __PRETTY_FUNCTION__ << "| " << this << endl;
  if (! FactorGraphInterface::removeFactor(f))
    return false;
  auto it = factors().find(f->graphId());
  factors().erase(it);
  return true;
}

bool FactorGraph::removeVariable(VariableBasePtr v)
{
  return removeVariable(v.get());
}

bool FactorGraph::removeVariable(VariableBase* v)
{
  if (! FactorGraphInterface::removeVariable(v))
    return false;
  auto it = variables().find(v->graphId());
  _variables.erase(it);
  return true;
}

void FactorGraph::deserialize(ObjectData & odata, IdContext & context)
{
  (void)context;
  _variables.clear();
  _factors.clear();
  ArrayData* var_data = dynamic_cast<ArrayData*>(odata.getField("variables"));
  for (auto it = var_data->begin(); it != var_data->end(); ++it) {
    ValueData* data = *it;
    VariableBasePtr v =
      std::dynamic_pointer_cast<VariableBase>(data->getPointer()->getSharedPtr());
    addVariable(v);
    _last_graph_id = std::max(v->graphId(), _last_graph_id);
  }

  ArrayData* fact_data = dynamic_cast<ArrayData*>(odata.getField("factors"));
  for (auto it = fact_data->begin(); it != fact_data->end(); ++it) {
    ValueData* data = *it;
    FactorBasePtr f =
      std::dynamic_pointer_cast<FactorBase>(data->getPointer()->getSharedPtr());
    addFactor(f);
    _last_graph_id = std::max(f->graphId(), _last_graph_id);
  }
  ++_last_graph_id;

  bindFactors();
}

// convenience function to write a factor graph to a file
void FactorGraph::write(const std::string& filename)
{
  bindFactors();
  std::set<FactorBase*> selected_factors;

  for (auto it = factors().begin(); it != factors().end(); ++it) {
    FactorBase* f = it.value();
    if ((_level_serialization!=-1) && f->level() != _level_serialization) {
      continue;
    }
    selected_factors.insert(f);
  }
  
  Serializer ser;
  ser.setFilePath(filename);
  ser.setBinaryPath(filename+".d/<classname>.<nameAttribute>.<id>.<ext>");
  _write(ser, selected_factors);
  //ser.writeObject(*this);
}

// convenience functions a graph from a file
FactorGraphPtr FactorGraph::read(const std::string& filename)
{
  Deserializer des;
  des.setFilePath(filename);
  SerializablePtr o;
  cerr << "loading graph from [" << filename << "]" << std::endl;
  int object_count=0;
  std::list<VariableBasePtr> variables;
  std::list<FactorBasePtr> factors;
  while ((o = des.readObjectShared())) {
    ++object_count;
    if ( ! (object_count%1000) )
      cerr << "\rloading " << object_count << " objects";
    VariableBasePtr v=std::dynamic_pointer_cast<VariableBase>(o);
    if (v) {
      variables.push_back(v);
    }
    FactorBasePtr f=std::dynamic_pointer_cast<FactorBase>(o);
    if (f) {
      factors.push_back(f);
    }

    FactorGraphPtr graph = std::dynamic_pointer_cast<FactorGraph>(o);
    if (graph) {
      return graph;
    }
  }
  cerr << "no graph instance found, assembling objects in a new instance" << endl;
  FactorGraphPtr graph(new FactorGraph);
  for (auto v: variables) {
    graph->addVariable(v);
  }
  for (auto f: factors) {
    graph->addFactor(f);
  }
  graph->bindFactors();
  return graph;
}

VariableBasePtr FactorGraph::detachVariable(VariableBase* v)
{
  VariableBasePtr v_ret;
  auto v_it=_variables.container().find(v->graphId());
  if (v_it==_variables.container().end() ||
      v_it->second.get()!=v) {
    return v_ret;
  }
  v_ret=v_it->second;
  removeVariable(v);
  return v_ret;
}

FactorBasePtr   FactorGraph::detachFactor(FactorBase* f)
{
  FactorBasePtr f_ret;
  auto f_it=_factors.container().find(f->graphId());
  if (f_it==_factors.container().end() ||
      f_it->second.get()!=f) {
    return f_ret;
  }
  f_ret=f_it->second;
  removeFactor(f);
  return f_ret;
}

} // namespace srrg2_solver
