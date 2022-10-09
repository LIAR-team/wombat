#include "wombat_srrg/srrg_boss/id_context.h"
#include "wombat_srrg/srrg_boss/id_placeholder.h"

using namespace srrg2_core;
using namespace std;

IdentifiablePtr AbstractPlaceHolderAssigner::getSharedPtr(Identifiable* ident) {
  // std::cerr << "query ptr " << _context << " id: " << ident << " " << /*getSharedPtr(ident) <<*/ std::endl;
  // std::cerr << "shared ptr: " << _context->getSharedPtr(ident) << std::endl;
  return _context->getSharedPtr(ident);
}

void IdPlaceholder::resolve(Identifiable* instance) {
  for (vector<AbstractPlaceHolderAssigner*>::iterator it=_assigners.begin();it!=_assigners.end();it++) {
    (*it)->assign(instance);
  }
}

IdPlaceholder::~IdPlaceholder() {
  for (vector<AbstractPlaceHolderAssigner*>::iterator it=_assigners.begin();it!=_assigners.end();it++) {
    delete *it;
  }
}


