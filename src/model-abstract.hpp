#ifndef __VIAOPT_MODELABSTRACT__
#define __VIAOPT_MODELABSTRACT__

#include <Eigen/Core>
#include "viaopt/api.hpp"

namespace viaopt
{

  class VIAOPT_EXPORT ModelAbstract
  {
  private:
 
  protected:
 
  public:
    ModelAbstract (void) {}
    void display (std::ostream & os) const;

  };

  inline std::ostream& 
  operator<< (std::ostream& os, const ModelAbstract& m) { m.display(os); return os;}

} // namespace viaopt


#endif // #ifndef __VIAOPT_MODELABSTRACT__
