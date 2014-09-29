#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <unistd.h>

#include "polygonization/polygonization.h"
#include "polygonization/voronoi_diagram.h"

#include "graphconstruction/img_stream.h"


namespace graphconstruction
{
    
class graphconstruction_t
{
public:
    graphconstruction_t (){};
    virtual ~graphconstruction_t ();

private:
    /* data */
};

} /* graphconstruction */