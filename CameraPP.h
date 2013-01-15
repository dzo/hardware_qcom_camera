/*

*/
#include <camera/CameraParameters.h>
namespace android {

class CameraPP {
    CameraPP();
    ~CameraPP();
    void setParameters(const CameraParameters& params);
    void handleHDRPicture(unsigned char *,void *);
    void handleAutoEnhance(unsigned char *,void *);
};

} // namespace android
