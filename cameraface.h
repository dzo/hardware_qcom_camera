namespace android {

typedef int face_config_type;

class CameraFace {

public:
    int getParameters(CameraParameters &);
    int setParameters(CameraParameters  const&);
    void startFaceDetection(int,int);
    void stopFaceDetection(int,int);
    void initialize(face_config_type *);
    ~CameraFace();
    CameraFace(void);
    void processPreview(char *,CameraParameters &,int,int);
    void ZoomChanged(android_native_rect_t);
    void processImage(char *,int,int);

    int rect[4];
};

}
