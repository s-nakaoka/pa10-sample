#include "Interpolator.h"
#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include <cnoid/ForceSensor>
#include <cnoid/Camera>
#include <cnoid/EigenUtil>
#include <fmt/format.h>
#include <mutex>
#include <thread>
#include <condition_variable>

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

const double pgain[] = {
    35000.0, 35000.0, 35000.0, 35000.0, 35000.0, 35000.0, 35000.0,
    17000.0, 17000.0 };

const double dgain[] = {
    220.0, 220.0, 220.0, 220.0, 220.0, 220.0, 220.0,
    220.0, 220.0 };

enum Phase {
    MoveToHomePosition,
    InitializeMoveForward,
    MoveForward,
    RemainStill,
    InitializeMoveBack,
    MoveBack,
    MoveToHomePositionToFinish,
    EnterErrorState,
    Finish
};

}

class PA10ScrewController : public SimpleController
{
    SimpleControllerIO* io;
    Body* ioBody;
    BodyPtr ikBody;
    shared_ptr<JointPath> jointPath;
    VectorXd q_home;
    VectorXd q_ref, q_old, q_ref_old;
    Interpolator<VectorXd> jointInterpolator;
    Interpolator<VectorXd> endInterpolator;
    ForceSensorPtr forceSensor;
    CameraPtr camera;
    ScopedConnection cameraConnection;
    shared_ptr<const Image> latestCameraImage;
    std::mutex cameraImageMutex;
    std::thread cameraImageThread;
    std::condition_variable cameraImageCondition;
    bool isCameraImageProcessingRequested;
    bool isControllerStopped;
    int phase;
    double timeStep;
    double phaseTime;
    double waitingTime;
    double vx;

public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->io = io;
        ioBody = io->body();

        ikBody = ioBody->clone();
        Link* baseLink = ikBody->rootLink();
        Link* endLink = ikBody->findUniqueEndLink();
        jointPath = JointPath::getCustomPath(ikBody, baseLink, endLink);
        baseLink->position().setIdentity();

        const int nj = ioBody->numJoints();
        q_old.resize(nj);
        for(int i=0; i < nj; ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
            double q = joint->q();
            ikBody->joint(i)->q() = q;
            q_old[i] = q;
        }
        
        jointPath->calcForwardKinematics();
        q_ref = q_old;
        q_ref_old = q_old;

        q_home.resize(nj);
        q_home << 0.0, radian(-58.7), 0.0, radian(112.0), 0.0, radian(36.3), 0.0;

        jointInterpolator.clear();

        forceSensor = ioBody->findDevice<ForceSensor>();
        io->enableInput(forceSensor);

        camera = ioBody->findDevice<Camera>();
        cameraConnection =
            camera->sigStateChanged().connect(
                [&](){ onCameraStateChanged(); });
        latestCameraImage.reset();
        io->enableInput(camera);
        isCameraImageProcessingRequested = false;
        isControllerStopped = false;
        cameraImageThread = std::thread([&](){ processCameraImage(); });
        
        phase = MoveToHomePosition;

        timeStep = io->timeStep();
        phaseTime = 0.0;

        return true;
    }

    virtual bool control() override
    {
        bool isActive = true;

        switch(phase){

        case MoveToHomePosition:
        case MoveToHomePositionToFinish:
            if(jointInterpolator.empty()){
                jointInterpolator.appendSample(0.0, q_ref);
                jointInterpolator.appendSample(2.0, q_home);
                jointInterpolator.update();
            }
            q_ref = jointInterpolator.interpolate(phaseTime);
            if(phaseTime < jointInterpolator.domainUpper()){
                phaseTime += timeStep;
            } else {
                jointInterpolator.clear();
                if(phase == MoveToHomePosition){
                    requestCameraImageProcessing();
                    phase = InitializeMoveForward;
                } else {
                    phase = Finish;
                }
            }
            break;

        case InitializeMoveForward:
            {
                for(int i=0; i < jointPath->numJoints(); ++i){
                    jointPath->joint(i)->q() = q_ref[i];
                }
                jointPath->calcForwardKinematics();
                Position T = jointPath->endLink()->T();
                T.linear() = rotFromRpy(radian(Vector3(0.0, 90.0, 0.0)));
                jointPath->calcInverseKinematics(T);
                vx = 0.0;
                phase = MoveForward;
            }
            
        case MoveForward:
            if(forceSensor->f().z() > -20.0){
                Position T = jointPath->endLink()->T();
                if(vx < 0.4){
                    vx += 1.0 * timeStep;
                }
                T.translation().x() += (vx * timeStep);
                if(!jointPath->calcInverseKinematics(T)){
                    phase = EnterErrorState;
                } else {
                    for(int i=0; i < jointPath->numJoints(); ++i){
                        q_ref[i] = jointPath->joint(i)->q();
                    }
                }
            } else {
                io->os() << "The end effector has touched to the wall." << endl;
                phaseTime = 0.0;
                phase = RemainStill;
            }
            break;

        case RemainStill:
            if(phaseTime <= 0.5){
                phaseTime += timeStep;
            } else {
                requestCameraImageProcessing();
                phase = InitializeMoveBack;
            }
            break;

        case InitializeMoveBack:
            {
                Position T = jointPath->endLink()->T();
                VectorXd p_current(6);
                p_current.head<3>() = T.translation();
                p_current.tail<3>() = rpyFromRot(T.linear());
                VectorXd p_back = p_current;
                p_back(0) -= 0.1;
                endInterpolator.clear();
                endInterpolator.appendSample(0.0, p_current);
                endInterpolator.appendSample(1.0, p_back);
                endInterpolator.update();
                phaseTime = 0.0;
                phase = MoveBack;
            }
            
        case MoveBack:
            {
                VectorXd p = endInterpolator.interpolate(phaseTime);
                if(phaseTime < endInterpolator.domainUpper()){
                    Position T;
                    T.translation() = p.head<3>();
                    T.linear() = rotFromRpy(p.tail<3>());
                    if(!jointPath->calcInverseKinematics(T)){
                        phase = EnterErrorState;
                    } else {
                        for(int i=0; i < jointPath->numJoints(); ++i){
                            q_ref[i] = jointPath->joint(i)->q();
                        }
                        phaseTime += timeStep;
                    }
                } else {
                    phaseTime = 0.0;
                    phase = MoveToHomePositionToFinish;
                }
                break;
            }

        case EnterErrorState:
            io->os() << "Error occured." << endl;
            phase = Finish;
            
        case Finish:
            io->os() << "Control finished." << endl;
        default:
            isActive = false;
            break;
        }
            
        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            double q = joint->q();
            double dq = (q - q_old[i]) / timeStep;
            double dq_ref = (q_ref[i] - q_ref_old[i]) / timeStep;
            joint->u() = (q_ref[i] - q) * pgain[i] + (dq_ref - dq) * dgain[i];
            q_old[i] = q;
        }

        q_ref_old = q_ref;
        
        return isActive;
    }

    void stop()
    {
        if(cameraImageThread.joinable()){
            {
                std::lock_guard<std::mutex> lock(cameraImageMutex);
                isControllerStopped = true;
            }
            cameraImageCondition.notify_all();
            cameraImageThread.join();
        }
    }

    void onCameraStateChanged()
    {
        std::lock_guard<std::mutex> lock(cameraImageMutex);
        if(latestCameraImage != camera->sharedImage()){
            latestCameraImage = camera->sharedImage();
        }
    }

    void requestCameraImageProcessing()
    {
        {
            std::lock_guard<std::mutex> lock(cameraImageMutex);
            isCameraImageProcessingRequested = true;
        }
        cameraImageCondition.notify_all();
    }

    void processCameraImage()
    {
        int cameraImageCounter = 0;
        while(true){
            shared_ptr<const Image> cameraImage;
            {
                std::unique_lock<std::mutex> lock(cameraImageMutex);
                cameraImageCondition.wait(
                    lock, [&]{ return isCameraImageProcessingRequested || isControllerStopped; });
                if(isControllerStopped){
                    break;
                }
                cameraImage = latestCameraImage;
                isCameraImageProcessingRequested = false;
            }
            if(cameraImage){
                string filename = format("handeye{}.png", cameraImageCounter++);
                cameraImage->save(filename);
                io->os() << format("The current camera image has been saved to {}", filename) << endl;
            }
        }
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PA10ScrewController)
