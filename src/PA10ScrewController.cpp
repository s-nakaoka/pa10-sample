#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include <cnoid/ForceSensor>
#include <cnoid/EigenUtil>
#include "Interpolator.h"

using namespace std;
using namespace cnoid;

namespace {

const double pgain[] = {
    35000.0, 35000.0, 35000.0, 35000.0, 35000.0, 35000.0, 35000.0,
    17000.0, 17000.0 };

const double dgain[] = {
    220.0, 220.0, 220.0, 220.0, 220.0, 220.0, 220.0,
    220.0, 220.0 };

enum Phase {
    REMAIN_STILL,
    MOVE_TO_HOME_POSITION,
    MOVE_FORWARD,
    MOVE_BACK,
    MOVE_TO_HOME_POSITION_AND_FINISH
};

}

class PA10ScrewController : public SimpleController
{
    Body* ioBody;
    BodyPtr ikBody;
    shared_ptr<JointPath> jointPath;
    VectorXd q_home;
    VectorXd qref, qold, qref_old;
    Interpolator<VectorXd> jointInterpolator;
    Interpolator<VectorXd> endInterpolator;
    ForceSensorPtr forceSensor;
    int phase;
    double time;
    double timeStep;
    double interpolationTime;

public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();

        forceSensor = ioBody->findDevice<ForceSensor>();
        
        ikBody = ioBody->clone();
        Link* baseLink = ikBody->rootLink();
        Link* endLink = ikBody->findUniqueEndLink();
        jointPath = JointPath::getCustomPath(ikBody, baseLink, endLink);
        baseLink->position().setIdentity();

        const int nj = ioBody->numJoints();
        qold.resize(nj);
        for(int i=0; i < nj; ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
            double q = joint->q();
            ikBody->joint(i)->q() = q;
            qold[i] = q;
        }
        
        jointPath->calcForwardKinematics();
        qref = qold;
        qref_old = qold;

        q_home.resize(nj);
        q_home << 0.0, radian(-58.7), 0.0, radian(112.0), 0.0, radian(36.3), 0.0;

        jointInterpolator.clear();
        jointInterpolator.appendSample(0.0, qold);
        jointInterpolator.appendSample(2.0, q_home);
        jointInterpolator.update();

        phase = MOVE_TO_HOME_POSITION;

        time = 0.0;
        timeStep = io->timeStep();
        interpolationTime = 0.0;

        return true;
    }

    virtual bool control() override
    {
        bool isActive = false;

        switch(phase){

        case MOVE_TO_HOME_POSITION:
        case MOVE_TO_HOME_POSITION_AND_FINISH:
            qref = jointInterpolator.interpolate(interpolationTime);
            interpolationTime += timeStep;
            isActive = true;
            if(interpolationTime >= jointInterpolator.domainUpper()){
                if(phase == MOVE_TO_HOME_POSITION){
                    phase = MOVE_FORWARD;
                } else {
                    phase = REMAIN_STILL;
                }
            }
            break;

        case MOVE_FORWARD:
            break;

        case MOVE_BACK:
            break;

        default:
            break;
        }
            
        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            double q = joint->q();
            double dq = (q - qold[i]) / timeStep;
            double dq_ref = (qref[i] - qref_old[i]) / timeStep;
            joint->u() = (qref[i] - q) * pgain[i] + (dq_ref - dq) * dgain[i];
            qold[i] = q;
        }

        qref_old = qref;
        time += timeStep;
        
        return isActive;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PA10ScrewController)
