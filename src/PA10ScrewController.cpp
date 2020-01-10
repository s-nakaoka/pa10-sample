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

}

class PA10ScrewController : public SimpleController
{
    Body* ioBody;
    BodyPtr ikBody;
    Link* ikWrist;
    shared_ptr<JointPath> jointPath;
    VectorXd q_home;
    ForceSensorPtr forceSensor;
    VectorXd qref, qold, qref_old;
    Interpolator<VectorXd> jointInterpolator;
    Interpolator<VectorXd> endInterpolator;
    int phase;
    double time;
    double timeStep;
    double dq_hand;

public:

    Vector3 toRadianVector3(double x, double y, double z)
    {
        return Vector3(radian(x), radian(y), radian(z));
    }
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();

        forceSensor = ioBody->findDevice<ForceSensor>();
        
        ikBody = ioBody->clone();
        ikWrist = ikBody->link("J7");
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
        q_home << 0.0, -58.7, 0.0, 112.0, 0.0, 36.3, 0.0;

        jointInterpolator.clear();
        jointInterpolator.appendSample(0.0, qold);
        jointInterpolator.appendSample(1.0, q_home);
        jointInterpolator.update();
        
        phase = 0;
        time = 0.0;
        timeStep = io->timeStep();

        return true;
    }

    virtual bool control() override
    {
        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            double q = joint->q();
            double dq = (q - qold[i]) / timeStep;
            double dq_ref = (qref[i] - qref_old[i]) / timeStep;
            joint->u() = (qref[i] - q) * pgain[i] + (dq_ref - dq) * dgain[i];
            qold[i] = q;
        }
        
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PA10ScrewController)
