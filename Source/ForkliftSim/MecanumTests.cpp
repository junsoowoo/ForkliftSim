#include "Misc/AutomationTest.h"
#include "MecanumModel.h"
#include "RigidSpin.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMecanumKinematics,"ForkliftSim.Mecanum.Kinematics",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FMecanumKinematics::RunTest(const FString&)
{
    for(const FVector T:{FVector(1,0,0),FVector(0,1,0),FVector(.7,.7,0),FVector(0,0,.4),FVector(.8,-.3,-.2)})
    {
        double W[4];Mecanum::Solve(FVector2D(T.X,T.Y),T.Z,1.6,1.3,W);const FVector Back=Mecanum::Forward(W,1.6,1.3);
        TestTrue(TEXT("Forward/inverse round trip in Unreal axes"),(Back-T).Size()<1e-12);
        for(int I=0;I<4;++I)
        {
            const double X=(I<2?1:-1)*.8,Y=(I%2==0?-1:1)*.65;
            const FVector A=FVector(1,Mecanum::Hand(I),0).GetSafeNormal();
            const FVector Contact(T.X-T.Z*Y-W[I],T.Y+T.Z*X,0);
            TestTrue(TEXT("Zero contact velocity along passive roller axis"),FMath::Abs(FVector::DotProduct(A,Contact))<1e-12);
        }
        const double Scale=Mecanum::Desaturate(W,.4);const FVector Limited=Mecanum::Forward(W,1.6,1.3);
        TestTrue(TEXT("Common saturation preserves commanded planar twist"),(Limited-T*Scale).Size()<1e-12);
        for(double V:W) TestTrue(TEXT("Each shaft stays within surface-speed limit"),FMath::Abs(V)<=.4+1e-12);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMecanumRigidSpin,"ForkliftSim.Mecanum.SymmetricRollerSpin",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FMecanumRigidSpin::RunTest(const FString&)
{
    const FVector I(2,3,3),W0(5,7,0),L0=I*W0;
    FQuat Q=FQuat::Identity;FVector W=W0;
    const double Energy0=.5*FVector::DotProduct(I*W0,W0),Dt=.0020833333333333333;
    for(int K=0;K<4800;++K) TestTrue(TEXT("Axisymmetric flow supported"),RigidSpin::Symmetric(Q,W,I,Dt));
    const FVector L=Q.RotateVector(I*Q.UnrotateVector(W));
    TestTrue(TEXT("Ten seconds conserve world angular momentum"),(L-L0).Size()<1e-8);
    TestTrue(TEXT("Ten seconds conserve kinetic energy"),FMath::Abs(.5*FVector::DotProduct(I*Q.UnrotateVector(W),Q.UnrotateVector(W))-Energy0)<1e-7);
    // Independent Rodrigues prediction for the symmetry axis: it precesses
    // around the constant world momentum at |L| / I_perpendicular.
    const FVector N=L0.GetSafeNormal(),A=FVector::ForwardVector;const double Angle=L0.Size()/3*10;
    const FVector Expected=A*FMath::Cos(Angle)+FVector::CrossProduct(N,A)*FMath::Sin(Angle)+N*FVector::DotProduct(N,A)*(1-FMath::Cos(Angle));
    TestTrue(TEXT("Physical symmetry axis follows analytic precession"),(Q.RotateVector(A)-Expected).Size()<1e-8);
    W=-W;
    for(int K=0;K<4800;++K) RigidSpin::Symmetric(Q,W,I,Dt);
    TestTrue(TEXT("Reversed angular velocity returns the initial attitude"),Q.AngularDistance(FQuat::Identity)<1e-7);
    TestTrue(TEXT("Reversed angular velocity returns the initial speed"),(W+W0).Size()<1e-7);
    TestFalse(TEXT("Triaxial body must not silently use symmetric approximation"),RigidSpin::Symmetric(Q,W,FVector(2,3,4),Dt));
    const FQuat Pose(FVector(1,2,3).GetSafeNormal(),.7);const FVector HubW(4,-3,2),Axis=Pose.GetAxisX();
    const FVector Spin=HubW+Axis*17,Tensor(.002,.014,.014);
    const FVector Torque=RigidSpin::BearingTransport(Pose,Spin,Tensor,HubW);
    TestTrue(TEXT("Passive bearing couple has no torque on its free axis"),FMath::Abs(FVector::DotProduct(Torque,Axis))<1e-12);
    TestTrue(TEXT("Internal bearing couple does no work at compatible relative spin"),FMath::Abs(FVector::DotProduct(Torque,Spin-HubW))<1e-12);
    const double Eps=1e-6;const FQuat Next=FQuat(HubW.GetSafeNormal(),HubW.Size()*Eps)*Pose;
    auto Momentum=[&](FQuat P) {const FVector Velocity=HubW+P.GetAxisX()*17;return P.RotateVector(Tensor*P.UnrotateVector(Velocity));};
    TestTrue(TEXT("Bearing couple equals independent momentum derivative"),((Momentum(Next)-Momentum(Pose))/Eps-Torque).Size()<1e-5);
    return true;
}

#include "ReducedMecanum.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FReducedMecanumContact,"ForkliftSim.Mecanum.ReducedContactPassivity",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FReducedMecanumContact::RunTest(const FString&)
{
    const FVector Axis=FVector(1,1,0).GetSafeNormal();
    for(double Mu:{0.,.05,.8})for(double Speed:{-10.,-.1,0.,.1,10.})
    {
        const double Dt=1./480,InverseMass=.08;const FVector V=Axis*Speed;
        const FVector F=ReducedMecanum::Force(V,Axis,7000,Mu,InverseMass,Dt);
        TestTrue(TEXT("Coulomb limit"),F.Size()<=Mu*7000+1e-8);
        TestTrue(TEXT("Friction never supplies initial slip power"),FVector::DotProduct(F,V)<=1e-8);
        const double Before=.5*Speed*Speed/InverseMass,After=.5*FMath::Square(Speed+FVector::DotProduct(F,Axis)*Dt*InverseMass)/InverseMass;
        TestTrue(TEXT("Uncoupled contact impulse does not gain kinetic energy"),After<=Before+1e-8);
    }
    TestTrue(TEXT("Free roller direction is unforced"),ReducedMecanum::Force(FVector(1,-1,0),Axis,7000,.8,.08,1./480).IsNearlyZero());
    TestTrue(TEXT("No normal load gives no traction"),ReducedMecanum::Force(Axis,Axis,0,.8,.08,1./480).IsNearlyZero());
    TestTrue(TEXT("Bottom roller contact is eligible"),ReducedMecanum::IsRollingContact(FVector::UpVector,FVector::RightVector));
    TestTrue(TEXT("A cap against a step wall is not another roller patch"),!ReducedMecanum::IsRollingContact(FVector::RightVector,FVector::RightVector));
    TestTrue(TEXT("Ceiling contact does not become ground support"),!ReducedMecanum::IsRollingContact(-FVector::UpVector,FVector::RightVector));
    for(const FVector N:{FVector::UpVector,FVector(-.2,0,1).GetSafeNormal(),FVector(0,.2,1).GetSafeNormal()})for(double Hand:{-1.,1.})
    {
        const FVector D=ReducedMecanum::ContactDirection(FVector::RightVector,N,Hand);
        TestTrue(TEXT("Roller traction is a unit vector tangent to the actual contact surface"),FMath::Abs(D.Size()-1)<1e-10 && FMath::Abs(FVector::DotProduct(D,N))<1e-10);
        if(N==FVector::UpVector)TestTrue(TEXT("Flat-floor mecanum handedness is unchanged"),(D-FVector(1,Hand,0).GetSafeNormal()).Size()<1e-10);
    }
    // Independent mechanical energy inequality, including off-diagonal point
    // inertia. It catches storing a clipped damping force as elastic preload.
    FRandomStream Random(10431);
    for(int I=0;I<2000;++I)
    {
        const double A=Random.FRandRange(.001,5),B=Random.FRandRange(.001,5),Cross=Random.FRandRange(-.9,.9)*FMath::Sqrt(A*B);
        const double VX=Random.FRandRange(-10,10),VY=Random.FRandRange(-10,10),X=Random.FRandRange(-.1,.1),H=I%2?1./960:1./1920;
        const double K=200000,C=2000,L=FMath::Pow(10.,Random.FRandRange(-2,5))*H;
        const double JY=-FMath::Sign(VY)*FMath::Min(L,FMath::Min(.8*FMath::Abs(VY)/B,(1.5+5*FMath::Abs(VY))*H));
        const double Left=FMath::Sqrt(FMath::Max(0.,L*L-JY*JY));
        const auto Step=ReducedMecanum::BrakeStep(VX+Cross*JY,X,A,H,K,C,Left/H);
        const double JX=Step.Impulse;
        const double Gain=JX*VX+JY*VY+.5*(A*JX*JX+2*Cross*JX*JY+B*JY*JY)+.5*K*(Step.Shear*Step.Shear-X*X);
        TestTrue(TEXT("Coupled contact kick plus elastic energy cannot increase without input"),Gain<1e-6);
        TestTrue(TEXT("Combined contact impulse respects the full Coulomb cone"),JX*JX+JY*JY<=L*L+1e-7);
    }
    const double Held=.006;
    TestTrue(TEXT("A shaft constrained at rest retains its loaded contact spring"),FMath::Abs(ReducedMecanum::SolvedShear(Held,0,1./960,200000,7000,0)-Held)<1e-12);
    for(int I=0;I<2000;++I)
    {
        const double X=Random.FRandRange(-.03,.03),V=Random.FRandRange(-10,10),Work=Random.FRandRange(-100,100),K=200000;
        const double Updated=ReducedMecanum::SolvedShear(X,V,1./960,K,7000,Work);
        TestTrue(TEXT("Solved spring never exceeds the available storage budget"),.5*K*Updated*Updated<=FMath::Max(0.,.5*K*X*X-Work)+1e-8);
        TestTrue(TEXT("Solved spring respects the Coulomb storage cap"),FMath::Abs(K*Updated)<=7000+1e-8);
    }
    return true;
}
