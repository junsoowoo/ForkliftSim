#include "SwerveKinematics.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSwerveKinematicTest,"ForkliftSim.Swerve.RigidBodyVelocityAndSteering",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSwerveKinematicTest::RunTest(const FString& Parameters)
{
    // Reconstruct wheel velocities from optimized signed speed/angle. Verify
    // the rigid-body relation even when steering wraps or reverses wheel spin.
    for(double Heading:{-179.,-90.,0.,90.,179.})
    for(double Omega:{-1.,0.,1.})
    for(FVector2D V:{FVector2D(0,0),FVector2D(2,0),FVector2D(0,-2),FVector2D(1,1)})
    for(FVector2D P:{FVector2D(.8,-.65),FVector2D(.8,.65),FVector2D(-.8,-.65),FVector2D(-.8,.65)})
    {
        const auto T=Swerve::Solve(P,V,Omega,Heading);
        const double A=FMath::DegreesToRadians(T.AngleDeg);
        const FVector2D Reconstructed=FVector2D(FMath::Cos(A),FMath::Sin(A))*T.SpeedMps;
        TestTrue(TEXT("Wheel velocity agrees with rigid-body translation + rotation"),(Reconstructed-V-FVector2D(-Omega*P.Y,Omega*P.X)).Size()<1e-9);
        TestTrue(TEXT("No unnecessary turn greater than 90 degrees"),FMath::Abs(FMath::FindDeltaAngleDegrees(Heading,T.AngleDeg))<=90.000001);
        if(V.IsNearlyZero() && Omega!=0) TestTrue(TEXT("Spin wheel direction is tangent, not radial"),FMath::Abs(FVector2D::DotProduct(Reconstructed,P))<1e-9);
    }
    TestTrue(TEXT("Slew crosses wrap by the shortest path"),FMath::Abs(Swerve::Slew(179,-170,3)+178)<1e-9);
    TestTrue(TEXT("Steering target step respects rate"),FMath::Abs(Swerve::Slew(0,90,.5)-.5)<1e-9);
    Swerve::FTarget Arc[4];
    for(int I=0;I<4;++I) Arc[I]=Swerve::Solve(FVector2D(I<2?.8:-.8,I%2==0?-.65:.65),FVector2D(1,0),PI/4,0);
    TestEqual(TEXT("Outer-wheel headroom preserves chassis cruise on an arc"),Swerve::DesaturationScale(Arc,4,4),1.);
    const double Scale=Swerve::DesaturationScale(Arc,4,1);
    TestTrue(TEXT("Chassis speed used as wheel cap wrongly shrinks the whole motion"),Scale<.7);
    for(const auto& T:Arc) TestTrue(TEXT("Wheel cap obeyed when reached"),FMath::Abs(T.SpeedMps*Scale)<=1+1e-9);
    FVector2D Positions[4]={FVector2D(.8,-.65),FVector2D(.8,.65),FVector2D(-.8,-.65),FVector2D(-.8,.65)};
    double Angles[4]; double Yaw=.8;
    for(int I=0;I<4;++I) Angles[I]=Swerve::Solve(Positions[I],FVector2D(1,0),Yaw,0).AngleDeg;
    for(int Tick=0;Tick<300;++Tick)
    {
        Yaw=Swerve::LimitYawForSteering(Positions,Angles,4,FVector2D(1,0),Yaw,-.8,.5);
        for(int I=0;I<4;++I)
        {
            const auto T=Swerve::Solve(Positions[I],FVector2D(1,0),Yaw,Angles[I]);
            TestTrue(TEXT("S-curve common yaw obeys every module's 30 deg/s steering limit at 60 Hz"),FMath::Abs(FMath::FindDeltaAngleDegrees(Angles[I],T.AngleDeg))<=.500001);
            Angles[I]=T.AngleDeg;
        }
    }
    TestTrue(TEXT("Coordinated command reaches opposite curve"),FMath::Abs(Yaw+.8)<1e-6);
    for(double Fraction:{.9,.5,.01}) for(const FVector2D& P:Positions)
    {
        const auto Before=Swerve::Solve(P,FVector2D(1,0),.8,0);
        const auto After=Swerve::Solve(P,FVector2D(Fraction,0),Swerve::DecelerationYaw(.8,1,Fraction),Before.AngleDeg);
        TestTrue(TEXT("Full release preserves wheel direction while slowing a turn"),FMath::Abs(After.AngleDeg-Before.AngleDeg)<1e-8);
        TestTrue(TEXT("Full release scales wheel speed, not curvature"),FMath::Abs(After.SpeedMps-Before.SpeedMps*Fraction)<1e-8);
    }
    TestEqual(TEXT("Stopped translation does not start a new yaw command"),Swerve::DecelerationYaw(.8,0,0),0.);
    double Actual[4]={0,0,0,0},Previous[4]={0,0,0,0}; double TrackedYaw=0;
    for(int Tick=0;Tick<300;++Tick)
    {
        TrackedYaw=Swerve::LimitYawForSteering(Positions,Previous,4,FVector2D(1,0),TrackedYaw,.8,.5,Actual,5);
        for(int I=0;I<4;++I)
        {
            const auto T=Swerve::Solve(Positions[I],FVector2D(1,0),TrackedYaw,Previous[I]);
            TestTrue(TEXT("Common yaw does not run ahead of a loaded steering module"),FMath::Abs(FMath::FindDeltaAngleDegrees(Actual[I],T.AngleDeg))<=5.000001);
            Previous[I]=T.AngleDeg;
            Actual[I]=Swerve::Slew(Actual[I],T.AngleDeg,.25); // slower actual steering
        }
    }
    TestTrue(TEXT("Loaded steering still reaches the requested curve"),FMath::Abs(TrackedYaw-.8)<1e-6);
    return true;
}
#endif
