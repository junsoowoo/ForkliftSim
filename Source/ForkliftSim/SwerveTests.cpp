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
    return true;
}
#endif
