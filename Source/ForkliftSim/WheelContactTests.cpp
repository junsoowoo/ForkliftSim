#include "WheelContactGeometry.h"
#include "WheelContactData.h"
#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWheelContactGeometryTest,"ForkliftSim.Validation.SmoothCylinderContact",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FWheelContactGeometryTest::RunTest(const FString& Parameters)
{
    for(double Heading:{0.,45.,90.,173.}) for(double Roll:{0.,3.,15.})
    {
        const FQuat Q=FRotator(0,Heading,Roll).Quaternion();
        const FVector Axis=Q.RotateVector(FVector(0,1,0)), N(0,0,1), C(20,10,30);
        for(double Shoulder:{-9.,9.})
        {
            FVector P; TestTrue(TEXT("Lateral cylinder contact exists"),WheelContactGeometry::Project(C,Axis,N,C+Axis*Shoulder,30,9,P));
            const FVector D=P-C; const double Axial=FVector::DotProduct(D,Axis);
            TestTrue(TEXT("Exact rolling radius"),FMath::Abs((D-Axis*Axial).Size()-30)<1e-9);
            TestTrue(TEXT("Shoulder/roll lever arm preserved"),FMath::Abs(Axial-Shoulder)<1e-9);
            TestTrue(TEXT("Radial contact lies toward road"),FVector::DotProduct(D-Axis*Axial,N)<0);
            // Rotation about the cylinder axis must not create radial faceting.
            FVector Again; WheelContactGeometry::Project(C,Axis,N,P,30,9,Again);
            TestTrue(TEXT("Projection is idempotent"),Again.Equals(P,1e-8));
        }
    }
    FVector P;
    TestFalse(TEXT("Sidewall uses native collision"),WheelContactGeometry::Project(FVector::ZeroVector,FVector(0,0,1),FVector(0,0,1),FVector::ZeroVector,30,9,P));
    TestFalse(TEXT("Invalid radius rejected"),WheelContactGeometry::Project(FVector::ZeroVector,FVector(0,1,0),FVector(0,0,1),FVector::ZeroVector,0,9,P));
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWheelEdgeTest,"ForkliftSim.Validation.CircularTyreBoxEdge",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FWheelEdgeTest::RunTest(const FString&)
{
    const FVector Min(400,-400,-17.5),Max(4000,400,2.5),A(0,1,0);
    for(double R:{15.,30.}) for(double Angle:{.01,.2,.6,1.2,PI/2-.001}) for(double Gap:{-.02,0.,.03})
    {
        const FVector N(-FMath::Cos(Angle),0,FMath::Sin(Angle));
        const FVector C=FVector(400,0,2.5)+N*(R+Gap);
        for(double Along:{-5.,5.})
        {
            FVector W,P,Computed;
            TestTrue(TEXT("Axle parallel convex edge accepted"),WheelContactGeometry::ProjectEdge(C,A,FVector(400,Along,2.5),Min,Max,R,5,W,P,Computed));
            TestTrue(TEXT("Analytic normal continuous over edge"),Computed.Equals(N,1e-9));
            TestTrue(TEXT("Circular radius preserved"),FMath::Abs((W-C-A*Along).Size()-R)<1e-9);
            TestTrue(TEXT("Signed separation exact"),FMath::Abs(FVector::DotProduct(W-P,Computed)-Gap)<1e-9);
            TestTrue(TEXT("Road point remains on edge"),FMath::Abs(P.X-400)<1e-9 && FMath::Abs(P.Z-2.5)<1e-9);
        }
    }
    FVector W,P,N;
    TestFalse(TEXT("Skew edge remains native"),WheelContactGeometry::ProjectEdge(FVector(390,0,15),FVector(.1,.995,0).GetSafeNormal(),FVector(400,0,2.5),Min,Max,15,5,W,P,N));
    TestFalse(TEXT("Top face is not treated as an edge"),WheelContactGeometry::ProjectEdge(FVector(410,0,15),A,FVector(410,0,2.5),Min,Max,15,5,W,P,N));
    TestFalse(TEXT("Wrong native feature not relocated"),WheelContactGeometry::ProjectEdge(FVector(390,0,15),A,FVector(400,0,-17.5),Min,Max,15,5,W,P,N));
    TestFalse(TEXT("End cap remains native"),WheelContactGeometry::ProjectEdge(FVector(390,399,15),A,FVector(400,399,2.5),Min,Max,15,5,W,P,N));
    TestTrue(TEXT("Near parallel tolerance accepted"),WheelContactGeometry::ProjectEdge(FVector(390,0,15),FVector(0,1,.001).GetSafeNormal(),FVector(400,5,2.5),Min,Max,15,5,W,P,N));
    TestTrue(TEXT("Near parallel road point stays exactly on box edge"),FMath::Abs(P.X-400)<1e-9 && FMath::Abs(P.Z-2.5)<1e-9);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWheelContactBalanceTest,"ForkliftSim.Validation.ContactDistributionBalance",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FWheelContactBalanceTest::RunTest(const FString&)
{
    const FVector Center(1,2,.15),A(0,1,0),N(0,0,1);
    FWheelContactData Left,Right;
    Left.AddPoint(FVector(1,1.95,0),Center,A,N,3,.05);
    Right.AddPoint(FVector(1,2.05,0),Center,A,N,1,.05);
    Left.Add(Right);
    TestTrue(TEXT("Band impulses close exactly"),FMath::Abs(Left.BandNs[0]+Left.BandNs[1]+Left.BandNs[2]-Left.NormalNs)<1e-12);
    TestTrue(TEXT("Weighted load center obeys moment balance"),(Left.PointWeightedMNs/Left.NormalNs).Equals(FVector(1,1.975,0),1e-12));
    TestTrue(TEXT("Asymmetric shoulder load produces roll moment"),Left.NormalMomentNms.Equals(FVector(-.1,0,0),1e-12));
    TestTrue(TEXT("Resultant normal unchanged by diagnostics"),Left.NormalImpulseNs.Equals(N*4,1e-12));
    Left.AddPoint(Center,Center,A,N,0,.05);
    TestEqual(TEXT("Zero impulse does not invent support"),Left.NormalNs,4.);
    return true;
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFiniteCylinderSupportTest,"ForkliftSim.Contact.FiniteCylinderCapsAndSkewEdges",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FFiniteCylinderSupportTest::RunTest(const FString&)
{
    const FVector Axis(0,1,0),C=FVector::ZeroVector;
    const FVector Cap=WheelContactGeometry::CylinderSupport(C,Axis,Axis,FVector(3,0,4),10,2);
    TestTrue(TEXT("A cap contact keeps an interior disk point, not a fictitious tread normal"),(Cap-FVector(3,-2,4)).Size()<1e-9);
    const FVector Shoulder=WheelContactGeometry::CylinderSupport(C,Axis,FVector(0,1,1).GetSafeNormal(),C,10,2);
    TestTrue(TEXT("Tilted plane touches the finite cylinder rim"),(Shoulder-FVector(0,-2,-10)).Size()<1e-8);
    const FVector Min(-100,-100,-20),Max(100,0,0),Old(5,0,0);FVector W,P,N;
    bool OK=WheelContactGeometry::ProjectCylinderEdge(FVector(5,4,12),Axis,Old,Min,Max,10,2,W,P,N);
    TestTrue(TEXT("Perpendicular axle edge resolves a finite cap/rim corner"),OK);
    TestTrue(TEXT("Independent rectangular projection gives sqrt(2) corner components"),(W-FVector(5,2,2)).Size()<1e-4 && (N-FVector(0,1,1).GetSafeNormal()).Size()<1e-4);
    OK=WheelContactGeometry::ProjectCylinderEdge(FVector(5,-4,12),Axis,Old,Min,Max,10,2,W,P,N);
    TestTrue(TEXT("A native edge candidate resolves to the actual top face"),OK && (N-FVector::UpVector).Size()<1e-7 && FMath::Abs(P.Z)<1e-7 && P.Y<0);
    const FVector Skew=FVector(.02,1,.04).GetSafeNormal(),Center(5,4,12);
    OK=WheelContactGeometry::ProjectCylinderEdge(Center,Skew,Old,Min,Max,10,2,W,P,N);
    TestTrue(TEXT("Skewed axle edge has a finite exact support solution"),OK);
    if(OK)
    {
        const FVector D=W-Center;const double Axial=FVector::DotProduct(D,Skew),Radial=(D-Skew*Axial).Size();
        TestTrue(TEXT("Witness stays on the finite cylinder"),FMath::Abs(Axial)<=2.00001 && Radial<=10.00001 && (FMath::Abs(FMath::Abs(Axial)-2)<1e-5 || FMath::Abs(Radial-10)<1e-5));
        TestTrue(TEXT("Witness stays on the box edge and its outward cone"),FMath::Abs(P.Y)+FMath::Abs(P.Z)+FMath::Abs(N.X)<1e-7 && N.Y>=0 && N.Z>=0);
        TestTrue(TEXT("Contact separation has no invented tangential displacement"),((W-P)-N*FVector::DotProduct(W-P,N)).Size()<.025);
    }
    return true;
}
#endif
