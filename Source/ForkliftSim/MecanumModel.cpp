#include "MecanumModel.h"
#include "WheelActor.h"
#include "Vehicle3DHelpers.h"
#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "PhysicsEngine/BodySetup.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "MeshDescription.h"
#include "StaticMeshAttributes.h"
#include "Engine/StaticMesh.h"

bool FMecanumSettings::Load(FString& Error)
{
    FString Text; TSharedPtr<FJsonObject> J;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("Mecanum.json"))) ||
       !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J) || !J) {Error=TEXT("Cannot read Mecanum.json");return false;}
    FString Geometry;double Pos=0,Vel=0;
    FString Integrator;double MinHz=0;
    if(!J->TryGetStringField(TEXT("bearing_integrator"),Integrator) || Integrator!=TEXT("symmetric_spin_verlet") || !J->TryGetBoolField(TEXT("adaptive_step"),bAdaptiveStep))
    {Error=TEXT("Mecanum profile requires symmetric_spin_verlet and adaptive_step");return false;}
    bBearingIntegrator=true;
    if(!J->TryGetNumberField(TEXT("minimum_physics_hz"),MinHz) || (MinHz!=480 && MinHz!=960 && MinHz!=1920)) {Error=TEXT("Invalid minimum mecanum physics frequency");return false;}
    MinimumPhysicsHz=int(MinHz);
    if(!J->TryGetStringField(TEXT("contact_geometry"),Geometry) || (Geometry!=TEXT("inscribed_9_spheres") && Geometry!=TEXT("barrel_convex")) ||
       !J->TryGetNumberField(TEXT("position_iterations"),Pos) || !J->TryGetNumberField(TEXT("velocity_iterations"),Vel) ||
       !J->TryGetNumberField(TEXT("max_control_step_s"),MaxControlStepS) || !FMath::IsFinite(Pos) || !FMath::IsFinite(Vel) ||
       Pos!=FMath::FloorToDouble(Pos) || Vel!=FMath::FloorToDouble(Vel) || Pos<8 || Pos>127 || Vel<8 || Vel>127 || !FMath::IsFinite(MaxControlStepS) || MaxControlStepS<1./120 || MaxControlStepS>1./30+1e-9)
    {Error=TEXT("Invalid mecanum contact/numerical profile");return false;}
    PositionIterations=int(Pos);VelocityIterations=int(Vel);bSphereContact=Geometry==TEXT("inscribed_9_spheres");
    double N=0,Angle=0;
    auto Read=[&](const TCHAR* K,double& V) {return J->TryGetNumberField(K,V) && FMath::IsFinite(V);};
    if(!Read(TEXT("roller_count"),N) || !Read(TEXT("wheel_width_m"),WidthM) || !Read(TEXT("hub_radius_m"),HubRadiusM) || !Read(TEXT("hub_width_m"),HubWidthM) ||
       !Read(TEXT("roller_max_radius_m"),RollerRadiusM) || !Read(TEXT("roller_half_length_m"),RollerHalfLengthM) ||
       !Read(TEXT("roller_mass_kg"),RollerMassKg) || !Read(TEXT("roller_bearing_coulomb_nm"),RollerCoulombNm) ||
       !Read(TEXT("roller_bearing_viscous_nms"),RollerViscousNms) || !Read(TEXT("roller_angle_deg"),Angle) ||
       !Read(TEXT("max_translation_mps"),MaxTranslationMps) || !Read(TEXT("velocity_feedback_gain"),VelocityKp) ||
       !Read(TEXT("velocity_integral_gain_per_s"),VelocityKi) || !Read(TEXT("max_velocity_correction_mps"),MaxCorrectionMps) ||
       N!=FMath::FloorToDouble(N) || N<8 || N>16 || Angle!=45 || WidthM<.1 || WidthM>.3 ||
       HubRadiusM<.05 || HubRadiusM>.3 || HubWidthM<.03 || HubWidthM>WidthM || RollerRadiusM<.01 || RollerRadiusM>.06 || RollerHalfLengthM<.03 || RollerHalfLengthM>.12 ||
       RollerMassKg<.2 || RollerMassKg>3 || RollerCoulombNm<0 || RollerCoulombNm>2 || RollerViscousNms<0 || RollerViscousNms>1 ||
       MaxTranslationMps<=0 || MaxTranslationMps>2 || VelocityKp<0 || VelocityKp>3 || VelocityKi<0 || VelocityKi>2 || MaxCorrectionMps<0 || MaxCorrectionMps>2)
    {Error=TEXT("Mecanum dimensions/masses/bearing losses outside supported design bounds");return false;}
    const TSharedPtr<FJsonObject>* Reference=nullptr;
    if(!J->TryGetObjectField(TEXT("catalogue_comparison"),Reference) || !Reference ||
       !(*Reference)->TryGetStringField(TEXT("model"),ComparisonModel) || ComparisonModel.IsEmpty() ||
       !(*Reference)->TryGetStringField(TEXT("url"),ComparisonUrl) || !ComparisonUrl.StartsWith(TEXT("https://")) ||
       !(*Reference)->TryGetNumberField(TEXT("listed_load_kg"),ComparisonLoadKg) || !FMath::IsFinite(ComparisonLoadKg) || ComparisonLoadKg<=0 ||
       !(*Reference)->TryGetNumberField(TEXT("listed_mass_kg"),ComparisonMassKg) || !FMath::IsFinite(ComparisonMassKg) || ComparisonMassKg<=0)
    {Error=TEXT("Invalid mecanum catalogue comparison; custom wheel remains unrated");return false;}
    Count=int(N);
    bReduced=FParse::Param(FCommandLine::Get(),TEXT("ReducedMecanum"));
    if(bReduced)
    {
        int ReducedHz=960;FParse::Value(FCommandLine::Get(),TEXT("ReducedPhysicsHz="),ReducedHz);
        if(ReducedHz!=480 && ReducedHz!=960 && ReducedHz!=1920){Error=TEXT("ReducedPhysicsHz requires 480/960/1920");return false;}
        // Retain the configured 32/32 contact iterations: 16/16 failed the
        // compact loaded half-step energy convergence check at 960/1920 Hz.
        MinimumPhysicsHz=ReducedHz;bAdaptiveStep=false;bBearingIntegrator=false;
    }
    return true;
}

void UWheelTyreComponent::BuildMecanumRoller(double R,double MaxRadius,double HalfLength,UStaticMesh* SharedMesh)
{
    // Inscribe the complete rotating roller in the wheel's cylindrical envelope.
    // X is the passive axle, Z the outward radial direction at assembly time.
    // Find a profile radius whose maximum projected wheel-plane radius is R.
    constexpr int Rings=13,Sides=32;
    const double Rc=R-MaxRadius,C=FMath::Sqrt(.5);
    TArray<FVector> Points;
    for(int I=0;I<Rings;++I)
    {
        const double X=-HalfLength+2*HalfLength*I/(Rings-1),T=FMath::Abs(X)*C;
        double Low=0,High=MaxRadius;
        for(int It=0;It<38;++It)
        {
            const double A=(Low+High)*.5;
            // Squared distance is concave in sin(phi) on the outward half.
            double L=0,H=UE_DOUBLE_PI*.5;
            auto D=[&](double P) {return FMath::Square(Rc+A*FMath::Cos(P))+FMath::Square(T+A*C*FMath::Sin(P));};
            for(int K=0;K<28;++K) {const double P=L+(H-L)/3,Q=H-(H-L)/3;if(D(P)<D(Q)) L=P;else H=Q;}
            if(D((L+H)*.5)>R*R) High=A;else Low=A;
        }
        const double Radius=(Low+High)*.5;
        for(int K=0;K<Sides;++K) {const double A=2*UE_DOUBLE_PI*K/Sides;Points.Add(FVector(X,Radius*FMath::Sin(A),Radius*FMath::Cos(A)));}
    }
    if(!SharedMesh)
    {
        FMeshDescription D; FStaticMeshAttributes At(D); At.Register();
        auto Pos=At.GetVertexPositions(); auto Norm=At.GetVertexInstanceNormals(); auto UV=At.GetVertexInstanceUVs();UV.SetNumChannels(1);
        const auto G=D.CreatePolygonGroup();At.GetPolygonGroupMaterialSlotNames()[G]=TEXT("Roller");
        TArray<FVertexID> Vs;for(FVector P:Points){const auto V=D.CreateVertex();Pos[V]=FVector3f(P);Vs.Add(V);}
        auto Tri=[&](int A,int B,int Cc)
        {
            const FVector3f N=FVector3f(FVector::CrossProduct(Points[B]-Points[A],Points[Cc]-Points[A]).GetSafeNormal());
            TArray<FVertexInstanceID> Corners;for(int K:{A,Cc,B}){const auto V=D.CreateVertexInstance(Vs[K]);Norm[V]=N;UV.Set(V,0,FVector2f(Points[K].X/R,Points[K].Y/R));Corners.Add(V);}D.CreateTriangle(G,Corners);
        };
        for(int I=0;I<Rings-1;++I) for(int K=0;K<Sides;++K) {const int A=I*Sides+K,B=I*Sides+(K+1)%Sides;Tri(A,B+Sides,B);Tri(A,A+Sides,B+Sides);}
        for(int K=1;K<Sides-1;++K){Tri(0,K,K+1);Tri((Rings-1)*Sides,(Rings-1)*Sides+K+1,(Rings-1)*Sides+K);}
        SharedMesh=NewObject<UStaticMesh>(this);SharedMesh->GetStaticMaterials().Add(FStaticMaterial(nullptr,TEXT("Roller")));
        UStaticMesh::FBuildMeshDescriptionsParams P;P.bFastBuild=true;P.bMarkPackageDirty=false;P.bBuildSimpleCollision=false;P.bAllowCpuAccess=true;
        check(SharedMesh->BuildFromMeshDescriptions({&D},P));
    }
    SetStaticMesh(SharedMesh);RollingCollision=NewObject<UBodySetup>(this);RollingCollision->CollisionTraceFlag=CTF_UseSimpleAsComplex;
    FKConvexElem Hull;Hull.VertexData=Points;Hull.UpdateElemBox();RollingCollision->AggGeom.ConvexElems.Add(MoveTemp(Hull));
    RollingCollision->InvalidatePhysicsData();RollingCollision->CreatePhysicsMeshes();RecreatePhysicsState();
}

void UWheelTyreComponent::UseMecanumSphereContact(int32 Count)
{
    const auto Points=RollingCollision->AggGeom.ConvexElems[0].VertexData;
    const FVector ReferenceI=GetInertiaTensor();const FTransform ReferenceMass=BodyInstance.GetMassSpaceLocal();const double Mass=GetMass();
    constexpr int Rings=13,Sides=32;const double Half=Points.Last().X;
    auto* Analytic=NewObject<UBodySetup>(this);Analytic->CollisionTraceFlag=CTF_UseSimpleAsComplex;
    for(int I=0;I<Count;++I)
    {
        const double X=-Half*2/3+Half*4/3*I/(Count-1);double Radius=Half-FMath::Abs(X);
        for(int K=0;K<Rings-1;++K)
        {
            const FVector A=Points[K*Sides],B=Points[(K+1)*Sides];
            const double R0=A.Z*FMath::Cos(UE_DOUBLE_PI/Sides),R1=B.Z*FMath::Cos(UE_DOUBLE_PI/Sides),Slope=(R1-R0)/(B.X-A.X);
            Radius=FMath::Min(Radius,(R0+Slope*(X-A.X))/FMath::Sqrt(1+Slope*Slope));
        }
        FKSphereElem Sphere;Sphere.Center=FVector(X,0,0);Sphere.Radius=Radius;Analytic->AggGeom.SphereElems.Add(Sphere);
    }
    RollingCollision=Analytic;RollingCollision->CreatePhysicsMeshes();RecreatePhysicsState();SetMassOverrideInKg(NAME_None,Mass);
    // Overlapping spheres represent only the contact envelope. Preserve the
    // authored barrel's mass, COM and inertia; do not count overlaps as material.
    FPhysicsCommand::ExecuteWrite(BodyInstance.GetPhysicsActor(),[ReferenceI,ReferenceMass](const FPhysicsActorHandle& Actor)
    {
        FPhysicsInterface::SetMassSpaceInertiaTensor_AssumesLocked(Actor,ReferenceI);
        FPhysicsInterface::SetComLocalPose_AssumesLocked(Actor,ReferenceMass);
    });
    checkf((GetInertiaTensor()-ReferenceI).Size()<1e-3,TEXT("Analytic contact must preserve authored barrel inertia"));
}

void AWheelActor::BuildMecanum(const FVehicle3DSettings& S,UPhysicalMaterial* Material,int Slot)
{
    if(!bMecanum) return;
    if(S.Mecanum.bReduced){BuildReducedMecanum(S,Material,Slot);return;}
    MecanumSpec=S.Mecanum;RollerHand=Mecanum::Hand(Slot);bSphereContact=S.Mecanum.bSphereContact || FParse::Param(FCommandLine::Get(),TEXT("MecanumSphereContact"));if(FParse::Param(FCommandLine::Get(),TEXT("MecanumConvexContact"))) bSphereContact=false;
    Vehicle3D::Color(Tyre,FLinearColor(.22,.30,.34));
    const FVector P=Tyre->GetComponentLocation(),Axis=Knuckle->GetRightVector();const FQuat Q=Tyre->GetComponentQuat();
    UStaticMesh* Mesh=nullptr;
    for(int I=0;I<MecanumSpec.Count;++I)
    {
        const double A=2*UE_DOUBLE_PI*I/MecanumSpec.Count;
        const FVector Radial=Q.RotateVector(FVector(FMath::Cos(A),FMath::Sin(A),0));
        const FVector RollerAxis=(-FVector::CrossProduct(Axis,Radial)+RollerHand*Axis).GetSafeNormal();
        const FQuat Pose=FRotationMatrix::MakeFromXZ(RollerAxis,Radial).ToQuat();
        auto* B=NewObject<UWheelTyreComponent>(this);B->SetMobility(EComponentMobility::Movable);B->RegisterComponent();
        B->SetWorldLocationAndRotation(P+Radial*(S.RadiusM-MecanumSpec.RollerRadiusM)*100,Pose);
        B->BuildMecanumRoller(S.RadiusM*100,MecanumSpec.RollerRadiusM*100,MecanumSpec.RollerHalfLengthM*100,Mesh);Mesh=B->GetStaticMesh();
        Vehicle3D::Dynamic(B,MecanumSpec.RollerMassKg);B->SetCollisionObjectType(TreadChannel);B->SetCollisionResponseToChannel(TreadChannel,ECR_Ignore);
        B->SetPhysMaterialOverride(Material);B->SetLinearDamping(0);B->SetAngularDamping(0);B->SetNotifyRigidBodyCollision(true);
        B->OnComponentHit.AddDynamic(this,&AWheelActor::OnTyreHit);
        Vehicle3D::Color(B,I%2?FLinearColor(.09,.13,.15):FLinearColor(.045,.065,.08));
        if(bSphereContact) B->UseMecanumSphereContact(9);
        auto* J=NewObject<UPhysicsConstraintComponent>(this);J->RegisterComponent();J->SetWorldLocationAndRotation(B->GetComponentLocation(),Pose);
        Vehicle3D::Lock(J);J->ConstraintInstance.ProfileInstance.bUseLinearJointSolver=FParse::Param(FCommandLine::Get(),TEXT("MecanumLinearJoints"));J->SetAngularTwistLimit(ACM_Free,0);J->SetConstrainedComponents(Tyre,NAME_None,B,NAME_None);
        J->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);J->SetAngularVelocityTarget(FVector::ZeroVector);
        Rollers.Add(B);RollerJoints.Add(J);
        RollerLocalCenters.Add(Q.UnrotateVector(B->GetComponentLocation()-P));
    }
    RollerNormalN.Init(0,Rollers.Num());RollerOmega.Init(0,Rollers.Num());RollerBearingNm.Init(0,Rollers.Num());RollerContactPointM.Init(FVector::ZeroVector,Rollers.Num());
    RollerAnchorErrorM.Init(0,Rollers.Num());
    UE_LOG(LogTemp,Display,TEXT("MECANUM_WHEEL slot=%d rollers=%d hand=%.0f hub_mass_kg=%.3f roller_total_kg=%.3f inherited_max_omega_rad_s=%.6f"),Slot,Rollers.Num(),RollerHand,Tyre->GetMass(),MecanumSpec.Count*MecanumSpec.RollerMassKg,Tyre->BodyInstance.GetMaxAngularVelocityInRadians());
}

void AWheelActor::MeasureMecanum()
{
    double Sum=0;
    for(int I=0;I<Rollers.Num();++I)
    {
        const FVector A=Rollers[I]->GetForwardVector();
        RollerOmega[I]=FVector::DotProduct(Rollers[I]->GetPhysicsAngularVelocityInRadians()-Tyre->GetPhysicsAngularVelocityInRadians(),A);
        RollerAnchorErrorM[I]=(Rollers[I]->GetComponentLocation()-Tyre->GetComponentLocation()-Tyre->GetComponentQuat().RotateVector(RollerLocalCenters[I])).Size()/100.;
        Sum+=RollerNormalN[I];
    }
    // Same inner-step contact, pose and relative velocity. Never evaluate an
    // old averaged contact point at the current render-frame wheel pose.
    LongSlipMps=SolverSlipWeight>0?FMath::Sqrt(SolverSlipAlongSum/SolverSlipWeight):0;
    LateralSlipMps=SolverSlipWeight>0?FMath::Sqrt(SolverSlipAcrossSum/SolverSlipWeight):0;
    bGrounded=bReducedMecanum?NormalLoadN>0:Sum>0;bRollingRadiusValid=false;EstimatedPatchLengthM=EstimatedPeakPressurePa=0;
}

void AWheelActor::ResistRollers(bool Enabled)
{
    for(int I=0;I<Rollers.Num();++I)
    {
        const double Cap=Enabled?MecanumSpec.RollerCoulombNm+MecanumSpec.RollerViscousNms*FMath::Abs(RollerOmega[I]):0;
        // Passive finite torque bearing: equal/opposite reactions are supplied
        // by the joint. No explicit opposing torque can push it through zero.
        RollerJoints[I]->SetAngularVelocityDriveTwistAndSwing(Cap>0,false);
        RollerJoints[I]->SetAngularDriveParams(0,1.e6,FMath::Max(Cap,.000001)*10000);
    }
}
