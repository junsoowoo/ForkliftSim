#include "TreadModel.h"
#include "WheelActor.h"
#include "Vehicle3DHelpers.h"
#include "PhysicsEngine/BodySetup.h"
#include "MeshDescription.h"
#include "StaticMeshAttributes.h"
#include "Engine/StaticMesh.h"

void UWheelTyreComponent::BuildTreadSector(double R,double Depth,double Width,int32 Count,UStaticMesh* SharedMesh)
{
    const double CenterR=R-Depth*.5, Half=UE_DOUBLE_PI/Count;
    const int ArcSteps=FMath::Max(8,256/Count);
    TArray<FVector> Polygon;
    for(int I=0;I<=ArcSteps;++I)
    {
        const double A=-Half+2*Half*I/ArcSteps;
        Polygon.Add(FVector(R*FMath::Cos(A)-CenterR,R*FMath::Sin(A),0));
    }
    // Inner chord, not a concave annular sector: one valid convex per pad.
    Polygon.Add(FVector((R-Depth)*FMath::Cos(Half)-CenterR,(R-Depth)*FMath::Sin(Half),0));
    Polygon.Add(FVector((R-Depth)*FMath::Cos(Half)-CenterR,-(R-Depth)*FMath::Sin(Half),0));
    TArray<FVector> Points;
    for(double Z:{-Width*.5,Width*.5}) for(FVector P:Polygon) { P.Z=Z; Points.Add(P); }
    if(!SharedMesh)
    {
        FMeshDescription D; FStaticMeshAttributes Attributes(D); Attributes.Register();
        auto Positions=Attributes.GetVertexPositions(); auto Normals=Attributes.GetVertexInstanceNormals();
        auto UVs=Attributes.GetVertexInstanceUVs(); UVs.SetNumChannels(1);
        const auto Group=D.CreatePolygonGroup();
        Attributes.GetPolygonGroupMaterialSlotNames()[Group]=TEXT("Tread");
        TArray<FVertexID> Vertices;
        for(const FVector& P:Points) { const auto V=D.CreateVertex(); Positions[V]=FVector3f(P); Vertices.Add(V); }
        auto Triangle=[&](int A,int B,int C)
        {
            const FVector3f N=FVector3f(FVector::CrossProduct(Points[B]-Points[A],Points[C]-Points[A]).GetSafeNormal());
            TArray<FVertexInstanceID> Corners;
            // Unreal render faces use clockwise front winding. Keep the
            // outward geometric normal while reversing raster index order.
            for(int I:{A,C,B}) { const auto V=D.CreateVertexInstance(Vertices[I]); Normals[V]=N; UVs.Set(V,0,FVector2f(Points[I].X/R,Points[I].Y/R)); Corners.Add(V); }
            D.CreateTriangle(Group,Corners);
        };
        const int N=Polygon.Num();
        for(int I=1;I<N-1;++I) { Triangle(0,I+1,I); Triangle(N,N+I,N+I+1); }
        for(int I=0;I<N;++I) { const int J=(I+1)%N; Triangle(I,J,N+J); Triangle(I,N+J,N+I); }
        SharedMesh=NewObject<UStaticMesh>(this);
        SharedMesh->GetStaticMaterials().Add(FStaticMaterial(nullptr,TEXT("Tread")));
        UStaticMesh::FBuildMeshDescriptionsParams Params; Params.bFastBuild=true; Params.bMarkPackageDirty=false;
        Params.bBuildSimpleCollision=false; Params.bAllowCpuAccess=true;
        const bool Built=SharedMesh->BuildFromMeshDescriptions({&D},Params); check(Built);
    }
    SetStaticMesh(SharedMesh);
    RollingCollision=NewObject<UBodySetup>(this);
    RollingCollision->CollisionTraceFlag=CTF_UseSimpleAsComplex;
    FKConvexElem Hull; Hull.VertexData=Points; Hull.UpdateElemBox();
    RollingCollision->AggGeom.ConvexElems.Add(MoveTemp(Hull));
    RollingCollision->InvalidatePhysicsData(); RollingCollision->CreatePhysicsMeshes(); RecreatePhysicsState();
}

TArray<UStaticMeshComponent*> AWheelActor::PhysicalBodies() const
{
    TArray<UStaticMeshComponent*> B={Knuckle.Get(),BeltCarrier.Get(),Tyre.Get()};
    for(UWheelTyreComponent* P:TreadPads) B.Add(P);
    for(UWheelTyreComponent* P:Rollers) B.Add(P);
    return B;
}

void AWheelActor::BuildTread(const FVehicle3DSettings& S,UPhysicalMaterial* Material)
{
    if(!Compliance.Tread.bEnabled) return;
    const auto& T=Compliance.Tread;
    // Replace only the aggregate RADIAL spring. Keep the non-spinning X/Y shear
    // guide. Pad/rim radial springs now carry actual normal contact reactions.
    ComplianceGuide->SetLinearPositionDrive(false,false,false);
    ComplianceGuide->SetLinearVelocityDrive(false,false,false);
    ComplianceGuide->SetLinearZLimit(LCM_Locked,0);
    if(T.bCoupled)
    {
        // Local material shear replaces the former whole-wheel proxy.
        ComplianceGuide->SetLinearXLimit(LCM_Locked,0); ComplianceGuide->SetLinearYLimit(LCM_Locked,0);
        ShearGuide->SetLinearPositionDrive(false,false,false); ShearGuide->SetLinearVelocityDrive(false,false,false);
        ShearGuide->SetLinearXLimit(LCM_Free,0); ShearGuide->SetLinearYLimit(LCM_Free,0);
    }
    const FQuat HubQ=Tyre->GetComponentQuat(); const FVector HubP=Tyre->GetComponentLocation();
    UStaticMesh* Mesh=nullptr;
    for(int I=0;I<T.Count;++I)
    {
        const double A=2*UE_DOUBLE_PI*I/T.Count+T.PhaseRad;
        const FVector Local(FMath::Cos(A),FMath::Sin(A),0), Radial=HubQ.RotateVector(Local);
        const FVector P=HubP+Radial*(S.RadiusM-T.DepthM*.5)*100;
        auto* Pad=NewObject<UWheelTyreComponent>(this);
        Pad->SetMobility(EComponentMobility::Movable); Pad->RegisterComponent();
        Pad->SetWorldLocationAndRotation(P,HubQ*FQuat(FVector::UpVector,A));
        Pad->BuildTreadSector(S.RadiusM*100,T.DepthM*100,S.WidthM*100,T.Count,Mesh);
        Mesh=Pad->GetStaticMesh(); Vehicle3D::Dynamic(Pad,T.MassKg/T.Count);
        // Filter same-wheel pairs BEFORE convex narrow phase. Each wheel has a
        // distinct object channel, so inter-wheel contacts remain enabled.
        Pad->SetCollisionObjectType(TreadChannel); Pad->SetCollisionResponseToChannel(TreadChannel,ECR_Ignore);
        Pad->SetLinearDamping(0); Pad->SetAngularDamping(0); Pad->SetPhysMaterialOverride(Material);
        Vehicle3D::Color(Pad,I%2?FLinearColor(.055,.07,.08):FLinearColor(.035,.045,.055));
        Pad->SetNotifyRigidBodyCollision(true); Pad->OnComponentHit.AddDynamic(this,&AWheelActor::OnTyreHit);
        auto* J=NewObject<UPhysicsConstraintComponent>(this); J->RegisterComponent();
        J->SetWorldLocationAndRotation(P,FRotationMatrix::MakeFromZX(Radial,HubQ.GetAxisZ()).ToQuat());
        Vehicle3D::Lock(J); J->SetLinearZLimit(LCM_Limited,T.TravelM*100);
        if(T.bCoupled) J->ConstraintInstance.ProfileInstance.bUseLinearJointSolver=false;
        // Chaos reverses component order in its solver and expresses linear
        // drives in solver frame 0. The foundation axes must follow the rim,
        // not the deforming pad. Keep the legacy radial-only fixture unchanged.
        if(T.bCoupled) J->SetConstrainedComponents(Pad,NAME_None,Tyre,NAME_None);
        else J->SetConstrainedComponents(Tyre,NAME_None,Pad,NAME_None);
        J->SetLinearPositionDrive(false,false,true); J->SetLinearVelocityDrive(false,false,true);
        J->SetLinearPositionTarget(FVector::ZeroVector); J->SetLinearVelocityTarget(FVector::ZeroVector);
        J->SetLinearDriveParams(T.PadK(S.RadiusM,S.WidthM),T.PadC(S.RadiusM,S.WidthM),0);
        if(T.bCoupled)
        {
            // X is the wheel axle, Y the local tangent, Z the radial axis.
            J->SetLinearYLimit(LCM_Free,0); J->SetLinearZLimit(LCM_Limited,T.TravelM*100);
            J->SetLinearPositionDrive(false,true,true); J->SetLinearVelocityDrive(false,true,true);
            J->ConstraintInstance.SetLinearDriveParams(FVector(0,T.PadShearK(S.RadiusM,S.WidthM),T.PadK(S.RadiusM,S.WidthM)),
                FVector(0,T.PadShearC(S.RadiusM,S.WidthM),T.PadC(S.RadiusM,S.WidthM)),FVector::ZeroVector);
            J->SetAngularTwistLimit(ACM_Free,0); J->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
            J->SetOrientationDriveTwistAndSwing(true,false); J->SetAngularVelocityDriveTwistAndSwing(true,false);
            Vehicle3D::AngularSpring(J,T.PadRotK(S.RadiusM,S.WidthM),T.PadRotC(S.RadiusM,S.WidthM));
        }
        TreadPads.Add(Pad); TreadJoints.Add(J);
    }
    if(T.bCoupled) for(int I=0;I<T.Count;++I)
    {
        const double A=2*UE_DOUBLE_PI*(I+.5)/T.Count+T.PhaseRad;
        const FVector Dir=HubQ.RotateVector(FVector(FMath::Cos(A),FMath::Sin(A),0));
        auto* J=NewObject<UPhysicsConstraintComponent>(this); J->RegisterComponent();
        J->SetWorldLocationAndRotation(HubP+Dir*(S.RadiusM-T.DepthM*.5)*100,FRotationMatrix::MakeFromZX(Dir,HubQ.GetAxisZ()).ToQuat());
        Vehicle3D::Lock(J); J->SetLinearXLimit(LCM_Free,0); J->SetLinearYLimit(LCM_Free,0); J->SetLinearZLimit(LCM_Free,0);
        J->SetAngularTwistLimit(ACM_Free,0); J->SetAngularSwing1Limit(ACM_Free,0); J->SetAngularSwing2Limit(ACM_Free,0);
        J->ConstraintInstance.ProfileInstance.bUseLinearJointSolver=false;
        J->SetConstrainedComponents(TreadPads[I],NAME_None,TreadPads[(I+1)%T.Count],NAME_None);
        J->SetLinearPositionDrive(false,true,true); J->SetLinearVelocityDrive(false,true,true);
        J->SetLinearDriveParams(T.LinkK(S.RadiusM,S.WidthM),T.LinkC(S.RadiusM,S.WidthM),0);
        J->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
        J->SetOrientationDriveTwistAndSwing(true,false); J->SetAngularVelocityDriveTwistAndSwing(true,false);
        Vehicle3D::AngularSpring(J,T.BendK(S.RadiusM,S.WidthM),T.BendC(S.RadiusM,S.WidthM));
        TreadLinks.Add(J);
    }
    TreadCompressionM.SetNumZeroed(T.Count); TreadRateMps.SetNumZeroed(T.Count);
    TreadShearM.SetNumZeroed(T.Count); TreadRotationRad.SetNumZeroed(T.Count);
    TreadNormalN.SetNumZeroed(T.Count);
    TreadContactPointM.SetNumZeroed(T.Count);
    UE_LOG(LogTemp,Display,TEXT("TREAD_BUILD segments=%d hub_kg=%.3f tread_kg=%.3f pad_k_npm=%.3f pad_c_nspm=%.3f bulk_radial_spring=OFF"),T.Count,Tyre->GetMass(),T.MassKg,T.PadK(S.RadiusM,S.WidthM),T.PadC(S.RadiusM,S.WidthM));
}

void AWheelActor::MeasureTread(double R,double W)
{
    TreadMaxCompressionM=TreadStoredJ=TreadSupportN=TreadShearJ=TreadRotationJ=TreadLinkJ=TreadMaxShearM=TreadMaxRotationRad=0; TreadMinCompressionM=1e30; bTreadBottomed=false;
    const auto& T=Compliance.Tread; const FQuat Q=Tyre->GetComponentQuat(); const FVector Hub=Tyre->GetComponentLocation();
    const double Rest=R-T.DepthM*.5, K=T.PadK(R,W), C=T.PadC(R,W);
    for(int I=0;I<TreadPads.Num();++I)
    {
        UWheelTyreComponent* P=TreadPads[I]; const double A=2*UE_DOUBLE_PI*I/T.Count+T.PhaseRad;
        const FVector Dir=Q.RotateVector(FVector(FMath::Cos(A),FMath::Sin(A),0));
        const FVector Offset=P->GetComponentLocation()-Hub;
        const double D=Rest-FVector::DotProduct(Offset,Dir)/100.;
        const FVector Relative=P->GetPhysicsLinearVelocityAtPoint(P->GetComponentLocation())-Tyre->GetPhysicsLinearVelocityAtPoint(Hub)-FVector::CrossProduct(Tyre->GetPhysicsAngularVelocityInRadians(),Offset);
        const double V=-FVector::DotProduct(Relative,Dir)/100.;
        TreadCompressionM[I]=D; TreadRateMps[I]=V;
        const FVector Tangent=Q.RotateVector(FVector(-FMath::Sin(A),FMath::Cos(A),0));
        TreadShearM[I]=FVector::DotProduct(Offset,Tangent)/100.;
        const FQuat Rel=(Q*FQuat(FVector::UpVector,A)).Inverse()*P->GetComponentQuat();
        TreadRotationRad[I]=std::remainder(2*FMath::Atan2(Rel.Z,Rel.W),2*UE_DOUBLE_PI);
        if(T.bCoupled)
        {
            TreadShearJ+=.5*T.PadShearK(R,W)*FMath::Square(TreadShearM[I]);
            TreadRotationJ+=.5*T.PadRotK(R,W)*FMath::Square(TreadRotationRad[I]);
            TreadMaxShearM=FMath::Max(TreadMaxShearM,FMath::Abs(TreadShearM[I]));
            TreadMaxRotationRad=FMath::Max(TreadMaxRotationRad,FMath::Abs(TreadRotationRad[I]));
        }
        TreadMaxCompressionM=FMath::Max(TreadMaxCompressionM,D); TreadMinCompressionM=FMath::Min(TreadMinCompressionM,D);
        TreadStoredJ+=.5*K*D*D; TreadSupportN-=(K*D+C*V)*FVector::DotProduct(Dir,Knuckle->GetUpVector());
        bTreadBottomed |= FMath::Abs(D)>=T.TravelM*.98;
        if(auto* Material=Cast<UMaterialInstanceDynamic>(P->GetMaterial(0)))
        {
            const double Fraction=NormalLoadN>1?FMath::Clamp(TreadNormalN[I]/NormalLoadN*2,0.,1.):0;
            Material->SetVectorParameterValue(TEXT("Tint"),FMath::Lerp(FLinearColor(.04,.055,.065),FLinearColor(.9,.38,.06),Fraction));
        }
    }
    if(T.bCoupled) for(int I=0;I<TreadLinks.Num();++I)
    {
        auto* J=TreadLinks[I].Get(); auto* A=TreadPads[I].Get(); auto* B=TreadPads[(I+1)%T.Count].Get();
        const FTransform F0=J->ConstraintInstance.GetRefFrame(EConstraintFrame::Frame1)*A->GetComponentTransform();
        const FTransform F1=J->ConstraintInstance.GetRefFrame(EConstraintFrame::Frame2)*B->GetComponentTransform();
        const FVector Delta=F0.GetRotation().UnrotateVector(F1.GetLocation()-F0.GetLocation())/100.;
        const double Angle=std::remainder(TreadRotationRad[(I+1)%T.Count]-TreadRotationRad[I],2*UE_DOUBLE_PI);
        TreadLinkJ+=.5*T.LinkK(R,W)*(Delta.Y*Delta.Y+Delta.Z*Delta.Z)+.5*T.BendK(R,W)*Angle*Angle;
    }
    TreadStoredJ+=TreadShearJ+TreadRotationJ+TreadLinkJ;
}
