#include "StabilityLab.h"
#include "ContactLoadSampler.h"
#include "ForkActor.h"
#include "WheelActor.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

void AStabilityLab::PrepareEnergyWork()
{
    if(!LoadSampler) return;
    auto* In=LoadSampler->GetProducerInputData_External();
    In->bAuditWork=Settings.bSwerve;
    auto Id=[](UStaticMeshComponent* B){return B->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;};
    In->ForkCarriageId=Id(Fork->Carriage);In->MastStageId=Fork->MovingMast?Id(Fork->MovingMast):INDEX_NONE;In->ForkDampingCount=0;
    for(const AForkTineActor* T:Fork->Tines) if(T->Structure.bEnabled)
        for(int I=0;I<T->Sections.Num();++I) if(!T->Bends[I]->IsBroken())
        {
            const int K=In->ForkDampingCount++;const double D=T->Structure.LengthM/T->Sections.Num();
            In->ForkDampingParent[K]=Id(I?T->Sections[I-1].Get():Fork->Carriage.Get());In->ForkDampingChild[K]=Id(T->Sections[I]);
            In->ForkDampingC[K]=2*T->Structure.DampingRatio*FMath::Sqrt(T->Structure.JointK(I)*T->Sections[I]->GetMass()*D*D/3.);
        }
    for(int I=0;I<4;++I)
    {
        const auto* W=Wheels[I].Get(); const FVector Axis=W->Knuckle->GetRightVector();
        In->DriveTorqueNm[I]=Axis*W->DriveTorqueNm;
        In->RollingTorqueNm[I]=Axis*W->RollingTorqueNm;
    }
}
void AStabilityLab::UpdateEnergyAudit()
{
    auto& A=EnergyAudit; const auto& H=Fork->HydraulicState;
    double BrakeElastic=0;for(const AWheelActor* W:Wheels) BrakeElastic+=W->BrakeElasticJ;
    A.ReducedLossJ=A.PlasticRelaxationJ=A.FractureReleaseJ=0;
    for(const AWheelActor* W:Wheels)A.ReducedLossJ+=W->ReducedContactLossJ;
    for(const AForkTineActor* T:Fork->Tines){A.PlasticRelaxationJ+=T->PlasticRelaxationJ;A.FractureReleaseJ+=T->FractureReleaseJ;}
    // A common settled reference removes the arbitrary gravitational datum.
    // Queue outputs and HydraulicState both describe the completed frame here.
    if(!A.bReferenced && TimeS>1 && PreviousPhysicsDt>0)
    {
        A.bReferenced=true; A.ReferenceJ=TyreMechanicalEnergyJ();
        A.ReferenceDriveJ=A.DriveJ; A.ReferenceRollingJ=A.RollingJ; A.ReferenceSteeringJ=A.SteeringJ;
        A.ReferenceDcJ=H.DcJ; A.ReferenceHeatJ=H.HeatJ; A.ReferenceTyreJ=TyreState.DampingJ;
        A.ReferenceChainLossJ=H.ChainLossJ;
        A.RefReducedLossJ=A.ReducedLossJ;A.RefPlasticRelaxationJ=A.PlasticRelaxationJ;A.RefFractureReleaseJ=A.FractureReleaseJ;
        A.RefContactNormalJ=A.ContactNormalJ;A.RefContactTangentJ=A.ContactTangentJ;A.RefCorrectionPseudoJ=A.CorrectionPseudoJ;A.RefForkDampingEstimateJ=A.ForkDampingEstimateJ;
        A.ReferenceBearingTransportJ=A.BearingTransportJ; A.ReferenceRollerBearingJ=A.RollerBearingJ;A.ReferenceShaftJointJ=A.ShaftJointJ;A.ReferenceBrakeElasticJ=BrakeElastic;
    }
    if(!A.bReferenced) return;
    A.DeltaStoredJ=TyreMechanicalEnergyJ()-A.ReferenceJ;
    A.InputJ=A.DriveJ-A.ReferenceDriveJ+A.SteeringJ-A.ReferenceSteeringJ+H.DcJ-A.ReferenceDcJ;
    A.ShaftLossJ=-(A.ShaftJointJ-A.ReferenceShaftJointJ)-(BrakeElastic-A.ReferenceBrakeElasticJ);
    A.KnownLossJ=A.ShaftLossJ+H.HeatJ-A.ReferenceHeatJ+H.ChainLossJ-A.ReferenceChainLossJ+TyreState.DampingJ-A.ReferenceTyreJ-(A.RollingJ-A.ReferenceRollingJ)-(A.RollerBearingJ-A.ReferenceRollerBearingJ);
    A.KnownLossJ+=A.ReducedLossJ-A.RefReducedLossJ+A.PlasticRelaxationJ-A.RefPlasticRelaxationJ+A.FractureReleaseJ-A.RefFractureReleaseJ;
    // Contact work uses a whole-solve mean velocity: joints interact with the
    // contact impulses. It is a signed diagnostic, not an exact loss allocation.
    // Fork damping is likewise a sampled constitutive estimate. Neither is
    // added to KnownLossJ; positional pseudo-work is never physical heat.
    // Signed residual includes UNMEASURED contact/beam/body losses,
    // frame sampling, hydraulic discretization and solver error. Not full closure.
    A.UnresolvedJ=A.DeltaStoredJ-A.InputJ+A.KnownLossJ;
    A.PeakPositiveJ=FMath::Max(A.PeakPositiveJ,A.UnresolvedJ);
}
FString AStabilityLab::EnergyCsv(bool Header) const
{
    if(Header) return TEXT(",energy_reference_valid,energy_stored_delta_j,energy_drive_work_j,energy_steering_joint_work_j,energy_rolling_work_j,energy_hydraulic_dc_j,energy_hydraulic_heat_j,energy_tyre_damping_j,energy_known_input_j,energy_known_loss_j,energy_unresolved_j,energy_positive_peak_j,energy_full_closure,energy_roller_bearing_loss_j,energy_shaft_joint_work_j,energy_brake_elastic_delta_j,energy_shaft_loss_j,energy_bearing_transport_work_j,energy_chain_damping_j,energy_reduced_contact_loss_j,energy_plastic_relaxation_j,energy_fracture_release_j,energy_contact_normal_work_estimate_j,energy_contact_tangent_work_estimate_j,energy_contact_correction_pseudo_j,energy_fork_damping_estimate_j");
    const auto& A=EnergyAudit; const auto& H=Fork->HydraulicState;
    double BrakeElastic=0;for(const AWheelActor* W:Wheels) BrakeElastic+=W->BrakeElasticJ;
    return FString::Printf(TEXT(",%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,0,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f"),A.bReferenced?1:0,
        A.DeltaStoredJ,A.DriveJ-A.ReferenceDriveJ,A.SteeringJ-A.ReferenceSteeringJ,A.RollingJ-A.ReferenceRollingJ,
        H.DcJ-A.ReferenceDcJ,H.HeatJ-A.ReferenceHeatJ,TyreState.DampingJ-A.ReferenceTyreJ,A.InputJ,A.KnownLossJ,A.UnresolvedJ,A.PeakPositiveJ,-(A.RollerBearingJ-A.ReferenceRollerBearingJ),A.ShaftJointJ-A.ReferenceShaftJointJ,BrakeElastic-A.ReferenceBrakeElasticJ,A.ShaftLossJ,A.BearingTransportJ-A.ReferenceBearingTransportJ,H.ChainLossJ-A.ReferenceChainLossJ)
        +FString::Printf(TEXT(",%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f"),A.ReducedLossJ-A.RefReducedLossJ,A.PlasticRelaxationJ-A.RefPlasticRelaxationJ,A.FractureReleaseJ-A.RefFractureReleaseJ,A.ContactNormalJ-A.RefContactNormalJ,A.ContactTangentJ-A.RefContactTangentJ,A.CorrectionPseudoJ-A.RefCorrectionPseudoJ,A.ForkDampingEstimateJ-A.RefForkDampingEstimateJ);
}
