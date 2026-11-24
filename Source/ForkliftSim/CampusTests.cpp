#include "CampusInventory.h"
#include "PalletLoadActor.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "PalletPerception.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCampusReservations,"Forklift.Campus.ExclusiveInventory",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCampusReservations::RunTest(const FString&)
{
    CampusInventory::FRegistry R;CampusInventory::FLoad L;L.Id=TEXT("PALLET");R.Loads.Add(L.Id,L);CampusInventory::FSlot S;S.Id=TEXT("SLOT");R.Slots.Add(S.Id,S);
    TestTrue(TEXT("First claimant"),R.Reserve(L.Id,TEXT("AGV-01")));TestFalse(TEXT("No second claimant"),R.Reserve(L.Id,TEXT("AGV-02")));TestFalse(TEXT("Other cannot release"),R.Release(L.Id,TEXT("AGV-02")));
    TestTrue(TEXT("Destination reservation"),R.ReserveSlot(S.Id,TEXT("AGV-01")));TestFalse(TEXT("Other cannot place"),R.Place(L.Id,S.Id,TEXT("AGV-02")));TestTrue(TEXT("Owner places"),R.Place(L.Id,S.Id,TEXT("AGV-01")));
    TestEqual(TEXT("Occupied"),R.Slots[S.Id].Pallet,L.Id);TestFalse(TEXT("Occupied slot cannot be re-reserved"),R.ReserveSlot(S.Id,TEXT("AGV-02")));R.Depart(L.Id);TestTrue(TEXT("Pickup clears source"),R.Slots[S.Id].Pallet.IsEmpty());
    R.Loads[L.Id].Moving=true;TestFalse(TEXT("Moving load not released"),R.Release(L.Id,TEXT("AGV-01")));R.Loads[L.Id].Moving=false;TestTrue(TEXT("Stopped released"),R.Release(L.Id,TEXT("AGV-01")));R.Slots[S.Id].Blocked=true;TestFalse(TEXT("Unavailable slot"),R.ReserveSlot(S.Id,TEXT("AGV-01")));return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCampusFourWayGeometry,"Forklift.Campus.FourWayCollisionCorridors",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCampusFourWayGeometry::RunTest(const FString&)
{
    FPayloadSettings S;FString Error;TestTrue(TEXT("Load defaults"),S.Load(Error));TestTrue(TEXT("Read actual collision parts"),S.UseCampusGeometry(Error));
    if(!Error.IsEmpty())return false;TestTrue(TEXT("Free cargo"),!S.bCargoSecured);TestEqual(TEXT("Tine spacing matches channels"),S.ForkSpacingM,.46);
    for(int Axis=0;Axis<2;++Axis)for(double Side:{-.23,.23})for(int I=0;I<S.BoardCentersM.Num();++I)
    {const auto C=S.BoardCentersM[I],D=S.BoardSizesM[I];const bool OverlapZ=C.Z-D.Z*.5<.075&&C.Z+D.Z*.5>.025;const bool OverlapSide=C[1-Axis]-D[1-Axis]*.5<Side+.06&&C[1-Axis]+D[1-Axis]*.5>Side-.06;TestFalse(TEXT("120x50 mm blade traverses full assumed opening"),OverlapZ&&OverlapSide);}
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCampusMeasuredPallet,"Forklift.Campus.MeasuredFourWayDepthReplay",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCampusMeasuredPallet::RunTest(const FString&)
{
    TArray<uint8> Bytes;TestTrue(TEXT("Recorded GPU frame with sensor errors"),FFileHelper::LoadFileToArray(Bytes,*(FPaths::ProjectContentDir()/TEXT("../ContentSource/Campus/Fixtures/MeasuredFront1100.bin"))));
    const int N=320*180;if(Bytes.Num()!=N*5){AddError(TEXT("Unexpected frame size"));return false;}
    DepthCamera::FFrame F;F.Sequence=1;F.CaptureS=1;F.DeliveryS=1.1;F.DepthM.SetNum(N);F.Validity.SetNum(N);FMemory::Memcpy(F.DepthM.GetData(),Bytes.GetData(),N*4);FMemory::Memcpy(F.Validity.GetData(),Bytes.GetData()+N*4,N);
    PalletPerception::FSpec S;S.LayerSearchM=.16;S.Spacing=.46;S.OpeningWidth=.28;S.OpeningHeight=.09;S.SupportHalfWidth=.46;S.Width=1.1;S.ForkSpacing=.46;S.FullDepthM=1.;
    const auto D=PalletPerception::Detect(F,S);TestTrue(TEXT("Feet layer survives a recessed cargo plane"),D.Valid);TestTrue(TEXT("Measured front distance"),FMath::Abs(D.DistanceM-1.4275)<.035);
    F.Validity.Init(uint8(DepthCamera::EValidity::NoReturn),N);TestFalse(TEXT("Expanded hypotheses do not fabricate missing openings"),PalletPerception::Detect(F,S).Valid);return true;
}
