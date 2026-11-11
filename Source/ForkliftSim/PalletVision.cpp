#include "PalletPerception.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonSerializer.h"
namespace
{
    enum class EProbe {Unknown,Face,Far,Censored,Occluded};
    struct FDepthEvidenceImage
    {
        const DepthCamera::FFrame& F;const PalletPerception::FSpec& S;double Fx,Cx,Cy;FQuat Rotation;
        // Hypotheses revisit image pixels many times. Cache only the measured
        // 3x3 median/category within this frame; geometry tests remain exact.
        mutable TArray<uint8> PixelState;mutable TArray<double> PixelMedian;
        FDepthEvidenceImage(const DepthCamera::FFrame& In,const PalletPerception::FSpec& Spec):F(In),S(Spec){Fx=F.Spec.Width/(2*FMath::Tan(FMath::DegreesToRadians(F.Spec.FovDeg)*.5));Cx=(F.Spec.Width-1)*.5;Cy=(F.Spec.Height-1)*.5;Rotation=FRotator(S.CameraPitchDeg,0,0).Quaternion();PixelState.Init(0,F.DepthM.Num());PixelMedian.SetNumUninitialized(F.DepthM.Num());}
        FVector Point(int X,int Y,double D) const{return Rotation.RotateVector(FVector(D,(X-Cx)*D/Fx,(Cy-Y)*D/Fx));}
        EProbe Probe(FVector P,double* Error=nullptr) const
        {
            const FVector Optical=Rotation.UnrotateVector(P);if(Optical.X<=0)return EProbe::Unknown;
            const int X=FMath::RoundToInt(Cx+Fx*Optical.Y/Optical.X),Y=FMath::RoundToInt(Cy-Fx*Optical.Z/Optical.X);if(X<1||X>=F.Spec.Width-1||Y<1||Y>=F.Spec.Height-1)return EProbe::Unknown;
            const int Pixel=Y*F.Spec.Width+X;
            if(!PixelState[Pixel])
            {
                double V[9];int Count=0,Censored=0;
                for(int DY=-1;DY<=1;++DY)for(int DX=-1;DX<=1;++DX){const int K=(Y+DY)*F.Spec.Width+X+DX;if(F.Validity[K]==uint8(DepthCamera::EValidity::Valid))V[Count++]=F.DepthM[K];else if(F.Validity[K]==uint8(DepthCamera::EValidity::TooFar))++Censored;}
                PixelState[Pixel]=Count>=3?2:Censored>=5?3:1;
                if(Count>=3){for(int I=0;I<Count;++I)for(int J=I+1;J<Count;++J)if(V[J]<V[I])Swap(V[J],V[I]);PixelMedian[Pixel]=V[Count/2];}
            }
            if(PixelState[Pixel]!=2)return PixelState[Pixel]==3&&P.Size()+.15<F.Spec.MaxM?EProbe::Censored:EProbe::Unknown;
            const double E=PixelMedian[Pixel]-Optical.X;if(Error)*Error=E;
            const double Tol=.018+.003*Optical.X*Optical.X;
            return FMath::Abs(E)<=Tol?EProbe::Face:E>.09?EProbe::Far:E< -Tol?EProbe::Occluded:EProbe::Unknown;
        }
    };
    struct FVisionPlane {double D=0,Yaw=0;int Votes=0;};
}
bool PalletPerception::FSpec::Load(FString& Error)
{
    FString Text;TSharedPtr<FJsonObject> J;if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("PalletPerception.json")))||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J){Error=TEXT("Cannot read PalletPerception.json");return false;}
    struct F {const TCHAR* K;double* V;};F Fields[]={{TEXT("opening_spacing_m"),&Spacing},{TEXT("opening_width_m"),&OpeningWidth},{TEXT("opening_height_m"),&OpeningHeight},{TEXT("support_half_width_m"),&SupportHalfWidth},{TEXT("pallet_width_m"),&Width},{TEXT("frame_stale_s"),&FrameStaleS},{TEXT("track_max_s"),&TrackMaxS},{TEXT("track_max_distance_m"),&TrackMaxDistanceM},{TEXT("minimum_confidence"),&MinConfidence},{TEXT("full_insertion_depth_m"),&FullDepthM},{TEXT("decision_dwell_s"),&DwellS}};
    for(auto Field:Fields)if(!J->TryGetNumberField(Field.K,*Field.V)||!FMath::IsFinite(*Field.V)){Error=TEXT("Invalid pallet perception field");return false;}
    if(Spacing<.3||Spacing>.8||OpeningWidth<.2||OpeningWidth>=Spacing||OpeningHeight<.07||OpeningHeight>.2||SupportHalfWidth<Spacing*.8||Width<.8||Width>1.4||FrameStaleS<.2||FrameStaleS>1||TrackMaxS<.5||TrackMaxS>20||TrackMaxDistanceM<.2||TrackMaxDistanceM>1.8||MinConfidence<.6||MinConfidence>.95||FullDepthM<.9||FullDepthM>1.15||DwellS<.1||DwellS>.5){Error=TEXT("Pallet perception specification outside supported bounds");return false;}return true;
}
PalletPerception::FDetection PalletPerception::Detect(const DepthCamera::FFrame& Frame,const FSpec& Spec)
{
    FDetection Best;Best.Sequence=Frame.Sequence;Best.CaptureS=Frame.CaptureS;Best.DeliveryS=Frame.DeliveryS;
    if(Frame.Spec.Width<32||Frame.Spec.Height<24||Frame.DepthM.Num()!=Frame.Spec.Width*Frame.Spec.Height||Frame.Validity.Num()!=Frame.DepthM.Num())return Best;
    FDepthEvidenceImage Image(Frame,Spec);TArray<FVector> Cloud;
    for(int Y=2;Y<Frame.Spec.Height-2;Y+=3)for(int X=2;X<Frame.Spec.Width-2;X+=3){const int K=Y*Frame.Spec.Width+X;if(Frame.Validity[K]==uint8(DepthCamera::EValidity::Valid)&&FMath::IsFinite(Frame.DepthM[K])&&Frame.DepthM[K]>.1){const auto P=Image.Point(X,Y,Frame.DepthM[K]);if(P.Z>-.45&&P.Z<.45)Cloud.Add(P);}}
    TArray<FVisionPlane> Planes;
    for(int A=-20;A<=20;A+=5)
    {
        const double Yaw=FMath::DegreesToRadians(double(A)),C=FMath::Cos(Yaw),S=FMath::Sin(Yaw);int Histogram[180]={};
        for(auto P:Cloud){const int K=FMath::RoundToInt((C*P.X+S*P.Y)/.02);if(K>15&&K<180)++Histogram[K];}
        for(int K=16;K<179;++K)if(Histogram[K]>=15&&Histogram[K]>=Histogram[K-1]&&Histogram[K]>=Histogram[K+1])Planes.Add({K*.02,Yaw,Histogram[K]});
    }
    Planes.Sort([](const FVisionPlane& A,const FVisionPlane& B){return A.Votes>B.Votes;});TArray<FVisionPlane> Selected;
    // Cargo can protrude several centimetres in front of the pallet. Keep
    // those distinct hypotheses rather than merging away the pallet face.
    for(auto P:Planes){bool Duplicate=false;for(auto Q:Selected)Duplicate|=FMath::Abs(P.D-Q.D)<.025&&FMath::Abs(P.Yaw-Q.Yaw)<.13;if(!Duplicate)Selected.Add(P);if(Selected.Num()>=5)break;}
    const auto Seeds=Selected;Selected.Empty();
    // A weak pallet face need not be a local histogram maximum when the cargo
    // face dominates. Inspect the nearby depth layers without declaring them
    // occupied; every candidate still needs all measured structural evidence.
    for(auto Seed:Seeds)for(double Shift:{0.,-.02,.02,-.04,.04}){auto P=Seed;P.D+=Shift;if(P.D>.3)Selected.Add(P);}
    Best.PlaneCandidates=Selected.Num();
    for(auto Plane:Selected)
    {
        const FVector N(FMath::Cos(Plane.Yaw),FMath::Sin(Plane.Yaw),0),T(-N.Y,N.X,0);
        for(double Z=-.34;Z<=.30;Z+=.015)for(double Y=-.70;Y<=.70;Y+=.025)
        {
            auto At=[&](double U,double V){return N*Plane.D+T*(Y+U)+FVector(0,0,Z+V);};
            int Faces=0;double Errors=0;FVector Observed[3];for(double U:{-Spec.SupportHalfWidth,0.,Spec.SupportHalfWidth}){double E=0;const FVector Pred=At(U,0);if(Image.Probe(Pred,&E)==EProbe::Face){const double OpticalX=Image.Rotation.UnrotateVector(Pred).X;Observed[Faces]=Pred*((OpticalX+E)/OpticalX);++Faces;Errors+=E*E;}}if(Faces<3)continue;
            ++Best.SupportPass;int Roof=0;for(double U:{-.36,-.18,0.,.18,.36})Roof+=Image.Probe(At(U,Spec.OpeningHeight*.5+.014))==EProbe::Face;if(Roof<4)continue;++Best.RoofPass;
            int Far=0,Censored=0;bool Both=true;
            for(int H=0;H<2;++H){int ValidGap=0,CensoredGap=0;for(double U:{-.10,0.,.10})for(double V:{-.025,0.,.025}){const auto R=Image.Probe(At((H?1:-1)*Spec.Spacing*.5+U,V));ValidGap+=R==EProbe::Far;CensoredGap+=R==EProbe::Censored;}Both&=ValidGap>=2&&ValidGap+CensoredGap>=5;Far+=ValidGap;Censored+=CensoredGap;}if(!Both)continue;
            if(Both)++Best.GapPass;
            // Positive face transitions bound each opening. Unknown pixels and
            // unreturned rays never stand in for an observed opening edge.
            double Left[2],Right[2];bool Edges=true;
            for(int H=0;H<2;++H){const double Center=(H?1:-1)*Spec.Spacing*.5;Left[H]=Right[H]=0;for(int Side:{-1,1}){bool Found=false;for(double D=.10;D<=.25;D+=.01)if(Image.Probe(At(Center+Side*D,0))==EProbe::Face){(Side<0?Left[H]:Right[H])=Center+Side*(D-.005);Found=true;break;}Edges&=Found;}const double W=Right[H]-Left[H];Edges&=W>.25&&W<.43;}if(!Edges)continue;
            double RoofEdges[2]={};bool RoofBounded=true;
            for(int H=0;H<2;++H){bool Found=false;const double Center=(Right[H]+Left[H])*.5;for(double V=-.03;V<=.10;V+=.005)if(Image.Probe(At(Center,V))==EProbe::Face&&Image.Probe(At(Center,V+.005))==EProbe::Face){RoofEdges[H]=Z+V-.0025;Found=true;break;}RoofBounded&=Found;}if(!RoofBounded)continue;
            ++Best.EdgePass;const double Confidence=.55+.20*Roof/5.+.25*Far/18.;if(Confidence<=Best.Confidence)continue;
            double MeanX=0,MeanY=0;for(auto P:Observed){MeanX+=P.X/3;MeanY+=P.Y/3;}double YY=0,XY=0;for(auto P:Observed){YY+=FMath::Square(P.Y-MeanY);XY+=(P.X-MeanX)*(P.Y-MeanY);}const double Slope=YY>1e-6?XY/YY:0,FitYaw=FMath::Atan(-Slope);if(FMath::Abs(FitYaw-Plane.Yaw)>.07)continue;
            const FVector RefN(FMath::Cos(FitYaw),FMath::Sin(FitYaw),0),RefT(-RefN.Y,RefN.X,0);
            Best.Valid=Confidence>=Spec.MinConfidence;Best.Confidence=Confidence;Best.DistanceM=(MeanX-Slope*MeanY)*RefN.X;Best.CenterZ=(RoofEdges[0]+RoofEdges[1])*.5-Spec.OpeningHeight*.5;Best.Yaw=FitYaw;Best.Rms=FMath::Sqrt(Errors/3);Best.FaceEvidence=Faces+Roof;Best.GapEvidence=Far;Best.CensoredEvidence=Censored;
            for(int H=0;H<2;++H){Best.HoleWidth[H]=(Right[H]-Left[H])*FVector::DotProduct(T,RefT);Best.HoleY[H]=FVector::DotProduct(N*Plane.D+T*(Y+(Right[H]+Left[H])*.5),RefT);}Best.CenterY=(Best.HoleY[0]+Best.HoleY[1])*.5;
            if(Best.Confidence>=.999&&Best.Rms<.02)return Best;
        }
    }
    return Best;
}
void PalletPerception::FDecision::Step(const FSignals& In,const FSpec& Spec)
{
    ConfirmedFull=false;FString Next=TEXT("unknown");Reason.Empty();
    if(!In.Fresh)Reason=TEXT("required_measurement_unavailable");
    else if(In.UnilateralLoad){Next=TEXT("uneven_support");Reason=TEXT("one_sided_contact_load");}
    else if(In.Loaded){Next=TEXT("supported");Reason=TEXT("contact_support_is_not_current_insertion_geometry");}
    else if(!In.VisionFresh&&!In.Tracked)Reason=TEXT("vision_missing_or_tracking_expired");
    else if(!In.Alignment){Next=TEXT("misaligned");Reason=TEXT("opening_clearance_or_angle");}
    else if(FMath::Min(In.Depth[0],In.Depth[1])-In.UncertaintyM>=Spec.FullDepthM)
    {
        bool All=true;for(bool P:In.Presence)All&=P;
        if(All){Next=TEXT("fully_inserted");Reason=TEXT("tracked_depth_and_four_presence_points");}else{Next=TEXT("partial");Reason=TEXT("full_depth_without_complete_presence_evidence");}
    }
    else if(FMath::Max(In.Depth[0],In.Depth[1])>0){Next=TEXT("partial");Reason=TEXT("insertion_depth_below_threshold");}
    else {Next=TEXT("aligned");Reason=TEXT("not_inserted");}
    if(Next!=Candidate){Candidate=Next;Since=In.Now;}
    if(Next==TEXT("unknown")||Next==TEXT("misaligned")||Next==TEXT("uneven_support")||In.Now-Since>=Spec.DwellS)State=Next;
    else if(State!=Next)State=TEXT("observing");
    ConfirmedFull=State==TEXT("fully_inserted")&&Next==State;
    if(ConfirmedFull){LastFullS=In.Now;LastFullBeforeSupport=true;}
    if(Next==TEXT("aligned")||Next==TEXT("partial")||Next==TEXT("misaligned")||Next==TEXT("uneven_support")||(!In.Loaded&&Next==TEXT("unknown")))LastFullBeforeSupport=false;
}
