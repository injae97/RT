////////////////////////////////////////////////////////////////////////////////
//
//		File Version History
//			V0.1 : 2026-02-xx First Release
//
//	NOTE: This class implements the Part Direction Mark & Barcode operation for the CSeqCycle. 
//			It is is one of the 5 main steps (dump, nozzle change, pick, align, place) used by CSeqCycle.
// 
// CStepPickTape supports the common CStep interface for use by CSeqCycle.
// 
// PPI(= Pre PickUp Inspection / = Pre Pick Insp) 
// 클래스 역할: 부품 픽업 전, 수행해야할 행동 + 검사 단계의 표준 구현을 제공한다. (비전 검사로직 구체화 및 CStePrePickInsp 상속받음)
// 

#ifndef __STEP_PPI_STD_H__
#define __STEP_PPI_STD_H__

#include "StepPrePickInsp.h"
#include "SeqCycle.h"

#ifndef __RT_STEP_CLASS_H__
#include "RtStepClass.h"
#endif

class CStepPrePickInspStd;
// 피듀셜 카메라로 인식을 해야하므로 CCamera 선언
class CCamera;

extern CStepPrePickInspStd	g_StepPrePickInspStd[MAX_STEP_PRE_PICK_INSP + 1];


//---------------------------------------------
// CLASS	CRtStepClassPrePickInspStd		Declaration
//---------------------------------------------

class CRtStepClassPrePickInspStd : public CRtStepClass
{
public:
	// 1. 생성자 선언 (이게 없으면 .cpp에서 redefinition에러 발생함.)
	CRtStepClassPrePickInspStd();

	// CRtStepClass Overrides
	// 2. 부모의 순수 가상 함수 선언
	virtual eStatus	Configure();
	virtual eStatus	UnConfigure();
};



//-----------------------------------------
// CLASS	CStepPbiStd		Declaration
//-----------------------------------------
//This class implements the Fly head place operation for the CSeqCycle. 
//It is is one of the 5 main steps (dump, nozzle change, pick, align, place) 
//  used by CSeqCycle.
//CStepPickTape supports the common CStep interface for use by CSeqCycle.
//Currently this class handles the Fly head.

class CStepPrePickInspStd : public CStepPrePickInsp
{
private:
	CHead* m_pHeads[_countof(((StCycleData*)(0))->placeData)];

	long			m_Cycle;				// RUN	Cycle number
	long	m_MapHeadToHeadData[SYS_MAX_HEAD + 1];// CFG	Maps a head unit number to a HeadData index;

	struct HeadData
	{
		bool		bUse;					// RUN	Indicates this head will be processed by this step in this cycle
		bool		bCounted;				// RUN	Indicates this head was interval counted in this cycle
		eStatus		status;					// RUN	eOK=this head can be scanned, eNG=do not scan
		HeadID		headID;
		CHead* pHead;
		CCamera* pCameraUp;				// RUN	Camera performing the UP corner finding
		CCameraDown* pCameraDn;				// RUN	Camera performing the DN corner finding

		//		int			scanGroup;				// RUN  Group number for heads that will be scanned together.
		CProfile	profile;				// RUN  Profile of the model
		//		StPlaceData	pd;						// RUN	Cached placement data for use by MANUAL PBI

		//		long		headScanR;				// RUN	(degrees) Desired head angle at which the part should be scanned.(INFO ONLY)
		//		long		partScanR;				// RUN	(degrees) Desired Angle at which the part should be scanned.
		//		long		placeR;					// RUN	(degrees) Placement Angle, which is also the scan angle when "bUsePlaceR" is true
		//		long		partVectorR;			// RUN	(degrees) Vector from the center of the part to the nozzle.
		//		CFxyzt		partVector;				// RUN	Vector from the center of the part to the nozzle.
		//											//		Value is in Part Coordinates.
		CFxyt		visCornerVector[MAX_CORNER_DATA];	// RSLT	Vectors from part found position to each corner (in image coordinates)

		//		ePbiMethod	visMethod;				// RUN	UP Camera Inspection method: Whole Body Find  OR  Corner Finding
		//StVisBody	visBody;				// Body Finding Data
		long		nCorners;
		//StVisCorner	visCorner[MAX_CORNER_DATA + D_MAX_VERIFY_WINDOWS_NUM];	// Corner Finding Data

		//		long		visModelID;				// RUN  ModelID for model to find
		//		CFxyt		visExpectedPos;			// RUN  Expected position of the model in global coordinates
		CFxyt		visUncertainty;			// RUN  Uncertainty of the model in global image coordinates
		long		visFindID;				// RSLT Model find ID for the find request
		CFxyt		visModelVector;			// RSLT Result from the vision find request
		long		visConfidenceFound;		// RSLT Output of Find command: Confidence level (-1 to 100). Only supported for Fiducial model types. This value is set to -1 for other model types.
		eStatus		visCmdStatus;			// RSLT Non-process result status of the most recent vision operation
		eStatus		visFindStatus;			// RSLT Process status of the most recent vision operation
		//		NOTE: This is a simple interpretation of visResult[] below.
		long		visResult;				// RSLT VISION SPECIFIC Result status of the most recent vision operation
		ePartFailReason	visFailureClass;		// RSLT VISION SPECIFIC Result status of the most recent vision operation
		CFxyt		resultL;				// RSLT [ab] Result vector from the found model position to the 
		//             calibrated head position
		//ePbiFind	resultRunCode;			// RSLT Runtime action that should be taken as a result of the find operation.
	};

	long			m_nHeads;
	HeadData		m_HeadData[SYS_MAX_HEAD_PER_GANTRY + 1];
	CFixedArrayList< StReportError, SYS_MAX_HEAD_PER_GANTRY* CSeqCycle::eMAX_EXE_STEPS >	m_SavedReportError;

public:
	CStepPrePickInspStd( );
	// Assigns the selected heads to this step. 
	// 3. 생성자 선언 (이게 없으면 배열 생성 시, 에러 발생.)
	void					Config(GantryID aGantryID);
	// Indicates this CStepPbiStd is not used.
	void					UnConfig();
	bool					IsHeadData(HeadID aHeadID) const;

private:


	void					ResetPlanning();
	long					GetWorkSheetIndex(CPCBoard* apBrd, const TPlaceCadID& aPlaceId);

	// Returns a reference to m_HeadData by HeadID.
	const HeadData&			GetHeadData(HeadID aHeadID) const;
	HeadData&				GetHeadData(HeadID aHeadID);
	long					GetHeadDataIdx(HeadID aHeadID) const;
	//	eCycleStatus			ExecuteFindBody		( StCycleData& arCycleData );

	const StPlaceWs* GetPlacementData	(
								const CPlacementId& aPlacementId,
								const CStationId& aStationId) const;

	// CStepPbi Overrides
	virtual eCycleStatus	AssignHeads		(StCycleData& arCycleData);

	virtual eManPbiStatus	ManExecute(long& arConfidence,
									CFxyt& arPartAdjust,
									long		aSysCameraID,
									bool		abDrawOutline,
									const CFxyt& aExpectedPartPos,
									const CFxy& axyUncertainty,
									const CPlacementId& aPlacementId,
									const CProfile& aProfile,
									CStationId		aStationId,
									long& aThreshold,
									long		aCornerIdx = 0);

	// CStep Overrides
	virtual CHead*			GetFirstHead();
	virtual void			GetFirstMove(CFxyzt& aNextTarget, HeadID aHeadID);

	// NOTE: m_xyMoveOpt is also output from this function. 
	// NOTE: m_PlaceLoc   [ aIdxPlace] is also an OUTPUT of ExecuteFirstMoveXY
	// NOTE: m_PlaceLocGan[ aIdxPlace] is also an OUTPUT of ExecuteFirstMoveXY
	virtual eStatus			ExecuteFirstMoveXY(RQTag& aXyReq, ePartVector& aPvStatus, CFxyzt& aFirstMove, const StCycleData& arCycleData, long aPlaceDataIdx);

	virtual eStatus			GetFirstNearlyDoneZ(double& aEscHeight, eMoveOpt& aMoveOpt, HeadID aHeadID);
	virtual void			GetPlaceDataOrder(TPDOrder& aPlaceDataOrder, const StCycleData& arCycleData);
	virtual eCycleStatus	PlanStep(StCycleData& arCycleData);
	virtual eCycleStatus	PlanRegions(StCycleData& arCycleData);
	virtual eCycleStatus	Execute(StCycleData& arCycleData);
	virtual eCycleStatus	PostExecute(StCycleData& arCycleData, long aCyDataIndex);
	virtual eCycleStatus	PostCycleExecute(StCycleData& arCycleData);

	// Private Functions for Saved Upcamera Corner Data.

};
#endif //__STEP_PPI_STD_H__