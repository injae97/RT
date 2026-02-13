////////////////////////////////////////////////////////////////////////////////
// 
//		File Version History
//			V0.1 : 2026-02-xx First Release
// 
// NOTE: This class only exists as a BASE CLASS for the actual derived class PrePickInsp steps.
//		 Otherwise, only static function exist to configure PrePickInsp steps into a cycle.
// 
// PPI(= Pre PickUp Inspection / = Pre Pick Insp)
// 클래스 역할 : 모든 PPI step(표준 PPI 또는 향후 추가 PPI 구현)의 공통 인터페이스와 관리 로직을 제공한다.
// 


#ifndef __STEP_PRE_PICK_INSP_H__
#define __STEP_PRE_PICK_INSP_H__

#include "Step.h"

class CStepPrePickInsp;

// 실제 메모리 할당은 한곳에서만 이루어지고 extern을 쓴곳은 메모리 주소를 참조만 한다. "데이터 단일성"을 보장한다.
// CFixedArrayList에 포인터(CStepPrePickInsp*)를 담고 있는데, 이는 각 검사 단계의 상세 정보(파라미터, 결과 등)를 동적으로 관리하겠다는 뜻이다.
// extern이 붙었으므로 시퀀스 제어부, UI부 등 어디서든 이 리스트에 접근해 검사 상태를 접근할 수 있다. Ex)SeqCycle.h : g_SeqCycle..
extern CFixedArrayList< CStepPrePickInsp*, MAX_STEP_PRE_PICK_INSP>	g_StepPrePickInspList;

// Static 선언 : Static을 쓰는 이유는 전체 시스템(또는 파일) 수준에서 하나만 존재해야하는 데이터/동작을 표현하기 위함이다. (중복 생성 방지)
//				객체 생성없이 클래스 이름만으로 즉시 호출 가능하므로 Real Time Operating System에서 메모리 관리에 유리한 구조이다.
//				(외부 파일에서 이 변수 이름을 호출해도 보이지 않음 + 데이터 오염 방지)
static long g_PpiIntervalRemaining[eBOTH][MAX_STEP_PRE_PICK_INSP];



class CStepPrePickInsp : public CStep
{
public:
	//1. 생성자를 선언해주지 않으면, .cpp에서 Redefinition에러 발생
	CStepPrePickInsp( )		{ m_TypePrimary=eSTEP_PRE_PICK_INSP; }
	~CStepPrePickInsp( )	{ m_TypePrimary=eSTEP_NONE; }

	static void	AddInstalledStep		( CStepPrePickInsp* apStep );
	static void	RemoveInstalledStep		( CStepPrePickInsp* apStep );

	// Get an installed step associated with the specified gantry and step type (NOT PRIMARY TYPE);
	static CStepPrePickInsp*		GetStepPrePickInsp		( GantryID aGantryID, eStepType aStepType );

	//Assigns the correct heads to each CStepPrePickInspXXX object in aStepList. Assigns
	//  a head by examining the placements in arCycleData.
	//Outputs:
	//	arCycleData - Indicates which heads have been handled for the pick step so far. 
	static eCycleStatus		AssignHeadsPrePickInsp( StCycleData& arCycleData );


	static void				OrderSteps		( int aCount, CStep** ppStepPpi);

	// For each head assigns arCycleData.placeData[].pStep[eSTEP_PBI] to the CStepPrePickInspXxx for that head.
	virtual eCycleStatus	AssignHeads		( StCycleData& arCycleData )=0;

	virtual eManPbiStatus	ManExecute			(	long&		arConfidence,
													CFxyt&		arPartAdjust,										
													long		aSysCameraID,
													bool		abDrawOutline,
												const CFxyt&	aExpectedPartPos,
												const CFxy&		axyUncertainty,
										const CPlacementId&		aPlacementId,
											const CProfile&		aProfile,
												CStationId		aStationId,
													long&		aThreshold,
													long		cornerIdx)=0;

protected:

	// Use a HeadID() as an index to return the CStepPlace derived object
	// assigned to that CHead.
	static CStepPrePickInsp*		m_HeadToPrePickInspStep[ SYS_MAX_HEAD +1 ];};

#endif // __STEP_PRE_PICK_INSP_H__

