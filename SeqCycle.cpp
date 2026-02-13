////////////////////////////////////////////////////////////////////////////////
//			Copyright (c) 2007 Samsung TechWin Co., Ltd.
//					All rights reserved.
//
//	This source code contains confidential, trade secret material of
//	Samsung TechWin. Any attempt or participation in deciphering, decoding,
//	reverse engineering or in any way altering the source code is
//	strictly prohibited, unless the prior written consent of Samsung TechWin
//	is obtained.
//
//	File Version History
//		V0.1 : 2007-xx-xx First Release
//
//

#include "stdafx.h"
#include <math.h>
#include <float.h>

#include "IlluminatorSideControl.h"
#include "SeqCycle.h"
#include "Step.h"
#include "StepAlignCameraLso.h"
#include "StepDump.h"
#include "StepVacStuff.h"
#include "StepAncPutPreInsp.h"
#include "StepAncPutPostInsp.h"
#include "StepAncGetPreInsp.h"
#include "StepAncGetPostInsp.h"
#include "StepAncGetPostNozTypeInsp.h"
#include "StepAncPut.h"
#include "StepAncGet.h"
#include "StepRetryFeeding.h"
#include "StepRetryFeedingStd.h"
#include "StepPocketXy.h"
#include "StepPocketXyStd.h"
#include "StepPrePickInsp.h"
#include "StepPick.h"
#include "StepPickStickFly.h"
#include "StepPickTapeFly.h"
#include "StepPickTray.h"
//#include "StepPickTrayFly.h"
#include "StepPickTrayFeeder.h"
#include "StepPickTrayFeederFly.h"
//#include "StepFlux.h"
#include "StepPreFlux.h"
#include "StepPostFlux.h"

#include "StepLeadCheck.h"

#include "..\HeadAP\StepCycleOptimize.h"

#include "StepPlace.h"
#include "StepPbi.h"
#include "StepSideView.h"
#include "ManualSmt.h"
#include "DumpStop.h"
#include "ZHeightMap.h"
#include "ZHeightMapSMT.h"
#include "SeqEngine.h"
#include "StageCameras.h"
#include "FlyCameras.h"
#include "UpCameras.h"
#include "CameraUp.h"
#include "ThermalMapping.h"
#include "FactoryDebug.h"
#include "FactoryConst.h"
#include "SystemConst.h"
#include "Buzzer.h"
#include "DumpStop.h"
#include "RunOptions.h"
#include "RunTimeBackUp.h"
#include "AutoHeadCleanMgr.h"
#include "Feederbase.h"
#include "Conveyor.h"
#include "Anc.h"
#include "Tray.h"
#include "StickFeeder.h"
#include "PcbSetFunc.h"
#include "GlobalSmtFunctions.h"
#include "ApcMgr.h"

#include "HeadAP\StepPickTapeAP.h"
#include "headpmf\StepPickTapePmf.h"

#undef THIS_FILE
static char THIS_FILE[] = __FILE__;


CSeqCycle	g_SeqCycle[ MAX_SEQ_CYCLE+1];


bool	g_bUseOldPickRScanRMethod = false;

TStatIncRecList	CSeqCycle::m_StatisticsIncRecTape	[ eMAX_PCB_FEEDER_BASE	+1][ eMAX_PCB_TAPE	+1];
TStatIncRecList	CSeqCycle::m_StatisticsIncRecStick	[ eMAX_PCB_STICKUNIT	+1][ eMAX_PCB_STICK	+1];
TStatIncRecList	CSeqCycle::m_StatisticsIncRecTray	[ eMAX_PCB_TRAYUNIT		+1][ eMAX_PCB_TRAY	+1][eMAX_PCB_TRAY_POSITIONS	+1];

long			CSeqCycle::m_StatisticsMispicksTape	[ eMAX_PCB_FEEDER_BASE	+1][ eMAX_PCB_TAPE	+1];
long			CSeqCycle::m_StatisticsMispicksStick[ eMAX_PCB_STICKUNIT	+1][ eMAX_PCB_STICK	+1];
long			CSeqCycle::m_StatisticsMispicksTray	[ eMAX_PCB_TRAYUNIT		+1][ eMAX_PCB_TRAY	+1][eMAX_PCB_TRAY_POSITIONS	+1];

long			CSeqCycle::m_PickUpMissCountTape	[ eMAX_PCB_FEEDER_BASE	+1][ eMAX_PCB_TAPE	+1];  	// PickUp miss count without vision error
long			CSeqCycle::m_PickUpMissCountStick	[ eMAX_PCB_STICKUNIT	+1][ eMAX_PCB_STICK +1];
long			CSeqCycle::m_PickUpMissCountTray	[ eMAX_PCB_TRAYUNIT		+1][ eMAX_PCB_TRAY	+1][eMAX_PCB_TRAY_POSITIONS +1];

long			CSeqCycle::m_PartNGCountTape		[ eMAX_PCB_FEEDER_BASE	+1][ eMAX_PCB_TAPE	+1];	// PickUp miss count witho only vision error
long			CSeqCycle::m_PartNGCountStick		[ eMAX_PCB_STICKUNIT	+1][ eMAX_PCB_STICK +1];
long			CSeqCycle::m_PartNGCountTray		[ eMAX_PCB_TRAYUNIT		+1][ eMAX_PCB_TRAY	+1][eMAX_PCB_TRAY_POSITIONS +1];

bool			CSeqCycle::m_NonStopRetryTape			[ eMAX_PCB_FEEDER_BASE	+1][ eMAX_PCB_TAPE	+1];

int				CSeqCycle::m_PickUpTapeCount			[ eMAX_PCB_FEEDER_BASE	+1][ eMAX_PCB_TAPE	+1];
ePartFailReason CSeqCycle::m_PickUpMissTapeRateCount	[ eMAX_PCB_FEEDER_BASE	+1][ eMAX_PCB_TAPE	+1][eMAX_PICK_RATE_COUNT];
bool			CSeqCycle::m_bPickUpMissTapePocketTeach [ eMAX_PCB_FEEDER_BASE	+1][ eMAX_PCB_TAPE	+1];
int				CSeqCycle::m_PickUpMissTapeRateNextIndex[ eMAX_PCB_FEEDER_BASE	+1][ eMAX_PCB_TAPE	+1];
int				CSeqCycle::m_PickUpMissTapeRateCheckIndex[ eMAX_PCB_FEEDER_BASE	+1][ eMAX_PCB_TAPE	+1];
int				CSeqCycle::m_PickUpMissTapeRateCheckIndex2[ eMAX_PCB_FEEDER_BASE	+1][ eMAX_PCB_TAPE	+1];

bool			CSeqCycle::m_bUseNegativePick = true;

//Cycle selection priority.
//	First one('eCYCLE_TYPE_PP_SS') is highest priority to be chosen.
eCycleType g_CycleTypeTable[] =
{
    eCYCLE_TYPE_PP_SS        ,          
    eCYCLE_TYPE_PP_S         ,  
    eCYCLE_TYPE_PP_M         ,  
    eCYCLE_TYPE_PP_L         ,  
    eCYCLE_TYPE_FLY_SS       ,  
    eCYCLE_TYPE_FLY_S        ,  
    eCYCLE_TYPE_FLY_M        ,  
    eCYCLE_TYPE_FIX_SS       ,  
    eCYCLE_TYPE_FIX_S        ,  
    eCYCLE_TYPE_FIX_M        ,  
    eCYCLE_TYPE_FIX_L        , 
    eCYCLE_TYPE_FLY_SIDE_SS  ,  
    eCYCLE_TYPE_FLY_SIDE_S   ,
	eCYCLE_TYPE_LSO_SS		 ,	
	eCYCLE_TYPE_LSO_S		 ,	
	eCYCLE_TYPE_LSO_M		 ,	
	eCYCLE_TYPE_LSO_L		 ,
    eCYCLE_TYPE_NONE
};


const char*
g_StatEraseReasonText[] =
{
	"None",				// eSTAT_ERASE_NONE (should never occur)
	"FdrErr|PartEmpty",	// eSTAT_ERASE_ERROR_OR_IT_EMPTY
	"~Splice&RetryEx",	// eSTAT_ERASE_NOTSPLICEAREA_RETRYEXCEEDED
	"StartRetryFeed",	// eSTAT_ERASE_STARTRETRYFEEDING
	"Max"				// eSTAT_ERASE_MAX (should never occur)
};

// Keep the table above up to date with eStatEraseReason
CompileTimeAssert( _countof(g_StatEraseReasonText)==(CSeqCycle::eSTAT_ERASE_MAX+1), SeqCycleStatEraseReasonTextTableNeedsUpdate);


const char*
g_PartFailReasonText[] =
{
	"None",			// ePART_FAIL_NONE
	"VacPick",		// ePART_FAIL_VAC_PICK
	"Vision",		// ePART_FAIL_VISION
	"VisShift",		// ePART_FAIL_VIS_SHIFT_PICK
	"SvsPick",		// ePART_FAIL_SVS_PICK
	"SvsVision",	// ePART_FAIL_SVS_VISION
	"VacPrePlace",	// ePART_FAIL_VAC_PRE_PLACE
	"FailForce",	// ePART_FAIL_FORCE_PLACE
	"FailSvs",		// ePART_FAIL_SVS
	"LeadScan",		// ePART_FAIL_LEAD_SCAN
	"LcrOK",		// ePART_FAIL_LCR_OK
	"LcrNG",		// ePART_FAIL_LCR_NG
	"SkipPick",		// ePART_FAIL_SKIP_PICK
	"Max"			// ePART_FAIL_MAX
};

// Keep the table above up to date with eStatEraseReason
CompileTimeAssert( _countof(g_PartFailReasonText)==(ePART_FAIL_MAX+1), SeqCyclePartFailReasonTextTableNeedsUpdate);



CSeqCycle::CSeqCycle() : m_SectionID( 1)
{
	int i;
	int j;
	m_bDualRegion = false;
	m_bTrayStatusUpdated = false;
	m_GantryID = 0;
	m_pGantry = NULL;
	m_ReturnStatus = eCYCLE_OK;
	m_EventFindTrigger.Reset( -1);
	m_EventFindResults.Reset( -1);
	m_EventFind3DResults.Reset( -1);
	m_CycleSpeedPickUpZ = eHSPEED_NONE;
	m_CycleSpeedXY = eHSPEED_NONE;
	m_CycleType = eCYCLE_TYPE_NONE;
	m_frRunningLane = eFRONT;
	m_PrefetchOp = eCYCLE_PREFETCH_NONE;
	m_PrevPrefetchOp = eCYCLE_PREFETCH_NONE;
	memset( m_Steps, 0, sizeof(m_Steps));
	for ( i=0; i<_countof(m_TrayFeederBufferChg); i++)	m_TrayFeederBufferChg[i] = eTRAY_BUFCHG_YES;
	memset( m_TrayFeederNowPallet , 0, sizeof(m_TrayFeederNowPallet ));
	memset( m_TrayFeederNxtPallet , 0, sizeof(m_TrayFeederNxtPallet ));
	m_LastStepType = eSTEP_NONE;
	m_bCycleUseHRS = 0;
	m_bFastCycleBk = false;
	m_bFineGrab = false;
	m_bFineXY = false;
	m_bFirstCycleHalfPalletOut = false;
	m_bPostCycleHalfPalletOut = false;
	m_b2ndStepPalletOut = false;
	m_bBarcodeCycle		= false;	

	for (i = 0; i<_countof(m_bLastTrayCycle); i++)	m_bLastTrayCycle[i] = false;

	m_nSteps = 0;

	for (i=0; i<_countof(m_PickUpMissCountTape);  i++) 
	{
		for (j=0; j<_countof(m_PickUpMissCountTape[0]);  j++) 
		{ 
			m_PickUpMissCountTape[i][j] = 0;
		}
	}

	for (i=0; i<_countof(m_NonStopRetryTape);  i++) 
	{
		for (j=0; j<_countof(m_NonStopRetryTape[0]);  j++) 
		{ 
			m_NonStopRetryTape[i][j] = false;
		}
	}
}


CSeqCycle::~CSeqCycle()
{

}


eStatus
CSeqCycle::NotifyFeederPlace( int aPlaceDataIdx, CFeederId aActualFeederId )
{
	return eOK;
}


void
CSeqCycle::ClearAllErrors( SectionID aSectionID, eFrontRear aFR )
{
	aFR = g_StateReport.GetIndependentLaneBOTH( aFR);

	int i;
	int j;
	int k;
	SectionID	sectionID;
	eFrontRear	fr;

	// Remove all statistics increment records
	for (i=0; i<_countof(m_StatisticsIncRecTape);  i++) 
	{
		CFeederbase* pBase = ::GetFeederbase( i );
		if ( pBase == NULL)					continue;
		sectionID	= pBase->GetSectionID();
		fr			= pBase->GetFrontOrRear();

		if ( aSectionID != ALL_SECTIONS	&&   sectionID != aSectionID)	continue;
		if (		aFR != eBOTH		&&			fr != aFR)			continue;

		for (j=0; j<_countof(m_StatisticsIncRecTape[0]);  j++) 
			m_StatisticsIncRecTape[i][j].RemoveAll();
	}

	for (i=0; i<_countof(m_StatisticsIncRecStick); i++) 
	{
		for (j=0; j<_countof(m_StatisticsIncRecStick[0]); j++) 
		{
			CStickFeeder& stick = g_StickFeeder[i][j];
			if ( stick.IsInstalled() )
			{
				CFeederbase* pBase = ::GetFeederbase( stick.GetBaseID() );
				if ( pBase == NULL)					continue;

				sectionID	= pBase->GetSectionID();
				fr			= pBase->GetFrontOrRear();

				if ( aSectionID != ALL_SECTIONS	&&   sectionID != aSectionID)	continue;
				if (		aFR != eBOTH		&&			fr != aFR)			continue;
			}
			
			m_StatisticsIncRecStick[i][j].RemoveAll();
		}
	}

	for (i=0; i<_countof(m_StatisticsIncRecTray);  i++)
	{
		for (j=0; j<_countof(m_StatisticsIncRecTray[0]);  j++) 
		{
			for (k=0; k<_countof(m_StatisticsIncRecTray[0][0]); k++) 
			{
				CTray& tray = g_Tray[i][j][k];
				if ( tray.IsInstalled() )
				{
					CTrayFeeder* pTrayFeeder = ::GetTrayFeeder( tray.GetID() );
					if ( pTrayFeeder == NULL)			continue;

					sectionID	= pTrayFeeder->GetSectionID();
					fr			= pTrayFeeder->GetFrontOrRear();

					if ( aSectionID != ALL_SECTIONS	&&   sectionID != aSectionID)	continue;
					if (		aFR != eBOTH		&&			fr != aFR)			continue;
				}
				m_StatisticsIncRecTray[i][j][k].RemoveAll();
			}
		}
	}

	// Zero all mispick counters
	for (i=0; i<_countof(m_StatisticsMispicksTape);  i++) 
	{
		CFeederbase* pBase = ::GetFeederbase( i );
		if ( pBase == NULL)					continue;

		sectionID	= pBase->GetSectionID();
		fr			= pBase->GetFrontOrRear();

		if ( aSectionID!=ALL_SECTIONS  &&  sectionID != aSectionID )		continue;
		if ( 			   aFR!=eBOTH  &&  fr != aFR)						continue;

		for (j=0; j<_countof(m_StatisticsMispicksTape[0]);  j++) 
		{
			m_StatisticsMispicksTape[i][j] = 0;
			m_PickUpMissCountTape[i][j] = 0;
			m_PartNGCountTape[i][j] = 0;
		}
	}

	for (i = 0; i < _countof(m_StatisticsMispicksStick); i++)
	{
		for (j = 0; j < _countof(m_StatisticsMispicksStick[0]); j++)
		{
			CStickFeeder& stick = g_StickFeeder[i][j];
			if (stick.IsInstalled())
			{
				CFeederbase* pBase = ::GetFeederbase(stick.GetBaseID());
				if (pBase == NULL)					continue;

				sectionID = pBase->GetSectionID();
				fr = pBase->GetFrontOrRear();

				if (aSectionID != ALL_SECTIONS  &&  sectionID != aSectionID)		continue;
				if (aFR != eBOTH  &&  fr != aFR)						continue;
			}

			m_StatisticsMispicksStick[i][j] = 0;
			m_PickUpMissCountStick[i][j] = 0;
			m_PartNGCountStick[i][j] = 0;
		}
	}

	for (i = 0; i < _countof(m_StatisticsMispicksTray); i++)
	{
		for (j = 0; j < _countof(m_StatisticsMispicksTray[0]); j++)
		{
			for (k = 0; k < _countof(m_StatisticsMispicksTray[0][0]); k++)
			{
				CTray& tray = g_Tray[i][j][k];
				if (tray.IsInstalled())
				{
					CTrayFeeder* pTrayFeeder = ::GetTrayFeeder(tray.GetID());
					if (pTrayFeeder == NULL)			continue;

					sectionID = pTrayFeeder->GetSectionID();
					fr = pTrayFeeder->GetFrontOrRear();

					if (aSectionID != ALL_SECTIONS  &&  sectionID != aSectionID)		continue;
					if (fr != eBOTH && aFR != eBOTH && fr != aFR)						continue;
				}
				m_StatisticsMispicksTray[i][j][k] = 0;
				m_PickUpMissCountTray[i][j][k] = 0;
				m_PartNGCountTray[i][j][k] = 0;
			}
		}
	}

	for (i=0; i<_countof(m_NonStopRetryTape);  i++) 
	{
		for (j=0; j<_countof(m_NonStopRetryTape[0]);  j++) 
		{ 
			m_NonStopRetryTape[i][j] = false;
		}
	}

	for (i=0; i<_countof(m_PickUpTapeCount);  i++) 
	{
		CFeederbase* pBase = ::GetFeederbase( i );
		if ( pBase == NULL)					continue;
		sectionID	= pBase->GetSectionID();
		fr			= pBase->GetFrontOrRear();
		
		if ( aSectionID != ALL_SECTIONS	&&   sectionID != aSectionID)	continue;
		if (		aFR != eBOTH		&&			fr != aFR)			continue;
		
		for (j=0; j<_countof(m_PickUpTapeCount[0]);  j++) 
		{ 
			m_PickUpTapeCount[i][j] = 0;
		}
	}

	for (i=0; i<_countof(m_PickUpMissTapeRateCount);  i++) 
	{
		CFeederbase* pBase = ::GetFeederbase( i );
		if ( pBase == NULL)					continue;
		sectionID	= pBase->GetSectionID();
		fr			= pBase->GetFrontOrRear();
		
		if ( aSectionID != ALL_SECTIONS	&&   sectionID != aSectionID)	continue;
		if (		aFR != eBOTH		&&			fr != aFR)			continue;
		
		for (j=0; j<_countof(m_PickUpMissTapeRateCount[0]);  j++) 
		{ 
			for (k=0; k<_countof(m_PickUpMissTapeRateCount[0][0]); k++) 
			{
				m_PickUpMissTapeRateCount[i][j][k] = ePART_FAIL_NONE;
			}
		}
	}

	for (i=0; i<_countof(m_bPickUpMissTapePocketTeach);  i++) 
	{
		CFeederbase* pBase = ::GetFeederbase( i );
		if ( pBase == NULL)					continue;
		sectionID	= pBase->GetSectionID();
		fr			= pBase->GetFrontOrRear();
		
		if ( aSectionID != ALL_SECTIONS	&&   sectionID != aSectionID)	continue;
		if (		aFR != eBOTH		&&			fr != aFR)			continue;

		for (j=0; j<_countof(m_bPickUpMissTapePocketTeach[0]);  j++) 
		{ 
			m_bPickUpMissTapePocketTeach[i][j] = false;
		}
	}

	for (i=0; i<_countof(m_PickUpMissTapeRateNextIndex);  i++) 
	{
		CFeederbase* pBase = ::GetFeederbase( i );
		if ( pBase == NULL)					continue;
		sectionID	= pBase->GetSectionID();
		fr			= pBase->GetFrontOrRear();
		
		if ( aSectionID != ALL_SECTIONS	&&   sectionID != aSectionID)	continue;
		if (		aFR != eBOTH		&&			fr != aFR)			continue;

		for (j=0; j<_countof(m_PickUpMissTapeRateNextIndex[0]);  j++) 
		{ 
			m_PickUpMissTapeRateNextIndex[i][j] = false;
		}
	}

	for (i=0; i<_countof(m_PickUpMissTapeRateCheckIndex);  i++) 
	{
		CFeederbase* pBase = ::GetFeederbase( i );
		if ( pBase == NULL)					continue;
		sectionID	= pBase->GetSectionID();
		fr			= pBase->GetFrontOrRear();
		
		if ( aSectionID != ALL_SECTIONS	&&   sectionID != aSectionID)	continue;
		if (		aFR != eBOTH		&&			fr != aFR)			continue;

		for (j=0; j<_countof(m_PickUpMissTapeRateCheckIndex[0]);  j++) 
		{ 
			m_PickUpMissTapeRateCheckIndex[i][j] = false;
		}
	}

	for (i=0; i<_countof(m_PickUpMissTapeRateCheckIndex2);  i++) 
	{
		CFeederbase* pBase = ::GetFeederbase( i );
		if ( pBase == NULL)					continue;
		sectionID	= pBase->GetSectionID();
		fr			= pBase->GetFrontOrRear();
		
		if ( aSectionID != ALL_SECTIONS	&&   sectionID != aSectionID)	continue;
		if (		aFR != eBOTH		&&			fr != aFR)			continue;

		for (j=0; j<_countof(m_PickUpMissTapeRateCheckIndex2[0]);  j++) 
		{ 
			m_PickUpMissTapeRateCheckIndex2[i][j] = false;
		}
	}
}


void
CSeqCycle::Config( CGantry* apGantry, bool abDualRegion )
{
	(SectionID&)m_SectionID = apGantry->GetSectionID();
	m_frRunningLane = apGantry->GetFrontOrRear();
	m_pGantry = apGantry;
	m_GantryID = m_pGantry->Unit();
	m_bDualRegion = abDualRegion;
	m_PrevPrefetchOp = eCYCLE_PREFETCH_NONE;
	
//#pragma message( "For debug: Force m_bDualRegion false" )
//	m_bDualRegion = false;

	PostSVSResult( 0xFF );
	PostPlaceSVSBegin( 0xFF);
}


void
CSeqCycle::ResetPlaceData( )
{
	m_CycleData.ancPutData.RemoveAll();
	m_CycleData.ancGetData.RemoveAll();
	m_CycleData.dumpData.RemoveAll();
	m_CycleData.nCount = 0;
	m_CycleData.nCountOrg = 0;
	m_CycleData.cycle  = 0;
	m_CycleData.pPcbBoard = NULL;
	m_CycleData.bPickupVerifyOnly = false;
	m_CycleData.bVirtualPick = false;
	for (int i=0; i<_countof(m_CycleData.headIdToPlaceDataIdx); i++) m_CycleData.headIdToPlaceDataIdx[i] = NOT_AN_INDEX;
	m_CycleData.stationId.Undefine();

	m_CycleData.nextCycleData.RemoveAll();

	m_CycleData.bCycleStepOptimized = false;

}


void
CSeqCycle::SetManualPlaceData( const TPlaceWsList& aPlaceDataArray, eCyclePrefetchOp aPrefetchOp, CManualSmt* apManual)
{
	m_PrefetchOp = aPrefetchOp;
	m_bFineXY = false;
	m_bFineGrab = false;
	m_bCycleUseHRS = false;	// do NOT use HRS for manual command
	m_CycleSpeedXY = eHSPEED_FASTEST;
	m_CycleSpeedPickUpZ = eHSPEED_FASTEST;

	long cycle = 0;
	for (int i=0; i<aPlaceDataArray.GetCount() ; i++)
	{

		StPlaceWs& placeWs = *aPlaceDataArray[i].m_pPlaceWs;

		StPlaceData& cyclePlaceData = m_CycleData.placeData[i];

		// Ignore placements with bad data
		//if ( placeWs.placeCadID >= g_pPcbs[n]->placeCad.GetCount())
		//	continue;
		
		//const StPlaceCad& pcbPlaceCad = g_pPcbs[n]->GetPlaceCad( placeWs.placeCadID );

		//if ( pcbPlaceCad.partID >= g_pTable->parts.GetCount())
		//	continue;

		//if ( pcbPlaceCad.arrayNo >= g_pPcbs[n]->arrays.GetCount())
		//	continue;

		//if ( pcbPlaceCad.placeFidID >= g_pPcbs[n]->fidLocalMaps.GetCount())
		//	continue;

		if ( placeWs.head >= _countof(g_Head))
			continue;

		// Sets cycle number of each step. Assume every step has same cycle number.
		cycle								= aPlaceDataArray[i].m_Cycle;

		cyclePlaceData.status				= ePLACE_OK;
		cyclePlaceData.bStoppedByDumpStop	= false;

		cyclePlaceData.placeCadID			= placeWs.placeCadID;
		cyclePlaceData.placeMapID			= placeWs.placeMapID;
		cyclePlaceData.cycle				= cycle;
		cyclePlaceData.pickOrder			= placeWs.pickOrder;
		cyclePlaceData.feederID				= placeWs.feederID;
		cyclePlaceData.pickedFeederID		= CFeederId(0);
		cyclePlaceData.placedFeederID		= CFeederId(0);

		//<2011.1.31>
		//Initialize picked pocket index
		cyclePlaceData.mPocketPickedIndex	= -1;

		cyclePlaceData.bAutoFeeder			= false;
		cyclePlaceData.head					= placeWs.head;
		cyclePlaceData.correctPartVector.Undefine();
		cyclePlaceData.correctModelVector.Undefine();

		m_CycleData.headIdToPlaceDataIdx[ placeWs.head] = i;

		cyclePlaceData.pHead				= &g_Head[ placeWs.head];
											
		cyclePlaceData.arrayId.blockNo		= NOT_AN_INDEX;
		cyclePlaceData.arrayId.arrayNo		= aPlaceDataArray[i].m_ArrayNo;
		//cyclePlaceData.placeFidID			= pcbPlaceCad.placeFidID;

		// NOTE : parameter need
		//const StPart& pcbPart				= g_pTable->GetPart( pcbPlaceCad.partID );
		
		const StPart& pcbPart				= apManual->GetPart();

		// TODO : define this
		//cyclePlaceData.partID				= pcbPlaceCad.partID;

		// NOTE : Assign valid Part ID
		cyclePlaceData.partID				= pcbPart.partID;

		memset( cyclePlaceData.partName, 0, sizeof(cyclePlaceData.partName) );
		strncpy( cyclePlaceData.partName, pcbPart.partName, sizeof(cyclePlaceData.partName)-1);
		cyclePlaceData.profileID			= pcbPart.profileID;
		cyclePlaceData.bPolarized			= pcbPart.bPolarized!=0;

		// NOTE : parameter need
		//const StProfile& pcbProfile		= g_pTable->GetProfile( pcbPart.profileID);
		const CProfile& pcbProfile			= apManual->GetProfile( placeWs.head );
		cyclePlaceData.profile				= pcbProfile;

		cyclePlaceData.nozzleId				= placeWs.nozzleId;
		cyclePlaceData.nozzleType1			= pcbProfile.common.nozzleType1;
		cyclePlaceData.nozzleType2			= pcbProfile.common.nozzleType2;
		cyclePlaceData.hdNozzleId.Undefine();

		cyclePlaceData.placePos = cyclePlaceData.position;
//		if ( apPCBoard)
//			apPCBoard->MapPlacementToMachineCoord( cyclePlaceData.placePos, cyclePlaceData.arrayNo, cyclePlaceData.placeFidID );
//
		cyclePlaceData.pickPos.Undefine();

		// Cached/Calculated Information
		cyclePlaceData.pickInspR			= ePIR_PICK_ANY;
		cyclePlaceData.bFinePitchComp		= pcbProfile.IsFinePitchComp();
		cyclePlaceData.bFineGrabComp		= pcbProfile.IsFineGrabSize();
		cyclePlaceData.bFinalBacklash		= pcbProfile.IsFinalBacklashComp();
		cyclePlaceData.bSmallComp			= pcbProfile.IsSmallComp();
		cyclePlaceData.bVerySmallComp		= pcbProfile.IsVerySmallComp();
//		cyclePlaceData.bSoftPick			= pcbProfile.IsSoftPickComp( placeWs.feederID);

		if ( cyclePlaceData.bFinePitchComp)
			m_bFineXY = true;
		if ( cyclePlaceData.bFineGrabComp)
			m_bFineGrab = true;

		m_CycleSpeedXY		= max(m_CycleSpeedXY, pcbProfile.handling.speedLimitXY);
		m_CycleSpeedPickUpZ	= max(m_CycleSpeedPickUpZ, pcbProfile.handling.pickUpSpeedZ);

		for (int iStep=0; iStep<_countof(cyclePlaceData.pStep); iStep++) cyclePlaceData.pStep[iStep] = NULL;
	}

	m_CycleData.cycle	= cycle;
	m_CycleData.nCount = aPlaceDataArray.GetCount();
	m_CycleData.pCycleSeq = this;
	
	//m_CycleData.pPcbBoard = apPCBoard;

	// NOTE : parameter need
	//m_CycleData.stationId = aStation;
	m_CycleData.stationId = apManual->GetStationId();
	m_CycleData.bPickupVerifyOnly = true;
	m_CycleData.bVirtualPick = false;
}


StCycleData
CSeqCycle::GetManualCycleData( const TPlaceWsList& aPlaceDataArray, eCyclePrefetchOp aPrefetchOp, CStationId aStationID, CPCBoard* arpPCBoard)
{
	SetPlaceData( aPlaceDataArray, arpPCBoard, aStationID, aPrefetchOp );

	return m_CycleData;
}


void
CSeqCycle::SetPlaceData( const TPlaceWsList& aPlaceDataArray, CPCBoard* apPCBoard,
						const CStationId& aStation, eCyclePrefetchOp aPrefetchOp )
{
	bool	bMultiPcb = g_pPcbSet->IsMultiPcb(aStation.Lane());

	// Make sure there is only one Pcb choice
	SYS_ASSERT( apPCBoard!=NULL  ||  !bMultiPcb);

	m_frRunningLane = ::GetLaneFR( aStation.Lane());
	m_PrefetchOp = aPrefetchOp;
	m_bFineXY = false;
	m_bFineGrab = false;
	m_bCycleUseHRS = true;
	m_CycleSpeedXY = eHSPEED_FASTEST;
	m_CycleSpeedPickUpZ = eHSPEED_FASTEST;

	// start as normal pick
	m_CycleData.bVirtualPick = false;

	TPcbID		pcbID = (apPCBoard ? apPCBoard->GetPcbID() : g_pPcbSet->PreferPcbId( aStation.Lane()) );
	CPcbFunc*	pPcbFunc = g_pPcbs[ pcbID];

	long cycle = 0;
	int  nVirtualPick = 0;
	int  i = 0;
	for ( i=0; i<aPlaceDataArray.GetCount(); i++)
	{
		StPlaceWs& placeWs = *aPlaceDataArray[i].m_pPlaceWs;

		StPlaceData& cyclePlaceData = m_CycleData.placeData[i];
		const StPlaceCad& pcbPlaceCad = pPcbFunc->GetPlaceCad( placeWs.placeCadID );

		// Ignore placements with...
		// 1) Out of range CAD ID
		// 2) Out of range PART ID
		// 3) Out of range ARRAY NO
		// 4) Out of range PLACE FID ID
		// 5) Out of range HEAD ID
		if ( placeWs.placeCadID >= pPcbFunc->placeCad.GetCount())			continue;
//		if ( pcbPlaceCad.partID >= g_pTable->parts.GetCount())				continue;
		if ( !g_pTable->parts.IsExist( pcbPlaceCad.partID))					continue;
		if ( aPlaceDataArray[i].m_ArrayNo >= pPcbFunc->GetArrayCnt( pcbPlaceCad.blockNo ))	continue;
		if ( pcbPlaceCad.placeFidID >= pPcbFunc->fidLocalMaps.GetCount())	continue;
		if ( placeWs.head >= _countof(g_Head))								continue;

		// Sets cycle number of each step. Assume every step has same cycle number.
		cycle								= aPlaceDataArray[i].m_Cycle;

		cyclePlaceData.status				= ePLACE_OK;
		cyclePlaceData.bStoppedByDumpStop	= false;
		cyclePlaceData.placeCadID			= placeWs.placeCadID;
		cyclePlaceData.placeMapID			= placeWs.placeMapID;
		cyclePlaceData.cycle				= cycle;
		cyclePlaceData.pickOrder			= placeWs.pickOrder;
		cyclePlaceData.feederID				= placeWs.feederID;
		cyclePlaceData.pickedFeederID		= CFeederId(0);
		cyclePlaceData.placedFeederID		= CFeederId(0);

		//<2011.1.31>
		//Initialize picked pocket index
		cyclePlaceData.mPocketPickedIndex	= -1;

		cyclePlaceData.correctPartVector.Undefine();
		cyclePlaceData.correctModelVector.Undefine();

		cyclePlaceData.bAutoFeeder			= false;
		cyclePlaceData.head					= placeWs.head;

		m_CycleData.headIdToPlaceDataIdx[ placeWs.head] = i;

		cyclePlaceData.pHead				= &g_Head[ placeWs.head];
		
		cyclePlaceData.position				= pcbPlaceCad.position;
		cyclePlaceData.arrayId.arrayNo		= aPlaceDataArray[i].m_ArrayNo;
		cyclePlaceData.arrayId.blockNo		= pcbPlaceCad.blockNo;
		cyclePlaceData.placeFidID			= (short)pcbPlaceCad.placeFidID;
											
		const StPart& pcbPart				= g_pTable->GetPart( pcbPlaceCad.partID );
		cyclePlaceData.partID				= pcbPlaceCad.partID;
		StPart pcbPartFromFeeder			= pcbPart;

		if (cyclePlaceData.feederID.IsDefined( ))
		{
			switch (cyclePlaceData.feederID.Type( ))
			{
			case eFEEDTYPE_TAPE:
				pcbPartFromFeeder = g_pTable->GetPart( ::GetTapeFeeder( cyclePlaceData.feederID )->GetPartID( ) );
				cyclePlaceData.partID = ::GetTapeFeeder( cyclePlaceData.feederID )->GetPartID( );
				break;

			case eFEEDTYPE_STICK:
				pcbPartFromFeeder = g_pTable->GetPart( ::GetStickFeeder( cyclePlaceData.feederID )->GetPartID( ) );
				cyclePlaceData.partID = ::GetStickFeeder( cyclePlaceData.feederID )->GetPartID( );
				break;

			case eFEEDTYPE_TRAY:
				pcbPartFromFeeder = g_pTable->GetPart( ::GetTray( cyclePlaceData.feederID )->GetPartID( ) );
				cyclePlaceData.partID = ::GetTray( cyclePlaceData.feederID )->GetPartID( );
				break;
			}
		}

		memset( cyclePlaceData.partName, 0, sizeof(cyclePlaceData.partName));
		strncpy( cyclePlaceData.partName, pcbPartFromFeeder.partName, sizeof(cyclePlaceData.partName)-1 );
		cyclePlaceData.profileID			= pcbPartFromFeeder.profileID;
		cyclePlaceData.bPolarized			= pcbPartFromFeeder.bPolarized!=0;
											
		const CProfile& pcbProfile			= g_pTable->GetProfile( pcbPartFromFeeder.profileID);
		cyclePlaceData.profile				= pcbProfile;
											
		cyclePlaceData.nozzleId				= placeWs.nozzleId;
		cyclePlaceData.nozzleType1			= pcbProfile.common.nozzleType1;
		cyclePlaceData.nozzleType2			= pcbProfile.common.nozzleType2;
		cyclePlaceData.hdNozzleId.Undefine();

		cyclePlaceData.placePos = cyclePlaceData.position;
		if ( apPCBoard)
			apPCBoard->MapPlacementToMachineCoord( cyclePlaceData.placePos, m_GantryID, cyclePlaceData.arrayId.blockNo, cyclePlaceData.arrayId.arrayNo, cyclePlaceData.placeFidID );

		cyclePlaceData.pickPos.Undefine();

		// Cached/Calculated Information
		cyclePlaceData.pickInspR			= ePIR_PICK_ANY;
		cyclePlaceData.bFinePitchComp		= pcbProfile.IsFinePitchComp();
		cyclePlaceData.bFineGrabComp		= pcbProfile.IsFineGrabSize();
		cyclePlaceData.bFinalBacklash		= pcbProfile.IsFinalBacklashComp();
		cyclePlaceData.bSmallComp			= pcbProfile.IsSmallComp();
		cyclePlaceData.bVerySmallComp		= pcbProfile.IsVerySmallComp();
//		cyclePlaceData.bSoftPick			= pcbProfile.IsSoftPickComp( placeWs.feederID);

		// Set this cycle as virtual pick cycle if any step has virtual pick part.
		if ( pcbProfile.IsVirtualPick() )
			nVirtualPick++;

		if ( cyclePlaceData.bFinePitchComp)
			m_bFineXY = true;
		if ( cyclePlaceData.bFineGrabComp)
			m_bFineGrab = true;

		// Even one head has large part(larger than 3.2mm), give up HRS mode in fly camera.
		if ( !pcbProfile.IsSmallCompForHRS() )
			m_bCycleUseHRS = false;

		m_CycleSpeedXY		= max(m_CycleSpeedXY, pcbProfile.handling.speedLimitXY);
		m_CycleSpeedPickUpZ	= max(m_CycleSpeedPickUpZ, pcbProfile.handling.pickUpSpeedZ);

		for (int iStep=0; iStep<_countof(cyclePlaceData.pStep); iStep++) cyclePlaceData.pStep[iStep] = NULL;
	}

	if ( nVirtualPick != 0 && nVirtualPick == aPlaceDataArray.GetCount() )
		m_CycleData.bVirtualPick = true;

	m_CycleData.cycle	= cycle;
	m_CycleData.nCount = aPlaceDataArray.GetCount();
	m_CycleData.pCycleSeq = this;
	m_CycleData.pPcbBoard = apPCBoard;
	m_CycleData.stationId = aStation;

	m_CycleData.ClearNextCycleDataPtr();
	for ( i=0; i<_countof(m_NextCycleData.placeData); i++)
	{
		StPlaceData& cyclePlaceData = m_NextCycleData.placeData[i];
		for (int iStep=0; iStep<_countof(cyclePlaceData.pStep); iStep++) cyclePlaceData.pStep[iStep] = NULL;
	}

	m_CycleData.nCountOrg	= 0;


	
}

void
CSeqCycle::SetBarcodePlaceData( const TBarcodePlaceWsList& aPlaceDataArray, CPCBoard* apPCBoard,
								const CStationId& aStation, eCyclePrefetchOp aPrefetchOp )
{
//	const TGantryIdList& gantryIdList = g_Machine.GetGantryIds();
	m_PrefetchOp = aPrefetchOp;
	m_bFineXY = false;
	m_bFineGrab = false;
	m_bCycleUseHRS = true;
	m_CycleSpeedXY = eHSPEED_FASTEST;
	m_CycleSpeedPickUpZ = eHSPEED_FASTEST;

	// start as normal pick
	m_CycleData.bVirtualPick = false;

	TPcbID		pcbID = (apPCBoard ? apPCBoard->GetPcbID() : g_pPcbSet->PreferPcbId( aStation.Lane()) );
	CPcbFunc*	pPcbFunc = g_pPcbs[ pcbID];

	long cycle = 0;
	int  nVirtualPick = 0;
	for (int i=0; i<aPlaceDataArray.GetCount(); i++)
	{
		StBarcodePlaceWs& placeWs = *aPlaceDataArray[i].m_pPlaceWs;

		StPlaceData& cyclePlaceData = m_CycleData.placeData[i];
		const StBarcodePlaceCad& pcbPlaceCad = pPcbFunc->GetBarcodePlaceCad( placeWs.placeCadID );

		// Ignore placements with...
		// 1) Out of range CAD ID
		// 2) Out of range PART ID
		// 3) Out of range ARRAY NO
		// 4) Out of range PLACE FID ID
		// 5) Out of range HEAD ID
		if ( placeWs.placeCadID >= pPcbFunc->barcodePlaceCad.GetCount())			continue;
//		if ( pcbPlaceCad.partID >= pPcbFunc->parts.GetCount())			continue;
		if ( aPlaceDataArray[i].m_ArrayNo >= pPcbFunc->GetArrayCnt( pcbPlaceCad.blockNo ))	continue;
		if ( pcbPlaceCad.placeFidID >= pPcbFunc->fidLocalMaps.GetCount())	continue;

// 		if ( placeWs.head == SYS_MAX_HEAD +1 )
// 		{
// 			if ( placeWs.feederID.Unit() == 1 || gantryIdList.GetCount() == 1 )
// 				placeWs.head	=1;
// 			else if( placeWs.feederID.Unit() == 2 )
// 				placeWs.head	=7;
// 		}

		if ( placeWs.head >= _countof(g_Head))							continue;

		// Sets cycle number of each step. Assume every step has same cycle number.
//		cycle								= aPlaceDataArray[i].m_Cycle;
		cycle								= placeWs.cycle;

		cyclePlaceData.status				= ePLACE_OK;
		cyclePlaceData.bStoppedByDumpStop	= false;
		cyclePlaceData.placeCadID			= placeWs.placeCadID;
		cyclePlaceData.cycle				= cycle;
		cyclePlaceData.feederID				= placeWs.feederID;
		cyclePlaceData.pickedFeederID		= CFeederId(0);
		cyclePlaceData.placedFeederID		= CFeederId(0);
	
		//<2011.1.31>
		//Initialize picked pocket index
		cyclePlaceData.mPocketPickedIndex	= -1;

		cyclePlaceData.correctPartVector.Undefine();
		cyclePlaceData.correctModelVector.Undefine();

		cyclePlaceData.bAutoFeeder			= false;

		
		CHead*	pHead = GetHead( placeWs.head );
		if ( pHead && pHead->IsHeadDisabled() )
			placeWs.head++;

		cyclePlaceData.head					= placeWs.head;

		m_CycleData.headIdToPlaceDataIdx[ placeWs.head] = i;

		cyclePlaceData.pHead				= &g_Head[ placeWs.head];
		
		CFxyzt loc = pcbPlaceCad.position;
		cyclePlaceData.position				= loc;					//Same as StBarcodePlaceCad::position
		SYS_ASSERT(apPCBoard);
		// Converts from Barcode Place Cad data to Placement coordinates.
		// It will be modified into machine coord at the CStepPlace***::PlanStep() by CSeqCycle::GetPlaceNominalNoPV().
		apPCBoard->MapBarcodePlaceCadToPlacementCoord( m_GantryID, pcbPlaceCad.blockNo, pcbPlaceCad.arrayNo, loc);
		cyclePlaceData.placePos				= loc;					//Actual planned place position for this head.
		cyclePlaceData.arrayId.arrayNo		= pcbPlaceCad.arrayNo;
		cyclePlaceData.arrayId.blockNo		= pcbPlaceCad.blockNo;
		cyclePlaceData.placeFidID			= (short)pcbPlaceCad.placeFidID;
									
		const StPart& pcbPart				= g_pTable->GetPart( pcbPlaceCad.partID );
		
		cyclePlaceData.partID				= pcbPlaceCad.partID;
		strncpy( cyclePlaceData.partName, pcbPart.partName, sizeof(cyclePlaceData.partName));
		cyclePlaceData.partName[ sizeof(cyclePlaceData.partName)-1 ] = 0;
		cyclePlaceData.profileID			= pcbPart.profileID;
		cyclePlaceData.bPolarized			= pcbPart.bPolarized!=0;
											
		const CProfile& pcbProfile			= g_pTable->GetProfile( pcbPart.profileID);
		cyclePlaceData.profile				= pcbProfile;
											
		cyclePlaceData.nozzleId				= placeWs.nozzleId;
		cyclePlaceData.nozzleType1			= pcbProfile.common.nozzleType1;
		cyclePlaceData.nozzleType2			= pcbProfile.common.nozzleType2;
		cyclePlaceData.hdNozzleId.Undefine();

//		if ( g_FacConst.IsUseNozzleHandlingData())
//		{
//			eNozzleType	nozType	= eNOZZLE_TYPE_NONE;
//			if ( cyclePlaceData.nozzleId.IsDefined())
//			{
//				CAnc* pAnc = ::GetAnc( cyclePlaceData.nozzleId.Anc());
//				if ( pAnc && pAnc->IsHole( cyclePlaceData.nozzleId.HoleNum()))
//					nozType = pAnc->GetNozzleType( cyclePlaceData.nozzleId.HoleNum());
//			}
//			else if ( cyclePlaceData.nozzleType1 != eNOZZLE_TYPE_NONE)
//				nozType = cyclePlaceData.nozzleType1;
//			else
//				nozType = cyclePlaceData.nozzleType2;
//			
//			CHeadBlock* pHB = cyclePlaceData.pHead->GetHeadBlock();
//			pHB->UpdateNozzleHandlingData( nozType, cyclePlaceData.profile );
//		}

		CGantry* pGantry = pHead->Gantry();
		SYS_ASSERT(pGantry);
	
		CSeqEngine&	seqEngine = g_SeqEngine[ pGantry->Unit()];
		seqEngine.SetBarCodeWorkGantry(pGantry);
		cyclePlaceData.pickPos.Undefine();

		// Cached/Calculated Information
		cyclePlaceData.pickInspR			= ePIR_PICK_ANY;
		cyclePlaceData.bFinePitchComp		= pcbProfile.IsFinePitchComp();
		cyclePlaceData.bFineGrabComp		= pcbProfile.IsFineGrabSize();
		cyclePlaceData.bFinalBacklash		= pcbProfile.IsFinalBacklashComp();
		cyclePlaceData.bSmallComp			= pcbProfile.IsSmallComp();
		cyclePlaceData.bVerySmallComp		= pcbProfile.IsVerySmallComp();
//		cyclePlaceData.bSoftPick			= pcbProfile.IsSoftPickComp( placeWs.feederID);

		// Set this cycle as virtual pick cycle if any step has virtual pick part.
		if ( pcbProfile.IsVirtualPick() )
			nVirtualPick++;

		if ( cyclePlaceData.bFinePitchComp)
			m_bFineXY = true;
		if ( cyclePlaceData.bFineGrabComp)
			m_bFineGrab = true;

		// Even one head has large part(larger than 3.2mm), give up HRS mode in fly camera.
		if ( !pcbProfile.IsSmallCompForHRS() )
			m_bCycleUseHRS = false;

		m_CycleSpeedXY		= max(m_CycleSpeedXY, pcbProfile.handling.speedLimitXY);
		m_CycleSpeedPickUpZ	= max(m_CycleSpeedPickUpZ, pcbProfile.handling.pickUpSpeedZ);

		for (int iStep=0; iStep<_countof(cyclePlaceData.pStep); iStep++) cyclePlaceData.pStep[iStep] = NULL;
	}

	if ( nVirtualPick != 0 && nVirtualPick == aPlaceDataArray.GetCount() )
		m_CycleData.bVirtualPick = true;

	m_CycleData.cycle	= cycle;
	m_CycleData.nCount = aPlaceDataArray.GetCount();
	m_CycleData.pCycleSeq = this;
	m_CycleData.pPcbBoard = apPCBoard;
	m_CycleData.stationId = aStation;

	m_CycleData.ClearNextCycleDataPtr();
	
	for ( int j=0; j<_countof(m_NextCycleData.placeData); j++)
	{
		StPlaceData& cyclePlaceData = m_NextCycleData.placeData[j];
		for (int iStep=0; iStep<_countof(cyclePlaceData.pStep); iStep++) cyclePlaceData.pStep[iStep] = NULL;
	}

	m_CycleData.nCountOrg	= 0;
}

void
CSeqCycle::SetPlaceLineLCRData( const TPlaceWsList& aPlaceDataArray, CPCBoard* apPCBoard,
								const CStationId& aStation, eCyclePrefetchOp aPrefetchOp )
{
	bool	bMultiPcb = g_pPcbSet->IsMultiPcb( aStation.Lane( ) );

	// Make sure there is only one Pcb choice
	SYS_ASSERT( apPCBoard!=NULL  ||  !bMultiPcb );

	m_frRunningLane = ::GetLaneFR( aStation.Lane( ) );
	m_PrefetchOp = aPrefetchOp;
	m_bFineXY = false;
	m_bFineGrab = false;
	m_bCycleUseHRS = true;
	m_CycleSpeedXY = eHSPEED_FASTEST;
	m_CycleSpeedPickUpZ = eHSPEED_FASTEST;

	// start as normal pick
	m_CycleData.bVirtualPick = false;

	TPcbID		pcbID = (apPCBoard ? apPCBoard->GetPcbID( ) : g_pPcbSet->PreferPcbId( aStation.Lane( ) ));
	CPcbFunc*	pPcbFunc = g_pPcbs[pcbID];

	long cycle = 0;
	int  nVirtualPick = 0;
	int  i = 0;
	for (i=0; i<aPlaceDataArray.GetCount( ); i++)
	{
		StPlaceWs& placeWs = *aPlaceDataArray[i].m_pPlaceWs;

		StPlaceData& cyclePlaceData = m_CycleData.placeData[i];
		const StPlaceCad& pcbPlaceCad = pPcbFunc->GetPlaceCad( placeWs.placeCadID );

		// Ignore placements with...
		// 1) Out of range CAD ID
		// 2) Out of range PART ID
		// 3) Out of range ARRAY NO
		// 4) Out of range PLACE FID ID
		// 5) Out of range HEAD ID
		if (placeWs.placeCadID >= pPcbFunc->placeCad.GetCount( ))			continue;
		//		if ( pcbPlaceCad.partID >= g_pTable->parts.GetCount())				continue;
		if (!g_pTable->parts.IsExist( pcbPlaceCad.partID ))					continue;
		if (aPlaceDataArray[i].m_ArrayNo >= pPcbFunc->GetArrayCnt( pcbPlaceCad.blockNo ))	continue;
		if (pcbPlaceCad.placeFidID >= pPcbFunc->fidLocalMaps.GetCount( ))	continue;
		if (placeWs.head >= _countof( g_Head ))								continue;

		// Sets cycle number of each step. Assume every step has same cycle number.
		cycle								= aPlaceDataArray[i].m_Cycle;

		cyclePlaceData.status				= ePLACE_OK;
		cyclePlaceData.bStoppedByDumpStop	= false;
		cyclePlaceData.placeCadID			= placeWs.placeCadID;
		cyclePlaceData.placeMapID			= placeWs.placeMapID;
		cyclePlaceData.cycle				= cycle;
		cyclePlaceData.pickOrder			= placeWs.pickOrder;
		cyclePlaceData.feederID				= placeWs.feederID;
		cyclePlaceData.pickedFeederID		= CFeederId( 0 );
		cyclePlaceData.placedFeederID		= CFeederId( 0 );


		CPCBoard::StLineLCRData pcboardData;
		CTapeFeeder* pTapeFeeder = ::GetTapeFeeder( cyclePlaceData.feederID	);
		SYS_ASSERT( pTapeFeeder );

		eLineLCRType type = pTapeFeeder->GetLineLCRPartType( );
		apPCBoard->GetLineLCRDataToPcboard( pcboardData, type );

		if ((pcboardData.position.x != pPcbFunc->boardLineLCR[1].position[type].x)
			|| (pcboardData.position.y != pPcbFunc->boardLineLCR[1].position[type].y)
			|| (pcboardData.position.z != pPcbFunc->boardLineLCR[1].position[type].z)
			|| (pcboardData.position.t != pPcbFunc->boardLineLCR[1].position[type].t))
		{
			Telemetry(EV_CLASSHEAD, placeWs.head, "H%s Portable XYT[%.3f,%.3f,%.3f] BoardXYT[%.3f,%.3f,%.3f] Mismatch!", cyclePlaceData.pHead->GetName( )
				, pcboardData.position.x, pcboardData.position.y, pcboardData.position.t,
				pPcbFunc->boardLineLCR[1].position[type].x, pPcbFunc->boardLineLCR[1].position[type].y, pPcbFunc->boardLineLCR[1].position[type].t);

			apPCBoard->SetLineBoardDataToPcboard( );
			apPCBoard->GetLineLCRDataToPcboard( pcboardData, type );
		}
			
		//<2011.1.31>
		//Initialize picked pocket index
		cyclePlaceData.mPocketPickedIndex	= -1;

		cyclePlaceData.correctPartVector.Undefine( );
		cyclePlaceData.correctModelVector.Undefine( );

		cyclePlaceData.bAutoFeeder			= false;
		cyclePlaceData.head					= placeWs.head;

		m_CycleData.headIdToPlaceDataIdx[placeWs.head] = i;

		cyclePlaceData.pHead				= &g_Head[placeWs.head];

		cyclePlaceData.position				= pcboardData.position;/*pcbPlaceCad.position;*/
		cyclePlaceData.arrayId.arrayNo		= 1/*aPlaceDataArray[i].m_ArrayNo*/;
		cyclePlaceData.arrayId.blockNo		= pcbPlaceCad.blockNo;
		cyclePlaceData.placeFidID			= (short)pcbPlaceCad.placeFidID;

		const StPart& pcbPart				= g_pTable->GetPart( pcbPlaceCad.partID );
		cyclePlaceData.partID				= pcbPlaceCad.partID;
		StPart pcbPartFromFeeder			= pcbPart;

		if (cyclePlaceData.feederID.IsDefined( ))
		{
			switch (cyclePlaceData.feederID.Type( ))
			{
			case eFEEDTYPE_TAPE:
				pcbPartFromFeeder = g_pTable->GetPart( ::GetTapeFeeder( cyclePlaceData.feederID )->GetPartID( ) );
				cyclePlaceData.partID = ::GetTapeFeeder( cyclePlaceData.feederID )->GetPartID( );
				break;

			case eFEEDTYPE_STICK:
				pcbPartFromFeeder = g_pTable->GetPart( ::GetStickFeeder( cyclePlaceData.feederID )->GetPartID( ) );
				cyclePlaceData.partID = ::GetStickFeeder( cyclePlaceData.feederID )->GetPartID( );
				break;

			case eFEEDTYPE_TRAY:
				pcbPartFromFeeder = g_pTable->GetPart( ::GetTray( cyclePlaceData.feederID )->GetPartID( ) );
				cyclePlaceData.partID = ::GetTray( cyclePlaceData.feederID )->GetPartID( );
				break;
			}
		}

		memset( cyclePlaceData.partName, 0, sizeof( cyclePlaceData.partName ) );
		strncpy( cyclePlaceData.partName, pcbPartFromFeeder.partName, sizeof( cyclePlaceData.partName )-1 );
		cyclePlaceData.profileID			= pcbPartFromFeeder.profileID;
		cyclePlaceData.bPolarized			= pcbPartFromFeeder.bPolarized!=0;

		const CProfile& pcbProfile			= g_pTable->GetProfile( pcbPartFromFeeder.profileID );
		cyclePlaceData.profile				= pcbProfile;

		cyclePlaceData.nozzleId				= placeWs.nozzleId;
		cyclePlaceData.nozzleType1			= pcbProfile.common.nozzleType1;
		cyclePlaceData.nozzleType2			= pcbProfile.common.nozzleType2;
		cyclePlaceData.hdNozzleId.Undefine( );

		cyclePlaceData.placePos = cyclePlaceData.position;
		if (apPCBoard)
			apPCBoard->MapBoardToMachineCoord( cyclePlaceData.placePos );
//			apPCBoard->MapPlacementToMachineCoord( cyclePlaceData.placePos, m_GantryID, cyclePlaceData.arrayId.blockNo, cyclePlaceData.arrayId.arrayNo, cyclePlaceData.placeFidID );

		cyclePlaceData.pickPos.Undefine( );

		// Cached/Calculated Information
		cyclePlaceData.pickInspR			= ePIR_PICK_ANY;
		cyclePlaceData.bFinePitchComp		= pcbProfile.IsFinePitchComp( );
		cyclePlaceData.bFineGrabComp		= pcbProfile.IsFineGrabSize( );
		cyclePlaceData.bFinalBacklash		= pcbProfile.IsFinalBacklashComp( );
		cyclePlaceData.bSmallComp			= pcbProfile.IsSmallComp( );
		cyclePlaceData.bVerySmallComp		= pcbProfile.IsVerySmallComp( );
		//		cyclePlaceData.bSoftPick			= pcbProfile.IsSoftPickComp( placeWs.feederID);

				// Set this cycle as virtual pick cycle if any step has virtual pick part.
		if (pcbProfile.IsVirtualPick( ))
			nVirtualPick++;

		if (cyclePlaceData.bFinePitchComp)
			m_bFineXY = true;
		if (cyclePlaceData.bFineGrabComp)
			m_bFineGrab = true;

		// Even one head has large part(larger than 3.2mm), give up HRS mode in fly camera.
		if (!pcbProfile.IsSmallCompForHRS( ))
			m_bCycleUseHRS = false;

		m_CycleSpeedXY		= max( m_CycleSpeedXY, pcbProfile.handling.speedLimitXY );
		m_CycleSpeedPickUpZ	= max( m_CycleSpeedPickUpZ, pcbProfile.handling.pickUpSpeedZ );

		for (int iStep=0; iStep<_countof( cyclePlaceData.pStep ); iStep++) cyclePlaceData.pStep[iStep] = NULL;
	}

	if (nVirtualPick != 0 && nVirtualPick == aPlaceDataArray.GetCount( ))
		m_CycleData.bVirtualPick = true;

	m_CycleData.cycle	= cycle;
	m_CycleData.nCount = aPlaceDataArray.GetCount( );
	m_CycleData.pCycleSeq = this;
	m_CycleData.pPcbBoard = apPCBoard;
	m_CycleData.stationId = aStation;

	m_CycleData.ClearNextCycleDataPtr( );
	for (i=0; i<_countof( m_NextCycleData.placeData ); i++)
	{
		StPlaceData& cyclePlaceData = m_NextCycleData.placeData[i];
		for (int iStep=0; iStep<_countof( cyclePlaceData.pStep ); iStep++) cyclePlaceData.pStep[iStep] = NULL;
	}

	m_CycleData.nCountOrg	= 0;



}

void
CSeqCycle::SetNextCyclePlaceData( const TPlaceWsList& aPlaceDataArray, CPCBoard* apPCBoard,
						const CStationId& aStation, eCyclePrefetchOp aPrefetchOp )

{
	bool	bMultiPcb = g_pPcbSet->IsMultiPcb(aStation.Lane());

	// Make sure there is only one Pcb choice
	SYS_ASSERT( apPCBoard!=NULL  ||  !bMultiPcb);

//x	m_PcbID = g_pPcbs[x]->m_PcbID;
//x	m_PrefetchOp = aPrefetchOp;
//x	m_bFineXY = false;
//x	m_bFineGrab = false;
//x	m_bCycleUseHRS = true;
//x	m_CycleSpeedXY = eHSPEED_FASTEST;
//x	m_CycleSpeedPickUpZ = eHSPEED_FASTEST;


	if ( aPlaceDataArray.IsEmpty() )	return;

	StCycleData& rCycleData = m_NextCycleData;

	TPcbID		pcbID = (apPCBoard ? apPCBoard->GetPcbID() : g_pPcbSet->PreferPcbId( aStation.Lane()) );
	CPcbFunc*	pPcbFunc = g_pPcbs[ pcbID];

	for (int i=0; i<aPlaceDataArray.GetCount() ; i++)
	{
		StPlaceWs& placeWs = *aPlaceDataArray[i].m_pPlaceWs;

		StPlaceData& cyclePlaceData = rCycleData.placeData[i];
		const StPlaceCad& pcbPlaceCad = pPcbFunc->GetPlaceCad( placeWs.placeCadID );

		// Ignore placements with...
		// 1) Out of range CAD ID
		// 2) Out of range PART ID
		// 3) Out of range ARRAY NO
		// 4) Out of range PLACE FID ID
		// 5) Out of range HEAD ID
		if ( placeWs.placeCadID >= pPcbFunc->placeCad.GetCount())			continue;
//		if ( pcbPlaceCad.partID >= g_pTable->parts.GetCount())				continue;
		if ( !g_pTable->parts.IsExist( pcbPlaceCad.partID))					continue;
		if ( aPlaceDataArray[i].m_ArrayNo >= pPcbFunc->GetArrayCnt( pcbPlaceCad.blockNo ))	continue;
		if ( pcbPlaceCad.placeFidID >= pPcbFunc->fidLocalMaps.GetCount())	continue;
		if ( placeWs.head >= _countof(g_Head))								continue;

		cyclePlaceData.status				= ePLACE_OK;
		cyclePlaceData.bStoppedByDumpStop	= false;
		cyclePlaceData.placeCadID			= placeWs.placeCadID;
		cyclePlaceData.placeMapID			= placeWs.placeMapID;
		cyclePlaceData.cycle				= aPlaceDataArray[i].m_Cycle;
		cyclePlaceData.pickOrder			= placeWs.pickOrder;
		cyclePlaceData.feederID				= placeWs.feederID;
		cyclePlaceData.pickedFeederID		= CFeederId(0);
		cyclePlaceData.placedFeederID		= CFeederId(0);

		//<2011.1.31>
		//Initialize picked pocket index
		cyclePlaceData.mPocketPickedIndex	= -1;

		cyclePlaceData.correctPartVector.Undefine();
		cyclePlaceData.correctModelVector.Undefine();

		cyclePlaceData.bAutoFeeder			= false;
		cyclePlaceData.head					= placeWs.head;

		rCycleData.headIdToPlaceDataIdx[ placeWs.head] = i;

		cyclePlaceData.pHead				= &g_Head[ placeWs.head];
		
		cyclePlaceData.position				= pcbPlaceCad.position;
		cyclePlaceData.arrayId.blockNo		= pcbPlaceCad.blockNo;
		cyclePlaceData.arrayId.arrayNo		= aPlaceDataArray[i].m_ArrayNo;
		cyclePlaceData.placeFidID			= (short)pcbPlaceCad.placeFidID;
											
		const StPart& pcbPart				= g_pTable->GetPart( pcbPlaceCad.partID );
		
		cyclePlaceData.partID				= pcbPlaceCad.partID;

		memset( cyclePlaceData.partName, 0, sizeof(cyclePlaceData.partName));
		strncpy( cyclePlaceData.partName, pcbPart.partName, sizeof(cyclePlaceData.partName)-1 );
		cyclePlaceData.profileID			= pcbPart.profileID;
		cyclePlaceData.bPolarized			= pcbPart.bPolarized!=0;
											
		const CProfile& pcbProfile			= g_pTable->GetProfile( pcbPart.profileID);
		cyclePlaceData.profile				= pcbProfile;
											
		cyclePlaceData.nozzleId				= placeWs.nozzleId;
		cyclePlaceData.nozzleType1			= pcbProfile.common.nozzleType1;
		cyclePlaceData.nozzleType2			= pcbProfile.common.nozzleType2;
		cyclePlaceData.hdNozzleId.Undefine();

		cyclePlaceData.placePos = cyclePlaceData.position;
		if ( apPCBoard)
			apPCBoard->MapPlacementToMachineCoord( cyclePlaceData.placePos, m_GantryID, cyclePlaceData.arrayId.blockNo, cyclePlaceData.arrayId.arrayNo, cyclePlaceData.placeFidID );

		cyclePlaceData.pickPos.Undefine();

		// Cached/Calculated Information
		cyclePlaceData.pickInspR			= ePIR_PICK_ANY;
		cyclePlaceData.bFinePitchComp		= pcbProfile.IsFinePitchComp();
		cyclePlaceData.bFineGrabComp		= pcbProfile.IsFineGrabSize();
		cyclePlaceData.bFinalBacklash		= pcbProfile.IsFinalBacklashComp();
		cyclePlaceData.bSmallComp			= pcbProfile.IsSmallComp();
		cyclePlaceData.bVerySmallComp		= pcbProfile.IsVerySmallComp();
//x//	cyclePlaceData.bSoftPick			= pcbProfile.IsSoftPickComp( placeWs.feederID);
//x
//x		if ( cyclePlaceData.bFinePitchComp)
//x			m_bFineXY = true;
//x		if ( cyclePlaceData.bFineGrabComp)
//x			m_bFineGrab = true;
//x
//x		// Even one head has large part(larger than 3.2mm), give up HRS mode in fly camera.
//x		if ( !pcbProfile.IsSmallCompForHRS() )
//x			m_bCycleUseHRS = false;
//x
//x		m_CycleSpeedXY		= max(m_CycleSpeedXY, pcbProfile.handling.speedLimitXY);
//x		m_CycleSpeedPickUpZ	= max(m_CycleSpeedPickUpZ, pcbProfile.handling.pickUpSpeedZ);

		for (int iStep=0; iStep<_countof(cyclePlaceData.pStep); iStep++) cyclePlaceData.pStep[iStep] = NULL;
	}

	
	rCycleData.nCount = aPlaceDataArray.GetCount();
	rCycleData.pCycleSeq = this;
	rCycleData.pPcbBoard = apPCBoard;
	rCycleData.stationId = aStation;

	m_CycleData.SetNextCycleDataPtr( &rCycleData );

	rCycleData.nCountOrg	= 0;
	
}

void
CSeqCycle::SetNextCycleBarcodePlaceData( const TBarcodePlaceWsList& aPlaceDataArray, CPCBoard* apPCBoard,
										const CStationId& aStation, eCyclePrefetchOp aPrefetchOp )

{
//x	m_PcbID = pPcbFunc->m_PcbID;
//x	m_PrefetchOp = aPrefetchOp;
//x	m_bFineXY = false;
//x	m_bFineGrab = false;
//x	m_bCycleUseHRS = true;
//x	m_CycleSpeedXY = eHSPEED_FASTEST;
//x	m_CycleSpeedPickUpZ = eHSPEED_FASTEST;


	if ( aPlaceDataArray.IsEmpty() )	return;

//	const TGantryIdList& gantryIdList = g_Machine.GetGantryIds();
	StCycleData& rCycleData = m_NextCycleData;

	TPcbID		pcbID = (apPCBoard ? apPCBoard->GetPcbID() : g_pPcbSet->PreferPcbId( aStation.Lane()) );
	CPcbFunc*	pPcbFunc = g_pPcbs[ pcbID];

	for (int i=0; i<aPlaceDataArray.GetCount() ; i++)
	{
		StBarcodePlaceWs& placeWs = *aPlaceDataArray[i].m_pPlaceWs;

		StPlaceData& cyclePlaceData = rCycleData.placeData[i];
		const StBarcodePlaceCad& pcbPlaceCad = pPcbFunc->GetBarcodePlaceCad( placeWs.placeCadID );

		// Ignore placements with...
		// 1) Out of range CAD ID
		// 2) Out of range PART ID
		// 3) Out of range ARRAY NO
		// 4) Out of range PLACE FID ID
		// 5) Out of range HEAD ID
		if ( placeWs.placeCadID >= pPcbFunc->barcodePlaceCad.GetCount())			continue;
//		if ( pcbPlaceCad.partID >= pPcbFunc->parts.GetCount())			continue;
		if ( !g_pTable->parts.IsExist( pcbPlaceCad.partID))					continue;
		if ( aPlaceDataArray[i].m_ArrayNo >= pPcbFunc->GetArrayCnt( pcbPlaceCad.blockNo ))	continue;
		if ( pcbPlaceCad.placeFidID >= pPcbFunc->fidLocalMaps.GetCount())	continue;


// 		if ( placeWs.head == SYS_MAX_HEAD +1 )
// 		{
// 			if ( placeWs.feederID.Unit() == 1 || gantryIdList.GetCount() == 1 )
// 				placeWs.head	=1;
// 			else if( placeWs.feederID.Unit() == 2 )
// 				placeWs.head	=7;
// 		}

		if ( placeWs.head >= _countof(g_Head))							continue;


		cyclePlaceData.status				= ePLACE_OK;
		cyclePlaceData.bStoppedByDumpStop	= false;
		cyclePlaceData.placeCadID			= placeWs.placeCadID;
//		cyclePlaceData.cycle				= aPlaceDataArray[i].m_Cycle;
		cyclePlaceData.cycle				= placeWs.cycle;//placeCadID;
		cyclePlaceData.feederID				= placeWs.feederID;
		cyclePlaceData.pickedFeederID		= CFeederId(0);
		cyclePlaceData.placedFeederID		= CFeederId(0);

		//<2011.1.31>
		//Initialize picked pocket index
		cyclePlaceData.mPocketPickedIndex	= -1;

		cyclePlaceData.correctPartVector.Undefine();
		cyclePlaceData.correctModelVector.Undefine();

		cyclePlaceData.bAutoFeeder			= false;

		
		CHead*	pHead = GetHead( placeWs.head );
		if ( pHead && pHead->IsHeadDisabled() )
			placeWs.head++;

		cyclePlaceData.head					= placeWs.head;

		rCycleData.headIdToPlaceDataIdx[ placeWs.head] = i;

		cyclePlaceData.pHead				= &g_Head[ placeWs.head];
		
		CFxyzt loc = pcbPlaceCad.position;
		cyclePlaceData.position				= loc;					//Same as StBarcodePlaceCad::position
		SYS_ASSERT(apPCBoard);
		// Converts from Barcode Place Cad data to Placement coordinates.
		// It will be modified int o machine coord at the CStepPlace***::PlanStep() by CSeqCycle::GetPlaceNominalNoPV().
		apPCBoard->MapBarcodePlaceCadToPlacementCoord( m_GantryID, pcbPlaceCad.blockNo, pcbPlaceCad.arrayNo, loc);
		cyclePlaceData.placePos				= loc;					//Actual planned place position for this head.
		cyclePlaceData.arrayId.arrayNo		= pcbPlaceCad.arrayNo;
		cyclePlaceData.arrayId.blockNo		= pcbPlaceCad.blockNo;
		cyclePlaceData.placeFidID			= (short)pcbPlaceCad.placeFidID;
		
		const StPart& pcbPart				= g_pTable->GetPart( pcbPlaceCad.partID );
		
		cyclePlaceData.partID				= pcbPlaceCad.partID;
		strncpy( cyclePlaceData.partName, pcbPart.partName, sizeof(cyclePlaceData.partName));
		cyclePlaceData.partName [ sizeof(cyclePlaceData.partName)-1 ] = 0;
		cyclePlaceData.profileID			= pcbPart.profileID;
		cyclePlaceData.bPolarized			= pcbPart.bPolarized!=0;
											
		const CProfile& pcbProfile			= g_pTable->GetProfile( pcbPart.profileID);
		cyclePlaceData.profile				= pcbProfile;
											
		cyclePlaceData.nozzleId				= placeWs.nozzleId;
		cyclePlaceData.nozzleType1			= pcbProfile.common.nozzleType1;
		cyclePlaceData.nozzleType2			= pcbProfile.common.nozzleType2;
		cyclePlaceData.hdNozzleId.Undefine();

// 		cyclePlaceData.placePos = cyclePlaceData.position;
// 		if ( apPCBoard)
// 			//apPCBoard->MapPlacementToMachineCoord( cyclePlaceData.placePos, cyclePlaceData.arrayId.blockNo, cyclePlaceData.arrayId.arrayNo, cyclePlaceData.placeFidID );
// 			apPCBoard->MapBoardToMachineCoord( cyclePlaceData.placePos );

		cyclePlaceData.pickPos.Undefine();

		// Cached/Calculated Information
		cyclePlaceData.pickInspR			= ePIR_PICK_ANY;
		cyclePlaceData.bFinePitchComp		= pcbProfile.IsFinePitchComp();
		cyclePlaceData.bFineGrabComp		= pcbProfile.IsFineGrabSize();
		cyclePlaceData.bFinalBacklash		= pcbProfile.IsFinalBacklashComp();
		cyclePlaceData.bSmallComp			= pcbProfile.IsSmallComp();
		cyclePlaceData.bVerySmallComp		= pcbProfile.IsVerySmallComp();
//x//		cyclePlaceData.bSoftPick			= pcbProfile.IsSoftPickComp( placeWs.feederID);
//x
//x		if ( cyclePlaceData.bFinePitchComp)
//x			m_bFineXY = true;
//x		if ( cyclePlaceData.bFineGrabComp)
//x			m_bFineGrab = true;
//x
//x		// Even one head has large part(larger than 3.2mm), give up HRS mode in fly camera.
//x		if ( !pcbProfile.IsSmallCompForHRS() )
//x			m_bCycleUseHRS = false;
//x
//x		m_CycleSpeedXY		= max(m_CycleSpeedXY, pcbProfile.handling.speedLimitXY);
//x		m_CycleSpeedPickUpZ	= max(m_CycleSpeedPickUpZ, pcbProfile.handling.pickUpSpeedZ);

		for (int iStep=0; iStep<_countof(cyclePlaceData.pStep); iStep++) cyclePlaceData.pStep[iStep] = NULL;
	}

	
	rCycleData.nCount = aPlaceDataArray.GetCount();
	rCycleData.pCycleSeq = this;
	rCycleData.pPcbBoard = apPCBoard;
	rCycleData.stationId = aStation;

	m_CycleData.SetNextCycleDataPtr( &rCycleData );
	
	rCycleData.nCountOrg	= 0;
}


void
CSeqCycle::GetPlaceData( StPlaceData*& arPlaceData, int& arCount )
{
	arPlaceData = m_CycleData.placeData;
	arCount = m_CycleData.nCount;
}

void
CSeqCycle::GetPlaceDataOrg( StPlaceData*& arPlaceData, int& arCount )
{
	arPlaceData = m_CycleData.placeDataOrg;
	arCount = m_CycleData.nCountOrg;
}


bool
CSeqCycle::IsHeadInCycle( HeadID aHeadID) const
{
	return m_CycleData.IsHeadInCycle( aHeadID);
}


const StPlaceData&
CSeqCycle::GetPlaceData( HeadID aHeadID) const
{
	return m_CycleData.GetPlaceData( aHeadID);
}


bool
CSeqCycle::GetPlaceData( const StPlaceData*& arpPd, long aIdx) const
{
	bool bValid = 0 <= aIdx  &&  aIdx < m_CycleData.nCount;

	if ( bValid)
		arpPd = &m_CycleData.placeData[ aIdx];
	return bValid;
}


// Returns the pick order for this cycle in terms of placeData indexes.
void
CSeqCycle::GetPickOrder( TPDOrder& aPlaceDataOrder)
{
	aPlaceDataOrder.RemoveAll();

	// First determine if there are any pick steps in this cycle.
	long	idxStep;	// Index into m_Steps[].
	CStep** ppStep=NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
	long	nStep;		// Count of contiguous primary type of steps.

	nStep = GetSteps( eSTEP_PICK, m_frRunningLane, idxStep, ppStep);

	if ( idxStep < 0)	return;
	if ( nStep <= 0)	return;

	for (int i=idxStep; --nStep>=0; i++)
	{
		CStepPick* pStep = (CStepPick*)m_Steps[i];
		TPDOrder order;

		pStep->GetPlaceDataOrder( order, m_CycleData);

		for (int j=0; j<order.GetCount(); j++)
			aPlaceDataOrder.Append( order[j]);
	}
}


// Returns the place order for this cycle in terms of placeData indexes.
void
CSeqCycle::GetPlaceOrder( TPDOrder& aPlaceDataOrder)
{
	aPlaceDataOrder.RemoveAll();

	// First determine if there are any pick steps in this cycle.
	long	idxStep;	// Index into m_Steps[].
	CStep** ppStep=NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
	long	nStep;		// Count of contiguous primary type of steps.

	nStep = GetSteps( eSTEP_PLACE, m_frRunningLane, idxStep, ppStep);

	if ( idxStep < 0)	return;
	if ( nStep <= 0)	return;

	for (int i=idxStep; --nStep>=0; i++)
	{
		CStepPlace* pStep = (CStepPlace*)m_Steps[i];
		TPDOrder order;

		pStep->GetPlaceDataOrder( order, m_CycleData);

		for (int j=0; j<order.GetCount(); j++)
			aPlaceDataOrder.Append( order[j]);
	}
}



bool
IsMovableHeadSide4Gan( GantryID aGantryID)
{
//	int res;
//	long HeadNo;
//	long ExtHeadType;

	CIlluminatorSideControl* pIllumSide = ::GetIlluminatorSideControlByGantry( aGantryID);
	return pIllumSide != NULL;

//	HeadNo = GetHeadNoGanIndex( GanNo, 0 );
//	ExtHeadType = GetExtHeadType( HeadNo );
//	
//	switch( ExtHeadType & EXT_HEAD_TYPE_FLY_SIDE )
//	{
//	case EXT_HEAD_TYPE_FLY_SIDE_NONE :
//		res = NG;
//		break;
//	case EXT_HEAD_TYPE_FLY_SIDE_MOVABLE :
//		res = OK;
//		break;
//	case EXT_HEAD_TYPE_FLY_SIDE_FIX :
//		res = NG;
//		break;
//	default :
//		assert(0);
//		break;
//	}
//	return res;
}

bool
IsMovableHeadSide( HeadID aHeadID)
{
//	int res;
//	long ExtHeadType;
//	
	CHead*	pHead = ::GetHead( aHeadID);
	CGantry* pGantry = pHead->Gantry();
	CIlluminatorSideControl* pIllumSide = ::GetIlluminatorSideControlByGantry( pGantry->Unit());
	return pIllumSide != NULL;
//
//	ExtHeadType = GetExtHeadType( HeadNo );
//	
//	switch( ExtHeadType & EXT_HEAD_TYPE_FLY_SIDE )
//	{
//	case EXT_HEAD_TYPE_FLY_SIDE_NONE :
//		res = NG;
//		break;
//	case EXT_HEAD_TYPE_FLY_SIDE_MOVABLE :
//		res = OK;
//		break;
//	case EXT_HEAD_TYPE_FLY_SIDE_FIX :
//		res = NG;
//		break;
//	default :
//		assert(0);
//		break;
//	}
//	return res;
}


eStatus
CSeqCycle::DetermineCycleType( eCycleType& aCycleType )
{
	int i;
	int iHead;
	eStatus res;
	
	eProfileType	alignMethod;
	eCycleType cycleType;
	eCycleType cyType;
	eCameraGroupType camGrpType;
	long sideLightLevel;
	long camNo;
	double size;
	double sizeHyp;
	eCycleType	headCycleType[ _countof(m_CycleData.placeData)];	// Indexed by cycle place index
	bool		bMovableSide = true;
	
	for (i=0; i<_countof(headCycleType); i++) headCycleType[i] = eCYCLE_TYPE_NONE;
	
	if ( m_pGantry )
		bMovableSide = IsMovableHeadSide4Gan( m_pGantry->Unit() );

	for (iHead=0; iHead<m_CycleData.nCount; iHead++)
	{
		StProfile& profile = m_CycleData.placeData[iHead].profile;

		const CFxy& partSize = profile.common.bound;
		size = partSize.x;
		if ( size < partSize.y )
			size = partSize.y;
		sizeHyp = sqrt( partSize.x * partSize.x + partSize.y * partSize.y);

		alignMethod		= profile.type;
		sideLightLevel	= profile.visCommon.light[eSIDE_LIGHT];
		camNo			= profile.visCommon.cameraId;
		camGrpType		= profile.visCommon.cameraGroupType;

		res = GetCycleTypeOne( alignMethod, camGrpType, sideLightLevel, size, sizeHyp, bMovableSide, cycleType );
		if( res == eOK )
			headCycleType[iHead] = cycleType;
	}

	// Search Best Cycle Type 
	// Sets 'eNG' if each cycle type('NONE', 'FLY', 'FIX', 'FLY SIDE') is mixed with other cycle type.
	for ( i=0; i<_countof(g_CycleTypeTable); i++)
	{
		res = eOK;
		cyType = g_CycleTypeTable[i];
		if( cyType == eCYCLE_TYPE_NONE ) 
		{
			res = eNG;
			break;
		}

		for (iHead=0; iHead<m_CycleData.nCount; iHead++)
		{
			if( (cyType & headCycleType[iHead]) == 0)
				res = eNG;
		}
		if( res == eOK )
			break;
	}

	if( res != eOK ) 
	{
		long cycle = 0;
		for (iHead=0; iHead<m_CycleData.nCount; iHead++)
		{
			StPlaceData& pd = m_CycleData.placeData[iHead];
			cycle = pd.cycle;

			TRACE( "headCycleType[%d] = %x \n", iHead, headCycleType[iHead]);
		}
		TRACE("Error Message4\n");
		g_StateReport.ReportErrorExCycle( eMEC_CYCLE_TYPE_ERROR, eERR_LEVEL_WARNING, eMOTOR_OK,
									m_SectionID, m_frRunningLane, cycle,
									0, HeadNone, 0, 0, 0, 0, 0, __LINE__,
									cycle,
									headCycleType[0], headCycleType[1], headCycleType[2], 
									headCycleType[3], headCycleType[4], headCycleType[5] );
		res = eOK;//eFAULT;
//		g_StateReport.ReportErrorEx2( eMEC_CYCLE_TYPE_ERROR, eERR_LEVEL_FREEZE, eMOTOR_OK,
//									m_SectionID, m_frRunningLane, 
//									0, HeadNone, 0, 0, 0, 0, 0, __LINE__,
//									cycle,
//									headCycleType[0], headCycleType[1], headCycleType[2], 
//									headCycleType[3], headCycleType[4], headCycleType[5] );
//		res = eNG;//eOK;//eFAULT;
	}

	aCycleType = cyType;
	return res;
}


// Cycle Check
//	- Virtual Pick & Normal Pick
eStatus
CSeqCycle::CheckCycleIntegrity( )
{
	CHead*		pHead = NULL;
	eFrontRear	frntRear = eFRONT;

	if ( m_CycleData.nCount == 0 )
		return eOK;

	//Get first step data in cycle for error message.
	StPlaceData& placeData = m_CycleData.placeData[ 0 ];
	pHead	 = placeData.pHead;

	//
	//Integrity check for virtual pick.
	//SPEC. Normal Pick & Virtual Pick should not be mixed. -> SM321 SPEC. Can be mixed.
	//----------------------------------------------------------------------------------
	if ( m_CycleData.bVirtualPick )
	{
		bool bVirtualPick = false;
		int  nVirtualPick = 0;
		for (int i=0; i<m_CycleData.nCount; i++)
		{
			StPlaceData& pd = m_CycleData.placeData[ i ];
			bVirtualPick = pd.profile.IsVirtualPick();
			if ( bVirtualPick )
				nVirtualPick++;
		}
		if ( nVirtualPick != 0 && nVirtualPick != m_CycleData.nCount) 
		{
			g_StateReport.ReportErrorExCycle( eMEC_CYCLE_TYPE_ERROR, eERR_LEVEL_FREEZE, eMOTOR_OK,
										m_SectionID, m_frRunningLane, m_CycleData.cycle,
										0, HeadNone, 0, 0, 0, 0, 0, __LINE__,
										m_CycleData.cycle );
			return eFAULT;
		}
	}

	return eOK;
}

//eStatus
//CSeqCycle::CheckNozzleTipWithSVSCamera()
//{
//	eStatus status = eOK;
//	THeadBlkList headBlocks;
//	::GetHeadBlocks( headBlocks, m_pGantry->Unit());
//	for (POS pos=headBlocks.GetHeadPosition(); pos!=NULL;)
//	{
//		CHeadBlock* pHB = headBlocks.GetNext( pos);
//		status = pHB->FindSVSRunTimeOffset();
//		if(status !=eOK )
//			return status;
//	}
//
//	return status;
//}
//
//WARNING NOTE: This function is used in MANUAL operation too, so do NOT use any memeber variable
//				inside of this function.
eStatus
CSeqCycle::GetCycleTypeOne( eProfileType aAlignMethod, eCameraGroupType aCamGrpType, long aSideLightLevel,
							double aSize, double aSizeHyp, bool abMovableSide, eCycleType& aCycleType )
{
	eStatus		res;
	eCycleType	cycleType;

	res = eOK;	 
	cycleType = eCYCLE_TYPE_NONE;

	switch( aAlignMethod )
	{
	case ePROFILE_TYPE_NONE:
		if( (aSize <= GetCompSizeSSP()) || (aSize <= GetCompSizeSSPRoot()) )
		{
			cycleType = eCycleType( cycleType | eCYCLE_TYPE_PP_SS | eCYCLE_TYPE_FIX_SS | eCYCLE_TYPE_FLY_SS | eCYCLE_TYPE_FLY_SIDE_SS);
		}
		if( aSize <= GetCompSizeSP() )
		{
			cycleType = eCycleType( cycleType | eCYCLE_TYPE_PP_S  | eCYCLE_TYPE_FIX_S	| eCYCLE_TYPE_FLY_S  | eCYCLE_TYPE_FLY_SIDE_S);
		}
		if( aSize <= GetCompSizeMP() )
		{
			cycleType = eCycleType( cycleType | eCYCLE_TYPE_PP_M  | eCYCLE_TYPE_FIX_M	| eCYCLE_TYPE_FLY_M);
			cycleType = eCycleType( cycleType | eCYCLE_TYPE_PP_L  | eCYCLE_TYPE_FIX_L);
		}
		if( GetCompSizeLM() < aSize  )
		{
			cycleType = eCycleType( cycleType | eCYCLE_TYPE_PP_L | eCYCLE_TYPE_FIX_L);
		}
		break;

	case ePROFILE_TYPE_CHIP:
	case ePROFILE_TYPE_LEAD:
	case ePROFILE_TYPE_BGA:
	case ePROFILE_TYPE_FLIPCHIP:
	case ePROFILE_TYPE_POLYGON:
	case ePROFILE_TYPE_ODDFORM:
//	case ALIGNMETHOD_SSA_MULTIBGA:
//	case ALIGNMETHOD_SSA_GDE:
		//
		//PART Size SPEC.
		// SM411: Square 14 mm (Fly FOV16 Camera)
		// SM421: Square 14 mm (Fly FOV16 Camera), 22 mm (Fly FOV25 Camera), Other Large (Upcamera)
		//
		if( IsFlyCam( aCamGrpType))
		{
			if( aSideLightLevel == 0 )
			{
				if( aSize <= GetCompSizeSSP() )	//SAME SIZE WITH LEFT  || (aSize <= GetCompSizeSSPRoot()) )
				{			//g_CompSizeSSP: 12 mm (square)
					cycleType = eCycleType( cycleType | eCYCLE_TYPE_FLY_SS | eCYCLE_TYPE_FLY_SIDE_SS);
				}
				if( aSize <= GetCompSizeSP() )
				{			//g_CompSizeSP: 17 mm
					cycleType = eCycleType( cycleType | eCYCLE_TYPE_FLY_S	| eCYCLE_TYPE_FLY_SIDE_S);
				}
				if( aSize <= GetCompSizeMP() )
				{			//g_CompSizeMP: 22 mm
					cycleType = eCycleType( cycleType | eCYCLE_TYPE_FLY_M);
				}
			}
			else
			{
				if( aSize <= GetCompSizeSSP() )	//SAME SIZE WITH LEFT || (aSize <= GetCompSizeSSPRoot()) )
				{			//g_CompSizeSSP
					cycleType = eCycleType( cycleType | eCYCLE_TYPE_FLY_SIDE_SS);
				}
				if( aSize <= GetCompSizeSP() )
				{			//g_CompSizeSP
					cycleType = eCycleType( cycleType | eCYCLE_TYPE_FLY_SIDE_S);
				}
				else if( aSize <= GetCompSizeMP() )	//g_CompSizeMP
				{	// aSize > GetCompSizeSP() && aSize <= GetCompSizeMP()
					if ( abMovableSide )
					{	//SM421 Style machine, make it error if it is mixed. Set cycle type with Not-a-Side.
						cycleType = eCycleType( cycleType | eCYCLE_TYPE_FLY_M);
					}
					else
					{	//SM411 Style machine, let it go by setting to use side illuminator.
						//Out of SPEC.
						cycleType = eCycleType( cycleType | eCYCLE_TYPE_FLY_SIDE_S);
					}
				}
			}	 
		}
		else if( IsUpCam( aCamGrpType))
		{
			if( aSize <= GetCompSizeSSP() )		//SAME SIZE WITH LEFT || (aSize <= GetCompSizeSSPRoot()) )
			{
				cycleType = eCycleType( cycleType | eCYCLE_TYPE_FIX_SS);
			}
			if( aSize <= GetCompSizeSP() )
			{
				cycleType = eCycleType( cycleType | eCYCLE_TYPE_FIX_S);
			}
			if( aSize <= GetCompSizeMP() )
			{
				cycleType = eCycleType( cycleType | eCYCLE_TYPE_FIX_M);
			}
			if( GetCompSizeLM() < aSize  )
			{
				cycleType = eCycleType( cycleType | eCYCLE_TYPE_FIX_L | eCYCLE_TYPE_FIX_M | eCYCLE_TYPE_FIX_S | eCYCLE_TYPE_FIX_SS);
			}
		}
		else if( IsLsoCam( aCamGrpType))
		{
			if( aSize <= GetCompSizeSSP() )		//SAME SIZE WITH LEFT || (aSize <= GetCompSizeSSPRoot()) )
			{
				cycleType = eCycleType( cycleType | eCYCLE_TYPE_LSO_SS);
			}
			if( aSize <= GetCompSizeSP() )
			{
				cycleType = eCycleType( cycleType | eCYCLE_TYPE_LSO_S);
			}
			if( aSize <= GetCompSizeMP() )
			{
				cycleType = eCycleType( cycleType | eCYCLE_TYPE_LSO_M);
			}
			if( GetCompSizeLM() < aSize  )
			{
				cycleType = eCycleType( cycleType | eCYCLE_TYPE_LSO_L | eCYCLE_TYPE_LSO_M | eCYCLE_TYPE_LSO_S | eCYCLE_TYPE_LSO_SS);
			}
		}
		else
		{
			res = eNG;
		}
		break;
	default :
		res = eNG;
		break;
	}

	if( res != eOK )
	{
		TRACE("Error Message3\n");
		g_StateReport.ReportErrorEx2( eMEC_CYCLE_TYPE_ERROR, eERR_LEVEL_FREEZE, eMOTOR_OK,
									m_SectionID, m_frRunningLane, 
									0, HeadNone, 0, 0, 0, 0, 0, __LINE__,
									aAlignMethod, aCamGrpType, aSideLightLevel, aSize, cycleType, 0);
	}
	
	aCycleType = cycleType;
	return res;
}


bool
CSeqCycle::IsCheckHalfPickUpCycle( )
{
	if ( !IsUseHalfPickup())
		return false;
	switch( m_CycleType )
	{
	case eCYCLE_TYPE_FLY_S	:
	case eCYCLE_TYPE_FLY_SIDE_S  :
	case eCYCLE_TYPE_FLY_M	:
	case eCYCLE_TYPE_FLY_SIDE_M  :
		return true;
	default :
		break;		
	}
	return false;
}


//Returns the cycle that Mirror 'WAIT' position open is good enough for further Operation.
//	if this condition is not satisfied, mirror should be opened fully except align step.
bool
CSeqCycle::IsMirWaitOpenCycle( )
{
	if ( IsCheckHalfPickUpCycle())
		return false;
	switch( m_CycleType )
	{
	case eCYCLE_TYPE_PP_SS	:
	case eCYCLE_TYPE_PP_S	:
	case eCYCLE_TYPE_PP_M	:
	case eCYCLE_TYPE_FLY_SS :
	case eCYCLE_TYPE_FLY_S	:
	case eCYCLE_TYPE_FLY_M	:
	case eCYCLE_TYPE_FLY_SIDE_SS :
	case eCYCLE_TYPE_FLY_SIDE_S  :
		return true;
	default :
		break;		
	}
	return false;
}


// NOT USED
bool
CSeqCycle::IsMirFastCloseCycle( )
{
	switch( m_CycleType )
	{
	case eCYCLE_TYPE_FLY_SS :
	case eCYCLE_TYPE_FLY_S	:
	// case eCYCLE_TYPE_FLY_M  :
	case eCYCLE_TYPE_FLY_SIDE_SS :
	case eCYCLE_TYPE_FLY_SIDE_S  :
		return true;
	default :
		break;		
	}
	return false;
}


// NOT USED
bool
CSeqCycle::IsFlyAlignCycle( )
{
	switch( m_CycleType )
	{
	case eCYCLE_TYPE_FLY_SS :
	case eCYCLE_TYPE_FLY_S	:
	case eCYCLE_TYPE_FLY_M	:
	case eCYCLE_TYPE_FLY_SIDE_SS :
	case eCYCLE_TYPE_FLY_SIDE_S  :
		return true;
	default :
		break;		
	}
	return false;
}

// NOT USED
bool
CSeqCycle::IsFastPrePickUpCycle( )
{
	switch( m_CycleType )
	{
	case eCYCLE_TYPE_FLY_SS :
	case eCYCLE_TYPE_FLY_SIDE_SS :
		return true;
	default :
		break;		
	}
	return false;
}

// NOT USED
bool
CSeqCycle::IsFastPostPickUpCycle( )
{
	switch( m_CycleType )
	{
	case eCYCLE_TYPE_FLY_SS :
	case eCYCLE_TYPE_FLY_SIDE_SS :
		return true;
	default :
		break;		
	}
	return false;
}

//Returns true if cycle type is Fly and (SS or SIDE_SS),
//		Speed is over 90%, and
//		Nozzle is small type.
bool
CSeqCycle::IsMirFastEscapeCycle( )
{
	bool	bOK = true;
	int 	iHead;
	
	switch( m_CycleType )
	{
	case eCYCLE_TYPE_FLY_SS :
	case eCYCLE_TYPE_FLY_SIDE_SS :
		if ( GetSpeedFactor( m_pGantry->Unit(), m_frRunningLane) >= 90)
		{
			for (iHead=0; iHead<m_CycleData.nCount; iHead++)
			{
				StPlaceData& pd = m_CycleData.placeData[iHead];
				eNozzleType nozType = eNOZZLE_TYPE_NONE;
				CNozzleId hdNozId = pd.pHead->GetNozzle();
				if ( hdNozId.IsDefined())
				{
					CAnc* pAnc = ::GetAnc( hdNozId.Anc());
					SYS_ASSERTFAUX( pAnc, return false; );
					SYS_ASSERTFAUX( pAnc->IsHole( hdNozId.HoleNum()), return false; );

					nozType = pAnc->GetNozzleType( hdNozId.HoleNum());
				}

				switch( nozType & eDEVICETYPE_ANCHOLE_TYPE_MASK )				 
				{
				case eDEVICETYPE_ANCHOLE_TYPE_A : // CP-45 퓖E퀋E갋
					bOK = true;
					break;
				case eDEVICETYPE_ANCHOLE_TYPE_G : // CP-55 퓖E퀋E갋
					bOK = false;
					switch ( nozType)
					{
					case eNOZZLE_TYPE_HG_N01 :
					case eNOZZLE_TYPE_HG_N02 :
					case eNOZZLE_TYPE_HG_N03 :
					case eNOZZLE_TYPE_HG_N04 :
						bOK = true;
						break;
					default :
						bOK = false;
						break;
					}
					break;
				default :
					bOK = false;
					break;
				}
			}
		}
		else
		{
			bOK = false;
		}
		break;
	default :
		bOK = false;
		break;		
	}
	return bOK;
}


bool
CSeqCycle::IsHeadSideDnCycle( )
{
	switch( m_CycleType )
	{
	case eCYCLE_TYPE_FLY_SS:
	case eCYCLE_TYPE_FLY_SIDE_SS:
	case eCYCLE_TYPE_FLY_SIDE_S:
		return true;
	default :
		break;		
	}
	return false;
}


bool
CSeqCycle::IsHeadSideUpCycle( )
{
	//Default is keeping Side Illuminator as Downed.
	//Looking for the condition should move up the side.
	//	1. Over 17 mm part ( >17mm)							[Ref. SM321 SPEC. 17 mm]
	//	2. Between 12 ~ 17 mm without having Side value.	[Ref. SM321 SPEC. 12 mm]
	switch( m_CycleType )
	{
//	case eCYCLE_TYPE_FLY_SS:
	case eCYCLE_TYPE_FLY_S:	// Give space for theta rotation by side UP.
	case eCYCLE_TYPE_FLY_M:
	case eCYCLE_TYPE_FLY_L:
//	case eCYCLE_TYPE_FLY_SIDE_SS:
//	case eCYCLE_TYPE_FLY_SIDE_S:

	//20070809 FIXED side is downed if part is Very Big because _FIX_SS is also selected.
	//		   Modified to do NOT down the side if part is big enough.
	case eCYCLE_TYPE_FLY_SIDE_M:	//Never assigned actually
	case eCYCLE_TYPE_FLY_SIDE_L:	//Never assigned actually
	case eCYCLE_TYPE_FIX_SS:

	case eCYCLE_TYPE_FIX_S:	// Give space for theta rotation by side UP.
	case eCYCLE_TYPE_FIX_M:
	case eCYCLE_TYPE_FIX_L:
		return true;
	default :
		break;		
	}
	return false;
}

// NOT USED
bool
CSeqCycle::IsMntRotFreeCycle( )
{
	if( IsMovableHeadSide4Gan( m_pGantry->Unit()))
	{
		switch( m_CycleType )
		{
		case eCYCLE_TYPE_FLY_SIDE_S:
			return false;
		default:
			return true;
		}
	}
	else
	{
		switch( m_CycleType )
		{
		case eCYCLE_TYPE_FLY_SIDE_S:
		case eCYCLE_TYPE_FLY_SIDE_M:
		case eCYCLE_TYPE_FLY_S:
		case eCYCLE_TYPE_FLY_M:
			return false;
		default :
			return true;
		}
	}
	return false;
}

// NOT USED
bool
CSeqCycle::IsFlySideAlignCycle( )
{
	switch( m_CycleType )
	{
	case eCYCLE_TYPE_FLY_SIDE_SS :
	case eCYCLE_TYPE_FLY_SIDE_S  :
	case eCYCLE_TYPE_FLY_SIDE_M  :
		return true;
	default :
		break;		
	}
	return false;
}


bool
CSeqCycle::IsFixAlignCycle( )
{
	switch( m_CycleType )
	{
	case eCYCLE_TYPE_FIX_SS :
	case eCYCLE_TYPE_FIX_S :
	case eCYCLE_TYPE_FIX_M :
	case eCYCLE_TYPE_FIX_L :
		return true;
	default :
		break;		
	}
	return false;
}


// NOT USED
bool
CSeqCycle::IsPpAlignCycle( )
{
	switch( m_CycleType )
	{
	case eCYCLE_TYPE_PP_SS :
	case eCYCLE_TYPE_PP_S :
	case eCYCLE_TYPE_PP_M :
	case eCYCLE_TYPE_PP_L :
		return true;
	default :
		break;		
	}
	return false;
}


void
CSeqCycle::GetPlaceNominalCAD(long aIdxPlace, CFxyzt& aPlaceLoc)
{
	SYS_ASSERT(0 <= aIdxPlace  &&  aIdxPlace < m_CycleData.nCount);
	int i = aIdxPlace;
	StPlaceData& pd = m_CycleData.placeData[i];

	if (m_CycleData.pPcbBoard  &&  pd.position.IsDefined())
	{
		CSMap	machineTMap;
		CFxyzt	placeOffset;
		CFxyzt  placeAngle;
		placeOffset.Zero();
		placeAngle.Zero();
		double  placeOffsetAngle;

		// Get Part place offset from MMI
		placeOffset = pd.profile.GetPartPlaceOffset();
		placeOffset.ZeroUndefines();
		placeOffsetAngle = placeOffset.t;

		aPlaceLoc = pd.position;
		m_CycleData.pPcbBoard->CadPlacementToMachineCoord(aPlaceLoc, m_GantryID, pd.arrayId.blockNo, pd.arrayId.arrayNo, pd.placeFidID);

		// Apply for Part place offset
		aPlaceLoc.x += placeOffset.x;
		aPlaceLoc.y += placeOffset.y;
		aPlaceLoc.z += placeOffset.z;
		aPlaceLoc.t += placeOffsetAngle;

		CStationId stationID = m_CycleData.pPcbBoard->GetStationId();

		aPlaceLoc += g_pConveyor->GetLogicalBoardStopThrd(stationID);

		if (stationID.Lane() == eLANE_REAR)
		{
			aPlaceLoc.y += g_pConveyor->GetActRailWidth(stationID);

			CPcbFunc* pPcbFunc = m_CycleData.pPcbBoard->GetPcb();
			aPlaceLoc.y -= pPcbFunc->pcbSize.y;
		}

		aPlaceLoc += m_CycleData.pPcbBoard->GetPlaceOrigin();

		//Set theta to short position from Absolute Zero within +/-200 degree.
		aPlaceLoc.t = ::NormalizeRWith(aPlaceLoc.t, 200);
	}
	else
		aPlaceLoc = pd.placePos;

	// If the coordinate could not be converted ASSERT and stop.
	SYS_ASSERT(aPlaceLoc.IsDefined());

	//	if ( aPlaceLoc.t <= -180.0)
	//		aPlaceLoc.t += 360.0;
	//
	// Because we do not guarrent the result of "::NormalizeRWith" above, we normalize 
	if (aPlaceLoc.t >= 200)
		aPlaceLoc.t -= 360;
	else if (aPlaceLoc.t <= -200)
		aPlaceLoc.t += 360;
}


void
CSeqCycle::GetPlaceNominalNoPV( long aIdxPlace, CFxyzt& aPlaceLoc, CFxyzt& aPlaceLocGan)
{
	SYS_ASSERT( 0<=aIdxPlace  &&  aIdxPlace<m_CycleData.nCount);
	int i = aIdxPlace;
	StPlaceData& pd = m_CycleData.placeData[i];
	bool bSectionRunning = g_StateReport.IsMachineRunning( m_SectionID, m_frRunningLane );
	CRunOptions&	rRunOptions = *::GetRunOptions(m_frRunningLane);

	if ( m_CycleData.pPcbBoard  &&  pd.position.IsDefined())
	{
		CSMap	machineTMap;
		CFxyzt	placeOffset;
		CFxyzt  placeAngle;
		placeOffset.Zero();
		placeAngle.Zero();
		double  placeOffsetAngle;
		
		// Get Part place offset from MMI
		placeOffset = pd.profile.GetPartPlaceOffset();
		placeOffset.ZeroUndefines();
		
		// Apply Apc Offset
		if (g_pApcMgr
			 && g_pApcMgr->IsSupportInlineCalib( pd.pHead->GetNozzle( ) )
			 && bSectionRunning
			 && (m_CycleData.pPcbBoard != NULL))
		{
			CStationId	stationId = m_CycleData.pPcbBoard->GetStationId( );
			CGantry*	pGantry = pd.pHead->Gantry( );
			eFrontRear  fr = (stationId.Lane( ) == eLANE_REAR ? eREAR : eFRONT);

			if (g_pApcMgr->IsInlineCalibOn(stationId) && m_CycleData.pPcbBoard->GetReassignRefType(pd.placeCadID) == 0)
			{
				CStationId	stationId = m_CycleData.pPcbBoard->GetStationId( );
				GantryID	gantryId = pd.pHead->Gantry( )->Unit( );
				long		areaIndex = 0;
				bool		bMultiPcb = g_pPcbSet->IsMultiPcb( m_CycleData.stationId.Lane( ) );
				long		blockNo = pd.arrayId.blockNo;
				long		arrayNo = pd.arrayId.arrayNo;
				long		refNo = pd.placeMapID;

				if (bMultiPcb)
				{
					blockNo = (long)(m_CycleData.pPcbBoard ? m_CycleData.pPcbBoard->GetPcbID( ) : g_pPcbSet->PreferPcbId( m_CycleData.stationId.Lane( ) ));

					if (g_StateReport.IsIndependentLanes( ) && stationId.Lane( ) == eLANE_REAR)
					{
						blockNo -= eMAX_MULTIPCB_PER_PCB;
					}
				}
				CFxyzt apcStepOffset = g_pApcMgr->GetApcStepOffset( stationId, gantryId, blockNo, arrayNo, refNo, pd.position );
				placeOffset.t -= apcStepOffset.t;
				Telemetry( EV_CLASSHEAD, pd.pHead->Unit( ), "H%s  apcStepTOffset=%7.3f] block=%d, array=%d, ref=%d", pd.pHead->GetName( ), apcStepOffset.t, blockNo, arrayNo, refNo );
			}
		}

		// Calculate Map of Place Offset by Machine Coordinate
		placeAngle.t = pd.position.t;
		placeOffsetAngle = placeOffset.t;

		m_CycleData.pPcbBoard->MapPlacementToMachineCoord( placeAngle, m_GantryID, pd.arrayId.blockNo, pd.arrayId.arrayNo, pd.placeFidID );
		machineTMap.SetOrigin( CFxyzt( 0, 0, 0, placeAngle.t ) );
		machineTMap.MapToXY( placeOffset);

		aPlaceLoc = pd.position;
		if (rRunOptions.IsBGAAutoAccuracyReturn())
		{
			if(pd.pickInspR == ePIR_NEG_PLACE__PLACE)
				aPlaceLoc.t = 0.0;
			else
			{
				aPlaceLoc.t += GetPartR(pd.feederID);
				aPlaceLoc.t *= -1;
			}		
		}
		m_CycleData.pPcbBoard->MapPlacementToMachineCoord( aPlaceLoc, m_GantryID, pd.arrayId.blockNo, pd.arrayId.arrayNo, pd.placeFidID );

		// Apply for Part place offset
		aPlaceLoc.x += placeOffset.x;
		aPlaceLoc.y += placeOffset.y;
		aPlaceLoc.z += placeOffset.z;
		aPlaceLoc.t += placeOffsetAngle;

		//Set theta to short position from Absolute Zero within +/-200 degree.
		Telemetry( EV_CLASSHEAD, 0, "***Before NormalizeRWith aPlaceLoc.t = %.3f", aPlaceLoc.t);
		aPlaceLoc.t = ::NormalizeRWith( aPlaceLoc.t, 200 );
		Telemetry( EV_CLASSHEAD, 0, "***After NormalizeRWith aPlaceLoc.t = %.3f", aPlaceLoc.t);
	}
	else
		aPlaceLoc = pd.placePos;

	if (rRunOptions.IsBGAAutoAccuracyMeasure())
	{
		CFxyzt placeLoc = rRunOptions.GetBGAAutoAccuracyMeasurePlacePos(pd.head);
		if (placeLoc.IsDefinedXY())
		{
			aPlaceLoc.x = placeLoc.x;
			aPlaceLoc.y = placeLoc.y;
			aPlaceLoc.z = placeLoc.z;
		}
	}
	// If the coordinate could not be converted ASSERT and stop.
	SYS_ASSERT( aPlaceLoc.IsDefined() );

//	if ( aPlaceLoc.t <= -180.0)
//		aPlaceLoc.t += 360.0;
//
	// Because we do not guarrent the result of "::NormalizeRWith" above, we normalize 
	if(aPlaceLoc.t >= 200) 
		aPlaceLoc.t -= 360;
	else if(aPlaceLoc.t <= -200)
		aPlaceLoc.t += 360;
	// Provide a gantry XY position too.
	aPlaceLocGan = aPlaceLoc - CFxyzt(pd.pHead->GetOffset(), 0, 0);
}


void
CSeqCycle::GetPlaceNominalPV( long aIdxPlace, CFxyzt& aPlaceLoc, CFxyzt& aPlaceLocGan)
{
	SYS_ASSERT( 0<=aIdxPlace  &&  aIdxPlace<m_CycleData.nCount);
	int i = aIdxPlace;
	StPlaceData& pd = m_CycleData.placeData[i];
	CRunOptions&	rRunOptions = *::GetRunOptions(m_frRunningLane);

	GetPlaceNominalNoPV( aIdxPlace, aPlaceLoc, aPlaceLocGan);

	// Get the BEST PART VECTOR available to offset the placement.
	CFxyzt partVector = ( pd.pHead->GetPartVectorPickup().IsDefined()
										? pd.pHead->GetPartVectorPickup()
										: pd.pHead->GetPartVectorPrepickup());

	// Pass only the NOMINAL PV to offset the placement.
	GetHeadPositionThruPV( pd.pHead, partVector, pd.profile.common.bPolarized, aPlaceLoc);
	// aPlaceLoc is the NOMINAL HEAD position required for this placement

	// Add the PART height to the target location
//	aPlaceLoc.z += (pd.profile.common.height - pd.profile.common.depthZ - pd.profile.common.pinlength);
	aPlaceLoc.z += (pd.profile.common.height - pd.profile.common.depthZ);

	if (rRunOptions.IsBGAAutoAccuracyMeasure())
	{
		CFxyzt placeLoc = rRunOptions.GetBGAAutoAccuracyMeasurePlacePos(pd.head);
		if (placeLoc.IsDefinedXY())
		{
			aPlaceLoc.x = placeLoc.x;
			aPlaceLoc.y = placeLoc.y;
			aPlaceLoc.z = placeLoc.z;
		}
	}
	// Provide a gantry XY position too.
	aPlaceLocGan = aPlaceLoc - CFxyzt(pd.pHead->GetOffset(), 0, 0); 
}

bool gDbgApplyRunoutOffsetForLSO=false;
void
CSeqCycle::GetPlaceCorrectedPV( long aIdxPlace, ePartVector& aPvStatus, CFxyzt& aPlaceLoc, CFxyzt& aPlaceLocGan)
{
	SYS_ASSERT( 0<=aIdxPlace  &&  aIdxPlace<m_CycleData.nCount);
	CFxy		 runOutOffset;
	int			 i = aIdxPlace;
	StPlaceData& pd = m_CycleData.placeData[i];
	CFxyzt		nominalPos(0, 0, 0, 0);

	// Calculate Cad Position for Apc.
	nominalPos = aPlaceLoc;
	GetPlaceNominalCAD(aIdxPlace, nominalPos);
	///////////////////////////////////

	GetPlaceNominalNoPV( aIdxPlace, aPlaceLoc, aPlaceLocGan);

	// Get the BEST PART VECTOR available to offset the placement.
	CFxyzt partVector( pd.pHead->GetPartVectorAligned( aPvStatus));
	if ( aPvStatus != ePART_VECTOR_OK)
		partVector = pd.pHead->GetPartVectorPickup();

	GetHeadPositionThruPV( pd.pHead, partVector, pd.profile.common.bPolarized, aPlaceLoc);
	// aPlaceLoc is the HEAD position required for this placement

	eCameraGroupType
			camGrpType	= pd.profile.visCommon.cameraGroupType;
	bool	bStageCameraAlign= (camGrpType & eCAMERA_GROUP_STAGE) != 0;
	bool	bFlyCameraAlign	= (camGrpType & eCAMERA_GROUP_FLY) != 0;
	bool	bUpCameraAlign	= (camGrpType & eCAMERA_GROUP_UP) != 0;
	bool	bLSOCameraAlign	= (camGrpType & eCAMERA_GROUP_LSO) != 0;

	// <CHANGWON 20071011> Only fly camera use 'FlyToFixOffset'.
	if ( bFlyCameraAlign )
	{
		// CORRECT HEAD POSITION due to RUN OUT at the Placement Height
		pd.pHead->GetRunOutOffset( aPlaceLoc.t, runOutOffset);
		aPlaceLoc -= runOutOffset;
	}

	// CORRECT HEAD POSITION due to RUN OUT at the Scanning Height
	CGantry* pGantry	= pd.pHead->Gantry();
	if ( pGantry )
	{
		CFlyCameraAssy* pFlyCamerasAssy = ::GetFlyCameraAssyByGantryID( pGantry->Unit());
//		CFlyCameras* pFlyCameras = ::GetFlyCameras( pGantry->Unit());
		if ( pFlyCamerasAssy && bFlyCameraAlign ) 
		{
			double	scanZ;
			long	scanT, placeT;
			CFxy	alignRunoutOffset(0.,0.), alignRunoutOffset4Scan(0.,0.);
			pFlyCamerasAssy->GetHeadScanZT( pd.head, scanZ, scanT);
			placeT = ::GetNearest45( (long)aPlaceLoc.t);
			if ( placeT-scanT != 0)
			{
				pd.pHead->GetAlignRunOutOffset( aPlaceLoc.t, alignRunoutOffset);
				if (scanT != 0)	// Because mountOffset's Base Angle is 0 Degree.
				{
					double	rotAngle = (placeT-scanT) * DtoR;
					CFxy	tmpOffset(0., 0.);
					pd.pHead->GetAlignRunOutOffset( (double)scanT, tmpOffset);
					alignRunoutOffset4Scan.x = tmpOffset.x * cos(rotAngle) - tmpOffset.y * sin(rotAngle);
					alignRunoutOffset4Scan.y = tmpOffset.x * sin(rotAngle) + tmpOffset.y * cos(rotAngle);
					alignRunoutOffset -= alignRunoutOffset4Scan;
				}
				aPlaceLoc -= alignRunoutOffset; 
			}
		}
		else
		if ( gDbgApplyRunoutOffsetForLSO && bLSOCameraAlign && pFlyCamerasAssy )
		{
			if ( g_SysConst.IsUseFlyRunOutToLSO() )
			{
				CUpCameraAssy* pLso = (pGantry ? ::GetUpCameraAssyByGantryID( pGantry->Unit()) : NULL );

				double	scanZ;
				long	scanT, placeT;
				CFxy	alignRunoutOffset(0.,0.), alignRunoutOffset4Scan(0.,0.);
				pLso->GetHeadScanZT( pd.head, scanZ, scanT);
				placeT = ::GetNearest45( (long)aPlaceLoc.t);
				if ( placeT-scanT != 0)
				{
					pd.pHead->GetAlignRunOutOffset( aPlaceLoc.t, alignRunoutOffset);
					if (scanT != 0)	// Because mountOffset's Base Angle is 0 Degree.
					{
						double	rotAngle = (placeT-scanT) * DtoR;
						CFxy	tmpOffset(0., 0.);
						pd.pHead->GetAlignRunOutOffset( (double)scanT, tmpOffset);
						alignRunoutOffset4Scan.x = tmpOffset.x * cos(rotAngle) - tmpOffset.y * sin(rotAngle);
						alignRunoutOffset4Scan.y = tmpOffset.x * sin(rotAngle) + tmpOffset.y * cos(rotAngle);
						alignRunoutOffset -= alignRunoutOffset4Scan;
					}
					aPlaceLoc -= alignRunoutOffset; 
				}
			}



		}
	}

	//Applying absolute XY shift offset due to gantry distortion between Upward Camera Position and place position.
	if ( bUpCameraAlign )
	{
		long sectionStageCamCnt = ::GetUpCameraCountInSection( m_SectionID);
// 		CUpCameraAssy* pStageCamerasAssy ;
		CStageCameras* pStageCameras;
		StPlaceData::eASSIGNED_STEP_TYPE stepType = StPlaceData::eASSIGNED_STEP_PICK_MAX;
		CFeederId feederID = pd.pHead->GetPickedFeeder();
		SectionID sectionID = ALL_SECTIONS;
		eFrontRear fr = eBOTH;
		long upCamUnit = pd.camId;

		if (sectionStageCamCnt > 1 && !pGantry->GetSharingGantry())
		{
			if (upCamUnit)
			{
	 			pStageCameras = ::GetStageCameras(upCamUnit);
// 	 			pStageCamerasAssy = ::GetUpCameraAssy( upCamUnit);
				SYS_ASSERT(pStageCameras);
			}
			else if ( feederID.IsDefined())
			{
				if ( feederID.Type() == eFEEDTYPE_TAPE )
				{
					CTapeFeeder* pTapeFeeder = ::GetTapeFeeder( feederID ); 
					sectionID = pTapeFeeder->GetSectionID();
					fr = pTapeFeeder->GetFrontOrRear();
				}
				else if ( feederID.Type() == eFEEDTYPE_STICK )
				{
					CStickFeeder* pFeeder = ::GetStickFeeder( feederID );
					sectionID = pFeeder->GetSectionID();
					fr = pFeeder->GetFrontRear();
				}
				else if ( feederID.Type() == eFEEDTYPE_TRAY )
				{
					CTrayFeeder* pTrayFeeder = ::GetTrayFeeder( feederID );
					sectionID = pTrayFeeder->GetSectionID();
					if ( pTrayFeeder->IsSideTrayType() || pTrayFeeder->GetFrontOrRear() == eBOTH)
					{
						if (pd.pickPos.y > pd.placePos.y )
							fr = eFRONT;
						else
							fr = eREAR;
					}
					else
						fr = pTrayFeeder->GetFrontOrRear();
				}
				else
					SYS_ASSERT(0);

				pStageCameras = ::GetBestStageCameras( sectionID, fr );
// 				pStageCamerasAssy = ::GetBestUpCameraAssy( sectionID, fr );
				SYS_ASSERT(pStageCameras);
				upCamUnit = pStageCameras->GetCameraID();
			}
			else
			{
				sectionID	= pGantry->GetSectionID();
				fr			= pGantry->GetFrontOrRear();
				pStageCameras = ::GetBestStageCameras( sectionID, fr );
// 				pStageCamerasAssy = ::GetBestUpCameraAssy( sectionID, fr );
				SYS_ASSERT(pStageCameras);
				upCamUnit = pStageCameras->GetCameraID();
			}
		}
		else
		{
			sectionID	= pGantry->GetSectionID();
			fr			= pGantry->GetFrontOrRear();
			pStageCameras = ::GetBestStageCameras( sectionID, fr );
// 			pStageCamerasAssy = ::GetBestUpCameraAssy( sectionID, fr );
			SYS_ASSERT(pStageCameras);
			upCamUnit = pStageCameras->GetCameraID();
		}

// 		long upCamUnit = pd.profile.visCommon.cameraId;
// 		SYS_ASSERT( upCamUnit > 0 && upCamUnit <= SYS_MAX_CAMERA_UP );
// 		CCameraUp* pCamUp = ::GetCameraUp( upCamUnit);
// 
// 		CUpCameraAssy* pUpCamerasAssy = ::GetUpCameraAssyByUpCam( pCamUp);
// //		CUpCameras* pUpCameras = ::GetUpCameras( upCamUnit);
// //		if ( pUpCameras )
// 		if ( pUpCamerasAssy && !pStageCameras)
// 		{
// 			//	double	scanZ;
// 			//	long	scanT, placeT;
// 			CFxyt	upCamMountOffset(0.0, 0.0, 0.0);
// 			//	CFxyt	upCamMountOffset4Scan(0.0, 0.0, 0.0);
// 			//	pUpCameras->GetHeadScanZT( pd.head, scanZ, scanT);
// 			//	placeT = ::GetNearest45( aPlaceLoc.t);
// 			//	if ( placeT-scanT != 0 )
// 			{
// 
// 			pd.pHead->GetUpCamMountOffset( upCamUnit, 0, upCamMountOffset,pd.profile.type );
// 
// 			//?	pd.pHead->GetUpCamMountOffset( upCamUnit, aPlaceLoc.t, upCamMountOffset );
// 			//	if ( scanT != 0 )	// Because mountOffset's Base Angle is 0 Degree.
// 			//	{
// 			//		double	rotAngle = (placeT-scanT) * DtoR;
// 			//		CFxy	tmpOffset(0.0, 0.0);
// 			//		pd.pHead->GetUpCamMountOffset( upCamUnit, (double)scanT, tmpOffset);
// 			//		upCamMountOffset4Scan.x = tmpOffset.x * cos(rotAngle) - tmpOffset.y * sin(rotAngle);
// 			//		upCamMountOffset4Scan.y = tmpOffset.x * sin(rotAngle) + tmpOffset.y * cos(rotAngle);
// 			//		upCamMountOffset -= upCamMountOffset4Scan;
// 			//	}
// 
// 			aPlaceLoc -= upCamMountOffset; 
// 
// 			}
// 		}

		// For the stage camera such as the EXCEN and the EXCEN PRO.
		if( pGantry )
		{
			if ( /*pStageCamerasAssy && */pStageCameras)
			{
				CFxy	alignRunoutOffset(0.,0.), alignRunoutOffset4Scan(0.,0.);

				// The effect of the Thermal Mapping.
				CThermalMapping* pgantryThermal = pGantry->GetThermalMapping();
				if ( pgantryThermal && pgantryThermal->GetEnhancedThermalMap() )
				{
					CFxy thermalEffect(0,0);
					CHeadBlock* pHB = pd.pHead->GetHeadBlock();
					int iApplyThermalEffect = 0;
					CRunOptions&	rRunOptions = *::GetRunOptions(m_frRunningLane);
					if (!rRunOptions.IsBGAAutoAccuracyReturn())
					{
						if (pHB->GetRefCount())
						{
							int scanGroup = pStageCameras->GetScanGroup(pd.pHead->GetHeadID());
							eScanStyle scanStyle = pStageCameras->GetScanStyle(scanGroup);
							if (scanStyle == eSCAN_STYLE_UP_CAMERA_CENTERED || scanStyle == eSCAN_STYLE_UP_CAMERA_MFOV)
							{
								thermalEffect = pStageCameras->GetThermalOffsetFrFidExRef();
								iApplyThermalEffect = 1;
							}
							else
							{
								thermalEffect = pStageCameras->GetThermalOffsetFrFidCam();
								iApplyThermalEffect = 2;
							}
						}
						else
						{
							thermalEffect = pStageCameras->GetThermalOffsetFrFidCam();
							iApplyThermalEffect = 3;
						}

						Telemetry(EV_CLASSHEAD, pd.pHead->Unit(), "H%s  Thermal Effect Offset[%7.3f,%7.3f]-[%7.3f,%7.3f] Idx:%d", pd.pHead->GetName(), aPlaceLoc.x, aPlaceLoc.y, thermalEffect.x, thermalEffect.y, iApplyThermalEffect);
					}
					aPlaceLoc -= thermalEffect;
					if( pgantryThermal->IsUseRThermalCorrect(pd.pHead->GetSpindle()) == true ){
						CFxy rThermalOffset = pgantryThermal->GetRThermalCorrectOffset();
						Telemetry( EV_CLASSHEAD, pd.pHead->Unit(), "H%s Raxis Thermal Offset[%7.3f,%7.3f]-[%7.3f,%7.3f]", pd.pHead->GetName(), aPlaceLoc.x, aPlaceLoc.y, rThermalOffset.x, rThermalOffset.y); 
						aPlaceLoc -= rThermalOffset;
					}
				}

				// Nozzle Insp Test
				// The align offset of a spindle.
				CHeadBlock* pHB = pd.pHead->GetHeadBlock();
				CFxyt	alignCOROfNozzle(0.0, 0.0, 0.0);
				CFxyt	pickPlaceCOROfNozzle(0.0, 0.0, 0.0);
				if ((g_GlobalSmt.GetAccuracyPCB(fr) || g_SysConst.IsSupportAllPCB())
					&& g_SysConst.IsSupportNozzleTipSearchApplyCOR()
					&& pHB->GetNozzleTipInspection(pd.pHead->GetHeadID(), pd.pHead->GetSpindle(), pd.pHead->GetNozzle(), alignCOROfNozzle, eTipPosAtAlignPosZUpCOR)
					&& pHB->GetNozzleTipInspection(pd.pHead->GetHeadID(), pd.pHead->GetSpindle(), pd.pHead->GetNozzle(), pickPlaceCOROfNozzle, eTipPosAtPickPosZDownCOR))
				{
					Telemetry(EV_CLASSHEAD, pd.pHead->Unit(), "H%s  Pick Place Nozzle COR Offset[%7.3f]=[%7.3f,%7.3f]", pd.pHead->GetName(), aPlaceLocGan.t, pickPlaceCOROfNozzle.x, pickPlaceCOROfNozzle.y);
					aPlaceLoc.x -= pickPlaceCOROfNozzle.x;
					aPlaceLoc.y -= pickPlaceCOROfNozzle.y;
				}
				

				// The align offset of a spindle.
				CFxyt	pickZDnCOR(0.0, 0.0, 0.0);
				CFxyt	pickZUpCOR(0.0, 0.0, 0.0);
				CFxyt	pickZDnRunoutOffsetOfNozzle(0.0, 0.0, 0.0);
				CFxyt	pickZUpRunoutOffsetOfNozzle(0.0, 0.0, 0.0);
				if ((g_GlobalSmt.GetAccuracyPCB(fr) || g_SysConst.IsSupportAllPCB())
					&& g_SysConst.IsSupportNozzleTipSearchApplyFlyToFix()
					&& pd.pHead->GetNozzleRunOutPickPosZDown(aPlaceLoc.t, pickZDnRunoutOffsetOfNozzle)
					&& pd.pHead->GetNozzleRunOutPickPosZUp(aPlaceLoc.t, pickZUpRunoutOffsetOfNozzle)
					&& pHB->GetNozzleTipInspection(pd.pHead->GetHeadID(), pd.pHead->GetSpindle(), pd.pHead->GetNozzle(), pickZDnCOR, eTipPosAtPickPosZDownCOR)
					&& pHB->GetNozzleTipInspection(pd.pHead->GetHeadID(), pd.pHead->GetSpindle(), pd.pHead->GetNozzle(), pickZUpCOR, eTipPosAtPickPosZUpCOR))
				{
					CSMap	scanRRot;
					scanRRot.SetOrigin(CFxyzt(0, 0, 0, aPlaceLoc.t));
					scanRRot.MapToXY(pickZDnRunoutOffsetOfNozzle);
					scanRRot.MapToXY(pickZUpRunoutOffsetOfNozzle);

					pickZDnRunoutOffsetOfNozzle -= pickZDnCOR;
					pickZUpRunoutOffsetOfNozzle -= pickZUpCOR;

					CFxyt flytoFix(0, 0, 0);
					flytoFix.x = (pickZDnRunoutOffsetOfNozzle.x - pickZUpRunoutOffsetOfNozzle.x);
					flytoFix.y = (pickZDnRunoutOffsetOfNozzle.y - pickZUpRunoutOffsetOfNozzle.y);

					Telemetry(EV_CLASSHEAD, pd.pHead->Unit(), "H%s  FlytoFix Insp Offset[%7.3f]=[%7.3f,%7.3f]", pd.pHead->GetName(), aPlaceLocGan.t, flytoFix.x, flytoFix.y);

					aPlaceLoc.x -= flytoFix.x;
					aPlaceLoc.y -= flytoFix.y;
				}
				else
				{
					// The align offset of a spindle.
					pd.pHead->GetRunOutOffset( aPlaceLoc.t, alignRunoutOffset);
					Telemetry( EV_CLASSHEAD, pd.pHead->Unit(), "H%s  Head to Align Offset[%7.3f]=[%7.3f,%7.3f]", pd.pHead->GetName(), aPlaceLocGan.t, alignRunoutOffset.x, alignRunoutOffset.y); 
					aPlaceLoc -= alignRunoutOffset; 
				}
				
				// The mount offset of a spindle.
				CFxyt	upCamMountOffset(0.0, 0.0, 0.0);
				{
					double temp = aPlaceLoc.t;

					if (0 /*g_FacDebug.IsUseNegativePick() && g_GlobalSmt.GetAccuracyPCB(fr)*/)
					{
						if ( pd.pickInspR == ePIR_NEG_PLACE__PLACE)
						{
							temp += pd.placePos.t + GetPartR(pd.feederID); 	// Because the Zero degree of the Motor is used in case of "ePIR_NEG_PLACE__PLACE".
						}
						
						// Temporary Code.... Fix it later. // ??????????????????????
						// S ...
						if ( pd.feederID.Type() == eFEEDTYPE_TRAY)
						{
							CTrayFeeder*	pTrayFeeder	= ::GetTrayFeeder( pd.feederID);
							
							if ( pTrayFeeder && pTrayFeeder->IsSideTrayType() )	// Side Tray Feeder.
							{
								if ( ( pGantry->IsFront() && pTrayFeeder->GetTrayPartR() == 180) ||
									( !pGantry->IsFront() && pTrayFeeder->GetTrayPartR() == 0) )	// Front gantry. Because the part should be located based on the rear side.
								{
									temp += 180.;
								}
							}
						}
						// E
					}
					pd.pHead->GetUpCamMountOffset( upCamUnit, temp/*aPlaceLoc.t*/, upCamMountOffset, pd.profile.type, pd.profile.partType );
					Telemetry( EV_CLASSHEAD, pd.pHead->Unit(), "H%s  AngleType(%d) Mount[%7.3f]=[%7.3f,%7.3f]", pd.pHead->GetName(), (int)pd.pickInspR, temp /*aPlaceLocGan.t*/, upCamMountOffset.x, upCamMountOffset.y); 
					if (m_CycleData.pPcbBoard)
					{
						CConveyor*	pConv = ::GetConveyorObj();
						CStationId stId = m_CycleData.pPcbBoard->GetStationId();
						if (pConv->IsWork(stId) && pConv->IsBuffer(stId))
						{
							aPlaceLoc -= g_FacDebug.GetAddMountOffset(pGantry->GetFrontOrRear(), stId.Lane());
						}
					}
					aPlaceLoc -= upCamMountOffset;
					Telemetry( EV_CLASSHEAD, pd.pHead->Unit(), "H%s  MountOffset(%d) Head[%7.3f]=[%7.3f,%7.3f]", pd.pHead->GetName(), (int)pd.pickInspR, temp /*aPlaceLocGan.t*/, aPlaceLoc.x, aPlaceLoc.y); 
				}
				if( g_FacDebug.IsUseGantryOffset() && m_CycleData.stationId !=  pGantry->GetPreferredStation() )
				{
					CFxy	mountOffset(0.0, 0.0);
					mountOffset = g_FacDebug.GantryMapOffset( pGantry->Unit() );
					Telemetry( EV_CLASSHEAD, pd.pHead->Unit(), "H%s  GantryMapOffset=[%7.3f,%7.3f]", pd.pHead->GetName(), mountOffset.x, mountOffset.y); 
					aPlaceLoc -= mountOffset; 
				}

				if ( g_SysConst.IsUseEnhancedThataCal())
				{
					double headScanR = pStageCameras->GetAlignAngle(pd.pHead->GetHeadID());
					double caliTAxisDiff = pStageCameras->CalcMountRelativeR(pd.pHead,aPlaceLoc.t, headScanR);
					Telemetry(EV_CLASSHEAD, pd.pHead->Unit(), "H%s Cali Angle Offset=[%7.3f] Scan Angle =[%7.3f] Place Angle =[%7.3f]", pd.pHead->GetName(), caliTAxisDiff, headScanR, aPlaceLoc.t);

					aPlaceLoc.t -= caliTAxisDiff;
				}
			}
		}
		else
			SYS_ASSERT(0);
	}
	else if ( bFlyCameraAlign )
	{
		CFlyCameraAssy* pFlyCamera = (pGantry ? ::GetFlyCameraAssyByGantryID( pGantry->Unit()) : NULL );

		//double	scanZ;
		//long	scanT, placeT;
		//CFxy	alignRunoutOffset(0.,0.), alignRunoutOffset4Scan(0.,0.);
		CFxyt	flyMountOffset(0.0, 0.0, 0.0);
		long	lsoCamUnit = pFlyCamera->Unit();

		//pFlyCameras->GetHeadScanZT( pd.head, scanZ, scanT);
		//placeT = ::GetNearest45( aPlaceLoc.t);
		//if ( placeT-scanT != 0)
		//{

		//pd.pHead->GetMountOffset( aPlaceLoc.t, flyMountOffset);
		pd.pHead->GetMountOffset( lsoCamUnit, aPlaceLoc.t, flyMountOffset, pd.profile.type, pd.profile.partType );
		Telemetry( EV_CLASSHEAD, pd.pHead->Unit(), "H%s  Mount[%7.3f]=[%7.3f,%7.3f]", pd.pHead->GetName(), aPlaceLocGan.t, flyMountOffset.x, flyMountOffset.y);  
		if (m_CycleData.pPcbBoard)
		{
			CConveyor*	pConv = ::GetConveyorObj();
			CStationId stId = m_CycleData.pPcbBoard->GetStationId();
			if (pConv->IsWork(stId) && pConv->IsBuffer(stId))
			{
				aPlaceLoc -= g_FacDebug.GetAddMountOffset(pGantry->GetFrontOrRear(), stId.Lane());
			}
		}
		aPlaceLoc -= flyMountOffset; 

		//if (scanT != 0)	// Because mountOffset's Base Angle is 0 Degree.
		//{
		//	double	rotAngle = (placeT-scanT) * DtoR;
		//	CFxy	tmpOffset(0., 0.);
		//	pd.pHead->GetAlignRunOutOffset( (double)scanT, tmpOffset);
		//	alignRunoutOffset4Scan.x = tmpOffset.x * cos(rotAngle) - tmpOffset.y * sin(rotAngle);
		//	alignRunoutOffset4Scan.y = tmpOffset.x * sin(rotAngle) + tmpOffset.y * cos(rotAngle);
		//	alignRunoutOffset -= alignRunoutOffset4Scan;
		//}
		//}
	}
	else if ( bLSOCameraAlign )
	{
		CUpCameraAssy* pLso = (pGantry ? ::GetUpCameraAssyByGantryID( pGantry->Unit()) : NULL );
		//double	scanZ;
		//long	scanT, placeT;
		//CFxy	alignRunoutOffset(0.,0.), alignRunoutOffset4Scan(0.,0.);
		CFxyt	lsoMountOffset(0.0, 0.0, 0.0);
		
		//pFlyCameras->GetHeadScanZT( pd.head, scanZ, scanT);
		//placeT = ::GetNearest45( aPlaceLoc.t);
		//if ( placeT-scanT != 0)
		//{
		long lsoCamUnit = pLso->Unit();
		//pd.pHead->GetMountOffset( aPlaceLoc.t, mountOffset);
		
		pd.pHead->GetUpCamMountOffset( lsoCamUnit, aPlaceLoc.t, lsoMountOffset, pd.profile.type, pd.profile.partType );
		Telemetry( EV_CLASSHEAD, pd.pHead->Unit(), "H%s  Mount[%7.3f]=[%7.3f,%7.3f]", pd.pHead->GetName(), aPlaceLocGan.t, lsoMountOffset.x, lsoMountOffset.y); 
		if (m_CycleData.pPcbBoard)
		{
			CConveyor*	pConv = ::GetConveyorObj();
			CStationId stId = m_CycleData.pPcbBoard->GetStationId();
			if (pConv->IsWork(stId) && pConv->IsBuffer(stId))
			{
				aPlaceLoc -= g_FacDebug.GetAddMountOffset(pGantry->GetFrontOrRear(), stId.Lane());
			}
		}
		aPlaceLoc -= lsoMountOffset; 
		
		//if (scanT != 0)	// Because mountOffset's Base Angle is 0 Degree.
		//{
		//	double	rotAngle = (placeT-scanT) * DtoR;
		//	CFxy	tmpOffset(0., 0.);
		//	pd.pHead->GetAlignRunOutOffset( (double)scanT, tmpOffset);
		//	alignRunoutOffset4Scan.x = tmpOffset.x * cos(rotAngle) - tmpOffset.y * sin(rotAngle);
		//	alignRunoutOffset4Scan.y = tmpOffset.x * sin(rotAngle) + tmpOffset.y * cos(rotAngle);
		//	alignRunoutOffset -= alignRunoutOffset4Scan;
		//}
		
		CHeadBlock* pHB = pd.pHead->GetHeadBlock();
		
		CFxyt hbScaleMountOffset = pHB->GetScaleMountOffset(pd.pHead->GetSpindle());
		aPlaceLoc -= hbScaleMountOffset;
		
		CFxyt hbMountOffset = pHB->GetMountOffset();
		aPlaceLoc -= hbMountOffset;
	}

	bool bSectionRunning = g_StateReport.IsMachineRunning(m_SectionID, m_frRunningLane);

	if ( g_pApcMgr 
		&& bSectionRunning 
		&& (m_CycleData.pPcbBoard != NULL))
	{
		CStationId	stationId = m_CycleData.pPcbBoard->GetStationId();
		eFrontRear  fr = (stationId.Lane() == eLANE_REAR ? eREAR : eFRONT);
		Telemetry(EV_CLASSNOZZLE, pd.pHead->Unit(), "H%s apcState=%d, IsApcNozzle=%d", pd.pHead->GetName(), (int)g_pApcMgr->IsInlineCalibOn(stationId), (int)g_pApcMgr->IsSupportInlineCalib(pd.pHead->GetNozzle()));
	}

	// Apply Apc Offset
	if ( g_pApcMgr 
		&& g_pApcMgr->IsSupportInlineCalib(pd.pHead->GetNozzle())
		&& bSectionRunning 
		&& (m_CycleData.pPcbBoard != NULL))
	{
		CStationId	stationId = m_CycleData.pPcbBoard->GetStationId();
		CGantry*	pGantry = pd.pHead->Gantry();
		eFrontRear  fr = (stationId.Lane() == eLANE_REAR ? eREAR : eFRONT);

		if (g_pApcMgr->IsInlineCalibOn(stationId) && m_CycleData.pPcbBoard->GetReassignRefType(pd.placeCadID) == 0)
		{
			GantryID	gantryId = pGantry->Unit();
			long		headId = (long)pd.pHead->GetSpindle();
			CNozzleId	nozzleId = pd.pHead->GetNozzle();
			long		areaIndex = 0;
			long		placeT = (long)::GetNearest90PlusOnly((long)aPlaceLoc.t);
			bool		bMultiPcb = g_pPcbSet->IsMultiPcb(m_CycleData.stationId.Lane());
			long		blockNo = pd.arrayId.blockNo;
			long		arrayNo = pd.arrayId.arrayNo;
			long		refNo = pd.placeMapID;

			Telemetry(EV_CLASSHEAD, pd.pHead->Unit(), "H%s ApcOffsetID=%s, BrdSerial=%u, Barcode=%s", pd.pHead->GetName(), g_pApcMgr->GetCurrentApcOffsetID(stationId), m_CycleData.pPcbBoard->GetApcSerialNum(), m_CycleData.pPcbBoard->m_ApcSerialData.barCodeData.c_str());

			if  (bMultiPcb)
			{ 
				blockNo = (long)(m_CycleData.pPcbBoard ? m_CycleData.pPcbBoard->GetPcbID() : g_pPcbSet->PreferPcbId(m_CycleData.stationId.Lane()));

				if (g_StateReport.IsIndependentLanes() && stationId.Lane() == eLANE_REAR)
				{
					blockNo -= eMAX_MULTIPCB_PER_PCB;
				}
			}
			
			if (pGantry->GetFrontOrRear() == eREAR)
			{
				placeT = (long)::GetNearest90PlusOnly((long)aPlaceLoc.t + 180);
			}

			CFxyzt apcStepOffset = g_pApcMgr->GetApcStepOffset(stationId, gantryId, blockNo, arrayNo, refNo, nominalPos);
			aPlaceLoc.x -= apcStepOffset.x;
			aPlaceLoc.y -= apcStepOffset.y;
			Telemetry(EV_CLASSHEAD, pd.pHead->Unit(), "H%s  apcStep=[%7.3f,%7.3f,%7.3f,%7.3f] block=%d, array=%d, ref=%d", pd.pHead->GetName(), apcStepOffset.x, apcStepOffset.y, apcStepOffset.z, apcStepOffset.t, blockNo, arrayNo, refNo);

			CFxyzt apcXYAreaOffset = g_pApcMgr->GetApcXYAreaOffset(stationId, gantryId, areaIndex, headId, nozzleId, nominalPos);
			if (apcXYAreaOffset.x != 0.0 || apcXYAreaOffset.y != 0.0)
			{
				aPlaceLoc -= apcXYAreaOffset;
				Telemetry(EV_CLASSHEAD, pd.pHead->Unit(), "H%s  apcArea=[%7.3f,%7.3f,%7.3f,%7.3f] Idx=%d", pd.pHead->GetName(), apcXYAreaOffset.x, apcXYAreaOffset.y, apcXYAreaOffset.z, apcXYAreaOffset.t, areaIndex);
			}

			CFxyzt apcAlignOffset = g_pApcMgr->GetApcAlignOffset(stationId, gantryId, headId, nozzleId);
			if (apcAlignOffset.x != 0.0 || apcAlignOffset.y != 0.0)
			{
				aPlaceLoc -= apcAlignOffset;
				Telemetry(EV_CLASSHEAD, pd.pHead->Unit(), "H%s  apcAlign=[%7.3f,%7.3f,%7.3f,%7.3f]", pd.pHead->GetName(), apcAlignOffset.x, apcAlignOffset.y, apcAlignOffset.z, apcAlignOffset.t);
			}

			CFxyzt apcNozzleOffset = g_pApcMgr->GetApcNozzleOffset(stationId, gantryId, areaIndex, headId, nozzleId, placeT, nominalPos);
			if (apcNozzleOffset.x != 0.0 || apcNozzleOffset.y != 0.0)
			{
				aPlaceLoc -= apcNozzleOffset;
				Telemetry(EV_CLASSHEAD, pd.pHead->Unit(), "H%s  apcNozzle=[%7.3f,%7.3f,%7.3f,%7.3f]", pd.pHead->GetName(), apcNozzleOffset.x, apcNozzleOffset.y, apcNozzleOffset.z, apcNozzleOffset.t);
			}

			Telemetry(EV_CLASSHEAD, pd.pHead->Unit(), "H%s  apcCAD [%7.3f,%7.3f] Idx=%d", pd.pHead->GetName(), nominalPos.x, nominalPos.y, areaIndex);
		}

		//		else if ( g_pApcMgr->IsInlineCalibOnInit(stationId) )
		//		{
		//			CFxy thermalInitLoc = aPlaceLoc;
		//
		//			CSMap* thermalInitMap = g_pApcMgr->GetMapThermalInit(pGantry->Unit(), stationId.Lane());
		//
		//			if ( thermalInitMap && thermalInitMap->IsKnown() )
		//			{
		//				thermalInitMap->MapToXY( thermalInitLoc );
		//				
		//				Telemetry( EV_CLASSHEAD, pd.pHead->Unit(), "H%s  ThermalInit=[%7.3f,%7.3f]", pd.pHead->GetName(), thermalInitLoc.x-aPlaceLoc.x, thermalInitLoc.y-aPlaceLoc.y); 
		//
		//				aPlaceLoc.x = thermalInitLoc.x;
		//				aPlaceLoc.y = thermalInitLoc.y;
		//			}
		//		}

	}


	// Add the PART height to the target location
	aPlaceLoc.z += (pd.profile.common.height - pd.profile.common.depthZ);

	// Provide a gantry XY position too.
	aPlaceLocGan = aPlaceLoc - CFxyzt(pd.pHead->GetOffset(), 0, 0); 
}


void
CSeqCycle::GetHeadPositionThruPV( CHead* apHead, const CFxyzt& aPartVector, bool abPolarized, CFxyzt& aTarget)
{
	double cost0;
	double sint0;
	CFxyzt rotPV;				// Rotated part vector
	CFxyzt pv( aPartVector);	// The part vector

	// Ignore any undefined values
	pv.ZeroUndefines();

	if ( !abPolarized && !g_GlobalSmt.GetAccuracyPCB(apHead->GetFrontOrRear()))
	{
		double placeTheta = aTarget.t + pv.t;
		placeTheta = fmod(placeTheta, 360.0);
		long placeT = (long)::GetNearest90(placeTheta);
		if (placeT >= 180 && placeT <= 270)
			aTarget.t -= 180;
		else if (placeT >= -180 && placeT <= -90)
			aTarget.t += 180;
		else if (placeTheta < -225)
			aTarget.t += 360;
		else if (placeTheta > 315)
			aTarget.t -= 360;
		/*
		//For a non-polarized part we must find which results in the smallest theta:
		//rotating to the placement angle or placement angle + 180 deg.
		//Get the current theta of the head.
		CFxyzt currHeadLoc = apHead->GetLastPosition();

		//Compute the actual placement theta including the partVector correction.
		double placeTheta = aTarget.t + pv.t;

		//Keep it in the range +2PI to -2PI
		placeTheta = fmod( placeTheta, 360.0 );
		//Keep it in the range 0 to +2PI.
		if ( placeTheta < 0 )
			placeTheta += 360.0;

		currHeadLoc.t = fmod( currHeadLoc.t, 360.0 );
		//Keep it in the range 0 to +2PI.
		if ( currHeadLoc.t < 0 )
			currHeadLoc.t += 360.0 ;

		double placeToHeadTheta = fabs( currHeadLoc.t - placeTheta);
		if ( placeToHeadTheta > 90.0 && placeToHeadTheta < 270.0 )
		{ // The angle between the head and the placement between 190 and 270 deg.
		 // which means the 180 deg end of the part is closer to the placement angle.
			//Change the placement by 180 deg.
			aTarget.t += 180.0;

			//Keep it in the range +2PI to -2PI
			aTarget.t = fmod( aTarget.t, 360.0 );
			//Keep it in the range 0 to +2PI.
			if ( aTarget.t < 0 )
				aTarget.t += 360.0;
		}*/
	}

	// Add in the PART VECTOR
	rotPV = pv;

	//The correction in partVector is in part coordinates.
	//Convert the partVector from part coordinates to table coordinates
	//by rotating the vector through same angle as the part.

	cost0 = cos( aTarget.t * DtoR );
	sint0 = sin( aTarget.t * DtoR );

	rotPV.x = pv.x * cost0  -  pv.y * sint0;
	rotPV.y = pv.x * sint0  +  pv.y * cost0;
 
//T_DEBUG_POINT
	//ex) SM321 H1
	//Head  1 - Vis (-0.0181, 0.0812, 2.2051), PartVecAligned (0.0149, -0.0819, -2.2051) 
	//Head  1 - cycle=6, LastT = 90.0, PlaceR=90.0, FinalOffset (0.01, -0.08, -2.21) 
//	TRACE( "Head %2d - cycle=%d, PlaceR=%.1f,(-)LastT=%.1f,(+)PartVT=%.1f, FinalOffset (%.2f, %.2f, %.2f) \n", 
//								pd.pHead->Unit(), pd.cycle, aTarget.t, lastLocT, partVectorT,
//								rotPV.x, rotPV.y, rotPV.t ); 

	// Correct the position of the nozzle by the partVector.
	aTarget += rotPV;

	//Set theta to short position from Absolute Zero within +/-200 degree.
	aTarget.t = ::NormalizeRWith( aTarget.t, 200 );
}


ePickInspectR
CSeqCycle::CalcPickInspectRCycle( CNozzleId aNozzleId, const CProfile& aProfile, CFeederId aFeederID ) const
{
	eCameraGroupType
			camGrpType		= aProfile.visCommon.cameraGroupType;

//TODO: TEMPORARY UNTIL MMI CAN SPECIFY UP vs. FLY CAMERA TYPE FOR THE STAGE CAMERA
//	camGrpType		= eCAMERA_GROUP_STAGE;
//TODO: TEMPORARY UNTIL MMI CAN SPECIFY UP vs. FLY CAMERA TYPE FOR THE STAGE CAMERA

	bool	bUpCamera		= (camGrpType & eCAMERA_GROUP_UP) != 0;
	bool	bInspOnlyZero	= camGrpType==eCAMERA_GROUP_UP  &&  aProfile.visCommon.bUseInspOnlyZero;
	bool	bSmOnly			=  aProfile.IsSmallComp()
							&& !aProfile.IsVerySmallComp()
							&& !aProfile.IsFinePitchComp()
							&& camGrpType!=eCAMERA_GROUP_UP;
	bool	bSmFineOrUp		=  aProfile.IsSmallComp()
							|| aProfile.IsVerySmallComp()
							|| aProfile.IsFinePitchComp()
							|| camGrpType==eCAMERA_GROUP_UP;
	bool	bPickRZero;
	bool	bOrientedNozzle;

	if ( bInspOnlyZero)
		return ePIR_PICK_ZERO;

	bPickRZero		= IsPickRZero( aFeederID);
	bOrientedNozzle	= IsOrientedNozzle( aNozzleId, aProfile.common.nozzleType1, aProfile.common.nozzleType2);

	long		lOrientedNozzleMode = g_FacDebug.GetModeOrientNozzle();

	if( 0 != lOrientedNozzleMode )
	{
		Telemetry( EV_CLASSNOZZLE, m_pGantry->Unit(),"OrientedNozzleMode[%d]", lOrientedNozzleMode); 
		if( -1 == lOrientedNozzleMode )
			return	ePIR_PICK_ANY;
		else
			return (ePickInspectR)lOrientedNozzleMode;
	}

	CRunOptions&	rRunOptions = *::GetRunOptions(m_frRunningLane);
	if (rRunOptions.IsBGAAutoAccuracyReturn() && m_CycleData.pPcbBoard->IsAccuracyPCB())
	{
		return ePIR_PICK_PLACE;
	}
	// See RT Exception MENU
	if ( bSmFineOrUp  &&  !bSmOnly  &&  bPickRZero  &&  !bOrientedNozzle  && g_FacDebug.IsUseNegativePick()
#ifdef WIN_SIM
		&&  IsUseNegativePick()
#endif
		)
	{
		return ePIR_NEG_PLACE__PLACE;
	}

//	if ( bUpCamera  &&  !bPickRZero  &&  !bOrientedNozzle)
//		return ePIR_NEG_PLACE__PLACE;
//
	if ( bSmOnly  &&  bPickRZero  &&  !bOrientedNozzle && g_FacDebug.IsUseNegativePick()
#ifdef WIN_SIM
		&&  IsUseNegativePick()
#endif
		)
	{
		return ePIR_NPL_ANY_PL_ANY;
	}

	if ( bSmFineOrUp)
		return ePIR_PICK_PLACE;

	if ( !bPickRZero)
		return ePIR_PICK_ANY;

	//<2009.11.31> for YuSung.
	//Support Oriented Nozzle
	if ( bOrientedNozzle )
		return ePIR_PICK_ANY;

#ifndef SM_MACHINE
	return ePIR_PICK_ZERO;
#else
	return ePIR_ANY_ANY;
#endif
}
//
//AP5 specfic Pick Scan rotation control type
//
// Assign pickAlignAngle as "ePIR_PICK_PLACE", or "ePIR_NEG_PLACE__PLACE" as possible as it can.
// Otherwise assign "ePIR_PICK_ZERO" if inspOnlySero is set.
ePickInspectR
CSeqCycle::CalcPickInspectRCycleAP( CNozzleId aNozzleId, const CProfile& aProfile, CFeederId aFeederID, double& arHeadPitch, double& arPlaceAngle ) const
{
	eCameraGroupType
			camGrpType		= aProfile.visCommon.cameraGroupType;

	double& headPitch		= arHeadPitch;
	double& partPlaceAngle	= arPlaceAngle;
	CFxy	partBound		= aProfile.GetBound();
	bool	bWillBeRotated	= false;
	double	partAngle		= 0;
	bool	bPIR_PICK_INSP_ZERO = false;

//	if ( aFeederID.IsDefined() && aFeederID.Type() == eFEEDTYPE_TAPE )
//	{
//		CTapeFeeder* pTapeFeeder = ::GetTapeFeeder( aFeederID ); 
//		partAngle = pTapeFeeder->GetPartR();
//		if ( partAngle != partPlaceAngle )
//			bWillBeRotated = true;
//	}
//
//	if ( bWillBeRotated )
//	{
//		if ( partBound.Magnitude() + 1.25 >= headPitch )
//			bPIR_PICK_INSP_ZERO = true;
//	}

//TODO: TEMPORARY UNTIL MMI CAN SPECIFY UP vs. FLY CAMERA TYPE FOR THE STAGE CAMERA
//	camGrpType		= eCAMERA_GROUP_STAGE;
//TODO: TEMPORARY UNTIL MMI CAN SPECIFY UP vs. FLY CAMERA TYPE FOR THE STAGE CAMERA

	bool	bUpCamera		= (camGrpType & eCAMERA_GROUP_UP) != 0;
	bool	bInspOnlyZero	= ( camGrpType==eCAMERA_GROUP_UP || camGrpType==eCAMERA_GROUP_LSO )  &&  aProfile.visCommon.bUseInspOnlyZero;
	bool	bSmOnly			=  aProfile.IsSmallComp()
							&& !aProfile.IsVerySmallComp()
							&& !aProfile.IsFinePitchComp()
							&& camGrpType!=eCAMERA_GROUP_UP;
	bool	bSmFineOrUp		=  aProfile.IsSmallComp()
							|| aProfile.IsVerySmallComp()
							|| aProfile.IsFinePitchComp()
							|| camGrpType==eCAMERA_GROUP_UP;
	bool	bPickRZero;
	bool	bOrientedNozzle;

	if ( bInspOnlyZero)
		return ePIR_PICK_ZERO;

	bPickRZero		= IsPickRZero( aFeederID);
	bOrientedNozzle	= IsOrientedNozzle( aNozzleId, aProfile.common.nozzleType1, aProfile.common.nozzleType2);

//	//<NOTE:>
//	//Pickup Part at pickup angle and inspect as place angle.
//	if ( bPIR_PICK_INSP_ZERO && !bOrientedNozzle)
//		return ePIR_PICK_PLACE;

	long		lOrientedNozzleMode = g_FacDebug.GetModeOrientNozzle();

	if( 0 != lOrientedNozzleMode && !bOrientedNozzle  )
	{
		Telemetry( EV_CLASSNOZZLE, m_pGantry->Unit(),"OrientedNozzleMode[%d]", lOrientedNozzleMode); 
		if( -1 == lOrientedNozzleMode )
			return	ePIR_PICK_PLACE;
		else
			return (ePickInspectR)lOrientedNozzleMode;
	}



	// See RT Exception MENU
	if ( bSmFineOrUp  &&  !bSmOnly  &&  bPickRZero  &&  !bOrientedNozzle  
#ifdef WIN_SIM
		&&  IsUseNegativePick()
#endif
		)
	{
		return ePIR_NEG_PLACE__PLACE;
	}

//	if ( bUpCamera  &&  !bPickRZero  &&  !bOrientedNozzle)
//		return ePIR_NEG_PLACE__PLACE;
//
	if ( bSmOnly  &&  bPickRZero  &&  !bOrientedNozzle
#ifdef WIN_SIM
		&&  IsUseNegativePick()
#endif
		)
	{
		return ePIR_NPL_ANY_PL_ANY;
	}

	if ( bSmFineOrUp)
		return ePIR_PICK_PLACE;

	if ( !bPickRZero)
		return ePIR_PICK_PLACE;

	//<2009.11.31> for YuSung.
	//Support Oriented Nozzle
	if ( bOrientedNozzle )
		return ePIR_PICK_PLACE;

	//Pickup Head "0" deg
	return ePIR_NEG_PLACE__PLACE;
}

//
//AP5 specfic spindle pitch & scan rotation decision
//
//Bellow rule is applied when feeder is tape feeder.
//Otherwise, pitch will be open for part rotation safe position.
//
// Calculate minimum pitch distance for part rotation.
// Determine if part should be rotate after pickup before scan part.
// This occur where
// 1) Place R and Part R is different
// And
// 2) InspOnlyZero is not set.
// If InspOnlyZero is set, then
// Part will be picked by pickup angle and 
// rotate to registered part angle and inspected
// Then rotate to place angle.
// Else, Head will be rotated neg place angle and pickup a part
// Rotate head to "0" deg which make part angle as place angle.
// Scan part as place angle and place it with head angle "0"
double
CSeqCycle::CalcSpindlePitchForPartRotateSafeAP2( const StCycleData& arCycleData, double aPitch, bool& arbPartWillRotate )
{
	CFixedMap< HeadID, bool, SYS_MAX_HEAD_PER_GANTRY > headIdMap;
	headIdMap.RemoveAll();

	CFixedArrayList< double, SYS_MAX_HEAD_PER_GANTRY > partSizeList;
	partSizeList.RemoveAll();

	CGantry*		pGantry			= NULL;
	CFeederId		feederID;
	bool			bPartWillRotate = false;
	double			partAngle		= 0.0;
	double			partPlaceAngle	= 0.0;
	bool			bInspZero		= false;
	CFxy			partSize;
	double			partSizeY		= 0;
	double			partSizeMax		= 0;
	bool			bAnyPartWillRotate = false;
	double			minPitchReturn = aPitch;
	bool			bPartR			= false;

	CTapeFeeder*	pTapeFeeder		= NULL;
	CStickFeeder*	pFeeder			= NULL;					
	CTray*			pTray			= NULL;
				

	for (int i=0; i<arCycleData.nCount; i++)
	{
		bPartWillRotate = false;

		const StPlaceData& pd = arCycleData.placeData[i];
		
		CHead*pHead = pd.pHead;

		headIdMap[pHead->Unit()] = true;

		if ( pGantry == NULL )
			pGantry = pHead->Gantry();

		const CProfile& profile = pd.profile;
		partSize = profile.GetBound();

		bInspZero = profile.visCommon.bUseInspOnlyZero;

		feederID = pd.feederID;

		bPartR = false;
		partAngle = 0;
		if ( feederID.IsDefined() )
		{
			if ( feederID.Type() == eFEEDTYPE_TAPE )
			{
				pTapeFeeder = ::GetTapeFeeder( feederID ); 
				partAngle = pTapeFeeder->GetPartR();
			}
			else if ( feederID.Type() == eFEEDTYPE_STICK )
			{
				pFeeder = ::GetStickFeeder( feederID ); 				
				partAngle = pFeeder->GetPartR();
			}
			else if ( feederID.Type() == eFEEDTYPE_TRAY )
			{
				pTray = ::GetTray( feederID ); 				
				partAngle = pTray->GetPartR();
			}
			else
				SYS_ASSERT(0);

			if ( ((int)partAngle % 180) != 0 )
				bPartR = true;

			partPlaceAngle = pd.placePos.t;

			if ( !bInspZero && fabs( partAngle - partPlaceAngle ) >= 15.0)
				bPartWillRotate = true;
			
		}
		else
		{
			if ( !bInspZero )
				bPartWillRotate = true;
		}

		if ( bPartWillRotate )
			partSizeY = partSize.Magnitude();
		else
		{
			partSizeY = partSize.y;
			if ( bPartR )
				partSizeY = partSize.x;
		}

		partSizeY += SYS_PART_SEPARATION_MARGIN_AP;

		if ( partSizeMax < partSizeY )
			partSizeMax = partSizeY;

		if ( bPartWillRotate && !bAnyPartWillRotate )
			bAnyPartWillRotate = true;

		partSizeList.Append(partSizeY);
	}

	arbPartWillRotate = bAnyPartWillRotate;

	//Consider remained head which may has part already.
	const THeadList& headList = GetHeadList( pGantry );
	if ( !headList.IsEmpty() )
	{
		CHead* pHead = NULL;
		for ( int h=0; h < headList.GetCount(); h++ )
		{
			pHead = headList[h];

			if ( headIdMap.IsExist( pHead->Unit() ) )
				continue;

			const CFRect& partBound = pHead->GetPickedPartBound();
			if ( !partBound.IsDefined() )
				continue;
			
			partSizeY = partBound.Height();

			partSizeY += SYS_PART_SEPARATION_MARGIN_AP;

			if ( partSizeMax < partSizeY )
				partSizeMax = partSizeY;

			partSizeList.Append(partSizeY);
		}
	}

	CFRect	thisRect;
	CFRect	thatRect;
	CFRect	interSect(0,0,0,0);
	CFRect	maxInterSect(0,0,0,0);
	double	distThis = aPitch;
	double	distThat = aPitch;
	
	minPitchReturn = max( minPitchReturn, partSizeMax * 0.5);

	for ( int p=0; p < partSizeList.GetCount(); p++ )
	{
		distThis = minPitchReturn * p;
		partSizeY = partSizeList[p];
		thisRect = CFRect( -0.5*partSizeY, -0.5*partSizeY, 0.5*partSizeY, 0.5*partSizeY);
		thisRect.OffsetRect( distThis, 0 );
		for ( int n=p+1; n < partSizeList.GetCount(); n++ )
		{
			distThat = minPitchReturn * n;
			partSizeY = partSizeList[n];
			thatRect = CFRect( -0.5*partSizeY, -0.5*partSizeY, 0.5*partSizeY, 0.5*partSizeY);
			thatRect.OffsetRect( distThat, 0 );

			interSect.IntersectRect(thisRect, thatRect );
			if ( interSect.IsDefined() && interSect.Width() > maxInterSect.Width())
				maxInterSect = interSect;
		}
	}

	if ( maxInterSect.IsDefined() && maxInterSect.Width() > 0 )
		minPitchReturn = minPitchReturn + maxInterSect.Width();

	return minPitchReturn;
}

double
CSeqCycle::CalcSpindlePitchForPartRotateSafeAP( const StCycleData& arCycleData, bool& arbPartWillRotate )
{

	CFixedMap< HeadID, bool, SYS_MAX_HEAD_PER_GANTRY > headIdMap;
	headIdMap.RemoveAll();

	CGantry*		pGantry			= NULL;
	CTapeFeeder*	pTapeFeeder		= NULL;
	CFeederId		feederID;
	bool			bPartWillRotate = false;
	double			partAngle		= 0.0;
	double			partPlaceAngle	= 0.0;
	bool			bInspZero		= false;

	double partSizeMax = 0;
	for (int i=0; i<arCycleData.nCount; i++)
	{
		const StPlaceData& pd = arCycleData.placeData[i];

		if ( pd.status != ePLACE_OK )	continue;
		
		CHead*pHead = pd.pHead;

		headIdMap[pHead->Unit()] = true;

		if ( pGantry == NULL )
			pGantry = pHead->Gantry();

		const CProfile& profile = pd.profile;

		bInspZero = profile.visCommon.bUseInspOnlyZero;

		const CFxy& partBoundThis = profile.GetBound();
		double partSizeThis = partBoundThis.Magnitude();
		
		partSizeThis += SYS_PART_SEPARATION_MARGIN_AP * 0.5;

		if ( partSizeMax <= partSizeThis )
			partSizeMax = partSizeThis;

		feederID = pd.feederID;

		if ( feederID.IsDefined() && feederID.Type() == eFEEDTYPE_TAPE )
		{
			pTapeFeeder = ::GetTapeFeeder( feederID ); 

			partAngle = pTapeFeeder->GetPartR();

			partPlaceAngle = pd.placePos.t;

			if ( !bInspZero && fabs( partAngle - partPlaceAngle ) >= 15.0)
				bPartWillRotate = true;
		}
		else
		{
			if ( !bInspZero )
				bPartWillRotate = true;
		}
	}

	const THeadList& headList = GetHeadList( pGantry );

	if ( !headList.IsEmpty() )
	{
		CHead* pHead = NULL;
		for ( int h=0; h < headList.GetCount(); h++ )
		{
			pHead = headList[h];

			if ( headIdMap.IsExist( pHead->Unit() ) )
				continue;

			const CFRect& partBound = pHead->GetPickedPartBound();
			CFxy partSize(partBound.Width(), partBound.Height());
			double partMag = partSize.Magnitude();
			partMag += 1.25;

			if ( partSizeMax <= partMag )
				partSizeMax = partMag;

		}
	}

	if ( partSizeMax < 0 )
		partSizeMax = 0;

	arbPartWillRotate = bPartWillRotate;

	return partSizeMax;
}

bool
CSeqCycle::IsPickRZero( CFeederId aFeederID) const
{
	bool	bPickRZero;

	double	pickR = GetPickR( aFeederID);
	bPickRZero = pickR == 0.0;

	return bPickRZero;
}


bool
CSeqCycle::IsOrientedNozzle( CNozzleId	aNozzleID,
							eNozzleType	aNozType1/*=eNOZZLE_TYPE_NONE*/,
							eNozzleType	aNozType2/*=eNOZZLE_TYPE_NONE*/) const
{
	bool	bOrientedNozzle;

	eNozzleType nozzleType = aNozType1;
	if ( nozzleType == eNOZZLE_TYPE_NONE )
		nozzleType = aNozType2;

	if ( aNozzleID.IsDefined())
	{
		long ancID = aNozzleID.Anc();
		long ancNo = aNozzleID.HoleNum();
		SYS_ASSERT( 0<ancID  &&  ancID<_countof(g_Anc));

		CAnc& anc = g_Anc[ ancID];
		SYS_ASSERT( anc.IsInstalled());

		nozzleType = anc.GetNozzleType(aNozzleID);
		
		if (!(aNozType1 == nozzleType || aNozType2 == nozzleType))
		{
			CRunOptions&	rRunOptions = *::GetRunOptions( m_frRunningLane );
			bool bFound = false;

			if (  rRunOptions.IsDrySkipPickUp() && rRunOptions.IsDrySkipANC() )
			{
				for (int holeId = 1; holeId <= anc.GetHoleCount( ); holeId++)
				{
					eNozzleType sysNoz = anc.GetNozzleType(holeId);

					if (!(aNozType1 == sysNoz || aNozType2 == sysNoz))
						continue;

					ancID = anc.GetNozzleID(holeId).Anc( );
					ancNo = anc.GetNozzleID(holeId).HoleNum( );
					nozzleType = sysNoz;
					bFound = true;

					Telemetry(EV_CLASSENGINE, m_pGantry->Unit( ), "G%s:IsOrientedNozzle - Dry Run Nozzle Data Change hole:%d", m_pGantry->NameBrief( ), holeId);
					break;
				}

				if (!bFound)
				{
					g_StateReport.ReportErrorEx2(eMEC_MN_ANC_GET_NOZ_TYPE_SEARCH_ERROR, eERR_LEVEL_EMER, eMOTOR_CUT, m_SectionID, anc.GetFrontOrRear( ),
						0, HeadNone, 0, 0, aNozzleID, 0, 0, __LINE__);
				}
			}
			else
			{
				g_StateReport.ReportErrorEx2(eMEC_MN_ANC_GET_NOZ_TYPE_SEARCH_ERROR, eERR_LEVEL_EMER, eMOTOR_CUT, m_SectionID, anc.GetFrontOrRear( ),
					0, HeadNone, 0, 0, aNozzleID, 0, 0, __LINE__);
				//SYS_ASSERT(0);
			}
		}

		bOrientedNozzle = anc.IsNozzleOriented( ancNo);
		bOrientedNozzle |= CAnc::IsNozzleOriented( nozzleType);   // Cross check by nozzle type
	}
	else
	{	// Nozzle is assigned with 'A'
		int ancIdx = 0;
		for (int i=0; i<_countof(g_Anc); i++)
		{
			if ( m_SectionID != g_Anc[i].GetSectionID())
				continue;
			if ( m_pGantry->GetFrontOrRear() != g_Anc[i].GetFrontOrRear())
				continue;

			ancIdx = i;
			break;
		}

		int holeCnt = g_Anc[ancIdx].GetHoleCount( );
		for (int holeId = 1; holeId <= holeCnt; holeId++)
		{
			if (aNozType1 == g_Anc[ancIdx].GetHole(holeId)->GetNozzleType( ))
			{
				nozzleType = aNozType1;
				break;
			}
			else if (aNozType2 != g_Anc[ancIdx].GetHole(holeId)->GetNozzleType( ))
			{
				nozzleType = aNozType2;
				break;
			}
			else
				continue;
		}

		bOrientedNozzle = CAnc::IsNozzleOriented( nozzleType);
	}

//GRIPPER_NOZZLE_SUPPORT
	StSysNozzle* pSysNoz = NULL;
	bool	bIsGripper = StSysNozzle::IsGripperNozzleType(aNozType1)
						|| StSysNozzle::IsGripperNozzleType(aNozType2);
	if ( CAnc::GetSysNozzle( aNozType1, pSysNoz))
	{
		if( pSysNoz && pSysNoz->bGripperCheck && !bIsGripper)
			bIsGripper = true;	
	}
	if ( CAnc::GetSysNozzle( aNozType2, pSysNoz))
	{
		if( pSysNoz && pSysNoz->bGripperCheck && !bIsGripper)
			bIsGripper = true;
	}

	bOrientedNozzle = bOrientedNozzle || bIsGripper;
//GRIPPER_NOZZLE_SUPPORT-END

	return bOrientedNozzle;
}


eStatus
CSeqCycle::GetNextPickFeederID( CFeederId& aFeederID, HeadID aHeadID) const
{
	eStatus		status = eNG;

	// First determine if there are any pick steps in this cycle.
	long	idxStep;	// Index into m_Steps[].
	CStep** ppStep=NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
	long	nStep;		// Count of contiguous primary type of steps.

	nStep = GetSteps( eSTEP_PICK, m_frRunningLane, idxStep, ppStep);

	if ( idxStep < 0)	return eNG;
	if ( nStep <= 0)	return eNG;

	for (int i=idxStep; --nStep>=0; i++)
	{
		CStepPick* pStep = (CStepPick*)m_Steps[i];

		status = pStep->GetNextPickFeederID( aFeederID, aHeadID, m_CycleData);
		if ( status == eOK)
			break;
	}
	return status;
}


eStatus
CSeqCycle::GetPickRPartRHead( HeadID aHeadID, double& aPickR, long& aPartR) const
{
	// NOTE: This function call ONLY VALID AFTER the
	//			eSTEP_PICK PlanStep() is called first.
	eStatus		status = eNG;
	CFeederId	feederID;

	// First determine if there are any pick steps in this cycle.
	long	idxStep;	// Index into m_Steps[].
	CStep** ppStep=NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
	long	nStep;		// Count of contiguous primary type of steps.

	nStep = GetSteps( eSTEP_PICK, m_frRunningLane, idxStep, ppStep);

	if ( idxStep < 0)	return eNG;
	if ( nStep <= 0)	return eNG;

	for (int i=idxStep; --nStep>=0; i++)
	{
		CStepPick* pStep = (CStepPick *)m_Steps[i];

		status = pStep->GetNextPickFeederID( feederID, aHeadID, m_CycleData);
//		pStep->GetNextPickFeederPickup( aHeadID, pickup);
		if ( status == eOK)
			break;
	}
	if (status != eOK)	return status;

	CTapeFeeder*	pTape	= NULL;
	CStickFeeder*	pStick	= NULL;
	CTray*			pTray	= NULL;

	switch ( feederID.Type())
	{
	case eFEEDTYPE_TAPE:	pTape=::GetTapeFeeder( feederID);
							if ( pTape != NULL )
							{
								aPartR = pTape->GetPartR();
								aPickR = pTape->GetPickR();
								return eOK;
							}
							break;
	case eFEEDTYPE_STICK:	pStick=::GetStickFeeder( feederID);
							if ( pStick != NULL )
							{
								aPartR = pStick->GetPartR();
								aPickR = pStick->GetPickR();
								return eOK;
							}
							break;
	case eFEEDTYPE_TRAY:	pTray = ::GetTray( feederID);
							if ( pTray != NULL )
							{
								aPartR = pTray->GetPartR();
								aPickR = pTray->GetPickR();
								return eOK;
							}
	}
	return eNG;
}


double
CSeqCycle::GetPickR( CFeederId aFeederID) const
{
	if ( !aFeederID.IsDefined())
		return NaN;

	switch ( aFeederID.Type())
	{
	case eFEEDTYPE_TAPE:{	CTapeFeeder* pTape = ::GetTapeFeeder( aFeederID );
							return (pTape==NULL ? NaN : pTape->GetPickR()); }

	case eFEEDTYPE_STICK:{	CStickFeeder* pStick = ::GetStickFeeder( aFeederID );
							return (pStick==NULL ? NaN : pStick->GetPickR()); }

	case eFEEDTYPE_TRAY:{	CTray* pTray = ::GetTray( aFeederID );
							return (pTray==NULL ? NaN : pTray->GetPickR()); }
	}
	return NaN;
}


long
CSeqCycle::GetPartR( CFeederId aFeederID) const
{
	if ( !aFeederID.IsDefined())
		return 0;

	switch ( aFeederID.Type())
	{
	case eFEEDTYPE_TAPE:{	CTapeFeeder* pTape = ::GetTapeFeeder( aFeederID );
							return (pTape==NULL ? 0 : pTape->GetPartR()); }

	case eFEEDTYPE_STICK:{	CStickFeeder* pStick = ::GetStickFeeder( aFeederID );
							return (pStick==NULL ? 0 : pStick->GetPartR()); }

	case eFEEDTYPE_TRAY:{	CTray* pTray = ::GetTray( aFeederID );
							return (pTray==NULL ? 0 : pTray->GetPartR()); }
	}
	return 0;
}


eCycleStatus
CSeqCycle::RunCycle( bool abResetRegions/*=true*/)
{
	// Reset the return status for the cycle sequencer.  This value will be modified
	// by each phase of the sequence.
	m_ReturnStatus = eCYCLE_OK;

	if( g_SysConst.GetImprovePIP() )
	{
		if( abResetRegions ||  eCYCLE_PREFETCH_ALIGN != m_PrevPrefetchOp  || eCYCLE_PREFETCH_NONE != m_PrefetchOp )
		{
			Telemetry( EV_CLASSENGINE, m_pGantry->Unit(), "Reset Find Result");
			m_EventFindTrigger.Reset( -1);
			m_EventFindResults.Reset( -1);
			m_EventFind3DResults.Reset( -1);
		}
	}
	else
	{
		m_EventFindTrigger.Reset( -1);
		m_EventFindResults.Reset( -1);
		m_EventFind3DResults.Reset( -1);
	}
	Telemetry( EV_CLASSENGINE, m_pGantry->Unit(), "____________RunCycle Start %s__________", m_pGantry->GetName() );

	// Calculate the cycle type for this cycle.
	eStatus			status;
	eCycleStatus	cyStatus = eCYCLE_OK;

	// NOTE that tray empty status has not been called in this cycle yet.
	m_bTrayStatusUpdated = false;

	//<2011.3.19>
	//Clear Assigned Pick Step Pointer
	ResetAssignedPickStep();


	status = DetermineCycleType( m_CycleType);
	if ( status != eOK)
		return eCYCLE_ABORT;

	status = CheckCycleIntegrity();
	if ( status != eOK)
		return eCYCLE_ABORT;

// 	status = CheckNozzleTipWithSVSCamera(); // after finishing evaluation on the machine,
// 	if ( status != eOK)
// 		return eCYCLE_ABORT;

	try
	{
		CheckAllTrayConfirmStatus();
		//<2010.12.16>
		//Applied V1 code
		//Seperate Dump Cycle execution from P&P execution
		//Because, the prepickup data will be gone after dump operation is executed.
		//Seperate execution will clearly re-build P&P cycle data.
		eCyclePrefetchOp prefetchOptBackup = m_PrefetchOp;

		//Process Dump Cycle 
		{			
			m_PrefetchOp = eCYCLE_PREFETCH_DUMP;
			
			PlanCycle();
			if ( m_ReturnStatus >= eCYCLE_FAILED )
				goto CycleDone;
			
			if ( m_bDualRegion)
			{
				PlanRegions();
				if ( m_ReturnStatus >= eCYCLE_FAILED )
					goto CycleDone;
			}
			
			// Prefetch Tray Check
			if( m_nSteps > 0 )
			{
				cyStatus = TrayFeederLookAheadCheck();
				m_ReturnStatus = (cyStatus > m_ReturnStatus ? cyStatus : m_ReturnStatus);
				if ( m_ReturnStatus >= eCYCLE_FAILED )
					goto CycleDone;
			}
			
			ExecuteCycle();
			if ( m_ReturnStatus >= eCYCLE_FAILED )
				goto CycleDone;
			
			PostCycle();
			
			// Reset the steps to be executed for this cycle, so they can be rebuilt.
			PlanCycleReset();

//<2011.1.31>
//Do not update Statistics after Dump Cycle
//
//			// Update partial cycle or full cycle statistics.
//			UpdateStatistics();
			
			if ( m_ReturnStatus >= eCYCLE_STOPPED )
				goto CycleDoneEnd;


// 			//<2011.07.14>
// 			//Check Machine State after dump cycle execution.
// 			//Release region after dump cycle execution
// 			// Holding region can cause machine Lockup Situation.
// 			eMachineState sectionState = g_StateReport.GetMachineState( m_SectionID, m_frRunningLane);
// 
// 			// If machine is FREEZE or worse, ZSafe Heads, BUT only if motor power will not be cut.
// 			if ( sectionState >= eMS_FREEZE  &&  !g_StateReport.IsMotorCutByReport())
// 				GantryMoveZSafe();

			if ( abResetRegions)
				m_pGantry->ResetRegions();
		}
		
		m_PrefetchOp = prefetchOptBackup;

		//Process Pick & Place Cycle
		PlanCycle();
		if ( m_ReturnStatus >= eCYCLE_FAILED )
		{
			goto CycleDone;
		}

		if ( m_bDualRegion)
		{
			PlanRegions();
			if ( m_ReturnStatus >= eCYCLE_FAILED )
			{
				goto CycleDone;
			}
		}

		if( g_SysConst.GetImprovePIP() )
		{
			if( abResetRegions ||  eCYCLE_PREFETCH_ALIGN != m_PrevPrefetchOp  || eCYCLE_PREFETCH_NONE != m_PrefetchOp )
			{
				// Prefetch Tray Check
				Telemetry( EV_CLASSFEEDER, m_pGantry->Unit(), "TF : TrayFeederLookAheadCheck- ALIGN STATUS, PLAN SKIP"); 
				cyStatus = TrayFeederLookAheadCheck();
				m_ReturnStatus = (cyStatus > m_ReturnStatus ? cyStatus : m_ReturnStatus);
				if ( m_ReturnStatus >= eCYCLE_FAILED )
				{
					goto CycleDone;
				}
			}
		}
		else
		{
			// Prefetch Tray Check
			cyStatus = TrayFeederLookAheadCheck();
			m_ReturnStatus = (cyStatus > m_ReturnStatus ? cyStatus : m_ReturnStatus);
			if ( m_ReturnStatus >= eCYCLE_FAILED )
			{
				goto CycleDone;
			}

		}

		ExecuteCycle();
		if ( m_ReturnStatus >= eCYCLE_FAILED )
		{
			goto CycleDone;
		}

CycleDone:
		// Update partial cycle or full cycle statistics.
		// !!! IMPORTANT NOTE: 20090205 - Added cycle placed data saving functions into non-volatile memeory.
		UpdateStatistics();

		PostCycle();

		m_PrevPrefetchOp = m_PrefetchOp;

		if ( m_ReturnStatus >= eCYCLE_STOPPED )
			goto CycleDoneEnd;

		// Don't allow post dumping when manual picks occur.
		if ( g_StateReport.IsManualCmdActive( m_frRunningLane) )
			goto CycleDoneEnd;

		// If PREFETCH without PICK STEP, then no need to dump failed parts
		if ( m_PrefetchOp!=eCYCLE_PREFETCH_NONE  &&  m_PrefetchOp<eCYCLE_PREFETCH_PICKUP)
			goto CycleDoneEnd;

		// DUMP any FAILED PARTS
		m_PrefetchOp = eCYCLE_PREFETCH_DUMP;

		// Reset the steps to be executed for this cycle, so they can be rebuilt.
		PlanCycleReset();

		PlanCycle();
		if ( m_ReturnStatus >= eCYCLE_FAILED )
			goto CycleDoneDump;

		if ( m_bDualRegion)
		{
			PlanRegions();
			if ( m_ReturnStatus >= eCYCLE_FAILED )
				goto CycleDoneDump;
		}

		//Buzz at dump cycle process
		long	idxStep;	// Index into m_Steps[].
		CStep** ppStep;		// Pointer to the beginning of a pointer array inside m_Steps[].
		long	nStep;		// Count of contiguous primary type of steps.

		CDumpStop*	pDumpStop = ::GetDumpStopObj(m_SectionID, m_frRunningLane);
		ppStep = NULL;
		nStep = GetSteps( eSTEP_DUMP, m_frRunningLane, idxStep, ppStep);
		if ( nStep > 0 )
		{
			bool bDumpStopBuz = false;

			//If dump stop is planned and buzz will be on, then skip buzzing at this time.
			if ( pDumpStop->IsDumpStopPlanned()
				&& ( g_SysConst.GetAdvancedOption() & eADV_BUZZER_AT_DUMPSTOP ))
				bDumpStopBuz = true;			

			if ( !bDumpStopBuz && ( g_SysConst.GetAdvancedOption() & eADV_BUZZER_AT_EVERY_DUMP ))
			{
				Telemetry( EV_CLASSENGINE, m_pGantry->Unit(), "DumpCycle[%s] Buzz", m_pGantry->GetName());
				g_pBuzzer->BeepN();
			}
		}

		ExecuteCycle();
		if ( m_ReturnStatus >= eCYCLE_FAILED )
			goto CycleDoneDump;

		m_PrefetchOp = eCYCLE_PREFETCH_NONE;

		if (pDumpStop->IsDumpStopPlanned())
			m_ReturnStatus = eCYCLE_STOPPED;

		// REPORT any RETRY EXCEEDED for feeder in this cycle.
		DetectExceededRetry();

CycleDoneDump:
		PostCycle();

CycleDoneEnd:
		// Send BufferChange commands to 1-Slot Tray Feeders if no pallets are required on near term subsequent cycles.
//  		cyStatus = TrayFeederBufferChangeCheck();
		m_ReturnStatus = (cyStatus > m_ReturnStatus ? cyStatus : m_ReturnStatus);
//No place to go if this fails
//		if ( m_ReturnStatus >= eCYCLE_FAILED )
//			goto CycleDone;

	}
	catch (CThrowAssert<> a)
	{
		// Just report the ASSERT and continue.
		a.Report();
		m_ReturnStatus = eCYCLE_ABORT;

		PostCycle();
	}
	catch (...)
	{
		m_ReturnStatus = eCYCLE_ABORT;
		SYS_ERROR_MSG1( "%s - Unknown SeqCycle exception occurred.", m_pGantry->GetName());

		PostCycle();
	}

// 	eMachineState sectionState = g_StateReport.GetMachineState( m_SectionID, m_frRunningLane);
// 
// 	// If machine is FREEZE or worse, ZSafe Heads, BUT only if motor power will not be cut.
// 	if ( sectionState >= eMS_FREEZE  &&  !g_StateReport.IsMotorCutByReport())
// 		GantryMoveZSafe();

	if ( abResetRegions)
		m_pGantry->ResetRegions();

	Telemetry( EV_CLASSENGINE, m_pGantry->Unit(), "____________RunCycle Stop %s status=%d__________", m_pGantry->GetName(), (long)m_ReturnStatus );
	return m_ReturnStatus;
}



void
CSeqCycle::PlanCycleReset( )
{
	// Reset the steps to be executed for this cycle, so they can be rebuilt.
	for (int i=0; i<m_CycleData.nCount; i++)
	{
		StPlaceData& pd = m_CycleData.placeData[i];
		for (int j=0; j<_countof(pd.pStep); j++) pd.pStep[j] = NULL;
	}
	m_CycleData.ancPutData.RemoveAll();
	m_CycleData.ancGetData.RemoveAll();
	m_CycleData.dumpData.RemoveAll();
}

void
CSeqCycle::ResetAssignedPickStep( )
{
	// Reset the steps to be executed for this cycle, so they can be rebuilt.
	for (int i=0; i<m_CycleData.nCount; i++)
	{
		StPlaceData& pd = m_CycleData.placeData[i];
		for (int j=0; j<_countof(pd.pStep); j++)
		{
			for( int k=0; k < _countof( pd.pStepPickAssigned ); k++ )
				pd.pStepPickAssigned[k] = NULL;
		}
	}
}


eCycleStatus
CSeqCycle::PlanCycle( )
{
	CDumpStop*		pDumpStop = ::GetDumpStopObj( m_SectionID, m_frRunningLane);
	CRunOptions&	rRunOptions = *::GetRunOptions( m_frRunningLane);
	// WARNING: CAREFUL !!!
	// Each planning can be effected by other device/step's planning.
	// ex) CStepAlign::AssignHead gets effect from what is the last pick head
	//	   for optimizing LSO position.

	eCycleStatus status;

PlanCylce:
	if ( m_PrefetchOp != eCYCLE_PREFETCH_DUMP && m_PrefetchOp != eCYCLE_PREFETCH_NOZZLE )
	{
		// Select step modules for each head-step of each placement
		status = CStepCycleOptimize	::AssignHeadsCycleOptimize	( m_CycleData);
		m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
		if (m_ReturnStatus >= eCYCLE_FAILED)
			goto CycleErr;
	}

	// Select step modules for each head-step of each placement
	status = CStepDump	::AssignHeadsDump	( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;

	// Check if planning should be cut-short here so that things like pushers won't be pushed
	// in anticipation of picks that will not really occur due to a dumping condition (below).
	if ( m_PrefetchOp == eCYCLE_PREFETCH_DUMP )
		goto PrefetchSkip1;

	// Limit planning if this is a dump stop cycle.
	// This check is needed for the case when production stops with parts on heads (usually 
	//    due to a previous dump stop), and is then restarted with more parts to dump stop.
	if ( pDumpStop->IsDumpStopPlanned() )
		goto DumpStopSkip1;

	status = CStepVacStuff::AssignHeadsVacStuff( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;

	status = CStepAncPut::AssignHeadsAncPut	( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;

	status = CStepAncGet::AssignHeadsAncGet	( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;

	// PutPreInsp depends on AncPut head assignment
	status = CStepAncPutPreInsp::AssignHeadsAncPutPreInsp( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;

	// PutPostInsp depends on AncPut head assignment
	status = CStepAncPutPostInsp::AssignHeadsAncPutPostInsp( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;

	// GetPreInsp depends on AncGet head assignment
	status = CStepAncGetPreInsp::AssignHeadsAncGetPreInsp( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;

	// GetPostInsp depends on AncGet head assignment
	status = CStepAncGetPostInsp::AssignHeadsAncGetPostInsp( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;

	// GetPostInspNozType depends on AncGet head assignment
	status = CStepAncGetPostNoz::AssignHeadsAncGetPostNozTypeInsp( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;

	status = CStepPocketXy::AssignHeadsPocketXy( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;


  	if ( m_PrefetchOp == eCYCLE_PREFETCH_NOZZLE && rRunOptions.IsManualPlaceSteps() )
 		goto PrefetchSkip1;

	status = CStepPrePickInsp::AssignHeadsPrePickInsp( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;

	status = CStepPick	::AssignHeadsPick	( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;

	// WARNING - Retry Feeding is assigned AFTER Pick, even though Retry Feeding
	//			 will execute BEFORE Pick.  Reason: Retry Feeding needs the
	//			 planning information from the Pick Step.
	status = CStepRetryFeeding::AssignHeadsRetryFeeding( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;
	
	if( m_CycleData.bReAssign && status == eCYCLE_OK)
	{
        PlanCycleReset();
		m_CycleData.bReAssign = false;
		goto PlanCylce;
	}
	
	if(m_PrefetchOp == eCYCLE_PREFETCH_NOZZLE)
	{
		// Remove Prefetch PickStep
		for (int i=0; i<m_CycleData.nCount; i++)
		{
			StPlaceData& pd = m_CycleData.placeData[i];
			for (int j=0; j<_countof(pd.pStep); j++)
			{
				if( !pd.pStep[j] || !pd.pStep[j]->IsInstalled())	continue;
				if( pd.pStep[j]->GetTypePrimary() == eSTEP_PICK)
					pd.pStep[j] = NULL;				
			}
		}
		goto PrefetchSkip1;
	}

	if ( m_PrefetchOp == eCYCLE_PREFETCH_PICKUP )
		goto PrefetchSkip1;

	status = CStepPreFlux	::AssignHeadsFlux	( m_CycleData, NULL);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;
	if ( m_PrefetchOp == eCYCLE_PREFETCH_PICKUP )
		goto PrefetchSkip1;

	status = CStepAlign	::AssignHeadsAlign	( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;
	if ( m_PrefetchOp == eCYCLE_PREFETCH_ALIGN )
		goto PrefetchSkip1;

	status = CStepLeadCheck	::AssignHeadsLeadCheck	( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;
	if ( m_PrefetchOp == eCYCLE_PREFETCH_ALIGN )
		goto PrefetchSkip1;

	status = CStepSideView	::AssignHeadsSideView	( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;
	if ( m_PrefetchOp == eCYCLE_PREFETCH_ALIGN )
		goto PrefetchSkip1;


	status = CStepPostFlux	::AssignHeadsFlux	( m_CycleData, NULL);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;
	if ( m_PrefetchOp == eCYCLE_PREFETCH_PICKUP )
		goto PrefetchSkip1;

	status = CStepPlace	::AssignHeadsPlace	( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;

	status = CStepPbi	::AssignHeadsPbi	( m_CycleData);
	m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;


DumpStopSkip1:
PrefetchSkip1:

	// Create a unique-step step list of selected step modules ordered by step type
	//
	CreateStepExecList();

	eFrontRear frStep;
	for ( frStep=eFRONT; frStep<=eBOTH; frStep=eFrontRear(frStep+1))
	{
		long	idxStep;	// Index into m_Steps[].
		CStep** ppStep=NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
		long	nStep;		// Count of contiguous primary type of steps.

		// Determines the device execution order within each device type.
		nStep = GetSteps( eSTEP_DUMP,				frStep, idxStep, ppStep); CStepDump::			OrderSteps( nStep, ppStep);

		if ( m_PrefetchOp == eCYCLE_PREFETCH_DUMP )
			goto PrefetchSkip2;

		// Limit planning if this is a dump stop cycle
		if ( pDumpStop->IsDumpStopPlanned())
			goto DumpStopSkip2;

		nStep = GetSteps( eSTEP_VAC_STUFF,			frStep, idxStep, ppStep); CStepVacStuff::		OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_ANC_PUT_PRE_INSP,	frStep, idxStep, ppStep); CStepAncPutPreInsp::	OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_ANC_PUT_POST_INSP,	frStep, idxStep, ppStep); CStepAncPutPostInsp::	OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_ANC_GET_PRE_INSP,	frStep, idxStep, ppStep); CStepAncGetPreInsp::	OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_ANC_GET_POST_INSP,	frStep, idxStep, ppStep); CStepAncGetPostInsp::	OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_ANC_GET_POST_NOZ,	frStep, idxStep, ppStep); CStepAncGetPostNoz::	OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_ANC_PUT,			frStep, idxStep, ppStep); CStepAncPut::			OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_ANC_GET,			frStep, idxStep, ppStep); CStepAncGet::			OrderSteps( nStep, ppStep);

		if ( m_PrefetchOp == eCYCLE_PREFETCH_NOZZLE )
			goto PrefetchSkip2;

		nStep = GetSteps( eSTEP_CYCLE_OPTIMIZE,		frStep, idxStep, ppStep); CStepCycleOptimize::	OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_RETRY_FEEDING,		frStep, idxStep, ppStep); CStepRetryFeeding::	OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_POCKET_XY,			frStep, idxStep, ppStep); CStepPocketXy::		OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_PRE_PICK_INSP,		frStep, idxStep, ppStep); CStepPick::			OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_PICK,				frStep, idxStep, ppStep); CStepPick::			OrderSteps( nStep, ppStep);

		if ( m_PrefetchOp == eCYCLE_PREFETCH_PICKUP )
			goto PrefetchSkip2;

		nStep = GetSteps( eSTEP_PRE_FLUX,			frStep, idxStep, ppStep); CStepPreFlux::		OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_ALIGN,				frStep, idxStep, ppStep); CStepAlign::			OrderSteps( nStep, ppStep);

		if ( m_PrefetchOp == eCYCLE_PREFETCH_ALIGN )
			goto PrefetchSkip2;

		nStep = GetSteps( eSTEP_POST_FLUX,			frStep, idxStep, ppStep); CStepPostFlux::		OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_PLACE,				frStep, idxStep, ppStep); CStepPlace::			OrderSteps( nStep, ppStep);
		nStep = GetSteps( eSTEP_PBI,				frStep, idxStep, ppStep); CStepPbi::			OrderSteps( nStep, ppStep);
DumpStopSkip2:
PrefetchSkip2:
		;
	}

	m_ReturnStatus = PlanSteps( m_ReturnStatus);
	if (m_ReturnStatus >= eCYCLE_FAILED)
		goto CycleErr;

#ifdef SIMULATION
	TelemetryPickOrder();
#endif


CycleErr:
	return m_ReturnStatus;
}


long
CSeqCycle::GetSteps( eStepType aPrimaryStepType, eFrontRear aStepFR, long& anIdxStep, CStep**& appStep) const
{
	anIdxStep = -1;
	long			nCount = 0;
	const CStep**	ppStep = NULL;
	for (int i=0; i<m_nSteps; i++)
	{
		CStep* pStep = m_Steps[i];
		if ( pStep == NULL)
			break;
		if ( pStep->m_TypePrimary != aPrimaryStepType)
		{
			if ( ppStep != NULL)
				break;
			continue;
		}
//TEMP DELETE until front/rear groupings of steps can be managed by all the callers.
//TEMP DELETE until front/rear groupings of steps can be managed by all the callers.
		if ( pStep->m_frRunningLane != aStepFR)
		{
			if ( ppStep != NULL)
				break;
			continue;
		}
		if ( ppStep == NULL)
		{
			ppStep = const_cast<const CStep**>( &(m_Steps[i]));
			anIdxStep = i;
		}
		nCount++;
	}
	appStep = const_cast<CStep**>( ppStep);
	return nCount;
}


void
CSeqCycle::CreateStepExecList()
{
	int i;
	int j;
	int k;
	int n;
	// Gather the step modules in lists by step type.
	m_nSteps = 0;
	memset( m_Steps, 0, sizeof(m_Steps));

	// Each placement, StPlaceData from m_CycleData, contains the steps objects to be executed for 1 head.
	// A typical StPlaceData::pStep[], for 1 head, will look like
	//		StPlaceData::pStep[ eSTEP_NONE]		= NULL
	//		StPlaceData::pStep[ eSTEP_DUMP]		= NULL
	//		StPlaceData::pStep[ eSTEP_VAC_STUFF]= NULL
	//		StPlaceData::pStep[ eSTEP_ANC_PUT]	= NULL
	//		StPlaceData::pStep[ eSTEP_ANC_GET]	= NULL
	//		StPlaceData::pStep[ eSTEP_POCKET_XY]= NULL
	//		StPlaceData::pStep[ eSTEP_PICK]		= ptr to CStepPickTapeFly
	//		StPlaceData::pStep[ eSTEP_FLUX]		= NULL
	//		StPlaceData::pStep[ eSTEP_ALIGN]	= ptr to CStepAlignFly
	//		StPlaceData::pStep[ eSTEP_PLACE]	= ptr to CStepPlaceFly
	//
	// But another head in the same cycle may look like.
	//		StPlaceData::pStep[ eSTEP_NONE]		= NULL
	//		StPlaceData::pStep[ eSTEP_DUMP]		= NULL
	//		StPlaceData::pStep[ eSTEP_VAC_STUFF]= NULL
	//		StPlaceData::pStep[ eSTEP_ANC_PUT]	= NULL
	//		StPlaceData::pStep[ eSTEP_ANC_GET]	= NULL
	//		StPlaceData::pStep[ eSTEP_POCKET_XY]= NULL
	//		StPlaceData::pStep[ eSTEP_PICK]		= ptr to CStepPickTapeFly
	//		StPlaceData::pStep[ eSTEP_FLUX]		= NULL
	//		StPlaceData::pStep[ eSTEP_ALIGN]	= ptr to CStepAlignUp		<-- note this is different
	//		StPlaceData::pStep[ eSTEP_PLACE]	= ptr to CStepPlaceFly
	//
	// The unique list of steps is gathered together in primary step order to produce 
	// an executable list of steps in the CSeqCycle::m_Step[] array.
	//		CSeqCycle::m_Step[ 0] = ptr to CStepPickTapeFly
	//		CSeqCycle::m_Step[ 1] = ptr to CStepAlignFly
	//		CSeqCycle::m_Step[ 2] = ptr to CStepAlignUp
	//		CSeqCycle::m_Step[ 3] = ptr to CStepPlaceFly
	//		CSeqCycle::m_Step[ 4] = NULL
	//		CSeqCycle::m_Step[ 5] = NULL
	//		CSeqCycle::m_Step[ 6] = NULL
	//
	// The starting index of each primary step in m_Step[] is recorded in m_StepStartIdx[]
	//		CSeqCycle::m_StepStartIdx[ eSTEP_NONE]		= 0
	//		CSeqCycle::m_StepStartIdx[ eSTEP_DUMP]		= 0
	//		CSeqCycle::m_StepStartIdx[ eSTEP_VAC_STUFF]	= 0
	//		CSeqCycle::m_StepStartIdx[ eSTEP_ANC_PUT]	= 0
	//		CSeqCycle::m_StepStartIdx[ eSTEP_ANC_GET]	= 0
	//		CSeqCycle::m_StepStartIdx[ eSTEP_POCKET_XY]	= 0
	//		CSeqCycle::m_StepStartIdx[ eSTEP_PICK]		= 0	<- the 0 index of the m_Step[] array points to the 1st pick object
	//		CSeqCycle::m_StepStartIdx[ eSTEP_FLUX]		= 0
	//		CSeqCycle::m_StepStartIdx[ eSTEP_ALIGN]		= 1
	//		CSeqCycle::m_StepStartIdx[ eSTEP_PLACE]		= 3
	//
	// and the count for each primary step type is also recorded in m_StepCount[].
	//		CSeqCycle::m_StepCount[ eSTEP_NONE]			= 0
	//		CSeqCycle::m_StepCount[ eSTEP_DUMP]			= 0
	//		CSeqCycle::m_StepCount[ eSTEP_VAC_STUFF]	= 0
	//		CSeqCycle::m_StepCount[ eSTEP_ANC_PUT]		= 0
	//		CSeqCycle::m_StepCount[ eSTEP_ANC_GET]		= 0
	//		CSeqCycle::m_StepCount[ eSTEP_POCKET_XY]	= 0
	//		CSeqCycle::m_StepCount[ eSTEP_PICK]			= 1 <- there is 1 pick object in the m_Step[] array.
	//		CSeqCycle::m_StepCount[ eSTEP_FLUX]			= 0
	//		CSeqCycle::m_StepCount[ eSTEP_ALIGN]		= 2
	//		CSeqCycle::m_StepCount[ eSTEP_PLACE]		= 1

	// Create a unique-step step list of selected step modules ordered by step type
	//
	CStep* pStep;
	for (i=0; i<eSTEP_MAX; i++)
	{
		int startStepIdx = m_nSteps; 

		for (j=0; j<m_CycleData.nCount; j++)
		{
			// Special handling for ANC put steps. Unmounts may include a head not in m_CycleData.placeData.

			pStep = NULL;
			if ( i == eSTEP_ANC_PUT )
			{
				for (k=0; k<m_CycleData.ancPutData.GetCount(); k++)
				{
					pStep = m_CycleData.ancPutData[k].pAncPutStep;
					if ( pStep == NULL )	continue;

					for ( n=0; n<m_nSteps; n++)	
						if ( pStep == m_Steps[n])	
							break;
					
					if ( n == m_nSteps)
					{
						m_Steps[ m_nSteps++] = pStep;
					}
					SYS_ASSERT( m_nSteps < eSTEP_MAX);
				}
				continue;
			}
			else if ( i == eSTEP_ANC_GET )
			{
				for (k=0; k<m_CycleData.ancGetData.GetCount(); k++)
				{
					pStep = m_CycleData.ancGetData[k].pAncGetStep;
					if ( pStep == NULL )
						continue;

					for ( n=0; n<m_nSteps; n++)	
						if ( pStep == m_Steps[n])	
							break;
					
					if ( n == m_nSteps)
					{
						m_Steps[ m_nSteps++] = pStep;
					}
					SYS_ASSERT( m_nSteps < eSTEP_MAX);
				}
				continue;
			}
			else if ( i == eSTEP_DUMP )
			{
				if ( j < m_CycleData.dumpData.GetCount() )
					pStep = m_CycleData.dumpData[j].pDumpStep;
				else
					pStep = NULL;
			}
			else
			{
				pStep = m_CycleData.placeData[j].pStep[i];
			}

			if ( pStep != NULL )
			{
				for ( k=0; k<m_nSteps; k++)	
					if ( pStep == m_Steps[k])	
						break;
				
				if ( k == m_nSteps)
				{
					m_Steps[ m_nSteps++] = pStep;
				}
				SYS_ASSERT( m_nSteps < eSTEP_MAX);
			}
		}
	}


//TEMP DELETE until front/rear groupings of steps can be managed by all the callers.
//TEMP DELETE until front/rear groupings of steps can be managed by all the callers.
	// Sort by FRONT/REAR, then by eStepType.
	qsort( m_Steps, m_nSteps, sizeof(CStep*), CSeqCycle::CompareStepFR);
}


int 
CSeqCycle::CompareStepFR( const void *apStep1, const void *apStep2)
{
	CStep* pStep1 = *((CStep**)apStep1);
	CStep* pStep2 = *((CStep**)apStep2);

	// All dumping must occur before any ANC because any spindle can get any nozzle hole.
	if ( pStep1->m_TypePrimary<=eSTEP_VAC_STUFF && pStep2->m_TypePrimary>eSTEP_VAC_STUFF)
		return -1;
	if ( pStep1->m_TypePrimary>eSTEP_VAC_STUFF && pStep2->m_TypePrimary<=eSTEP_VAC_STUFF)
		return +1;

	// ANC Puts and ANC Put Inspection can be grouped, but must occur before any ANC Get related steps.
	if ( pStep1->m_TypePrimary<=eSTEP_ANC_PUT_POST_INSP && pStep2->m_TypePrimary>eSTEP_ANC_PUT_POST_INSP)
		return -1;
	if ( pStep1->m_TypePrimary>eSTEP_ANC_PUT_POST_INSP && pStep2->m_TypePrimary<=eSTEP_ANC_PUT_POST_INSP)
		return +1;

	// ANC Gets and ANC Get Inspection can be grouped, but must occur before any picking related steps.
	if ( pStep1->m_TypePrimary<=eSTEP_ANC_GET_POST_NOZ && pStep2->m_TypePrimary>eSTEP_ANC_GET_POST_NOZ)
		return -1;
	if ( pStep1->m_TypePrimary>eSTEP_ANC_GET_POST_NOZ && pStep2->m_TypePrimary<=eSTEP_ANC_GET_POST_NOZ)
		return +1;

	// Picking, Fluxing and Aligning can be grouped, but must occur before any Placing or PBI related steps.
	if ( pStep1->m_TypePrimary<=eSTEP_POST_FLUX && pStep2->m_TypePrimary>eSTEP_POST_FLUX)
		return -1;
	if ( pStep1->m_TypePrimary>eSTEP_POST_FLUX && pStep2->m_TypePrimary<=eSTEP_POST_FLUX)
		return +1;

	// If a step is designated as eBOTH, then ignore F/R sorting.
	if ( pStep1->m_FrontRear!=eBOTH  &&  pStep2->m_FrontRear!=eBOTH)
	{
		if ( pStep1->m_FrontRear < pStep2->m_FrontRear)
			return -1;
		if ( pStep1->m_FrontRear > pStep2->m_FrontRear)
			return +1;
	}
	else
		int k=1;	// One of the steps is eBOTH, so don't sort by F/R

	if ( pStep1->m_TypePrimary < pStep2->m_TypePrimary)
		return -1;
	if ( pStep1->m_TypePrimary > pStep2->m_TypePrimary)
		return +1;
	return 0;
}



eStepType g_StepPlanOrderTbl[] =
{
	// WARNING - Only PRIMARY steps are allowed in this table.  DO NOT ADD any STEP VARIATIONS
	eSTEP_CYCLE_OPTIMIZE,	//Cycle Optimization
	eSTEP_DUMP,
	eSTEP_VAC_STUFF,
	eSTEP_ANC_PUT_PRE_INSP,
	eSTEP_ANC_PUT,
	eSTEP_ANC_PUT_POST_INSP,
	eSTEP_ANC_GET_PRE_INSP,
	eSTEP_ANC_GET,
	eSTEP_ANC_GET_POST_INSP,
	eSTEP_ANC_GET_POST_NOZ,
	eSTEP_PRE_PICK_INSP,
	eSTEP_PICK,
	eSTEP_RETRY_FEEDING,
	eSTEP_POCKET_XY,		// Plan this after pick planning so the pick feeder is resolved.
	eSTEP_PRE_FLUX,
	eSTEP_ALIGN,
	eSTEP_LEAD_CHECK,
	eSTEP_POST_FLUX,
	eSTEP_PLACE,
	eSTEP_PBI
};


eCycleStatus
CSeqCycle::PlanSteps( eCycleStatus aRetStatus)
{
	eCycleStatus status;

	CTelemetryPair tp( EV_CLASSENGINE, m_pGantry->Unit(), "G%s:PlanSteps", m_pGantry->NameBrief());

	// NOTE: Plan this step after the pick step because CStepPocketXy must
	//		 know the pick feeder ID for planning (even though this step
	//		 is actually executed BEFORE the CStepPick).

	// Plan the steps in the order specified by the planning table.
	for (int i=0; i<_countof(g_StepPlanOrderTbl); i++)
	{
		eStepType stepType = g_StepPlanOrderTbl[i];

		// Plan each step... Head ordering, grouping, simultaneous operations, etc...
		for (int idx=0; idx<m_nSteps; idx++)
		{
			CStep& step = *m_Steps[ idx];
			if ( step.GetTypePrimary() != stepType)		continue;

			// PLAN STEP, Optimize it
			status = step.PlanStep( m_CycleData);
			aRetStatus = (status > aRetStatus ? status : aRetStatus);
			if (aRetStatus >= eCYCLE_FAILED)
				return aRetStatus;
		}
	}
	return aRetStatus;
}

eCycleStatus
CSeqCycle::PlanStepsByType( eCycleStatus aRetStatus, eStepType aStepType )
{
	eCycleStatus status;

	// NOTE: Plan this step after the pick step because CStepPocketXy must
	//		 know the pick feeder ID for planning (even though this step
	//		 is actually executed BEFORE the CStepPick).

	// Plan the steps in the order specified by the planning table.
	//for (int i=0; i<_countof(g_StepPlanOrderTbl); i++)
	{
		//eStepType stepType = g_StepPlanOrderTbl[i];

		// Plan each step... Head ordering, grouping, simultaneous operations, etc...
		for (int idx=0; idx<m_nSteps; idx++)
		{
			CStep& step = *m_Steps[ idx];
			if ( step.GetTypePrimary() != aStepType )		continue;

			// PLAN STEP, Optimize it
			status = step.PlanStep( m_CycleData);
			aRetStatus = (status > aRetStatus ? status : aRetStatus);
			if (aRetStatus >= eCYCLE_FAILED)
				return aRetStatus;

			// MxCameraType	
			status = step.PlanRegions( m_CycleData);
			aRetStatus = (status > aRetStatus ? status : aRetStatus);
			if (aRetStatus >= eCYCLE_FAILED)
				return aRetStatus;
		}

		//Apply Expended region to motion manager
		m_pGantry->SetRegions();
	}
	return aRetStatus;
}


eCycleStatus
CSeqCycle::PlanRegions( )
{
	eCycleStatus status;

	CTelemetryPair tp( EV_CLASSENGINE, m_pGantry->Unit(), "G%s:PlanRegions", m_pGantry->NameBrief());

	m_pGantry->ResetNextRegions();

	// Plan each step...
	for (int i=0; i<m_nSteps; i++)
	{
		status = m_Steps[i]->PlanRegions( m_CycleData);
		m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
		if (m_ReturnStatus >= eCYCLE_FAILED)
			break;
	}

	if (m_ReturnStatus==eCYCLE_OK  &&  m_nSteps)
	{
		Telemetry( EV_CLASSENGINE, m_pGantry->Unit(), "G%s:PlanRegions - SetRegions", m_pGantry->NameBrief());
		m_pGantry->SetRegions();
	}
	return m_ReturnStatus;
}

#ifdef SIMULATION
void
CSeqCycle::TelemetryPickOrder( )
{
		// Spit out the pick order
	bool		bFirst = true;
	char		ctxt[20];
	TPDOrder	pickDataOrder;
	CFixString	szOrder = "PICK Order=";
	CFixString	szBrief = "   ";
	GetPickOrder( pickDataOrder);
	for (int i=0; i<pickDataOrder.GetCount(); i++)
	{
		int idxPd = pickDataOrder[i];
		StPlaceData& pd = m_CycleData.placeData[ idxPd];
		_itoa( pd.pHead->Unit(), ctxt, 10);
		if ( !bFirst)
		{
			szOrder += ", ";
			szBrief += ", ";
		}
		bFirst = false;
		szOrder += ctxt;
		szBrief += pd.pHead->NameBrief();
	}
	Telemetry( EV_CLASSFEEDER, 0, szOrder.c_str());
}
#endif



void
CSeqCycle::GetLastPickR( TAxRbyAxisMap& aAxRbyAxisMap) const
{
	aAxRbyAxisMap.RemoveAll();

	long	idxStep;	// Index into m_Steps[].
	CStep** ppStep=NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
	long	nStep;		// Count of contiguous primary type of steps.

	nStep = GetSteps( eSTEP_PICK, m_frRunningLane, idxStep, ppStep);
	for (int i=0; i<nStep; i++)
	{
		CStepPick* pStepPick = (CStepPick*)m_Steps[ idxStep+i];
		pStepPick->GetLastR( aAxRbyAxisMap);
	}
}

void
CSeqCycle::GetLastPickR( dTAxRbyAxisMap& aAxRbyAxisMap) const
{
	aAxRbyAxisMap.RemoveAll();
	
	long	idxStep;	// Index into m_Steps[].
	CStep** ppStep=NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
	long	nStep;		// Count of contiguous primary type of steps.

	nStep = GetSteps( eSTEP_PICK, m_frRunningLane, idxStep, ppStep);
	for (int i=0; i<nStep; i++)
	{
		CStepPick* pStepPick = (CStepPick*)m_Steps[ idxStep+i];
		pStepPick->GetLastR( aAxRbyAxisMap);
	}
}


	// Returns a step pointer to the place step.
CStepPlace*
CSeqCycle::GetPlaceStep( ) const
{
	long	idxStep;	// Index into m_Steps[].
	CStep** ppStep=NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
	long	nStep;		// Count of contiguous primary type of steps.

	nStep = GetSteps( eSTEP_PLACE, m_frRunningLane, idxStep, ppStep);
	if( idxStep < 0 )
		return NULL;
	return (CStepPlace*)m_Steps[ idxStep];
}


	// Returns a step pointer to the pocket teach step.
CStepPocketXy*
CSeqCycle::GetPocketXyStep( ) const
{
	long	idxStep;	// Index into m_Steps[].
	CStep** ppStep=NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
	long	nStep;		// Count of contiguous primary type of steps.

	nStep = GetSteps( eSTEP_POCKET_XY, m_frRunningLane, idxStep, ppStep);
	if( idxStep < 0 )
		return NULL;
	return (CStepPocketXy*)m_Steps[ idxStep];
}



// Returns a step pointer to the PBI step.
CStepPbi*
CSeqCycle::GetPbiStep( ) const
{
	long	idxStep;	// Index into m_Steps[].
	CStep** ppStep=NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
	long	nStep;		// Count of contiguous primary type of steps.

	nStep = GetSteps( eSTEP_PBI, m_frRunningLane, idxStep, ppStep);
	if( idxStep < 0 )
		return NULL;
	return (CStepPbi*)m_Steps[ idxStep];
}





CStepPickTrayFeeder*
CSeqCycle::GetTrayFeederStep( ) const
{
	long	idxStep;	// Index into m_Steps[].
	CStep** ppStep=NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
	long	nStep;		// Count of contiguous primary type of steps.

	nStep = GetSteps( eSTEP_PICK, m_frRunningLane, idxStep, ppStep);
	
	// No pick steps in some prefetch cycles
	if ( idxStep == -1)
		return NULL;

	for (int i=0; i<nStep; i++)
	{
		CStepPick* pStepPick = (CStepPick*)m_Steps[ idxStep+i];
		eStepType stepType = pStepPick->GetType();
		
		if ( stepType != eSTEP_PICK_TRAY)	continue;

		return (CStepPickTrayFeeder*)pStepPick;
	}
	return NULL;
}


CStep*
CSeqCycle::GetNextStep( const CStep* apStep) const
{
	int i;
	// Look for the specified step.
	for (i=0; i<m_nSteps; i++)
	{
		if (m_Steps[i] == apStep)
			break;
	}

	// Index to the following step
	i++;
	// Return the next step, if one exists
	return ( i<m_nSteps ? m_Steps[i] : NULL);
}


CStep*
CSeqCycle::GetPrevStep( const CStep* apStep) const
{
	CStep* pPrevStep = NULL;
	int i;
	// Look for the specified step.
	for (i=0; i<m_nSteps; i++)
	{
		if (m_Steps[i] == apStep)
			break;
		// Record this step as a previous step if its defined.
		if ( m_Steps[i])
			pPrevStep = m_Steps[i];
	}

	// Return the prev step, if one exists
	return pPrevStep;
}

CStep*
CSeqCycle::GetPrevPickStep( const CStep* apStep) const
{
	CStep* pPrevStep = NULL;
	int i=0;

	long	idxStep;	// Index into m_Steps[].
	CStep** ppStep=NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
	long	nStep;		// Count of contiguous primary type of steps.

	nStep = GetSteps( eSTEP_PICK, m_frRunningLane, idxStep, ppStep);

	int endIndex		= idxStep+nStep-1;

	// Look for the specified step.
	for (i=idxStep; i<=endIndex; i++)
	{
		if (m_Steps[i] == apStep)
			break;
		// Record this step as a previous step if its defined.
		if ( m_Steps[i])
			pPrevStep = m_Steps[i];
	}
	
	// Return the prev step, if one exists
	return pPrevStep;
}


double
CSeqCycle::GetNextZMove( CStep* apStep, HeadID aHeadID) const
{
	CFxyzt firstMove;

	while ( (apStep=GetNextStep( apStep)) != NULL  &&  firstMove.z==NaN)
		apStep->GetFirstMove( firstMove, aHeadID);
	return firstMove.z;
}



eStatus
CSeqCycle::GetNextNearlyDoneZMove( CStep* apStep, HeadID aHeadID, double& aEscHeight, eMoveOpt& aMoveOpt) const
{
	eStatus status = eNG;
	aEscHeight = NaN;

	while ( (apStep=GetNextStep( apStep)) != NULL  &&  status!=eFAULT  &&  status==eNG)
		status = apStep->GetFirstNearlyDoneZ( aEscHeight, aMoveOpt, aHeadID);
	return status;
}


eCycleStatus
CSeqCycle::ExecuteCycle( )
{
	eCycleStatus	status;
	eFeederType		feederType;
	eMachineState	machineState;
	TStepPickTrayFdrList	trayStepList;
	CStepPickTrayFeeder*	pStepPickTFNxt;

	long h;
	long m;
	bool bWorked = false;
	int lastTrayStep = -1;
	int i=0;
	bool bNeedTrayPrefetchrequired = false;

	::GetStepPickTrayFeederList( m_GantryID, trayStepList);

	for ( i=0; i<m_nSteps; i++)
	{
		CStep* pStep = m_Steps[i];
		eStepType stepType = pStep->GetType();
		if (eSTEP_PICK_TRAY_FLY <= stepType && stepType <= eSTEP_PICK_TRAY)
			lastTrayStep = i;		
	}

	// Execute each step...
	for ( i=0; i<m_nSteps; i++)
	{
		CStep* pStep = m_Steps[i];

		if(bWorked==false)
		{
			status = PreExecute( pStep, bWorked );
		}

		// Check Next Placement Data before STEP_PLACE
		if ( pStep->GetTypePrimary() == eSTEP_PLACE && m_NextCycleData.pCycleSeq != NULL )
		{
			status = PreFluxingPlanCycle( m_NextCycleData);
			m_ReturnStatus = ( status > m_ReturnStatus ? status : m_ReturnStatus);
			if ( m_ReturnStatus >= eCYCLE_STOPPED)
				return m_ReturnStatus;

			status = CStepPreFlux::PreExecuteFluxing( m_NextCycleData, bWorked);
			m_ReturnStatus = ( status > m_ReturnStatus ? status : m_ReturnStatus);
			if ( m_ReturnStatus >= eCYCLE_STOPPED)
				return m_ReturnStatus;

			if ( bWorked)
				Telemetry( EV_CLASSENGINE, 0, "Execute PreFluxing for Next Current Step- StepPreFluxStep");

			if ( !bWorked)
			{
				status = CStepPostFlux::PreExecuteFluxing( m_NextCycleData, bWorked);
				m_ReturnStatus = ( status > m_ReturnStatus ? status : m_ReturnStatus);
				if ( m_ReturnStatus >= eCYCLE_STOPPED)
					return m_ReturnStatus;
				if ( bWorked)
					Telemetry( EV_CLASSENGINE, 0, "Execute PreFluxing for Next Current Step- StepPostFluxStep");
			}
		}

 		m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
 		if ( m_ReturnStatus >= eCYCLE_STOPPED)
 			break;

		PrefetchRadialAxialFeeder( pStep->GetTypePrimary( ), __LINE__);

		status = pStep->Execute( m_CycleData);
		m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
		if ( m_ReturnStatus >= eCYCLE_STOPPED)
			break;

		machineState = g_StateReport.GetMachineState( m_SectionID, m_frRunningLane);
		if ( machineState >= eMS_FREEZE )
		{
			m_ReturnStatus = eCYCLE_ABORT;
			break;
		}

		// Do NOT call tray pre-loading during manual pick operation.
		if ( g_StateReport.IsManualCmdActive( m_frRunningLane) )
			continue;

		status = PostExecutePrepareNextCycle( );
		m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
		if ( m_ReturnStatus >= eCYCLE_STOPPED)
			break;

		if (CPCBoard::IsStackTrayNeeded())
		{
			//----------------------------------------------------------------
			//	Stack-Tray Feeder Operations.
			//----------------------------------------------------------------
			bool bTrayPost = false;
			m_LastStepType = pStep->GetTypePrimary();
			if (m_LastStepType == eSTEP_PLACE && g_FacDebug.IsUseReturnToStackTray())
			{
				for (int k = 0; k < m_CycleData.nCount; k++)
				{
					StPlaceData& pd = m_CycleData.placeData[k];
					eFeederType feedType = pd.feederID.Type();
					ePartFailReason	partFailReason = pd.pHead->GetPartFailReason();
					if (!pd.feederID.IsDefined())			continue;
					if (feedType != eFEEDTYPE_TRAY)			continue;

					// Wait vision align result about only tray type.
					if (pd.pHead->IsPickedFeeder()
						&& partFailReason != ePART_FAIL_NONE)
					{
						bTrayPost = false;
						break;
					}
					bTrayPost = true;
				}

				if (bTrayPost)
				{
					status = TrayFeederPostStepRefillStackTF(pStep->GetTypePrimary());
					m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
					if (m_ReturnStatus >= eCYCLE_STOPPED)
						return m_ReturnStatus;
				}
			}
		}
		else if (CPCBoard::IsMultiTrayNeeded())
		{
			//----------------------------------------------------------------
			//	Multi-Tray Feeder Operations.
			//----------------------------------------------------------------
			// Save Gantry Step for TrayFeeder
			m_LastStepType = pStep->GetTypePrimary();

			if (!CPCBoard::IsMultiTrayNeeded())		continue;

			eTrayPalletSpeed palletSpeed = g_SysConst.GetTrayPalletSpeed();

			// Check Empty Pallet.
			for (int iTray=0; iTray<trayStepList.GetCount(); iTray++)
			{
				pStepPickTFNxt = trayStepList[iTray];
				pStepPickTFNxt->CheckEmptyPallet( m_CycleData, m_LastStepType);
			}

			bool bIsFastOrFastest = (m_LastStepType==eSTEP_PICK   &&  palletSpeed==eTRAY_PALLET_FASTEST)
								||	(m_LastStepType==eSTEP_ALIGN  && (palletSpeed == eTRAY_PALLET_FAST || (palletSpeed == eTRAY_PALLET_FASTEST && bNeedTrayPrefetchrequired)));
			// If tray seq. is 'Normal', do NOT call next tray yet.
			if ( !bIsFastOrFastest )			continue;

			// "Next pallet" must plan after pick up. This is important.
			// Because of "actual tray pocket count".
			// Actually After pick up, we check "empty pallet", "tray pocket count"   
			TPlaceWsList	trayPlaceList;
			TTrayFeederWait	tfWaitList;
			eLookCycles		lookCycle;
			CSeqEngine&		seqEngine = g_SeqEngine[ m_pGantry->Unit()];
			CTrayFeeder*	pTrayFeeder = NULL;

			if ( m_PrefetchOp != eCYCLE_PREFETCH_NONE  &&  m_PrefetchOp < eCYCLE_PREFETCH_PICKUP)
				lookCycle = eLOOK_INCLUDE_CURRENT_CYCLE;
			else
				lookCycle = eLOOK_EXCLUDE_CURRENT_CYCLE;

			// Get advanced placements (ahead of the current cycle, and including the
			// current cycle if a no pick prefetch is being done)
			seqEngine.GetLookAheadTrayCycles( trayPlaceList, lookCycle);
			for ( h=0; h<_countof(m_TrayFeederNxtPallet); h++)	m_TrayFeederNxtPallet[h] = 0;
			for ( m=0; m<_countof(m_TrayFeederBufferChg); m++)	m_TrayFeederBufferChg[m] = eTRAY_BUFCHG_YES;
		
			if ( m_LastStepType==eSTEP_PICK	&& palletSpeed==eTRAY_PALLET_FASTEST)
			{
				// Release USE of any tray feeders used by this step.

				int		lastTrayPdIndex = -1;
				bool	bTrayPost = true;

				if ( lastTrayStep > -1 && lastTrayStep != i)		
					continue;

				if ( !PlanTrayFeederNextPallet( trayPlaceList, tfWaitList, m_TrayFeederNxtPallet, m_TrayFeederBufferChg))
					continue;

				for ( int k=0; k<m_CycleData.nCount; k++)
				{
					StPlaceData& pd = m_CycleData.placeData[k];
			
					if ( !pd.feederID.IsDefined())		continue;

					feederType	= pd.feederID.Type();
					if ( feederType != eFEEDTYPE_TRAY)	continue;

					CTrayFeeder* pTrayFeeder = ::GetTrayFeeder(pd.pickedFeederID);
					if (pd.pickedFeederID.IsDefined() && pd.pickedFeederID.Type() == eFEEDTYPE_TRAY)
					{
						CTray* pTray = ::GetTray(pd.pickedFeederID);
						if (m_bLastTrayCycle[pd.pickedFeederID.Unit()] && pTray && pTray->IsTrayEmpty())
						{
							bNeedTrayPrefetchrequired = true;
							bTrayPost = false;
						}
					}

					if ( pd.status > ePLACE_OK )
					{
						bTrayPost = false;
						break;
					}
				}

				if ( bTrayPost && CPCBoard::IsMultiTrayNeeded() )
				{
					// Prefetch pallets for tray feeders used in this step AND used in future cycles.*/
					status = TrayFeederPostStepPalletPrefetch( pStep->GetTypePrimary());
					m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
					if ( m_ReturnStatus >= eCYCLE_STOPPED)
						return m_ReturnStatus;
				}
			}
			else if ( m_LastStepType==eSTEP_ALIGN && (palletSpeed==eTRAY_PALLET_FAST || (palletSpeed == eTRAY_PALLET_FASTEST && bNeedTrayPrefetchrequired)))
			{
				eStatus				stat;
				bool				bTrayPost = true;

				if ( !PlanTrayFeederNextPallet( trayPlaceList, tfWaitList, m_TrayFeederNxtPallet, m_TrayFeederBufferChg))
				{
					// Release USE of any tray feeders used by this step.
					int l;
					for (l=0; l<m_CycleData.nCount; l++)
					{
						StPlaceData& pd					= m_CycleData.placeData[l];
						eFeederType feedType			= pd.feederID.Type();
						ePartFailReason	partFailReason	= pd.pHead->GetPartFailReason();
						pTrayFeeder = ::GetTrayFeeder(pd.feederID);
						if ( !pd.feederID.IsDefined())		continue;
						if ( pTrayFeeder == NULL)			continue;

						// If the placement has already failed, DO NOT  WaitAlignResult().
						if ( pd.status != ePLACE_OK)				continue;
						if ( feedType != eFEEDTYPE_TRAY)			continue;

						// Wait vision align result about only tray type.
						stat = WaitAlignResult( pd.head);
						if ( stat == eFAULT || stat == eNG)			continue;
						if ( partFailReason != ePART_FAIL_NONE )	continue;
					
						bTrayPost = true;
						pTrayFeeder->ReleaseUse( m_pGantry->Unit() );
					}

					continue;
				}
			
				for ( int m=0; m<m_CycleData.nCount; m++ )
				{
					StPlaceData& pd = m_CycleData.placeData[m];
				
					if ( !pd.feederID.IsDefined())		continue;

					feederType	= pd.feederID.Type();
					if ( feederType != eFEEDTYPE_TRAY)			continue;

					// NOTE :
					// If trayPickUp Head is failed to pickup, 
					// then the alignResult of this head will not be posted ever.
					// So, skip failed head.
					if ( pd.status > ePLACE_OK )
					{
						bTrayPost = false;
						break;
					}

					if ( pStep != pd.pStep[eSTEP_ALIGN] )		continue;

					pTrayFeeder = ::GetTrayFeeder( pd.feederID);
					if ( !pd.feederID.IsDefined())		continue;
					if ( pTrayFeeder == NULL)			continue;

					if ( m_TrayFeederNxtPallet[pTrayFeeder->Unit()] != pTrayFeeder->GetPalletOutNowNo())
					{
						// Wait vision align result about only tray type.
						stat = WaitAlignResult( pd.head);
						if ( stat == eFAULT || stat == eNG)
						{
							bTrayPost = false;
							break;
						}
				
						ePartFailReason		partFailReason = pd.pHead->GetPartFailReason();
						//if ( partFailReason == ePART_FAIL_VACUUM || partFailReason == ePART_FAIL_VISION)
						if ( partFailReason != ePART_FAIL_NONE )
						{
							bTrayPost = false;
							break;
						}
					}
				}
				
				// If Vision Inspection result is 'fail', don't call 'Next Pallet'. Because of 'Dump'.
				if (bTrayPost && CPCBoard::IsMultiTrayNeeded())
				{
					// Prefetch pallets for tray feeders used in this step AND used in future cycles.
					status = TrayFeederPostStepPalletPrefetch( pStep->GetTypePrimary());
					m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
					if ( m_ReturnStatus >= eCYCLE_STOPPED )
						return m_ReturnStatus;
				}
			}
		}
	}
		
	return m_ReturnStatus;
}


eCycleStatus
CSeqCycle::PreExecute( CStep* apStep, bool& abWorked )
{
	eCycleStatus status;
	
	// Fluxing Befor pick can be usable only at StepPreFluxVHP and StepPreFluxRHS
	if ( !CStepPreFlux::IsUsableFluxingBeforePick(m_CycleData) )
		return eCYCLE_OK;

	// make film
	eStepType stepType = apStep->GetTypePrimary();
	if ( eSTEP_PICK == stepType )
	{
		bool bWorked = false;
 		
//		if ( IsCycleForFluxingPlaning())
//		{
//			status = PreFluxingPlanCycle( m_CycleData );
//			m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
//			if ( m_ReturnStatus >= eCYCLE_STOPPED )
// 				return m_ReturnStatus;
//		}
//
		status = CStepPreFlux::PreExecuteFluxing( m_CycleData, bWorked );
		m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
		if ( m_ReturnStatus >= eCYCLE_STOPPED )
			return m_ReturnStatus;
		if ( bWorked )
			Telemetry( EV_CLASSENGINE, 0, "Execute PreFluxing for Current Step- StepPreFluxStep"); 


		if ( !bWorked )
		{
			status = CStepPostFlux::PreExecuteFluxing( m_CycleData, bWorked );
			m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
			if ( m_ReturnStatus >= eCYCLE_STOPPED )
				return m_ReturnStatus;
			if ( bWorked )
				Telemetry( EV_CLASSENGINE, 0, "Execute PreFluxing for Current Step- StepPostFluxStep");  			
		}	

		// Check Next Placement Data
		// 2025.12.16 Deleted : next cycle moved to ExecuteCycle
		abWorked = bWorked;
	}
	
	return eCYCLE_OK;
}

eCycleStatus
CSeqCycle::PreFluxingPlanCycle( StCycleData& arCycleData)
{
	CStepPreFlux*	pStepPreFlux;
	CStepPostFlux*	pStepPostFlux;
	eCycleStatus status;
	
	bool bSectionRunning = g_StateReport.IsMachineRunning( m_SectionID, m_frRunningLane);
	if ( bSectionRunning )
	{
		THeadIdMap preFluxAassignedHeads;
		THeadIdMap postFluxAassignedHeads;

		status = CStepPreFlux	::AssignHeadsFlux( arCycleData, &preFluxAassignedHeads, true);
		m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
		if ( m_ReturnStatus >= eCYCLE_STOPPED )
			return m_ReturnStatus;

		status = CStepPostFlux	::AssignHeadsFlux( arCycleData, &postFluxAassignedHeads );
		m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
		if ( m_ReturnStatus >= eCYCLE_STOPPED )
			return m_ReturnStatus;		
	
		for (int i=0; i<arCycleData.nCount; i++)
		{
			pStepPreFlux = (CStepPreFlux*)arCycleData.placeData[i].pStep[eSTEP_PRE_FLUX];	
			if ( pStepPreFlux != NULL )
			{
				status = pStepPreFlux->PlanStepFilmOnly( arCycleData, preFluxAassignedHeads );
				m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
				if ( m_ReturnStatus >= eCYCLE_STOPPED )
					return m_ReturnStatus;	

				break;
			}
			pStepPostFlux = (CStepPostFlux*)arCycleData.placeData[i].pStep[eSTEP_POST_FLUX];	
			if ( pStepPostFlux != NULL )
			{
				status = pStepPostFlux->PlanStepFilmOnly( arCycleData, postFluxAassignedHeads);
				m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
				if ( m_ReturnStatus >= eCYCLE_STOPPED )
					return m_ReturnStatus;	
				break;
			}
		}
	}	

	return eCYCLE_OK;
}	

//bool
//CSeqCycle::IsCycleForFluxingPlaning( )
//{
//	bool bCycleForPlanCycle = false;
//
//	for ( int i = 0; i < m_CycleData.nCount; i++ )
//	{
//		StPlaceData& pd = m_CycleData.placeData[ i];
//		
//		// Skip failed placements
//		// Skip flux steps that have already been claimed.
//		// Skip flux steps that don't require flux.
//		if ( pd.status != ePLACE_OK)								continue;
//		if ( pd.pStep[eSTEP_PRE_FLUX])								continue;
//		if ( pd.profile.common.fluxSeq == eFLUX_NO_USE)				continue;
//		if ( pd.profile.common.fluxSeq != eFLUX_BEFORE_VISION)		continue;
//		
//		// Skip if step is virtual pick step.
//		if ( pd.profile.IsVirtualPick() )							continue;
//
//		bCycleForPlanCycle = true;
//	}
//
//	return bCycleForPlanCycle;
//}


// Cycle preparation for specific steps
// such as "Pre open tapefeeder shutter befor pick operation is about to execute.
// So, this function does not do any harm on the member variable as m_CycleData.
eCycleStatus
CSeqCycle::PostExecutePrepareNextCycle( )
{
	eCycleStatus status;

	// Prepare next cycle needed for StepPick
	status = CStepPick::PostExecutePrepareNextCycle(m_CycleData);

	// Prepare next cycle needed for other steps
	// .....

	return status;
}

eCycleStatus
CSeqCycle::MarkHeadPlacedPart( StCycleData& arCycleData, CHead* apHead )
{
	eCycleStatus status = eCYCLE_OK;

	if ( arCycleData.nextCycleData.IsEmpty() )
		return eCYCLE_OK;

	for ( int i=0; i < arCycleData.nextCycleData.GetCount(); i++ )
	{
		StCycleData::StNextCycleFeederOpenData& cD = arCycleData.nextCycleData[i];
		SYS_ASSERT( cD.feederId.IsDefined() );

		// NOTE :
		// Check if previous heads which is planed to use this feeder finish place step.
		if ( cD.pickupHdList.IsEmpty() )
			return eCYCLE_OK;
		else
		{
			CFixedArrayList<HeadID, SYS_MAX_HEAD_PER_GANTRY> toDopickHdList;
			toDopickHdList.RemoveAll();

			for ( int j=0; j < cD.pickupHdList.GetCount(); j++ )
			{
				HeadID hdId = cD.pickupHdList[j];
				CHead* pHead = ::GetHead( hdId );
				SYS_ASSERT( pHead );

				if ( apHead->Unit() == hdId )
					continue;

				toDopickHdList.Append(hdId);
			}

			cD.pickupHdList.RemoveAll();
			cD.pickupHdList.Append(toDopickHdList);
		}
	}
	return status;
}

eCycleStatus
CSeqCycle::PostCycle( )
{
	eCycleStatus status;

	// Unconditionally PostExecute each step...
	for (int i=0; i<m_nSteps; i++)
	{
		status = m_Steps[i]->PostCycleExecute( m_CycleData);
		m_ReturnStatus = (status > m_ReturnStatus ? status : m_ReturnStatus);
	}
	return m_ReturnStatus;
}


void
CSeqCycle::DetectExceededRetry( )
{
	CFeederId			feederId;
	StStatisticsCycle	statErr;
	bool				bErrorReport = true;

	// FOR EACH PLACEMENT...
	// DETECT EXCEEDED RETRY and FAIL the FEEDER, FAIL the PLACEMENT, and REPORT the ERROR
	for (int i=0; i<m_CycleData.nCount; i++)
	{
		StPlaceData& pd = m_CycleData.placeData[i];

		if ( feederId != pd.pickedFeederID)
			bErrorReport = true;

		feederId = pd.pickedFeederID;

		if ( !feederId.IsDefined())		continue;

		eFeederType	feedType = feederId.Type();
		if (feedType == eFEEDTYPE_STICK)
		{
			CStickFeeder* pStickFeeder = ::GetStickFeeder(feederId);
			if ( pStickFeeder && pStickFeeder->IsStickerTapeFeeder())
			{
				if (!pStickFeeder->GetIsNeedToReportRetryExceeded())
				{
					return;
				}
			}
		}

		// Check if the feeder has failed due to excessive retries.
		eStatus status = CheckFeederRetryExceeded( feederId, pd.profile.common.retryCount, pd.profile.common.retryNgCount, &statErr, pd.profile.common.height, true);
		if ( status == eNG)
		{
			ePlacementStatus newPlaceStatus;

			// Report the failed condition and downgrade the placement status if necessary.
			if ( ReportFeederRetryExceeded( pd, newPlaceStatus, feederId, bErrorReport))
				pd.status = newPlaceStatus;

			// Remove the Index for the Pocket Teaching again.
			if ( feederId.Type() == eFEEDTYPE_TAPE && g_FacDebug.IsRetryOutPocketTeachStatusReset())
				CStepPocketXyStd::StaticSetPocketTeachStatus( feederId, eNG);

			if ( feederId.Type() == eFEEDTYPE_TAPE)
			{
				// Non-electric tapefeeders should consider a retry-exceeded as a possible splice.
				if ( !CFeederbase::IsElectric())
					CTapeFeeder::SetLcrCheckStatus( feederId, eNG);
			}
		}

		if (feederId.Type() == eFEEDTYPE_TAPE)
		{
			int idxSt = ::GetConveyorObj()->GetWorkStationArrayIndx(m_CycleData.stationId);
			StStatisticsCycle*	pStatCycle = &g_Statistics.statTapeFeeder[idxSt][feederId.Unit() - 1][feederId.No() - 1];
			long accumulatePickCnt = pStatCycle->pickFlyLsoCount + pStatCycle->pickUpwardCount;
			long periodPocketTeach = pd.profile.common.periodicPocketTeachCycle;
			CTapeFeeder* pTape = ::GetTapeFeeder(feederId);

			// Pocket Teach when occurred Pick Miss. //////
			bool bIsSmallPart = false;
			double smallPartSize = g_FacConst.GetSharedMapPartSize();
			if (pd.profile.common.bound.x < smallPartSize || pd.profile.common.bound.y < smallPartSize)
				bIsSmallPart = true;

			if (pTape && pd.profile.IsEnableAutoPocketTeach() && !pTape->IsEmbossReel() && bIsSmallPart && g_FacDebug.IsPocketTeachWhen1Miss() && GetMispickTape(feederId) > 0)
				pTape->SetPocketTeachStatus(ePOCKET_TEACH_RETEACH);
			////////////////////////////////////////////////

			if (periodPocketTeach > 0)
			{
				if (pTape)
				{
					PartID				partID = pTape->GetPartID();
					const StPart&		rPart = g_pTable->GetPart(partID);
					const CProfile&		rProfile = g_pTable->GetProfile(rPart.profileID);
					ePartType			partType = rProfile.GetPartType();
					eTapeFeederType		feederType = pTape->GetTapeFeederType();

//					if ((partType == ePARTTYPE_CHIPRECT_C0402 || partType == ePARTTYPE_CHIPRECT_R0402
//						|| partType == ePARTTYPE_CHIPRECT_C03015 || partType == ePARTTYPE_CHIPRECT_R03015
//						|| partType == ePARTTYPE_CHIPRECT_C0201 || partType == ePARTTYPE_CHIPRECT_R0201) && rProfile.IsEnableAutoPocketTeach())

					if ((eDEVICETYPE_TAPEELEC_8MM_V == feederType 
						|| eDEVICETYPE_TAPEELEC_8MM_0402_V == feederType
						|| eDEVICETYPE_TAPEELEC_VE8M_W4P1_V == feederType
						|| eDEVICETYPE_TAPEELEC_VE8M_W8P1_V == feederType
						|| eDEVICETYPE_TAPEELEC_AUTO_8MM_V == feederType
						|| eDEVICETYPE_TAPEELEC_8MM_HM == feederType
						|| eDEVICETYPE_TAPEELEC_8MM_W4P1_HM == feederType
						|| eDEVICETYPE_TAPEELEC_8MM_0402_HM == feederType
						|| eDEVICETYPE_TAPEELEC_8MM_LED_HM == feederType
						|| eDEVICETYPE_TAPEELEC_8MM_B_HM == feederType)
						&& rProfile.IsEnableAutoPocketTeach())
					{
						long lastResetCnt = pTape->GetLastPocketTeachResetcnt();
						{
							long diff = accumulatePickCnt - lastResetCnt;
							if ((diff) >= periodPocketTeach)
							{
								pTape->SetPocketTeachStatus(ePOCKET_TEACH_RETEACH);
								pTape->SetLastPocketTeachResetCnt(accumulatePickCnt);
							}
							else if ( diff < 0)
							{
								// Something Happened like JobChange or Production Data Clear.
								pTape->SetPocketTeachStatus(ePOCKET_TEACH_RETEACH);
								pTape->SetLastPocketTeachResetCnt(accumulatePickCnt);
							}
						}
					}
				}
			}
		}
	}
}

eCycleStatus
CSeqCycle::PrefetchRadialAxialFeeder( eStepType aNexyStepType, long lLine)
{
	eFrontRear   eFR = m_pGantry->GetFrontOrRear( );
	if ( g_StateReport.IsManualCmdActive( eFR))		return eCYCLE_OK;

	if ( !g_StepPickTapePmf[ m_pGantry->Unit()].IsInstalled( ))
		return eCYCLE_OK;

	eRadialFeederSpeed	radialFeederSpeed = g_SysConst.GetRadialFeederSpeed( );

	bool bDoPrefetch = false;

	if ( aNexyStepType == eSTEP_PLACE && radialFeederSpeed == eRADIAL_FEEDER_SPEED_FAST)
	{
		bDoPrefetch = true;
	}
	else if ( aNexyStepType == eSTEP_ALIGN && radialFeederSpeed == eRADIAL_FEEDER_SPEED_FASTEST)
	{
		bDoPrefetch = true;
	}

	if ( !bDoPrefetch)
	{
		return eCYCLE_OK;
	}

	CTelemetryPair tp( EV_CLASSFEEDER, 0, "__PrefetchRadialAxialFeeder__[ l=%d]__", lLine);

	eLookCycles		lookCycle;
	CFeederId		feederId;
	TPlaceWsList	placeList;
	StPlaceWs*		pPlaceWs = NULL;
	GantryID		gantryID = m_pGantry->Unit( );

	CSeqEngine&		seqEngine = g_SeqEngine[ gantryID];

	if ( m_PrefetchOp != eCYCLE_PREFETCH_NONE  &&  m_PrefetchOp < eCYCLE_PREFETCH_PICKUP)
		lookCycle = eLOOK_INCLUDE_CURRENT_CYCLE;
	else
		lookCycle = eLOOK_EXCLUDE_CURRENT_CYCLE;
	
	seqEngine.GetLookAheadCycles( placeList, lookCycle);

	long			cycle = 0;
	long			firstCycle = 0;
	int				i = 0;
	CHead*			pHead = NULL;

	CFixedMap< CFeederId, long, SYS_MAX_STEP_PER_CYCLE>	prefechedFeederMap;

	for ( i = 0; i < placeList.GetCount( ); i++)
	{
		const StPlcWsInfo&	placeWsInfo = placeList[ i];
		pPlaceWs = placeWsInfo.m_pPlaceWs;
		feederId = pPlaceWs->feederID;
		cycle = pPlaceWs->cycle;

		pHead = ::GetHead( pPlaceWs->head);

		// Detect the number of the first cycle.
		if ( firstCycle == 0)
			firstCycle = cycle;

		if ( cycle > firstCycle) //딱 다음 싸이클만 체크한다.
			break;

		if ( !feederId.IsDefined( ))
		{
			if ( !pHead)
				continue;
			
			TPcbID		pcbID = ( m_CycleData.pPcbBoard ? m_CycleData.pPcbBoard->GetPcbID( ) : g_pPcbSet->PreferPcbId( m_CycleData.stationId.Lane( )));
			CPcbFunc*	pPcbFunc = g_pPcbs[ pcbID];

			if ( !pPcbFunc)
				continue;

			const StPlaceCad& pcbPlaceCad = pPcbFunc->GetPlaceCad( pPlaceWs->placeCadID);

			StPlaceData placeData;

			placeData.status = ePLACE_OK;
			placeData.bStoppedByDumpStop = false;
			placeData.placeCadID = pPlaceWs->placeCadID;
			placeData.cycle = cycle;
			placeData.feederID = pPlaceWs->feederID;
			placeData.pickedFeederID = CFeederId( 0);
			placeData.placedFeederID = CFeederId( 0);

			placeData.bAutoFeeder = true;
			placeData.head = pHead->GetHeadID( );

			placeData.pHead = pHead;

			placeData.position = pcbPlaceCad.position;
			placeData.arrayId.arrayNo = 0;//aPlaceDataArray[i].m_ArrayNo;
			placeData.arrayId.blockNo = 0;//pcbPlaceCad.blockNo;
			placeData.placeFidID = 0;//pcbPlaceCad.placeFidID;

			const StPart& pcbPart = g_pTable->GetPart( pcbPlaceCad.partID);

			placeData.partID = pcbPlaceCad.partID;

			bool							bFeederFound;
			CStepPick::MapFdrIDtoN			usedFeederMap;	// Always empty for this operation, because we don't have access to the results of the current cycle

			eMachErrorCode  eErrorCode = eMEC_NOT_AN_ERROR;
			g_StepPickTapePmf[ gantryID].AssignAutomaticFeeder( placeData);
			feederId = g_StepPickTapePmf[ gantryID].FindFeeder( placeData.feederID, usedFeederMap, *pHead, 0, 0, placeData.bAutoFeeder, bFeederFound, eErrorCode);

			if ( !feederId.IsDefined( ))
				continue;
		}

		if ( feederId.Type( ) != eFEEDTYPE_TAPE)
			continue;

		CTapeFeeder* pFeeder = ::GetTapeFeeder( feederId);
		if ( pFeeder == NULL)
			continue;

		if ( !pFeeder->IsTypeAutoInsert( ))
			continue;

		if ( prefechedFeederMap.IsExist( pFeeder->GetID( )))	continue;

		prefechedFeederMap[ pFeeder->GetID( )] = 1;

		Telemetry( EV_CLASSFEEDER, pFeeder->Unit( ), "  __PrefetchRadialAxialFeeder [%s]", pFeeder->GetName( ));
		pFeeder->PresentPart( false);
	}

	return eCYCLE_OK;
}


eCycleStatus
CSeqCycle::TrayFeederLookAheadCheck( )
{
	// If manual command OR trays are not required... SKIP OUT
	if ( g_StateReport.IsManualCmdActive( m_frRunningLane))		return eCYCLE_OK;
	if ( !CPCBoard::IsMultiTrayNeeded())					return eCYCLE_OK;

	// Looks some cycles ahead of the current cycle about to be executed, looking for an opportunity
	// to prefetch a tray pallet not used in this cycle but used in a future cycle AND to plan 
	// prefetching of pallets after picking and after aligning depending on system constant settings.
	//
	bool			bPalletNow = false;
	int				i;
	eStatus			status;
	eLookCycles		lookCycle;
	CFeederId		feederId;
//	TPlaceWsList	placeList;
	TPlaceWsList	trayPlaceList;

	TTrayFeederWait	tfWaitList;
	CSeqEngine&		seqEngine = g_SeqEngine[ m_pGantry->Unit()];
	long			unit = m_pGantry->Unit();
	
	// UPDATE TRAY REFILLED STATUS if NOT YET done in this cycle 
	if ( !IsTrayEmptyStatusUpdated())
	{
		SetTrayEmptyStatusUpdated();

		bool bFastCheck = true;

		for (int trayUnit=1; trayUnit<_countof(g_TrayFeeder); trayUnit++)
		{
			CTrayFeeder* pTrayFeeder = &g_TrayFeeder[ trayUnit];
			
			if ( pTrayFeeder == NULL)							continue;
			if ( !pTrayFeeder->IsInstalled() )					continue;
			
			// cic, On this path, the condition "pTrayFeeder" cannot be false.
			eFrontRear   trayFrontRear   = pTrayFeeder->GetFrontOrRear();
			eFrontRear   frontRear       = m_pGantry->GetFrontOrRear();

			SectionID	 sectionId       = m_pGantry->GetSectionID();
			SectionID	 traySectionId   = pTrayFeeder->GetSectionID();

			if (( trayFrontRear == frontRear || trayFrontRear == eBOTH || !m_pGantry->GetSharingGantry() ) 
				&& sectionId == traySectionId )
			{
				status = CTrayFeeder::UpdateRefilledPallets( eTRAY_NOWAIT, tfWaitList, bFastCheck, trayUnit);
				SYS_ASSERT( status != eNG);
				if ( status != eOK)
					return eCYCLE_ABORT;
			}
		}	
	}

//	CStepPickTrayFeeder*	pStepPickTFCur = GetTrayFeederStep();

	// IF this is a prefetch cycle AND parts will NOT BE prefetched (and not prefetch planned)
	// THEN include the current cycle in the LOOKAHEAD logic to make sure that the current
	// cycle placements get top priority in the prefetch of pallets.
//	if ( m_PrefetchOp != eCYCLE_PREFETCH_NONE  &&  m_PrefetchOp < eCYCLE_PREFETCH_PICKUP)
//		lookCycle = eLOOK_INCLUDE_CURRENT_CYCLE;
//	else
//		lookCycle = eLOOK_EXCLUDE_CURRENT_CYCLE;
	

	// **Warning**  Always, Include current Cycle.
	lookCycle = eLOOK_INCLUDE_CURRENT_CYCLE;
	// Get advanced placements (ahead of the current cycle, and including the
	// current cycle if a no pick prefetch is being done)
	seqEngine.GetLookAheadTrayCycles( trayPlaceList, lookCycle);

	// Return if the look-ahead has no tray feeders
	if ( !IsMultiTrayFeederUsed( trayPlaceList))
	{
		for (i=0; i<_countof(m_TrayFeederNxtPallet); i++)	m_TrayFeederNxtPallet[i] = 0;
		return eCYCLE_OK;
	}
	
	if ( m_LastStepType == eSTEP_PICK) 			// When cycle is over, TrayFeederLookAheadCheck() function only execute one time.
	{											// But, When you check "Prefetch Parts Option, above function execute one more time. 
		Telemetry( EV_CLASSFEEDER, unit, "TF : TrayFeederLookAheadCheck- PICK STATUS, PLAN SKIP"); 
		return eCYCLE_OK;						
	}
	TTrayFeederPallet	trayFeederCurPallet;	// RUN	Tray Feeder look-ahead analysis results

	// List of tray feeder pallet "first" usage
	for (i=0; i<_countof(trayFeederCurPallet); i++)		trayFeederCurPallet[i] = 0;
	for (i=0; i<_countof(m_TrayFeederNowPallet); i++)	m_TrayFeederNowPallet[i] = 0;
	for (i=0; i<_countof(m_TrayFeederNxtPallet); i++)	m_TrayFeederNxtPallet[i] = 0;
	// Assume to buffer change at the end of the cycle
	for (i=0; i<_countof(m_TrayFeederBufferChg); i++)	m_TrayFeederBufferChg[i] = eTRAY_BUFCHG_YES;

	// Given the future placements and the current tray feeder pallet usage,
	// determine 1) trayfeeder pallets to be prefetched NOW because the tray feeders are NOT USED in this cycle
	// determine 2) trayfeeder pallets to be prefetched during this cycle after current pallet parts are picked/aligned.
	// determine 3) 1-slot tray feeders that should be buffer changed at the end of this cycle (because prefetching will not be done)
	if ( !PlanCurrentTrayFeederPrefetch( trayPlaceList, tfWaitList, trayFeederCurPallet, m_TrayFeederNowPallet, m_TrayFeederNxtPallet, m_TrayFeederBufferChg))
		return eCYCLE_OK;

	// Prefetch pallets for tray feeders not used in this cycle
	return TrayFeederPreCyclePrefetch( m_TrayFeederNowPallet);
}


bool
CSeqCycle::IsMultiTrayFeederUsed( const TPlaceWsList& aPlaceList)
{
	bool			bMultiPcb = g_pPcbSet->IsMultiPcb(m_CycleData.stationId.Lane());
	int				i;
	eFeederType		feederType;
	eTrayFeederType	trayFeederType;
	CFeederId		feederId;
	CTrayFeeder*	pTrayFeeder;
	StPlaceWs*		pPlaceWs;

	// Skip, if multiple board programs exist but one cannot be identified.
	if ( m_CycleData.pPcbBoard==NULL  &&  bMultiPcb)
		return false;

	// Return true if there are any tray feeders specified in the placement list.
	for (i=0; i<aPlaceList.GetCount(); i++)
	{
		const StPlcWsInfo&	placeWsInfo = aPlaceList[i];
		pPlaceWs	= placeWsInfo.m_pPlaceWs;
		feederId	= pPlaceWs->feederID;
		feederType	= feederId.Type();

		// For automatic feeders, just get any tray with a matching part ID
		if ( !feederId.IsDefined())
		{
			TPcbID		pcbID	 = (m_CycleData.pPcbBoard ? m_CycleData.pPcbBoard->GetPcbID() : g_pPcbSet->PreferPcbId( m_CycleData.stationId.Lane()) );
			CPcbFunc*	pPcbFunc = g_pPcbs[ pcbID];

			const StPlaceCad& pcbPlaceCad = pPcbFunc->GetPlaceCad( pPlaceWs->placeCadID );
			feederId = CStepPickTrayFeeder::GetAutomaticTray( m_SectionID, pcbPlaceCad.partID, m_frRunningLane );
		}

		feederType	= feederId.Type();
		if ( feederType != eFEEDTYPE_TRAY)					continue;

		pTrayFeeder = ::GetTrayFeeder( feederId);
		if ( pTrayFeeder == NULL)							continue;

		trayFeederType = pTrayFeeder->GetTrayFeederType();

		if ( trayFeederType == eDEVICETYPE_TRAY_SINGLE)		continue;
		if ( trayFeederType == eDEVICETYPE_TRAY_STACK)		continue;
		if ( trayFeederType == eDEVICETYPE_TRAY_DDF)		continue;
		if ( pTrayFeeder->GetMultiTFType() == eMULTI_TF_STACK)			continue;

		// Break if a multi-tray is found
		break;
	}

	// Return true (tray feeder is used) if a tray feeder was specified in the place list
	return (i != aPlaceList.GetCount());
}

// Move to System Constant(83) -> ONE_SLOT_BUFFER_CHANGE_CYCLES 
//#define ONE_SLOT_BUFFER_CHANGE_CYCLES	4	// The number of look ahead cycles with no required
											//		pallets before a Buffer Change can be sent

// Plan Only Current Cycle before Cycle Start.
bool
CSeqCycle::PlanCurrentTrayFeederPrefetch( const TPlaceWsList&	aPlaceList,
									const TTrayFeederWait&	aTFWaitList,// not used
									TTrayFeederPallet&	aTrayFeederCurPallet,
									TTrayFeederPallet&	aTrayFeederNowPallet,
									TTrayFeederPallet&	aTrayFeederNxtPallet,//not used
									TTrayFeederBufChg&	aTrayFeederBufferChg)
{
	// Given the future placements and the current tray feeder pallet usage,
	// determine 1) trayfeeder pallets to be prefetched NOW because the tray feeders are NOT USED in this cycle
	// determine 2) trayfeeder pallets to be prefetched during this cycle after current pallet parts are picked/aligned.
	// determine 3) 1-slot tray feeders that should be buffer changed at the end of this cycle (because prefetching will not be done)
	bool			bMultiPcb = g_pPcbSet->IsMultiPcb(m_CycleData.stationId.Lane());
	bool			bPalletNow = false;
	int				i;
	long			cycle;
	long			firstCycle = 0;
	long			lastCycle = 0;
	long			nCyclesAhead;
	long			trayUnit;
	long			pallet;
	long			outPallet;
	long			bufferChangeCycle = 0;
	eFeederType		feederType;
	CFeederId		feederId;
	StPlaceWs*		pPlaceWs;
	CHead*					pHead;
	CStepPickTrayFeeder*	pStepPickTFNxt;
	
	long			headFirstCycle = 1;
	long			nTrayCyclesAhead;
	bool			bOutOfRange = false;

	// Skip, if multiple board programs exist but one cannot be identified.
	if ( m_CycleData.pPcbBoard==NULL  &&  bMultiPcb)
		return false;

	m_bFirstCycleHalfPalletOut = true;


	bufferChangeCycle = g_SysConst.GetOneSlotBufferChangeCycles();

//	if ( 1 > bufferChangeCycle || bufferChangeCycle >4)
	if ( 1 > bufferChangeCycle)
		bufferChangeCycle = 4;

	// Look for the first required pallet for each tray feeder unit.
	// If the tray feeder unit does NOT appear in the current cycle, then this constitutes 
	//		an IMMEDIATE look-ahead operation.
	// If the tray feeder unit does appear in the current cycle, then this data constitutes 
	//		look-ahead operations that can be executed either after StepPick, or after 
	//		StepAlign executes depending on current optimization settings.
	// NOTE: This algorithem DOES NOT account for picks in the current cycle causing trays to go empty.

	for (i=0; i<aPlaceList.GetCount(); i++)
	{
		const StPlcWsInfo&	placeWsInfo = aPlaceList[i];
		pPlaceWs	= placeWsInfo.m_pPlaceWs;
		feederId	= pPlaceWs->feederID;
		cycle		= pPlaceWs->cycle;

		// Detect the number of the first cycle.
		if ( firstCycle == 0)
			firstCycle = cycle;

		// Keep track of the last cycle because this look ahead wraps around.
		if ( cycle > firstCycle)
			lastCycle = cycle;

		// Adjust the cycle number to extend past the end of the board program (wrap around).
		if ( cycle < lastCycle)
			cycle = lastCycle + cycle;

		// Compute the number cycle look ahead cycles this iteration represents.
		nCyclesAhead = cycle - firstCycle + 1;

		// For automatic feeders, just get any tray with a matching part ID.
		if ( !feederId.IsDefined())
		{
			TPcbID		pcbID	 = (m_CycleData.pPcbBoard ? m_CycleData.pPcbBoard->GetPcbID() 
															: g_pPcbSet->PreferPcbId( m_CycleData.stationId.Lane()) );
			CPcbFunc*	pPcbFunc = g_pPcbs[ pcbID];

			const StPlaceCad& pcbPlaceCad = pPcbFunc->GetPlaceCad( pPlaceWs->placeCadID );
			feederId = CStepPickTrayFeeder::GetAutomaticTray( m_SectionID, pcbPlaceCad.partID, m_frRunningLane);
		}

		feederType	= feederId.Type();
		pHead		= ::GetHead( pPlaceWs->head);

		if ( feederType != eFEEDTYPE_TRAY)		continue;
		
		CTrayFeeder* pTrayFeeder = ::GetTrayFeeder( feederId);
		if ( pTrayFeeder == NULL)				continue;
		
		
		bool							bAutoFeeder = true;
		bool							bFeederFound;
		bool							bEmptyPallet;
		bool							bUpperDisable;
		bool							bLowerDisable;
		bool							bUnclampDockingTray;
		CStepPick::MapFdrIDtoN				usedFeederMap;	// Always empty for this operation, because we don't have access to the results of the current cycle
		CStepPickTrayFeeder::TListTrayRec	trays;

		pStepPickTFNxt = ::GetStepPickTrayFeeder( m_GantryID, feederId.Unit());

		feederId = pStepPickTFNxt->FindFeeder( feederId, usedFeederMap, *pHead, 0,
												bAutoFeeder, trays, bFeederFound,
												bEmptyPallet, bUpperDisable, bLowerDisable, bOutOfRange, bUnclampDockingTray);

		feederType	= feederId.Type();
		trayUnit	= feederId.Unit();
		pallet		= feederId.No();

		eTrayFeederType	trayFeederType = pTrayFeeder->GetTrayFeederType();

		// First pickup part in cycle will use 2nd pallet out.
		if ( i == 0 && trayFeederType == eDEVICETYPE_TRAY_100VS)
			m_b2ndStepPalletOut = bOutOfRange;
		
		if ( ( trayFeederType != eDEVICETYPE_TRAY_SINGLE)	
			&& ( trayFeederType != eDEVICETYPE_TRAY_STACK)	
			&& ( trayFeederType != eDEVICETYPE_TRAY_DDF)
			&& ( pTrayFeeder->GetMultiTFType() != eMULTI_TF_STACK) )
		{
			if( bOutOfRange && trayFeederType == eDEVICETYPE_TRAY_100VS)
				outPallet = pTrayFeeder->Get2ndStepPalletOutNowNo();	
			else
				outPallet = pTrayFeeder->GetPalletOutNowNo();

			aTrayFeederCurPallet[trayUnit] = outPallet;
			
			if ( pallet == 0)
				return false;
			if ( pTrayFeeder->GetFrontOrRear() == eBOTH && m_PrefetchOp == eCYCLE_PREFETCH_NOZZLE)
					return false;
		}

		if ( feederType != eFEEDTYPE_TRAY)	continue;

		CStepPickTrayFeeder*	pStepPickTFCur = GetTrayFeederStep();
		// Get the pallet presentation state at the end of the current cycle
		// NOTE: StepPick may not exist if this is a prefetch cycle with no picking.
		// NOTE: StepPick may not exist if this is a cycle that was preceded by a prefetch with picking.

		if ( pStepPickTFCur)
		{
			pStepPickTFCur->GetLastTrayFeederPallet( aTrayFeederCurPallet);
		}
		
		// Don't PLAN if the tray feeder is currently loading a pallet.
		// i.e. A FAST status update was not possible so tray feeder status was not updated.
		//if ( aTFWaitList[ trayUnit] == eTRAY_NOWAIT)	continue;
		
		// If this tray unit is not being used in the current cycle, then this is an
		//	opportunity for IMMEDIATE look-ahead action.  Otherwise, it's an opportunity
		//	for post-pick look-ahead action if the pallet is not already presented.
		if ( aTrayFeederCurPallet[ trayUnit] == 0)
		{
			if ( aTrayFeederNowPallet[ trayUnit] == 0)
			{
				aTrayFeederNowPallet[ trayUnit] = pallet;
				bPalletNow = true;
				nTrayCyclesAhead = nCyclesAhead;

				// Prefetching for future cycles prevents buffer change at the end of this cycle
				// UNLESS the look ahead is far enough that a buffer change could be transparent
				if ( nCyclesAhead < bufferChangeCycle)
					aTrayFeederBufferChg[ trayUnit] = eTRAY_BUFCHG_NO;
				else
					aTrayFeederBufferChg[ trayUnit] = eTRAY_BUFCHG_YES;

				TPcbID pcbID = m_CycleData.pPcbBoard ? m_CycleData.pPcbBoard->GetPcbID() : g_pPcbSet->PreferPcbId(m_CycleData.stationId.Lane());
				CPcbFunc* pPcbFunc = g_pPcbs[pcbID];
				SYS_ASSERT(pPcbFunc);

				bool bLastTrayCycle = false;
				if ( pPcbFunc->bExtendedArray)
				{
					if ( m_CycleData.cycle >= pPlaceWs->cycle)
						bLastTrayCycle = true;
				}
				else
				{
					const StPlaceCad& pcbPlaceCad = pPcbFunc->GetPlaceCad(pPlaceWs->placeCadID);

					int blockCount = pPcbFunc->GetBlockCount() - 1;
					int blockArrayCount = pPcbFunc->GetArrayCnt(pcbPlaceCad.blockNo) - 1;

					if ( (m_CycleData.placeData->arrayId.blockNo == blockCount)
						&& (m_CycleData.placeData->arrayId.arrayNo == blockArrayCount)
						&& (m_CycleData.cycle >= pPlaceWs->cycle))
					{
						bLastTrayCycle = true;
					}
				}

				//CSeqEngine&	seqEngine = g_SeqEngine[m_pGantry->Unit()];

				//if(m_CycleData.cycle >= pPlaceWs->cycle)
				//if( seqEngine.IsLastCycle() )
				if ( bLastTrayCycle)
				{
					aTrayFeederBufferChg[ trayUnit] = eTRAY_BUFCHG_NO;
					m_bLastTrayCycle[ trayUnit] = true;
				}
				else
					m_bLastTrayCycle[ trayUnit] = false;
			}
		}
	}

	if ( bPalletNow)
	{
		if ( (headFirstCycle != nTrayCyclesAhead) && (nTrayCyclesAhead < bufferChangeCycle) )
			m_bFirstCycleHalfPalletOut = true;
		else 
			m_bFirstCycleHalfPalletOut = false;
	}

	return bPalletNow;
}

// Plan Only Next Pallet.
bool
CSeqCycle::PlanTrayFeederNextPallet( const TPlaceWsList&	aPlaceList,
								  const TTrayFeederWait&	aTFWaitList,
								  	  TTrayFeederPallet&	aTrayFeederNxtPallet,
									  TTrayFeederBufChg&	aTrayFeederBufferChg)
{
	
	bool			bMultiPcb = g_pPcbSet->IsMultiPcb(m_CycleData.stationId.Lane());
	int				i;
	long			cycle;
	long			firstCycle = 0;
	long			lastCycle = 0;
	long			lastTrayCycle = 0;
	long			nCyclesAhead;
	long			trayUnit;
	long			pallet;
	long			bufferChangeCycle = 0;
	long			aHeadOneCycle = 1;
	eFeederType		feederType;
	CFeederId		feederId;

	StPlaceWs*				pPlaceWs;
	CHead*					pHead;
	CStepPickTrayFeeder*	pStepPickTFNxt;

	// Skip, if multiple board programs exist but one cannot be identified.
	if ( m_CycleData.pPcbBoard==NULL  &&  bMultiPcb)
		return false;

	m_bPostCycleHalfPalletOut = true;
	for (i = 0; i<_countof(m_bLastTrayCycle); i++)	m_bLastTrayCycle[i] = false;
	
	bufferChangeCycle = g_SysConst.GetOneSlotBufferChangeCycles();

//	if ( 1 > bufferChangeCycle || bufferChangeCycle >4)
	if ( 1 > bufferChangeCycle)
		bufferChangeCycle = 4;
	
	// Look for the first required pallet for each tray feeder unit.
	// If the tray feeder unit does NOT appear in the current cycle, then this constitutes 
	//		an IMMEDIATE look-ahead operation.
	// If the tray feeder unit does appear in the current cycle, then this data constitutes 
	//		look-ahead operations that can be executed either after StepPick, or after 
	//		StepAlign executes depending on current optimization settings.
	// NOTE: This algorithem DOES NOT account for picks in the current cycle causing trays to go empty.

 	for (i=0; i<aPlaceList.GetCount(); i++)
	{
		const StPlcWsInfo&	placeWsInfo = aPlaceList[i];
		pPlaceWs	= placeWsInfo.m_pPlaceWs;
		feederId	= pPlaceWs->feederID;
		cycle		= pPlaceWs->cycle;


		if (feederId.Type() != eFEEDTYPE_TRAY)
			continue;

		if (cycle > lastTrayCycle)
			lastTrayCycle = cycle;
	}

	for (i = 0; i < aPlaceList.GetCount(); i++)
	{
		const StPlcWsInfo&	placeWsInfo = aPlaceList[i];
		pPlaceWs = placeWsInfo.m_pPlaceWs;
		feederId = pPlaceWs->feederID;
		cycle = pPlaceWs->cycle;
		// Detect the number of the first cycle.
		if ( firstCycle == 0)
			firstCycle = cycle;

		// Keep track of the last cycle because this look ahead wraps around.
		if ( cycle > firstCycle)
			lastCycle = cycle;

		// Adjust the cycle number to extend past the end of the board program (wrap around).
		if ( cycle < lastCycle)
			cycle = lastCycle + cycle;

		// Compute the number cycle look ahead cycles this iteration represents.
		nCyclesAhead = cycle - firstCycle + 1;

		// For automatic feeders, just get any tray with a matching part ID.
		if ( !feederId.IsDefined())
		{
			TPcbID		pcbID	 = (m_CycleData.pPcbBoard ? m_CycleData.pPcbBoard->GetPcbID() 
															: g_pPcbSet->PreferPcbId( m_CycleData.stationId.Lane()) );
			CPcbFunc*	pPcbFunc = g_pPcbs[ pcbID];

			const StPlaceCad& pcbPlaceCad = pPcbFunc->GetPlaceCad( pPlaceWs->placeCadID );
			pHead = ::GetHead(pPlaceWs->head);
			CGantry* pGantry = pHead->Gantry();
			eFrontRear frontRear = pGantry->GetFrontOrRear();
			
			// If Single Gantry, change Both side
			if (!pGantry->GetSharingGantry())
				frontRear = eBOTH;
				
			feederId = CStepPickTrayFeeder::GetAutomaticTray( m_SectionID, pcbPlaceCad.partID, frontRear);
		}

		feederType	= feederId.Type();
		pHead		= ::GetHead( pPlaceWs->head);

		if ( feederType != eFEEDTYPE_TRAY)		continue;
		
		CTrayFeeder* pTrayFeeder = ::GetTrayFeeder( feederId);
		if ( pTrayFeeder == NULL)				continue;
				
		bool							bAutoFeeder = true;
		bool							bFeederFound;
		bool							bEmptyPallet;
		bool							bUpperDisable;
		bool							bLowerDisable;
		bool							bOutOfRange = false;
		bool							bUnclampDockingTray;
CStepPick::MapFdrIDtoN					usedFeederMap;	// Always empty for this operation, because we don't have access to the results of the current cycle
	CStepPickTrayFeeder::TListTrayRec	trays;

		pStepPickTFNxt = ::GetStepPickTrayFeeder( m_GantryID, feederId.Unit());

		feederId = pStepPickTFNxt->FindFeeder( feederId, usedFeederMap, *pHead, 0,
												bAutoFeeder, trays, bFeederFound,
												bEmptyPallet, bUpperDisable, bLowerDisable, bOutOfRange, bUnclampDockingTray);

		feederType	= feederId.Type();
		trayUnit	= feederId.Unit();
		pallet		= feederId.No();

		// Check Single Tray
		eTrayFeederType	trayFeederType = pTrayFeeder->GetTrayFeederType();

		if ( ( trayFeederType != eDEVICETYPE_TRAY_SINGLE)	
		&& ( trayFeederType != eDEVICETYPE_TRAY_STACK)	
		&& ( trayFeederType != eDEVICETYPE_TRAY_DDF)
		&& ( pTrayFeeder->GetMultiTFType() != eMULTI_TF_STACK) )
		{
			if ( pallet == 0)
			{
				Telemetry( EV_CLASSFEEDER, trayUnit, "TF : PlanTrayFeederNextPallet- Find Pallet is none = %d", (long)feederId);
				return false;
			}
		}

		if ( feederType != eFEEDTYPE_TRAY)	continue;

//		CStepPickTrayFeeder*	pStepPickTFCur = GetTrayFeederStep();
		
		CTray*			pTray		= ::GetTray( feederId);
		long count = 1;
		if ( usedFeederMap.Lookup( feederId, count))
		{
			count = usedFeederMap.GetValueAt( feederId );
			count += 1;
		}

		Cxy pocket = pTray->GetNextPickN( count);
		if ( pTray->IsPocketEmpty( pocket))				continue;
		
		if ( aTrayFeederNxtPallet[ trayUnit] == 0)
		{
			aTrayFeederNxtPallet[ trayUnit] = pallet;
			Telemetry( EV_CLASSFEEDER, trayUnit, "TF : PlanTrayFeederNextPallet- Next Pallet Out Number = %d", pallet);
			// Prefetching for future cycles prevents buffer change at the end of this cycle
			// UNLESS the look ahead is far enough that a buffer change could be transparent
			if ( nCyclesAhead < bufferChangeCycle)
				aTrayFeederBufferChg[ trayUnit] = eTRAY_BUFCHG_NO;
			else
				aTrayFeederBufferChg[ trayUnit] = eTRAY_BUFCHG_YES;
		
			if ( (aHeadOneCycle != nCyclesAhead) && (nCyclesAhead < bufferChangeCycle) )
				m_bPostCycleHalfPalletOut = true;
			else
				m_bPostCycleHalfPalletOut = false;

			TPcbID pcbID = m_CycleData.pPcbBoard ? m_CycleData.pPcbBoard->GetPcbID() : g_pPcbSet->PreferPcbId( m_CycleData.stationId.Lane());
			CPcbFunc* pPcbFunc = g_pPcbs[pcbID];
			SYS_ASSERT(pPcbFunc);

			bool bLastTrayCycle = false;
			if ( pPcbFunc->bExtendedArray)
			{
				if ( m_CycleData.cycle >= lastTrayCycle)
					bLastTrayCycle = true;
			}
			else
			{
				const StPlaceCad& pcbPlaceCad = pPcbFunc->GetPlaceCad( pPlaceWs->placeCadID);

				int blockCount		= pPcbFunc->GetBlockCount() - 1;
				int blockArrayCount = pPcbFunc->GetArrayCnt(pcbPlaceCad.blockNo) - 1;

				if ( (m_CycleData.placeData->arrayId.blockNo == blockCount)
					&& (m_CycleData.placeData->arrayId.arrayNo == blockArrayCount)
					&& (m_CycleData.cycle >= lastTrayCycle))
				{
					bLastTrayCycle = true;
				}
			}

			//CSeqEngine&	seqEngine = g_SeqEngine[m_pGantry->Unit()];

			//if(m_CycleData.cycle >= lastTrayCycle)
			//if (seqEngine.IsLastCycle())
			if ( bLastTrayCycle)
			{
				aTrayFeederBufferChg[ trayUnit] = eTRAY_BUFCHG_NO;
				m_bLastTrayCycle[ trayUnit] = true;
			}
			else
				m_bLastTrayCycle[ trayUnit] = false;

			// First pickup part in cycle will use 2nd pallet out.
			if( i == 0 && trayFeederType == eDEVICETYPE_TRAY_100VS)
				m_b2ndStepPalletOut = bOutOfRange;
		}
	}
	
	return true;
}

eCycleStatus
CSeqCycle::TrayFeederPreCyclePrefetch( TTrayFeederPallet& aTrayFeederNowPallet)
{
	// Prefetch pallets for tray feeders not used in this cycle
	long				pallet;
	eStatus				status = eNG;
	long				presentedPallet;
	long				presentedHalfPallet;
	eTrayFeederType		trayFeederType;
	long				useHalfPallet;
		
	useHalfPallet = g_SysConst.IsUseHalfPalletOut();
		
	// Prefetch the specified pallet for each tray feeder.
	for (int trayUnit=1; trayUnit<_countof(aTrayFeederNowPallet); trayUnit++)
	{
		pallet = aTrayFeederNowPallet[ trayUnit];

		// SKIP prefetch if no pallet is needed now.
		// SKIP prefetch if a buffer change should be done instead
		if ( pallet == 0)											continue;
		if ( m_TrayFeederBufferChg[ trayUnit] == eTRAY_BUFCHG_YES)
			continue;

		CTrayFeeder* pTrayFeeder = ::GetTrayFeeder( trayUnit);
		if ( pTrayFeeder == NULL)									continue;

		// cic, On this path, the condition "pTrayFeeder" cannot be false.
//		eFrontRear	 trayfrontRear	 = (pTrayFeeder ? pTrayFeeder->GetFrontOrRear() : eFRONT);
		eFrontRear	 trayfrontRear	 = pTrayFeeder->GetFrontOrRear();
		eFrontRear	 frontRear		 = m_pGantry->GetFrontOrRear();
		
		SectionID	 sectionId       = m_pGantry->GetSectionID();
		SectionID	 traySectionId   = pTrayFeeder->GetSectionID();
		
		if ( (( trayfrontRear != frontRear && trayfrontRear != eBOTH && m_pGantry->GetSharingGantry() ) 
			|| sectionId != traySectionId ) )
			continue;
		
		trayFeederType = pTrayFeeder->GetTrayFeederType();
		if ( trayFeederType == eDEVICETYPE_TRAY_SINGLE)				continue;			
		if ( trayFeederType == eDEVICETYPE_TRAY_STACK)				continue;
		if ( trayFeederType == eDEVICETYPE_TRAY_DDF)				continue;
		if ( pTrayFeeder->GetMultiTFType() == eMULTI_TF_STACK)			continue;

		if(m_b2ndStepPalletOut)
			presentedPallet = pTrayFeeder->Get2ndStepPalletOutNowNo();
		else
			presentedPallet = pTrayFeeder->GetPalletOutNowNo();

		presentedHalfPallet = pTrayFeeder->GetHalfPalletOutNowNo();
		
		
		if ( (useHalfPallet && m_bFirstCycleHalfPalletOut) )
		{
			if ( presentedHalfPallet == 0 || presentedHalfPallet != pallet)
			{
				Telemetry( EV_CLASSFEEDER, trayUnit, "TF : TrayFeederPreCyclePrefetch- Half Pallet Out Number = %d", pallet);
				status = pTrayFeeder->HalfPalletOut( pallet, eTRAY_NOWAIT);
				
				if ( status != eOK)
				{
					presentedPallet = 0;
					return eCYCLE_ABORT;
				}

				continue;
			}
			
			if ( presentedHalfPallet == pallet)						continue;
		}

		if ( (presentedPallet == 0 || presentedPallet != pallet) && !m_bLastTrayCycle[trayUnit])
		{
			// BLOCK until this tray feeder becomes available (this is only a factor for shared Front/Rear tray feeders).
			// Grab use of the tray feeder
			status = pTrayFeeder->ReserveUse( m_pGantry->Unit(), false);
			if ( status == eFAULT)
				return eCYCLE_ABORT;

			eTrayWait trayWait = eTRAY_NOWAIT;
// 			if ( (pTrayFeeder->GetTrayFeederType() == eDEVICETYPE_TRAY_SM_STACK)
// 				|| (pTrayFeeder->GetTrayFeederType() == eDEVICETYPE_TRAY_STF_HD_200SJ))
// 			{
// 				trayWait = eTRAY_NOWAIT_DONE;
// 			}

			Telemetry( EV_CLASSFEEDER, trayUnit, "TF : TrayFeederPreStepPalletPrefetch-Pallet Out Number = %d", pallet); 
			status = pTrayFeeder->PalletOut( pallet, trayWait, m_b2ndStepPalletOut, m_pGantry->Unit());
			if (status != eOK)
			{
				presentedPallet = 0;
				return eCYCLE_ABORT;
			}
		}
	
	}
	return eCYCLE_OK;
}


eCycleStatus
CSeqCycle::TrayFeederPostStepPalletPrefetch( eStepType aDoneStepType)
{
	long presentedPallet;
	long bSkipNextPallet = false;
	long useHalfPallet;
	long presentedHalfPallet;

	useHalfPallet = g_SysConst.IsUseHalfPalletOut();

	if ( g_StateReport.IsManualCmdActive( m_frRunningLane) )
		return eCYCLE_OK;

	// Prefetch pallets for tray feeders used in this step AND used in future cycles.
	eFeederType			feederType;
	eTrayFeederType	trayFeederType;
	eTrayPalletSpeed	palletSpeed = g_SysConst.GetTrayPalletSpeed();

	if ( !(aDoneStepType==eSTEP_PICK   &&  palletSpeed==eTRAY_PALLET_FASTEST)
	&&	 !(aDoneStepType==eSTEP_ALIGN  &&  (palletSpeed==eTRAY_PALLET_FAST || palletSpeed == eTRAY_PALLET_FASTEST)))
		return eCYCLE_OK;

	for (int trayUnit=0; trayUnit<_countof( m_TrayFeederNxtPallet); trayUnit++)
	{
		long pallet = m_TrayFeederNxtPallet[ trayUnit];

		// SKIP prefetch if no pallet is needed now.
		// SKIP prefetch if a buffer change should be done instead
		//if ( pallet == 0)											continue;
// 		if ( m_TrayFeederBufferChg[ trayUnit] == eTRAY_BUFCHG_YES)
// 			continue;

		// Look for a reason NOT to prefetch a pallet.
		// Reason #1: a placement on this tray unit failed (and will need to be dumped)
		bool bSkipPrefetchPallet = false;
		for (int j=0; j<m_CycleData.nCount; j++)
		{
			StPlaceData& pd = m_CycleData.placeData[j];
			// SKIP placements not associated with this tray unit
			// SKIP placements that were successful (i.e. no need to skip prefetching due to a pending dump)
			if ( !pd.pickedFeederID.IsDefined())		continue;
			feederType	= pd.pickedFeederID.Type();
			if ( feederType != eFEEDTYPE_TRAY)			continue;
			if ( pd.pickedFeederID.Unit() != trayUnit)	continue;

			//<2014-5-21>
			// Skip Tray Prefetch after PickStep if this is the last cycle.
			// Cause, If this is last cycle, PCB_END_BUFFER_CHG may take place.
			// This may move Empty Pallet to Buffer.
			// But if alignment report error, and this cycle need to dump part,
			// Pallet out request for dump for a tray which is moving to buffer may cause Tray Error.
			// Solution.
			// Delay making decision of PCB_END BUFFER_CHG until align result are reported.
// 			if (m_bLastTrayCycle[trayUnit] && aDoneStepType != eSTEP_ALIGN )
// 			{
// 				bSkipPrefetchPallet = true;
// 				break;
// 			}

			if ( pd.status == ePLACE_OK)				continue;
			if ( pd.bStoppedByDumpStop)
				bSkipNextPallet = true;
			// A failed placement was detected, so don't prefetch
			bSkipPrefetchPallet = true;
			break;
		}
		
		// SKIP prefetching a pallet if a placement failure occurred, associated with this tray unit
		if ( bSkipPrefetchPallet)						continue;

		CTrayFeeder* pTrayFeeder = ::GetTrayFeeder( trayUnit);
		if ( pTrayFeeder == NULL)						continue;
		eFrontRear	 trayfrontRear	 = (pTrayFeeder ? pTrayFeeder->GetFrontOrRear() : eFRONT);
		eFrontRear	 frontRear		 = m_pGantry->GetFrontOrRear();
		
		SectionID	 sectionId       = m_pGantry->GetSectionID();
		SectionID	 traySectionId   = pTrayFeeder->GetSectionID();
		
		if ( (( trayfrontRear != frontRear && trayfrontRear != eBOTH && m_pGantry->GetSharingGantry() ) 
			|| sectionId != traySectionId ) )
			continue;

		// Check Empty Pallet.
		if ( pallet != 0 && pTrayFeeder->IsEmptyPallet( pallet))		continue;

		// Check Single Tray
		trayFeederType = pTrayFeeder->GetTrayFeederType();
		if ( trayFeederType == eDEVICETYPE_TRAY_SINGLE)	continue;
		if ( trayFeederType == eDEVICETYPE_TRAY_STACK)	continue;
		if ( trayFeederType == eDEVICETYPE_TRAY_DDF)	continue;
		if ( pTrayFeeder->GetMultiTFType() == eMULTI_TF_STACK)			continue;

		eStatus status = eOK;
		
		if (m_b2ndStepPalletOut)
			presentedPallet = pTrayFeeder->Get2ndStepPalletOutNowNo();	
		else
			presentedPallet = pTrayFeeder->GetPalletOutNowNo();

		presentedHalfPallet = pTrayFeeder->GetHalfPalletOutNowNo();
		
		if ( bSkipNextPallet)							continue;


		pTrayFeeder->ReleaseUse( m_pGantry->Unit( ) );
		CRunOptions&	rRunOptions = *::GetRunOptions( m_frRunningLane );

		if (m_bLastTrayCycle[trayUnit])
		{
			bool	pcbEndPalletChange = true;
			Telemetry(EV_CLASSFEEDER, trayUnit, "TF : PCBEndBufferChange-Pallet Out Number = %d", pallet);

			if (g_pPcbSet->IsMultiPcb(m_CycleData.stationId.Lane()))
			{
				if (!g_pPcbSet->IsMultiPcbUseSameFirstTray(m_CycleData.stationId))
					pcbEndPalletChange = false;
			}
			else if (g_pPcbSet->IsJoin(m_CycleData.stationId))
			{
				if (!g_pPcbSet->IsJoinPcbUseSameFirstTray(m_CycleData.stationId))
					pcbEndPalletChange = false;
			}

			if (!g_StateReport.IsIndependentLanes( ))
			{
				CConveyor::TStationIdList stationList;
				g_pConveyor->GetStationList(stationList);

				SYS_ASSERT(!stationList.IsEmpty( ));

				for (POS pos = stationList.GetTailPosition( ); pos != NULL;)
				{
					// Pass same station 
					CStationId  stationId = stationList.GetPrev(pos);
					if (stationId.Lane( ) == m_CycleData.stationId.Lane( ))
						continue;

					if (!g_pConveyor->IsWork(stationId))
						continue;

					CPCBoard* pBrd = g_pConveyor->GetBoard(stationId);
					if (pBrd && !pBrd->IsAllWorkDone(stationId))
					{
						long nextPalletNo = pBrd->CheckNextPCBPalletNo( m_pGantry->GetFrontOrRear(), stationId);
						
						if (nextPalletNo && pallet != nextPalletNo )
						{
							Telemetry(EV_CLASSFEEDER, trayUnit, "TF : Tray Pallet [Nxt=%d, NewNxt=%d, Cur=%d]", m_TrayFeederNxtPallet[trayUnit], nextPalletNo, presentedPallet);
							m_TrayFeederNxtPallet[trayUnit] = nextPalletNo;
							pallet = nextPalletNo;
						}

						if (presentedPallet == pallet)
						{
							pcbEndPalletChange = false;
							break;
						}
						else
							pcbEndPalletChange = true;

					}
				}
			}

			// Case1. Twin PCB
			// Case2. Multi Board & Same First Tray.
			// Case3. Join Board & Same First Tray.
			if (pcbEndPalletChange)
				status = pTrayFeeder->SetPcbEndBufferChange( pallet, eTRAY_WAIT);
			else
				status = pTrayFeeder->SetPcbEndBufferChange( 0, eTRAY_WAIT);

			if (status == eNG)
			{
				if (presentedPallet == 0 || presentedPallet != pallet)
				{
					eTrayWait trayWait = eTRAY_NOWAIT;
// 					if ( (pTrayFeeder->GetTrayFeederType() == eDEVICETYPE_TRAY_SM_STACK)
// 						|| (pTrayFeeder->GetTrayFeederType() == eDEVICETYPE_TRAY_STF_HD_200SJ))
// 					{
// 						trayWait = eTRAY_NOWAIT_DONE;
// 					}

					Telemetry(EV_CLASSFEEDER, 0, "TF1 : TrayFeederPostStepPalletPrefetch-Pallet Out Number = %d", pallet);
					status = pTrayFeeder->PalletOut(pallet, trayWait);

					if (status != eOK)
					{
						presentedPallet = 0;
						return eCYCLE_ABORT;
					}
				}
			}
			else if (status == eFAULT)
			{
				presentedPallet = 0;
				return eCYCLE_ABORT;
			}
			
			continue;
		}
		else
		{
			if ( m_TrayFeederBufferChg[ trayUnit] == eTRAY_BUFCHG_YES && pTrayFeeder->IsExistEmptyPallet())
			{
				if( pTrayFeeder->IsCycleBufferChange() )
				{	
					pTrayFeeder->ResetHalfPalletOutNo();

					Telemetry( EV_CLASSFEEDER, trayUnit, "TF : TrayFeederPostStepPalletPrefetch-Pallet Out Number = %d", pallet); 
					status = pTrayFeeder->SetCycleEndBufferChange( pallet, eTRAY_WAIT);
					if (status != eOK)
						return eCYCLE_ABORT;
					
					pTrayFeeder->SetCycleBufferEnable( false);
				}
			}	
			else
			{
				if ( pallet == 0)								continue;

				if ( useHalfPallet && m_bPostCycleHalfPalletOut)
				{
					if ( presentedHalfPallet == 0 || presentedHalfPallet != pallet)
					{
						Telemetry( EV_CLASSFEEDER, trayUnit, "TF : TrayFeederPostStepPalletPrefetch- Half Pallet Out Number = %d", pallet);
						status = pTrayFeeder->HalfPalletOut( pallet, eTRAY_NOWAIT);
						
						if ( status != eOK)
						{
							presentedPallet = 0;
							return eCYCLE_ABORT;
						}
						
						continue;
					}
					
					if ( presentedHalfPallet == pallet)			continue;
				}
				
				if ( presentedPallet == 0 || presentedPallet != pallet )
				{
					// BLOCK until this tray feeder becomes available (this is only a factor for shared Front/Rear tray feeders).
					// Grab use of the tray feeder
					status = pTrayFeeder->ReserveUse( m_pGantry->Unit(), false);
					
					if ( status == eFAULT)
						return eCYCLE_ABORT;
					
					eTrayWait trayWait = eTRAY_NOWAIT;
// 					if ( (pTrayFeeder->GetTrayFeederType() == eDEVICETYPE_TRAY_SM_STACK)
// 						|| (pTrayFeeder->GetTrayFeederType() == eDEVICETYPE_TRAY_STF_HD_200SJ))
// 					{
// 						trayWait = eTRAY_NOWAIT_DONE;
// 					}

					Telemetry( EV_CLASSFEEDER, trayUnit, "TF : TrayFeederPostStepPalletPrefetch-Pallet Out Number = %d", pallet); 
					status = pTrayFeeder->PalletOut( pallet, trayWait, m_b2ndStepPalletOut, m_pGantry->Unit());
					
					if (status != eOK)
					{
						presentedPallet = 0;
						return eCYCLE_ABORT;
					}
				}
			}
		}
	}
	return eCYCLE_OK;
}


eCycleStatus
CSeqCycle::TrayFeederPostStepRefillStackTF(eStepType aDoneStepType)
{
	Telemetry(EV_CLASSFEEDER, 0,  "TF : CSeqCycle::TrayFeederPostStepRefillStackTF - Start");

	long bSkipNextPallet = false;

	if (g_StateReport.IsManualCmdActive(m_frRunningLane))
		return eCYCLE_OK;

	// Prefetch pallets for tray feeders used in this step AND used in future cycles.
	eFeederType			feederType;
	CFeederId			feederId;
	eTrayFeederStatus	tfStatus;
	bool				bReady;
	eMagazine			magazine;
//	long				presentedPallet;
	long				presentingPallet = -1;
	long				trayUnit;
	long				pallet;
	eStatus				status = eNG;
	//CFixedMap< long, long, eMAX_PCB_TRAYUNIT> trayUnitBusyMap;
	
	bool bSkipRefillPallet = false;
	for (int j = 0; j < m_CycleData.nCount; j++)
	{
		StPlaceData& pd = m_CycleData.placeData[j];
		// SKIP placements not associated with this tray unit
		// SKIP placements that were successful (i.e. no need to skip prefetching due to a pending dump)
		if (!pd.pickedFeederID.IsDefined())		continue;
		feederType = pd.pickedFeederID.Type();
		feederId = pd.pickedFeederID;
		if (feederType != eFEEDTYPE_TRAY)			continue;

		if (pd.bStoppedByDumpStop)
			bSkipNextPallet = true;
		
		if (bSkipRefillPallet)	continue;

		CTrayFeeder* pTrayFeeder = ::GetTrayFeeder(feederId);
		long totalPalletCnt = pTrayFeeder->GetTotalPallets();
		trayUnit	= feederId.Unit();
		pallet		= feederId.No();
		
		if (pTrayFeeder->GetTrayFeederType() != eDEVICETYPE_TRAY_STF_HD_200SJ)
			continue;

		//if ( trayUnitBusyMap.Lookup(trayUnit, presentingPallet) ) continue;

		for (int k = 1; k < totalPalletCnt+1; k++)
		{
			pallet = k;
			feederId.No(pallet);

			// REPORT ERROR - If the magazine is not ready as it is expected
			tfStatus = pTrayFeeder->GetStatusCode();
			magazine = pTrayFeeder->GetMagazine(pallet);
			bReady = pTrayFeeder->IsReady(magazine);
			if (!bReady)
				goto ErrNotReady;
		
			//trayUnitBusyMap.Lookup(trayUnit, presentingPallet); // Map에서 trayUnit에 해당하는 pallet 를 찾아서 반환한다.				

			//if (presentingPallet != pallet)
			{			
				//eDEVICETYPE_TRAY_STACK 
				//eDEVICETYPE_TRAY_STF_HD_200SJ
			
				CTray* pTray = ::GetTray(feederId);
				Cxy nextPocket = pTray->GetNextPickN(1);
				if (nextPocket == Cxy(-1, -1))
				{
					bool bCanTrayOperation = true;
					bool bError = false, bEmpty = false, bDumpFull = false, bEmptyWarning = false, bDumpFullWarning = false, bReady1 = false, bReady2 = false, bRunning = false;
					pTrayFeeder->CheckTrayStatus(bError, bEmpty, bDumpFull, bEmptyWarning, bDumpFullWarning, bReady1, bReady2, bRunning, pallet);

					if (bError)
					{
						bCanTrayOperation = false;
						pTrayFeeder->ProcessMessage(eTFMSG_STACK_ERROR, 0);
					}
					if (bEmpty)
					{
						bCanTrayOperation = false;
						pTrayFeeder->ProcessMessage(eTFMSG_STACK_EMPTY, 0);
					}
					if (bDumpFull)
					{
						bCanTrayOperation = false;
						pTrayFeeder->ProcessMessage(eTFMSG_STACK_DUMP_FULL, 0);
					}

					if (bCanTrayOperation)
					{
						status = pTrayFeeder->PalletOut(pallet, eTRAY_NOWAIT, false, m_GantryID);
						if ( status != eOK)
							return eCYCLE_ABORT;
					}

					Telemetry(EV_CLASSFEEDER, trayUnit, "TF : CSeqCycle::TrayFeederPostStepRefillStackTF-pallet Out pallet = %d", pallet);
					TRACE("TF : CSeqCycle::TrayFeederPostStepRefillStackTF pallet Out pallet = %d\n", pallet);
				}
				else
				{
					status = eOK;
				}

//				trayUnitBusyMap[trayUnit] = pallet;
			}
		}
	}

	return eCYCLE_OK;

ErrNotReady:
	eMachErrorCode	mec = (magazine == 1 ? eMEC_TRAY_UP_MAG_NOT_READY : eMEC_TRAY_LO_MAG_NOT_READY);

	Telemetry(EV_CLASSFEEDER, trayUnit, "TF Error - Magazine NOT Ready, Tray Feeder Unit=%d Mag=%d", trayUnit, magazine);
	g_StateReport.ReportErrorEx2(mec, eERR_LEVEL_FREEZE, eMOTOR_OK,
									m_SectionID, m_frRunningLane,
									HeadID(0), HeadNone, PartID(0), feederId, CNozzleId(0), 0, AxisID(0), __LINE__,
									trayUnit, magazine);

	return eCYCLE_OK;
}


eCycleStatus
CSeqCycle::TrayFeederBufferChangeCheck( )
{
	if ( g_StateReport.IsManualCmdActive( m_frRunningLane) )
		return eCYCLE_OK;

	if ( !CPCBoard::IsMultiTrayNeeded() )
		return eCYCLE_OK;

	// Send BufferChange commands to 1-Slot Tray Feeders if no pallets are required on near term subsequent cycles.
	//
	long			pallet;
	eStatus			status;
	eTrayBufChg		bufferChg;
	eMultiTFType	multiTFType;
	CTrayFeeder*	pTrayFeeder;
	long			useHalfPallet;
	long			presentedPallet;

	TPlaceWsList		trayPlaceList;
	CSeqEngine&			seqEngine = g_SeqEngine[ m_pGantry->Unit()];

	seqEngine.GetLookAheadOneTrayCycles( trayPlaceList, eLOOK_INCLUDE_CURRENT_CYCLE);

	useHalfPallet = g_SysConst.IsUseHalfPalletOut();

	for (int trayUnit=1; trayUnit<_countof(m_TrayFeederBufferChg); trayUnit++)
	{
		pTrayFeeder = ::GetTrayFeeder( trayUnit);
		if ( pTrayFeeder == NULL)						continue;
		// cic, On this path, the condition "pTrayFeeder" cannot be false.
		eFrontRear	 trayFrontRear	 = pTrayFeeder->GetFrontOrRear();
		eFrontRear	 frontRear		 = m_pGantry->GetFrontOrRear();
		
		SectionID	 sectionId       = m_pGantry->GetSectionID();
		SectionID	 traySectionId   = pTrayFeeder->GetSectionID();
		
		if ( (( trayFrontRear != frontRear && trayFrontRear != eBOTH && m_pGantry->GetSharingGantry() ) 
			|| sectionId != traySectionId ) )
			continue;

		multiTFType = pTrayFeeder->GetMultiTFType();
		if ( multiTFType != eMULTI_TF_1SLOT_BUFFER)		continue;

		eTrayNonStopMethod trayNonStopMethod = pTrayFeeder->GetNonStopMethod();
		if ( trayNonStopMethod != eNONSTOP_ONE_SLOT_BUFFER)	continue;

		bufferChg = m_TrayFeederBufferChg[ trayUnit];
		if ( bufferChg == eTRAY_BUFCHG_NO)
		{
			Telemetry( EV_CLASSFEEDER, trayUnit, "TF : TrayFeederBufferChangeCheck - TRAY_BUFFER_CHANGE_NO");
			continue;
		}

		pallet = m_TrayFeederNowPallet[ trayUnit];
		if ( pallet == 0)
			pallet = m_TrayFeederNxtPallet[ trayUnit];

		presentedPallet = pTrayFeeder->GetPalletOutNowNo();
		
		if ( useHalfPallet && pallet == 0 && presentedPallet != 0 && trayPlaceList.GetCount() != 0)
		{
			Telemetry( EV_CLASSFEEDER, trayUnit, "TF : TrayFeederBufferChangeCheck- Half Pallet Out Number = %d", presentedPallet);
			status = pTrayFeeder->HalfPalletOut( presentedPallet, eTRAY_NOWAIT);
			
			if ( status != eOK)
			{
				presentedPallet = 0;
				return eCYCLE_ABORT;
			}
		}
		
		// Cannot CYCLE buffer change without target pallet
//		if ( pallet == 0)
//		{
//			Telemetry( EV_CLASSFEEDER, trayUnit, "TF : TrayFeederBufferChangeCheck - PALLET OUT NO - ZERO");
//			continue;
//		}
		
		if ( CTrayFeeder::IsTrayFeederRetryOver(sectionId, frontRear))
			return eCYCLE_OK;

// 		eFrontRear   trayFrontRear   = (pTrayFeeder ? pTrayFeeder->GetFrontOrRear() : eFRONT);
// 		eFrontRear   frontRear       = m_pGantry->GetFrontOrRear();
// 
// 		SectionID	 sectionId       = m_pGantry->GetSectionID();
// 		SectionID	 traySectionId   = pTrayFeeder->GetSectionID();

		if (( trayFrontRear == frontRear || trayFrontRear == eBOTH || !m_pGantry->GetSharingGantry() ) 
			&& sectionId == traySectionId )
        {
		    if( pTrayFeeder->IsCycleBufferChange() )
		    {	
			    pTrayFeeder->ResetHalfPalletOutNo();

			    status = pTrayFeeder->SetCycleEndBufferChange( pallet, eTRAY_WAIT);
			    if (status != eOK)
				    return eCYCLE_ABORT;

			    pTrayFeeder->SetCycleBufferEnable( false);
		    }
		}


// 		if( pTrayFeeder->IsCycleBufferChange() )
// 		{	
// 			pTrayFeeder->ResetHalfPalletOutNo();
// 
// 			status = pTrayFeeder->SetCycleEndBufferChange( pallet, eTRAY_WAIT);
// 			if (status != eOK)
// 				return eCYCLE_ABORT;
// 
// 			pTrayFeeder->SetCycleBufferEnable( false);
//		}
		
	}
	return eCYCLE_OK;
}


void
CSeqCycle::SetTrayFeederCycleBufferChange( eTrayBufChg aTrayBufChg)
{
	long i;
	
	for (i=0; i<_countof(m_TrayFeederBufferChg); i++)	m_TrayFeederBufferChg[i] = aTrayBufChg;
}


void
CSeqCycle::GantryMoveZSafe()
{
	THeadBlkList headBlocks;
	::GetHeadBlocks( headBlocks, m_pGantry->Unit());
	for (POS pos=headBlocks.GetHeadPosition(); pos!=NULL;)
	{
		CHeadBlock* pHB = headBlocks.GetNext( pos);
		pHB->MoveAllZSafe( true);
	}
}



// High Performance Alignment Coordination
//---------------------------------
void
CSeqCycle::PostFindTrigger( HeadID aHeadID )
{
	SYS_ASSERT( 1 <= aHeadID  &&  aHeadID < SYS_MAX_HEAD+1);
	long spindle = g_Head[ aHeadID].GetSpindle();
	SYS_ASSERT( 1 <= spindle  &&  spindle < 32);
	long bitHead = 1<<spindle;
	m_EventFindTrigger.Post( bitHead);
}


eStatus
CSeqCycle::WaitFindTrigger( long aHeadSpindleBits, long& aHeadSpindleTriggerBits)
{
	CSysEv32::eWaitStatus waitStatus;

	waitStatus = m_EventFindTrigger.Wait( aHeadSpindleBits, THIS_FILE, __LINE__);

	SYS_ASSERT( waitStatus==CSysEv32::WAIT_OK  ||  waitStatus==CSysEv32::WAIT_ABORT);

	aHeadSpindleTriggerBits = m_EventFindTrigger.Read();

	if ( waitStatus==CSysEv32::WAIT_OK)
		return eOK;
	else if ( waitStatus==CSysEv32::WAIT_ABORT)
		return eFAULT;
	return eNG;
}


void
CSeqCycle::UnPostAlignResult( HeadID aHeadID)
{
	SYS_ASSERT( 1 <= aHeadID  &&  aHeadID < SYS_MAX_HEAD+1);
	long spindle = g_Head[ aHeadID].GetSpindle();
	SYS_ASSERT( 1 <= spindle  &&  spindle < 32);
	long bitHead = 1<<spindle;
	m_EventFindResults.Reset( bitHead);
}


void
CSeqCycle::PostAlignResult( HeadID aHeadID)
{
	SYS_ASSERT( 1 <= aHeadID  &&  aHeadID < SYS_MAX_HEAD+1);
	long spindle = g_Head[ aHeadID].GetSpindle();
	SYS_ASSERT( 1 <= spindle  &&  spindle < 32);
	long bitHead = 1<<spindle;
	m_EventFindResults.Post( bitHead);


	long	idxStep;	// Index into m_Steps[].
	CStep** ppStep=NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
	long	nStep;		// Count of contiguous primary type of steps.

	nStep = GetSteps(eSTEP_PICK, m_frRunningLane, idxStep, ppStep);
	for (int i=0; i<nStep; i++)
	{
		CStepPick* pStepPick = (CStepPick*)m_Steps[idxStep+i];
		pStepPick->PostAlignResult(aHeadID);
	}
}

void
CSeqCycle::InitAlignResultAll( )
{
	SYS_ASSERT(m_pGantry);
	CStepPick::InitAlignResultAll(m_pGantry->Unit( ));
}

eStatus
CSeqCycle::WaitAlignResult( HeadID aHeadID, long aTimeout/*=INFINITE_TIMEOUT*/)
{
	CSysEv32::eWaitStatus waitStatus;

	SYS_ASSERT( 1 <= aHeadID  &&  aHeadID <= SYS_MAX_HEAD);
	long spindle = g_Head[ aHeadID].GetSpindle();
	SYS_ASSERT( 1<= spindle  &&  spindle < 32);
	long bitHead = 1<<spindle;

	CTelemetryPair tp(EV_CLASSHEAD, aHeadID, "H%s:WaitAlignResult ", g_Head[ aHeadID].NameBrief()); 

	waitStatus = m_EventFindResults.Wait( bitHead, THIS_FILE, __LINE__, aTimeout);
	
	SYS_ASSERT( waitStatus!=CSysEv32::WAIT_ERROR);
	
	if ( waitStatus==CSysEv32::WAIT_OK)
		return eOK;
	else if ( waitStatus==CSysEv32::WAIT_ABORT)
		return eFAULT;
	return eNG;
}


//For SVS Task operation Done.
void
CSeqCycle::UnPostSVSResult( long aOpMask)
{
	//SYS_ASSERT( 1 <= aHeadID  &&  aHeadID < SYS_MAX_HEAD+1);
	//long spindle = g_Head[ aHeadID].GetSpindle();
	//SYS_ASSERT( 1 <= spindle  &&  spindle < 32);
	//long bitHead = 1<<spindle;

	CTelemetryPair tp(EV_CLASSHEAD, m_pGantry->Unit(), "%s:UnPostSVSResult Mask:0x%x", m_pGantry->GetName(), aOpMask);

	m_EventSvsResults.Reset( aOpMask);
}


void
CSeqCycle::PostSVSResult( long aOpMask)
{
	//SYS_ASSERT( 1 <= aHeadID  &&  aHeadID < SYS_MAX_HEAD+1);
	//long spindle = g_Head[ aHeadID].GetSpindle();
	//SYS_ASSERT( 1 <= spindle  &&  spindle < 32);
	//long bitHead = 1<<spindle;

	CTelemetryPair tp(EV_CLASSHEAD, m_pGantry->Unit(), "%s:PostSVSResult Mask:0x%x", m_pGantry->GetName(), aOpMask); 

	PostPlaceSVSBegin(aOpMask);

	m_EventSvsResults.Post( aOpMask );
}


eStatus
CSeqCycle::WaitSVSResult( long aOpMask, long aTimeout/*=INFINITE_TIMEOUT*/)
{
	CSysEv32::eWaitStatus waitStatus;

	//SYS_ASSERT( 1 <= aHeadID  &&  aHeadID <= SYS_MAX_HEAD);
	//long spindle = g_Head[ aHeadID].GetSpindle();
	//SYS_ASSERT( 1<= spindle  &&  spindle < 32);
	//long bitHead = 1<<spindle;

	CTelemetryPair tp(EV_CLASSHEAD, 0, "WaitSVSResult Mask:0x%x", aOpMask); 

	waitStatus = m_EventSvsResults.Wait( aOpMask, THIS_FILE, __LINE__, aTimeout);
	
	SYS_ASSERT( waitStatus!=CSysEv32::WAIT_ERROR);
	
	if ( waitStatus==CSysEv32::WAIT_OK)
		return eOK;
	else if ( waitStatus==CSysEv32::WAIT_ABORT)
		return eFAULT;

	g_StateReport.ReportErrorEx2( eMEC_FLY_SVS_SEQ_SVS_RESULT_TIMEOUT, eERR_LEVEL_FREEZE, eMOTOR_OK, m_SectionID, eBOTH,
									0, HeadNone, 0, 0, 0, 0, 0, __LINE__);

	return eNG;
}

void
CSeqCycle::UnPostPlaceSVSBegin(long aOpMask)
{
	//SYS_ASSERT( 1 <= aHeadID  &&  aHeadID < SYS_MAX_HEAD+1);
	//long spindle = g_Head[ aHeadID].GetSpindle();
	//SYS_ASSERT( 1 <= spindle  &&  spindle < 32);
	//long bitHead = 1<<spindle;

	m_PlaceSvsCritcalMotion.Init();

	CTelemetryPair tp(EV_CLASSHEAD, m_pGantry->Unit(), "%s:UnPostPlaceSVSBegin Mask:0x%x", m_pGantry->GetName(), aOpMask);

	m_EventPlaceSvsBegin.Reset(aOpMask);
	m_EventPlaceSvsXYReady.Reset(aOpMask);
}

void
CSeqCycle::PostPlaceSVSBegin(long aOpMask, bool abSetPlaceSvs/*=false*/, long aStartTime/*=0*/, long aTriggerTime/*=0*/, long aCaptureTime/*=0*/)
{
	//SYS_ASSERT( 1 <= aHeadID  &&  aHeadID < SYS_MAX_HEAD+1);
	//long spindle = g_Head[ aHeadID].GetSpindle();
	//SYS_ASSERT( 1 <= spindle  &&  spindle < 32);
	//long bitHead = 1<<spindle;

	m_PlaceSvsCritcalMotion.bSet = abSetPlaceSvs;
	m_PlaceSvsCritcalMotion.startTime = aStartTime;
	m_PlaceSvsCritcalMotion.triggerTime = aTriggerTime;
	m_PlaceSvsCritcalMotion.captureTime = aCaptureTime;

	CTelemetryPair tp(EV_CLASSHEAD, m_pGantry->Unit(), "%s:PostPlaceSVSBegin Mask:0x%x", m_pGantry->GetName(), aOpMask);

	m_EventPlaceSvsBegin.Post(aOpMask);
}

eStatus
CSeqCycle::WaitPlaceSVSBegin(long aOpMask, bool& abSetPlaceSvs, long& aStartTime, long& aTriggerTime, long& aCaptureTime, long aTimeout/*=INFINITE_TIMEOUT*/)
{
	CSysEv32::eWaitStatus waitStatus;

	//SYS_ASSERT( 1 <= aHeadID  &&  aHeadID <= SYS_MAX_HEAD);
	//long spindle = g_Head[ aHeadID].GetSpindle();
	//SYS_ASSERT( 1<= spindle  &&  spindle < 32);
	//long bitHead = 1<<spindle;

	CTelemetryPair tp(EV_CLASSHEAD, m_pGantry->Unit(), "%s:WaitPlaceSVSBegin Mask:0x%x", m_pGantry->GetName(), aOpMask);

	waitStatus = m_EventPlaceSvsBegin.Wait(aOpMask, THIS_FILE, __LINE__, aTimeout);

	abSetPlaceSvs = m_PlaceSvsCritcalMotion.bSet;
	aStartTime = m_PlaceSvsCritcalMotion.startTime;
	aTriggerTime = m_PlaceSvsCritcalMotion.triggerTime;
	aCaptureTime = m_PlaceSvsCritcalMotion.captureTime;

	SYS_ASSERT(waitStatus != CSysEv32::WAIT_ERROR);

	if (waitStatus == CSysEv32::WAIT_OK)
		return eOK;
	else if (waitStatus == CSysEv32::WAIT_ABORT)
		return eFAULT;

	g_StateReport.ReportErrorEx2(eMEC_FLY_SVS_SEQ_PLC_BEGIN_TIMEOUT, eERR_LEVEL_FREEZE, eMOTOR_OK, m_SectionID, eBOTH,
		0, HeadNone, 0, 0, 0, 0, 0, __LINE__);
	return eNG;
}

void
CSeqCycle::PostPlaceSVSXYReady(long aOpMask, long aTriggerAccTime, long aTriggerTime)
{
	CTelemetryPair tp(EV_CLASSHEAD, m_pGantry->Unit(), "%s:PostPlaceSVSXYReady Mask:0x%x, AccT:%d, AbsT:%d ",
		m_pGantry->GetName(), aOpMask, aTriggerAccTime, aTriggerTime);
	m_PlaceSvsCritcalMotion.motionReadyTimeAbs = aTriggerTime;
	m_PlaceSvsCritcalMotion.motionReadyAccTime = aTriggerAccTime;

	m_EventPlaceSvsXYReady.Post(aOpMask);
}

eStatus
CSeqCycle::WaitPlaceSVSXYReady(long aOpMask, long& aTriggerAccTime, long& aTriggerTime, long aTimeout/*=INFINITE_TIMEOUT*/)
{
	CSysEv32::eWaitStatus waitStatus;

	//SYS_ASSERT( 1 <= aHeadID  &&  aHeadID <= SYS_MAX_HEAD);
	//long spindle = g_Head[ aHeadID].GetSpindle();
	//SYS_ASSERT( 1<= spindle  &&  spindle < 32);
	//long bitHead = 1<<spindle;

	CTelemetryPair tp(EV_CLASSHEAD, m_pGantry->Unit(), "%s:WaitPlaceSVSXYReady Mask:0x%x", m_pGantry->GetName(), aOpMask);

	waitStatus = m_EventPlaceSvsXYReady.Wait(aOpMask, THIS_FILE, __LINE__, aTimeout);

	aTriggerTime = m_PlaceSvsCritcalMotion.motionReadyTimeAbs;
	aTriggerAccTime = m_PlaceSvsCritcalMotion.motionReadyAccTime;

	SYS_ASSERT(waitStatus != CSysEv32::WAIT_ERROR);

	if (waitStatus == CSysEv32::WAIT_OK)
		return eOK;
	else if (waitStatus == CSysEv32::WAIT_ABORT)
		return eFAULT;

	g_StateReport.ReportErrorEx2(eMEC_FLY_SVS_SEQ_PLC_SVS_XY_TIMEOUT, eERR_LEVEL_FREEZE, eMOTOR_OK, m_SectionID, eBOTH,
		0, HeadNone, 0, 0, 0, 0, 0, __LINE__);
	return eNG;
}


void
CSeqCycle::UnPost3DAlignResult( HeadID aHeadID)
{
	SYS_ASSERT( 1 <= aHeadID  &&  aHeadID < SYS_MAX_HEAD + 1);
	long spindle = g_Head[aHeadID].GetSpindle();
	SYS_ASSERT( 1 <= spindle  &&  spindle < 32);
	long bitHead = 1 << spindle;
	m_EventFind3DResults.Reset(bitHead);
}


void
CSeqCycle::Post3DAlignResult( HeadID aHeadID)
{
	SYS_ASSERT( 1 <= aHeadID  &&  aHeadID < SYS_MAX_HEAD + 1);
	long spindle = g_Head[aHeadID].GetSpindle();
	SYS_ASSERT( 1 <= spindle  &&  spindle < 32);
	long bitHead = 1 << spindle;
	m_EventFind3DResults.Post(bitHead);


	long	idxStep;	// Index into m_Steps[].
	CStep** ppStep = NULL;// Pointer to the beginning of a pointer array inside m_Steps[].
	long	nStep;		// Count of contiguous primary type of steps.

	nStep = GetSteps(eSTEP_PICK, m_frRunningLane, idxStep, ppStep);
	for ( int i = 0; i < nStep; i++)
	{
		CStepPick* pStepPick = (CStepPick*)m_Steps[idxStep + i];
		pStepPick->PostAlignResult(aHeadID);
	}
}


eStatus
CSeqCycle::Wait3DAlignResult( HeadID aHeadID, long aTimeout/*=INFINITE_TIMEOUT*/)
{
	CSysEv32::eWaitStatus waitStatus;

	SYS_ASSERT( 1 <= aHeadID  &&  aHeadID <= SYS_MAX_HEAD);
	long spindle = g_Head[aHeadID].GetSpindle();
	SYS_ASSERT( 1 <= spindle  &&  spindle < 32);
	long bitHead = 1 << spindle;

	CTelemetryPair tp(EV_CLASSHEAD, aHeadID, "H%s:Wait3DAlignResult ", g_Head[aHeadID].NameBrief());

	waitStatus = m_EventFind3DResults.Wait(bitHead, THIS_FILE, __LINE__, aTimeout);

	SYS_ASSERT( waitStatus != CSysEv32::WAIT_ERROR);

	if ( waitStatus == CSysEv32::WAIT_OK)
		return eOK;
	else if ( waitStatus == CSysEv32::WAIT_ABORT)
		return eFAULT;
	return eNG;
}


void
CSeqCycle::UpdateStatistics( )
{
	CRunOptions& rRunOptions = *::GetRunOptions( m_frRunningLane);

	//Don't update statistics if this cycle(all step) is virtual pick.
	if ( m_CycleData.bVirtualPick )
		return;

	//Don't update statistics for manual operation.
	//	If you remove below, you have to check tray manual pick with Vac. check,
	//	You may get vacuum error and retry count over error continuously.
	if ( g_StateReport.IsManualCmdActive( m_frRunningLane) )
		return;

	// Don't update statistics for Dump Stop cycles, since nothing really happened
	if ( IsCycleDumpStopped())
		return;

	// Do NOT apply pick & place count during dry cycle in Real Customer if dry count is not enabled.
	if (	rRunOptions.IsDrySkipPickUp()
		|| (!g_FacDebug.IsDryPnPCountEnable() && rRunOptions.IsDryPassPartAlign()) )
		return;

	// <SAMEX 20070723> Do NOT apply if this is just pre-pick cycle, this will be updated on next cycle.
	if ( m_PrefetchOp != eCYCLE_PREFETCH_NONE &&  m_PrefetchOp < eCYCLE_PREFETCH_ALIGN/*m_PrefetchOp <= eCYCLE_PREFETCH_ALIGN */)
		return;

	InitStatCache();

	// Apply any statistics
	UpdatePickStatistics();
	UpdatePlaceStatistics();

	// Update the global cycle statistics.
	UpdateGlobalCycleStatValues();

	//DATABACKUP - 20090205
	// Update placed record into non-volatile saving device
	UpdatePlacedInfoBackUp();

	UpdateAccumulatedHeadCycleInfoBackUp();
}


//DATABACKUP - 20090205
void
CSeqCycle::UpdatePlacedInfoBackUp( )
{
	CRunOptions& rRunOptions = *::GetRunOptions( m_frRunningLane);

	// Ignore cycles that don't place parts
	if ( m_PrefetchOp != eCYCLE_PREFETCH_NONE)
		return;

	if ( g_pRunTimeBackUp == NULL )
		return;

	bool bSectionRunning = g_StateReport.IsMachineRunning( m_SectionID, m_frRunningLane);
	bool bRealPlace = ( !bSectionRunning || (!rRunOptions.IsDrySkipPickUp() && !rRunOptions.IsDrySkipIndexing()) );
	if ( !bRealPlace )
		return;

	int			idxSt = ::GetConveyorObj()->GetWorkStationArrayIndx( m_CycleData.stationId );
	SYS_ASSERT( idxSt != NOT_AN_INDEX );
	
	// Store Runtime backup data to permanent storage.
	g_pRunTimeBackUp->WritePlacedPartData( idxSt );

//	CStationId	stationId = m_CycleData.stationId;
//	long		cycle	  = m_CycleData.cycle;
//	long		headNoBit = 0;
//	int			idxSt	  = ::GetConveyorObj()->GetWorkStationArrayIndx( m_CycleData.stationId );
//	SYS_ASSERT( idxSt != NOT_AN_INDEX );
//
//	for (int i=0; i<m_CycleData.nCount; i++)
//	{
//		StPlaceData& pd = m_CycleData.placeData[i];
//
//		// Don't save placements that didn't place successfully
//		if ( pd.status != ePLACE_OK)			continue;
//		if ( !pd.placedFeederID.IsDefined())	continue;
//
//		if ( pd.profile.IsVirtualPick() )
//			continue;
//
//		//TODO: DATABACKUP - Add to save cycle placed data into memory.
//		
//		CGantry* pGantry		= pd.pHead->Gantry();
//		SYS_ASSERT( pGantry != NULL );
//		int			spindle		= pd.pHead->GetSpindle();	// One(1) based
//		GantryID	gantryId	= pGantry->Unit();			// One(1) based
//		SectionID	sectionId	= pGantry->GetSectionID();	// One(1) based
//		SYS_ASSERT( sectionId > 0 && sectionId <= SYS_MAX_GANTRY_PER_SECTION );
//		int gantryIdx = ::GetGantryIdxbySection( gantryId, sectionId ); // One(1) based
//
//		CArrayId	arrayId		= pd.arrayId;
//
//		long		headBit		= 0;
//		headBit = 0x01 << (spindle-1);
//		headNoBit |= headBit;
//
//		g_pRunTimeBackUp->SetPlacedCycleInfo( idxSt, gantryIdx-1, arrayId, (int)cycle, headNoBit );
//	}
}


void
CSeqCycle::UpdateAccumulatedHeadCycleInfoBackUp()
{
	if ( g_pRunTimeBackUp == NULL)
		return;

	CRunOptions& rRunOptions = *::GetRunOptions(m_frRunningLane);
	bool bSectionRunning = g_StateReport.IsMachineRunning(m_SectionID, m_frRunningLane);

	if ( !bSectionRunning || rRunOptions.IsDrySkipPickUp())
		return;

	if ( g_FacDebug.GetAutoSpindleFrictionCheckCount() == 0)
		return;

	for ( int i = 0; i < m_CycleData.nCount; i++)
	{
		StPlaceData& pd = m_CycleData.placeData[i];
		// Skip if step is virtual pick step.
		if ( pd.profile.IsVirtualPick())
			continue;

		CHeadBlock* pHB = pd.pHead->GetHeadBlock();
		SYS_ASSERT( pHB);

		if ( !pHB->IsRotary())
			continue;

		g_pRunTimeBackUp->IncAccumulatedHeadCycleData( pd.head);
	}
}


void
CSeqCycle::InitStatCache( )
{
	// Analyze the cycle in the pick order (not the place order).
	//
	for (int i=0; i<m_CycleData.nCount; i++)
	{
		StPlaceData& pd = m_CycleData.placeData[i];

		// Get the picked feeder as recorded by the CStepPick::Execute
		CFeederId feederId = pd.pickedFeederID;
		
		// If the part was not picked, then maybe it was already on the head
		// as recorded by CStepPlace::Execute
		if ( !feederId.IsDefined())
		{
			if ( pd.status == ePLACE_OK)
				feederId = pd.placedFeederID;
			else
				feederId = pd.pHead->GetPickedFeeder();
		}

		//NOTE: This can be happened if this place is 'Virtual Pick' part.
		if ( !feederId.IsDefined())
		{
			// Debug - do nothing line
			SYS_ASSERT( !feederId.IsDefined());
		}

		InitStatCache( i, pd, feederId);
	}
}


bool
CSeqCycle::IsCycleDumpStopped( )
{
	for (int i=0; i<m_CycleData.nCount; i++)
	{
		StPlaceData& pd = m_CycleData.placeData[i];

		// Don't count placements that didn't place successfully
		if ( pd.status != ePLACE_OK)	continue;
		if ( pd.bStoppedByDumpStop)		return true;
	}
	return false;
}


void
CSeqCycle::InitStatCache( int aIdxPd, const StPlaceData& aPD, CFeederId aPickedFeederId )
{
	int idxSt;

	// The Work Station index is the index for picks
	idxSt = ::GetConveyorObj()->GetWorkStationArrayIndx( m_CycleData.stationId );
	SYS_ASSERT( idxSt != NOT_AN_INDEX );

	// If the stat value for aPlaceData have already been cached...
	StStatCacheRec& rCacheRec = m_StatCacheRecs[ aIdxPd];
	
	// Cache the global stat values so the local copy can be updated during the cycle
	// without the MMI receiving partial results.
	rCacheRec.m_pDestHead = NULL;
	rCacheRec.m_pDestNozzle = NULL;
	rCacheRec.m_pDestFeeder = NULL;

	// Save the pointer to the destination global stat value.
	CGantry* pGantry		= aPD.pHead->Gantry();
	SYS_ASSERT( pGantry != NULL );
	int			spindle		= aPD.pHead->GetSpindle();	// One(1) based
	GantryID	gantryId	= pGantry->Unit();			// One(1) based
	SectionID	sectionId	= pGantry->GetSectionID();	// One(1) based
	SYS_ASSERT( sectionId > 0 && sectionId <= SYS_MAX_GANTRY_PER_SECTION );
	int gantryIdx = ::GetGantryIdxbySection( gantryId, sectionId ); // One(1) based

	rCacheRec.m_pDestHead = &g_Statistics.statHead[ sectionId-1 ][idxSt][ gantryIdx-1 ][ spindle-1 ];
	//rCacheRec.m_pDestHead = &g_Statistics.statHead[ aPD.head - 1];
	
	// Save the pointer to the destination global stat value.
	rCacheRec.m_pDestNozzle
			= &g_Statistics.statNozzle[idxSt][ aPD.pHead->GetNozzle().Anc()-1][ aPD.pHead->GetNozzle().HoleNum()-1];

	// CFeederId might be undefined if the pick & place cycle was aborted before a pick executed.
	if ( aPickedFeederId.IsDefined())
	{
		switch( aPickedFeederId.Type() )
		{
		case eFEEDTYPE_TAPE:
			rCacheRec.m_pDestFeeder = &g_Statistics.statTapeFeeder[ idxSt][ aPickedFeederId.Unit()-1][ aPickedFeederId.No()-1];
			break;
		case eFEEDTYPE_STICK:
			rCacheRec.m_pDestFeeder = &g_Statistics.statStickFeeder[ idxSt][ aPickedFeederId.Unit()-1][ aPickedFeederId.No()-1 ];
			break;
		case eFEEDTYPE_TRAY:
			rCacheRec.m_pDestFeeder	= &g_Statistics.statTrayFeeder[ idxSt][aPickedFeederId.Unit()-1][aPickedFeederId.No()-1][aPickedFeederId.ExtNo()-1];
			break;
		default:
			g_StateReport.ReportErrorEx2( eMEC_FEED_TYPE_ERROR, eERR_LEVEL_FREEZE, eMOTOR_OK, m_SectionID, eBOTH,
									0, HeadNone, 0, aPickedFeederId, eNOZZLE_TYPE_NONE, 0, 0, __LINE__, aPickedFeederId.Type() );
		}
	}

	// Clear the cache to accumulate counts;
	rCacheRec.m_Head.ClearAll();
	rCacheRec.m_Nozzle.ClearAll();
	rCacheRec.m_Feeder.ClearAll();
}



void
CSeqCycle::UpdateGlobalCycleStatValues( )
{
	CStatisticsInfo::CStatGetAndPutRes crit( g_StatisticsInfo );
	TFdrIDtoCache	fdrIDtoCache;

	for (int i=0; i<m_CycleData.nCount; i++)
	{
		StPlaceData& pd = m_CycleData.placeData[ i ];
		// Skip if step is virtual pick step.
		if ( pd.profile.IsVirtualPick() )
			continue;

		StStatCacheRec& rCacheRec = m_StatCacheRecs[i];

		// Accumulate Telemetry DATA for the updated statistics feeder errors
		//----------------
		CFeederId hdPickFeederID	= pd.pHead->GetPickedFeeder();
		CFeederId pickFeederID		= pd.pickedFeederID;
		if ( !pickFeederID.IsDefined() )
		{
			if ( pd.placedFeederID.IsDefined() )
				pickFeederID = pd.placedFeederID;
			else
				pickFeederID = hdPickFeederID;
		}
		StStatisticsCycle& cache = fdrIDtoCache[ pickFeederID];
		cache.Accumulate( rCacheRec.m_Feeder);
		//----------------

		// Accumulate the cache values back to the global statistics.
		AccumulateCycleData(rCacheRec.m_Head,   rCacheRec.m_pDestHead,
							rCacheRec.m_Nozzle, rCacheRec.m_pDestNozzle,
							rCacheRec.m_Feeder, rCacheRec.m_pDestFeeder );
		rCacheRec.m_Head.ClearAll();
		rCacheRec.m_Nozzle.ClearAll();
		rCacheRec.m_Feeder.ClearAll();

//AutoHeadClean
		CAutoHeadCleanMgr*	pCleaner = GetAutoHeadCleanMgr();
		if ( pCleaner )
		{
			pCleaner->IncrementPicks(pd.head);
		}
//AutoHeadClean-End
	}

	// Telemetry the accumulated data from above.
	for (POS pos=fdrIDtoCache.GetStartPosition(); pos!=NULL;)
	{
		CFeederId feederId;
		StStatisticsCycle& rCache = fdrIDtoCache.GetNextAssoc( pos, feederId);

		TelemetryPickErrors( "GLOBAL Errors=", feederId, rCache);
	}
}


int
CSeqCycle::TelemetryPickErrors( const char* apszTypeCount, CFeederId aFeederId, StStatisticsCycle& arStatCycle, bool abSuppressEmptyUpdates/*=true*/)
{
	char ctxt[50];
	long errCount = arStatCycle.pickMissVacPickCount +
					arStatCycle.pickMissPartNGCount +
					arStatCycle.pickMissVisShiftPickCount +
					arStatCycle.pickMissVacPrePlaceCount +
					arStatCycle.pickMissSvsPickCount +
					arStatCycle.pickMissSvsPartNGCount;
	if ( abSuppressEmptyUpdates  &&  errCount==0)
		return 0;

	Telemetry( EV_CLASSHEAD, 0, "Stat:%s %s%d  VacPk=%d PrtNG=%d ShftPk=%d VPrePl=%d SvsPk=%d SvsNG=%d",
								aFeederId.Format( ctxt, sizeof(ctxt)),
								apszTypeCount,
								errCount,
								arStatCycle.pickMissVacPickCount,
								arStatCycle.pickMissPartNGCount,
								arStatCycle.pickMissVisShiftPickCount,
								arStatCycle.pickMissVacPrePlaceCount,
								arStatCycle.pickMissSvsPickCount,
								arStatCycle.pickMissSvsPartNGCount);
	return errCount;
}



void
CSeqCycle::AccumulateCycleData( 
						const StStatisticsCycle& arCacheHead,   StStatisticsCycle* apGlobHead, 
						const StStatisticsCycle& arCacheNozzle, StStatisticsCycle* apGlobNozzle, 
						const StStatisticsCycle& arCacheFeeder, StStatisticsCycle* apGlobFeeder )
{
//	CGetAndPutRes crit( m_Critical );
//
	if ( apGlobHead)
		apGlobHead->Accumulate( arCacheHead);
	if ( apGlobNozzle)
		apGlobNozzle->Accumulate( arCacheNozzle);
	if ( apGlobFeeder)
		apGlobFeeder->Accumulate( arCacheFeeder);
}


long
CSeqCycle::GetMisPickCount(CFeederId aID)
{
	long misPickCount = 0;
	eFeederType	feedType = aID.Type();

	if (feedType == eFEEDTYPE_TAPE)
		misPickCount = GetMispickTape(aID);
	else if (feedType == eFEEDTYPE_TRAY)
		misPickCount = GetMispickTray(aID);
	else if (feedType == eFEEDTYPE_STICK)
		misPickCount = GetMispickStick(aID);
	else
		misPickCount = 0;

	return misPickCount;
}

long
CSeqCycle::GetPickUpMissCount(CFeederId aID)
{
	long pickMissCount = 0;
	eFeederType	feedType = aID.Type();

	if (feedType == eFEEDTYPE_TAPE)
		pickMissCount = GetPickUpMissCountTape(aID);
	else if (feedType == eFEEDTYPE_TRAY)
		pickMissCount = GetPickUpMissCountTray(aID);
	else if (feedType == eFEEDTYPE_STICK)
		pickMissCount = GetPickUpMissCountStick(aID);
	else
		pickMissCount = 0;

	return pickMissCount;
}

long
CSeqCycle::GetPartNGCount(CFeederId aID)
{
	long partNgCount = 0;
	eFeederType	feedType = aID.Type();

	if (feedType == eFEEDTYPE_TAPE)
		partNgCount = GetPartNGCountTape(aID);
	else if (feedType == eFEEDTYPE_TRAY)
		partNgCount = GetPartNGCountTray(aID);
	else if (feedType == eFEEDTYPE_STICK)
		partNgCount = GetPartNGCountStick(aID);
	else
		partNgCount = 0;

	return partNgCount;
}

long&
CSeqCycle::GetMispickTape( CFeederId aID)
{
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_StatisticsMispicksTape));
	SYS_ASSERT( slot<_countof(m_StatisticsMispicksTape[0]));
	return m_StatisticsMispicksTape[ unit][ slot];
}

long&
CSeqCycle::GetMispickStick( CFeederId aID)
{
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_StatisticsMispicksStick));
	SYS_ASSERT( slot<_countof(m_StatisticsMispicksStick[0]));
	return m_StatisticsMispicksStick[ unit][ slot];
}

long&
CSeqCycle::GetMispickTray( CFeederId aID)
{
	int unit = aID.Unit();  int slot=aID.No();	int extNo=aID.ExtNo();
	SYS_ASSERT( unit<_countof(m_StatisticsMispicksTray));
	SYS_ASSERT( slot<_countof(m_StatisticsMispicksTray[0]));
	SYS_ASSERT( extNo<_countof(m_StatisticsMispicksTray[0][0]));
	return m_StatisticsMispicksTray[ unit][ slot][ extNo];
}


long&
CSeqCycle::GetPickUpMissCountTape( CFeederId aID)
{
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_PickUpMissCountTape));
	SYS_ASSERT( slot<_countof(m_PickUpMissCountTape[0]));
	return m_PickUpMissCountTape[ unit][ slot];
}

long&
CSeqCycle::GetPickUpMissCountStick(CFeederId aID)
{
	int unit = aID.Unit();  int slot = aID.No();
	SYS_ASSERT(unit < _countof(m_PickUpMissCountStick));
	SYS_ASSERT(slot < _countof(m_PickUpMissCountStick[0]));
	return m_PickUpMissCountStick[unit][slot];
}

long&
CSeqCycle::GetPickUpMissCountTray(CFeederId aID)
{
	int unit = aID.Unit();  int slot = aID.No();	int extNo = aID.ExtNo();
	SYS_ASSERT(unit < _countof(m_PickUpMissCountTray));
	SYS_ASSERT(slot < _countof(m_PickUpMissCountTray[0]));
	SYS_ASSERT(extNo < _countof(m_PickUpMissCountTray[0][0]));
	return m_PickUpMissCountTray[unit][slot][extNo];
}


long&
CSeqCycle::GetPartNGCountTape( CFeederId aID)
{
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_PartNGCountTape));
	SYS_ASSERT( slot<_countof(m_PartNGCountTape[0]));
	return m_PartNGCountTape[ unit][ slot];
}

long&
CSeqCycle::GetPartNGCountStick(CFeederId aID)
{
	int unit = aID.Unit();  int slot = aID.No();
	SYS_ASSERT(unit < _countof(m_PartNGCountStick));
	SYS_ASSERT(slot < _countof(m_PartNGCountStick[0]));
	return m_PartNGCountStick[unit][slot];
}

long&
CSeqCycle::GetPartNGCountTray(CFeederId aID)
{
	int unit = aID.Unit();  int slot = aID.No();	int extNo = aID.ExtNo();
	SYS_ASSERT(unit < _countof(m_PartNGCountTray));
	SYS_ASSERT(slot < _countof(m_PartNGCountTray[0]));
	SYS_ASSERT(extNo < _countof(m_PartNGCountTray[0][0]));
	return m_PartNGCountTray[unit][slot][extNo];
}


TStatIncRecList&
CSeqCycle::GetStatisticsIncListTape( CFeederId aID)
{
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_StatisticsIncRecTape));
	SYS_ASSERT( slot<_countof(m_StatisticsIncRecTape[0]));
	return m_StatisticsIncRecTape[ unit][ slot];
}


TStatIncRecList&
CSeqCycle::GetStatisticsIncListStick( CFeederId aID)
{
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_StatisticsIncRecStick));
	SYS_ASSERT( slot<_countof(m_StatisticsIncRecStick[0]));
	return m_StatisticsIncRecStick[ unit][ slot];
}


TStatIncRecList&
CSeqCycle::GetStatisticsIncListTray( CFeederId aID)
{
	int unit = aID.Unit();  int slot=aID.No();	int extNo=aID.ExtNo();
	SYS_ASSERT( unit<_countof(m_StatisticsIncRecTray));
	SYS_ASSERT( slot<_countof(m_StatisticsIncRecTray[0]));
	SYS_ASSERT( extNo<_countof(m_StatisticsIncRecTray[0][0]));
	return m_StatisticsIncRecTray[ unit][ slot][ extNo];
}


bool&
CSeqCycle::GetNonStopRetryTape( CFeederId aID)
{
	/* This function must be called about Tape Feeder. */
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_NonStopRetryTape));
	SYS_ASSERT( slot<_countof(m_NonStopRetryTape[0]));
	return m_NonStopRetryTape[ unit][ slot];
}

int&
CSeqCycle::GetPickUpTapeCount( CFeederId aID)
{
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_PickUpTapeCount));
	SYS_ASSERT( slot<_countof(m_PickUpTapeCount[0]));
	return m_PickUpTapeCount[ unit][ slot];
}

ePartFailReason&
CSeqCycle::GetPickUpMissTapeRate( CFeederId aID, int aIndex )
{
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_PickUpMissTapeRateCount));
	SYS_ASSERT( slot<_countof(m_PickUpMissTapeRateCount[0]));
	SYS_ASSERT( aIndex >=0 );
	return m_PickUpMissTapeRateCount[ unit][ slot][aIndex];
}

void
CSeqCycle::SetPickUpMissTapePocketTeach( CFeederId aID, bool aPocketTeach)
{
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_bPickUpMissTapePocketTeach));
	SYS_ASSERT( slot<_countof(m_bPickUpMissTapePocketTeach[0]));
	m_bPickUpMissTapePocketTeach[ unit][ slot] = aPocketTeach;
}

int&
CSeqCycle::GetPickUpMissTapeRateNextCount( CFeederId aID)
{
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_PickUpMissTapeRateNextIndex));
	SYS_ASSERT( slot<_countof(m_PickUpMissTapeRateNextIndex[0]));
	return m_PickUpMissTapeRateNextIndex[ unit][ slot];
}

int&
CSeqCycle::GetPickUpMissTapeRateCheckCount( CFeederId aID)
{
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_PickUpMissTapeRateCheckIndex));
	SYS_ASSERT( slot<_countof(m_PickUpMissTapeRateCheckIndex[0]));
	return m_PickUpMissTapeRateCheckIndex[ unit][ slot];
}

int&
CSeqCycle::GetPickUpMissTapeRateCheckCount2( CFeederId aID)
{
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_PickUpMissTapeRateCheckIndex2));
	SYS_ASSERT( slot<_countof(m_PickUpMissTapeRateCheckIndex2[0]));
	return m_PickUpMissTapeRateCheckIndex2[ unit][ slot];
}

bool
CSeqCycle::GetPickUpMissTapePocketTeach( CFeederId aID)
{
	int unit = aID.Unit();  int slot=aID.No();
	SYS_ASSERT( unit<_countof(m_bPickUpMissTapePocketTeach));
	SYS_ASSERT( slot<_countof(m_bPickUpMissTapePocketTeach[0]));
	return m_bPickUpMissTapePocketTeach[ unit][ slot];
}


void
CSeqCycle::ClearMispickTape( CFeederId aID)
{
	long& misPickCount = GetMispickTape( aID);
	misPickCount = 0;
	ClearPickUpMissCountTape( aID);
	ClearPartNGCountTape ( aID);
}


void
CSeqCycle::ClearMispickStick( CFeederId aID)
{
	long& misPickCount = GetMispickStick( aID);
	misPickCount = 0;
	ClearPickUpMissCountStick( aID);
	ClearPartNGCountStick( aID);
}


void
CSeqCycle::ClearMispickTray( CFeederId aID)
{
	long& misPickCount = GetMispickTray( aID);
	misPickCount = 0;
	ClearPickUpMissCountTray( aID);
	ClearPartNGCountTray( aID);
}


void
CSeqCycle::ClearPickUpMissCountTape( CFeederId aID)
{
	long& pickUpMissCount = GetPickUpMissCountTape( aID);
	pickUpMissCount = 0;

	CTapeFeeder* pTape = ::GetTapeFeeder( aID );
	SYS_ASSERT( pTape )

	CStepRetryFeedingStd::StopRetryFeeding( pTape);
}

void
CSeqCycle::ClearPickUpMissCountStick(CFeederId aID)
{
	long& pickUpMissCount = GetPickUpMissCountStick(aID);
	pickUpMissCount = 0;
}

void
CSeqCycle::ClearPickUpMissCountTray(CFeederId aID)
{
	long& pickUpMissCount = GetPickUpMissCountTray(aID);
	pickUpMissCount = 0;
}


void
CSeqCycle::ClearPartNGCountTape( CFeederId aID)
{
	long& misPickCount = GetPartNGCountTape( aID);
	misPickCount = 0;
}

void
CSeqCycle::ClearPartNGCountStick(CFeederId aID)
{
	long& misPickCount = GetPartNGCountStick(aID);
	misPickCount = 0;
}

void
CSeqCycle::ClearPartNGCountTray(CFeederId aID)
{
	long& misPickCount = GetPartNGCountTray(aID);
	misPickCount = 0;
}


void
CSeqCycle::ClearPickUpMissTapeRate( CFeederId aID)
{
    int& pickCountMax = GetPickUpTapeCount(aID);
	for ( int i = 0; i <eMAX_PICK_RATE_COUNT; i++ )
	{
		ePartFailReason& misPickCount = GetPickUpMissTapeRate(aID, i);
		misPickCount = ePART_FAIL_NONE;
	}
	pickCountMax = 0;
}

void
CSeqCycle::ClearPickUpMissTapeRateNextCount( CFeederId aID)
{
	int& nextIndex = GetPickUpMissTapeRateNextCount(aID);
	nextIndex = 0;
}

void
CSeqCycle::ClearPickUpMissTapeRateCheckCount( CFeederId aID)
{
	int& checkIndex = GetPickUpMissTapeRateCheckCount(aID);
	checkIndex = 0;
}

void
CSeqCycle::ClearPickUpMissTapeRateCheckCount2( CFeederId aID)
{
	int& checkIndex = GetPickUpMissTapeRateCheckCount2(aID);
	checkIndex = 0;
}

void
CSeqCycle::IncMispickTape( CFeederId aID, ePartFailReason aPartFailReason )
{
	long& misPickCount = GetMispickTape( aID);
	misPickCount++;
	if ( ( aPartFailReason == ePART_FAIL_VAC_PICK ) || ( aPartFailReason == ePART_FAIL_SVS_PICK ) || (aPartFailReason == ePART_FAIL_VAC_PRE_PLACE))
		IncPickUpMissCountTape( aID );
	else if ( ( aPartFailReason == ePART_FAIL_VISION ) || ( aPartFailReason == ePART_FAIL_SVS_VISION ) || (aPartFailReason == ePART_FAIL_LEAD_SCAN))
	{
		IncPartNGCountTape( aID );
		//ClearPickUpMissCountTape( aID );
	}
	else
		ClearPickUpMissCountTape( aID );
}


void
CSeqCycle::IncMispickStick( CFeederId aID, ePartFailReason aPartFailReason)
{
	long& misPickCount = GetMispickStick( aID);
	misPickCount++;
	if ((aPartFailReason == ePART_FAIL_VAC_PICK) || (aPartFailReason == ePART_FAIL_SVS_PICK))
		IncPickUpMissCountStick(aID);
	else if ((aPartFailReason == ePART_FAIL_VISION) || (aPartFailReason == ePART_FAIL_SVS_VISION) || (aPartFailReason == ePART_FAIL_LEAD_SCAN))
	{
		IncPartNGCountStick(aID);
		//ClearPickUpMissCountStick(aID);
	}
	else
		ClearPickUpMissCountStick(aID);
}


void
CSeqCycle::IncMispickTray( CFeederId aID, ePartFailReason aPartFailReason)
{
	long& misPickCount = GetMispickTray( aID);
	misPickCount++;
	if ((aPartFailReason == ePART_FAIL_VAC_PICK) || (aPartFailReason == ePART_FAIL_SVS_PICK))
		IncPickUpMissCountTray(aID);
	else if ((aPartFailReason == ePART_FAIL_VISION) || (aPartFailReason == ePART_FAIL_SVS_VISION) || (aPartFailReason == ePART_FAIL_LEAD_SCAN))
	{
		IncPartNGCountTray(aID);
		//ClearPickUpMissCountTray(aID);
	}
	else
		ClearPickUpMissCountTray(aID);
}


void
CSeqCycle::IncPickUpMissCountTape( CFeederId aID)
{
	long& pickUpMissCount = GetPickUpMissCountTape( aID);
	pickUpMissCount++;
}

void
CSeqCycle::IncPickUpMissCountStick(CFeederId aID)
{
	long& pickUpMissCount = GetPickUpMissCountStick(aID);
	pickUpMissCount++;
}

void
CSeqCycle::IncPickUpMissCountTray(CFeederId aID)
{
	long& pickUpMissCount = GetPickUpMissCountTray(aID);
	pickUpMissCount++;
}


void
CSeqCycle::IncPartNGCountTape( CFeederId aID)
{
	long& misPickCount = GetPartNGCountTape( aID);
	misPickCount++;
}

void
CSeqCycle::IncPartNGCountStick(CFeederId aID)
{
	long& misPickCount = GetPartNGCountStick(aID);
	misPickCount++;
}

void
CSeqCycle::IncPartNGCountTray(CFeederId aID)
{
	long& misPickCount = GetPartNGCountTray(aID);
	misPickCount++;
}

void
CSeqCycle::IncPickUpTapeCount( CFeederId aID)
{
	int& pickCountMax = GetPickUpTapeCount( aID);
	if ( pickCountMax < eMAX_PICK_RATE_COUNT)
		pickCountMax++;
}

void
CSeqCycle::IncPickUpMissTapeRateNextCount( CFeederId aID)
{
	int& nextIndex = GetPickUpMissTapeRateNextCount(aID);
	nextIndex++;

	if ( nextIndex == eMAX_PICK_RATE_COUNT )
		ClearPickUpMissTapeRateNextCount(aID);
}

void
CSeqCycle::IncPickUpMissTapeRateCheckCount( CFeederId aID)
{
	int& checkIndex = GetPickUpMissTapeRateCheckCount(aID);
	checkIndex++;
	
	if ( checkIndex == eMAX_PICK_RATE_COUNT )
		ClearPickUpMissTapeRateCheckCount(aID);
}

void
CSeqCycle::IncPickUpMissTapeRateCheckCount2( CFeederId aID)
{
	int& checkIndex = GetPickUpMissTapeRateCheckCount2(aID);
	checkIndex++;
	
	if ( checkIndex == eMAX_PICK_RATE_COUNT )
		ClearPickUpMissTapeRateCheckCount2(aID);
}

void
CSeqCycle::SetPickUpMissTapeRate( CFeederId aID, int aIndex, ePartFailReason aPlaceStatus)
{
	ePartFailReason& misPickCount = GetPickUpMissTapeRate( aID, aIndex);
	misPickCount = aPlaceStatus;
}

void
CSeqCycle::ClearStatisticsIncTape( CFeederId aID)
{
	TStatIncRecList& incList = GetStatisticsIncListTape( aID);
	incList.RemoveAll( );
}

void
CSeqCycle::ClearStatisticsIncTapeEx( CFeederId aID, int aPocketIndex )
{
	TStatIncRecList& incList = GetStatisticsIncListTape( aID);
	if ( incList.IsEmpty() )	return;

	for ( int i = incList.GetCount()-1; i >= 0; i-- )
	{
		StStatIncRec& rec = incList[i];
		if ( rec.pocketIndex == aPocketIndex )
			incList.RemoveAt(i);
	}
}


void
CSeqCycle::ClearStatisticsIncStick( CFeederId aID)
{
	TStatIncRecList& incList = GetStatisticsIncListStick( aID);
	incList.RemoveAll( );
}


void
CSeqCycle::ClearStatisticsIncTray( CFeederId aID)
{
	TStatIncRecList& incList = GetStatisticsIncListTray( aID);
	incList.RemoveAll( );
}


eStatus
CSeqCycle::AddStatisticsIncRecTape( CFeederId aID, const StStatIncRec& aIncRec)
{
	TStatIncRecList& incList = GetStatisticsIncListTape( aID);

	if ( g_SysConst.IsSupportMultiPocketErrRateManagement() && 
		 CTapeFeeder::IsMultiPocketTapeFeeder( aID ) )
	{
		if (incList.IsFull())
		{
			//<2011.1.31>
			//Remove from old one if it is full
			//Remove 3set because they are same group.
			for ( int i=0; i < 3; i++ )
				incList.RemoveAt(0);
			
			//return eNG;
		}
#ifdef WIN_SIM
		if ( aIncRec.incDev == eSTAT_INC_HEAD )
		{
			char szStr[100];

			aID.Format( szStr,sizeof(szStr));
			TRACE("AddStatisticsInc H:%d, F:%s, P:%d\n\r",aIncRec.headID, szStr, aIncRec.pocketIndex);
		}
#endif
	}
	else
	{
		if ( incList.GetCount() >= SINGLE_POCKET_REEL_REC_COUNT )
			return eNG;
	}
	incList.Append( aIncRec);
	return eOK;
}


eStatus
CSeqCycle::AddStatisticsIncRecStick( CFeederId aID, const StStatIncRec& aIncRec)
{
	TStatIncRecList& incList = GetStatisticsIncListStick( aID);

	if (incList.IsFull())
		return eNG;
	incList.Append( aIncRec);
	return eOK;
}


eStatus
CSeqCycle::AddStatisticsIncRecTray( CFeederId aID, const StStatIncRec& aIncRec)
{
	TStatIncRecList& incList = GetStatisticsIncListTray( aID);
	
	if (incList.IsFull())
		return eNG;
	incList.Append( aIncRec);
	return eOK;
}

// Updates all required pickup statistic values when the pickup attempted, successful or not.
void
CSeqCycle::UpdatePickStatistics( )
{
	if ( g_SysConst.IsSupportMultiPocketErrRateManagement() )
		UpdatePickStatisticsMultiPocket();
	else
		UpdatePickStatisticsSinglePocket();
}

void
CSeqCycle::UpdatePickStatisticsSinglePocket( )
{
//	Pick Statistics Rules for TAPE
//	------------------------------
//	?? Pick...
//	1. SKIP un-inspected parts, they should NOT BE COUNTED AT ALL.
//
//	OK Pick...
//	2. No Issues
//	   a. INC Stat PICKED Cache, LOG Stat Cache.
//	   b. ERASE LOG Stat Cache.  (effectively this commits the counts)
//	   c. CLEAR ALL miss pick counts.
//
//	NG Pick...
//	3. NG Pick, ~SpliceAreaPick
//	   a. INC Stat PICKED Cache, LOG Stat Cache.
//	   b. misPick++
//	   c. misCount++,   if "vac" fail.
//	   d. misNGCount++, if inspect fail. And set misCount=0, And Stop Retry Feeding
//	   e. INC Stat BAD PICK Cache, LOG Stat Cache.
//
//	4. NG Pick, ERROR Feeder
//	   a. DEC Stat Cache (via LOG), ERASE LOG Stat Cache.
//
//	5. NG Pick, OK Feeder, ~SpliceAreaPick, RETRY==LIMIT, RetryFeeding ENABLED
//	   a. START Retry Feeding
//	   b. DEC Stat Cache (via LOG), ERASE LOG Stat Cache.
//	   (hitting the retry limit INVOKES the retry feeding solution)
//
//	6. NG Pick, OK Feeder, ~SpliceAreaPick, OVER RETRY
//	   a. DEC Stat Cache (via LOG), ERASE LOG Stat Cache.
//	   (hitting the retry limit ERASES the counts and ultimately makes an ERROR Feeder)
//
//
	TFdrIDtoCache	fdrIDtoPickBad;
	TFdrIDtoCache	fdrIDtoStatErr;
	TPDOrder		pickDataOrder;
	bool			bUseMissPickTapeRate  = g_FacDebug.IsUsePickUpMissTapeRate();
	bool			bUseMissPickTapeRate2 = g_FacDebug.IsUsePickUpMissTapeRate2();

	// Apply any pick statistics failures in the pick order
	GetPickOrder( pickDataOrder);

	// If there were no picks in this cycle, then use the place order
	TPDOrder placeDataOrder;
	GetPlaceOrder( placeDataOrder);

	// Use all place information for updating pick count if count is different.
	if ( pickDataOrder.GetCount() < placeDataOrder.GetCount() )
		GetPlaceOrder( pickDataOrder);

	// Analyze the cycle in the pick order (not the place order).
	//
	for (int i=0; i<pickDataOrder.GetCount(); i++)
	{
		int idxPd = pickDataOrder[i];
		StPlaceData& pd = m_CycleData.placeData[ idxPd];
		long retryLimit = pd.profile.common.retryCount;
		long retryNgLimit = pd.profile.common.retryNgCount;
		bool bVacVerify = (pd.profile.handling.bVacuumVerify && g_SysConst.IsSysPickVacCheck());

		if ( pd.profile.IsVirtualPick() )
// TAPE Rule #1: ?? Pick
			continue;

		//
		//IMPORTANT NOTE: <SAMEX 20070723>
		//	Do NOT process any pick count if a head still has a part, next time
		//	this pick count can be reset by operator before placing or dumping
		//	and makes 'pick count < place count' situation.
		//
		double seatedHeight = pd.pHead->GetSeatedHeight();
		//	DO NOT count as pick or error until its placed or dumped if part is still on the head.
		if ( seatedHeight != 0.0  &&  seatedHeight != NaN )	// Part is on the head.
		{
			// Part is not failed and not aligned state, this means part
			// will be processed on next cycle, so skip it.
			ePartFailReason partFailReason = pd.pHead->GetPartFailReason();
			ePartVector		partVecStatus  = pd.pHead->GetPartAlignedStatus();
			if (	partFailReason == ePART_FAIL_NONE 		// Not aligned yet, next time this part will be re-aligned.
				&&	partVecStatus  == ePART_VECTOR_UNALIGNED)
			{
				if ( m_PrefetchOp == eCYCLE_PREFETCH_ALIGN && m_LastStepType == eSTEP_ALIGN )
				{
					eStatus state;
					state = WaitAlignResult(pd.head, 3000);
					partFailReason = pd.pHead->GetPartFailReason();
					partVecStatus  = pd.pHead->GetPartAlignedStatus();

					if (	partFailReason == ePART_FAIL_NONE
						&&	partVecStatus  == ePART_VECTOR_UNALIGNED )
					{
// TAPE Rule #1: ?? Pick
						continue;
					}
					else if ( partVecStatus == ePART_VECTOR_NG )
					{
						pd.SetStatus(ePLACE_FAILED, THIS_FILE, __LINE__);
					}
				}
				else
				{
// TAPE Rule #1: ?? Pick
					continue;
				}
			}

			//NOTE: THE reason why commented...
			//		If part is already aligned with failure and got Error(EM or FREEZE), below will
			//		make error count is not counted on next time also. so, commented out.
			//x // Cycle will not be processed anymore including dump, do NOT count up as pick.
			//x if ( m_ReturnStatus >= eCYCLE_STOPPED )			// Part will not be dumped yet.
			//x 	continue;
		}

		bool bPickStisticsCounted	= pd.pHead->IsPickStatisticsCounted();
		CFeederId hdPickFeederID	= pd.pHead->GetPickedFeeder();
		CFeederId pickFeederID		= pd.pickedFeederID;

		//ADDED ON 20070511 - FIXING pick count and place count doesn't match.
		//Recursive update pick up count if this was not applied yet
		//due to manual pick up and then start auto production.
		//	If placement was successful, 'pd.placedFeederID' should be defined.
		//	else 'pickFeederID' or 'hdPickedFeederID' should be defined.
		if ( !pickFeederID.IsDefined() )
		{	//Part picked manually first or picked during previous production pick sequence
			//and stopped with EM, assign picked feeder ID.

			CFeederId placedFeederID = pd.placedFeederID;
			if ( placedFeederID.IsDefined() )	// successfully placed after manual pick up
				pickFeederID = placedFeederID;
			else								// failed to place after manual pick up
				pickFeederID = hdPickFeederID;
		}

		//THIS 'Not defined' situation can be happened, when part is picked manually and dumped.
		//And then start production, but hit the EM before picking up part.
		//SYS_ASSERT( pickFeederID.IsDefined() );

		//<20070723 SAMEX>
		//Fixed do NOT count as pick/vision error for tray return part because this part
		//can be re-used later. (Most of tray part failure is mis-teaching tray/part
		//or vision inspection failure due to vision algorithm.) 
		//#1. Pick count should be increased and dump count will be increased by MMI calculation.
		//#2. Keep to increasing RT managed mis-pick count for detecting retry over stop.
		bool bSkipErrorCountForReturnTray = false;
		bool bSkipErrorCountForReturnTrayVisionErr = false;
		bool bSkipErrorCountForSingleRetryCount = false;
		bool bSkipErrorCountEver = false;
		bool bIsPickErrorNoRetry = false;
		if ( pickFeederID.IsDefined())
		{
			ePartFailReason partFailReason = pd.pHead->GetPartFailReason();
			
			if ( pickFeederID.Type() == eFEEDTYPE_TRAY)
			{
				CTray* pFeeder = ::GetTray( pickFeederID);
				DumpID dumpId = pFeeder ? pFeeder->GetDumpID() : (-1);
				if ( dumpId == 0 )	// Return to tray.
				{
					bSkipErrorCountForReturnTray = !g_FacDebug.IsSupportErrorCountForReturnToTray();

					if ( g_FacDebug.IsSupportErrorCountForTrayOnlyVisionErr())
						if ( partFailReason == ePART_FAIL_VISION || partFailReason == ePART_FAIL_VIS_SHIFT_PICK 
							|| partFailReason == ePART_FAIL_SVS_VISION || partFailReason == ePART_FAIL_LEAD_SCAN)
							bSkipErrorCountForReturnTrayVisionErr = true;
				}
			}

			bIsPickErrorNoRetry = (g_FacDebug.IsPickupErrorNoRetry() && (retryLimit == 1 || retryNgLimit == 1));
			bSkipErrorCountForSingleRetryCount = g_FacDebug.IsSupportIncErrorCountforSingleRetryCount();
			bSkipErrorCountEver = g_FacDebug.IsSupportIncErrorCountEver();
			//Detect crash of Option
			SYS_ASSERT(!( bSkipErrorCountForSingleRetryCount && bSkipErrorCountEver ));
		}
		else
		{
			//NOTE: This can be happened if this place is 'Virtual Pick' part.
			// Debug - do nothing line
			SYS_ASSERT( !pickFeederID.IsDefined());
		}

		// Only collect pick statistics on executed picks, not skipped ones.
		// Update statistics with the pickup attempt.
		if ( pickFeederID.IsDefined())
		{
			if ( !bPickStisticsCounted )
			{
				ePartFailReason partFailReason = pd.pHead->GetPartFailReason();
				bool bSmartSplicing = pd.pHead->IsSpliceAreaPick()  &&  g_SysConst.IsUseSpliceZone();
				if ( partFailReason != ePART_FAIL_SKIP_PICK && (!bSmartSplicing || pd.status==ePLACE_OK))
				{
// TAPE Rule #2a: OK Pick, No Issues:		INC Stat PICKED Cache, LOG Stat Cache.
// TAPE Rule #3a: NG Pick, ~SplicePickArea:	INC Stat PICKED Cache, LOG Stat Cache.
					// Accumulate statistics for this placement...
					UpdatePickStatistics( idxPd, pd, pickFeederID, pd.pHead->GetNozzle());
				}

				//ADDED ON 20070511 - FIXING pick count and place count doesn't match.
				pd.pHead->SetPickStatisticsCounted( true);

				if ( (bUseMissPickTapeRate || bUseMissPickTapeRate2) && pickFeederID.Type()==eFEEDTYPE_TAPE )
				{
					int nextIndex = GetPickUpMissTapeRateNextCount(pickFeederID); 
					
					IncPickUpTapeCount(pickFeederID);
					int pickCnt = GetPickUpTapeCount(pickFeederID);

					SetPickUpMissTapeRate(pickFeederID, nextIndex, pd.pHead->GetPartFailReason());
					CheckFeederMissPickRate(pd, pickFeederID); 	

					if ( pickCnt >= g_FacDebug.GetPickUpMissTapeRateMaxCount())
						IncPickUpMissTapeRateCheckCount(pickFeederID);
					
					if ( pickCnt >= g_FacDebug.GetPickUpMissTapeRateMaxCount2())
						IncPickUpMissTapeRateCheckCount2(pickFeederID);

					IncPickUpMissTapeRateNextCount(pickFeederID);

				}		
			}
		}
		else
		{
			//NOTE: This can be happened if this place is 'Virtual Pick' part.
			// Debug - do nothing line
			pickFeederID = pd.pickedFeederID;
		}

		// On success, only reset mispicks on cycles that picked from the feeder (not prefetched 
		//	 parts) because a prefetch cycle would have already reset the mispicks if no pick 
		//	 problems occurred.
		if ( pd.status==ePLACE_OK)
		{
			if ( pickFeederID.IsDefined())
			{
// TAPE Rule #2b,c:  OK Pick, No Issues:		ERASE LOG Stat Cache,  CLEAR ALL miss pick counts
				// A successful pick means that any prior mispicks accumulated should stand.
				// This is done by simply erasing the mispick counts that detect when a feeder
				// should enter an ERROR state rather than counting its mispicks.
				CRunOptions& rRunOptions = *::GetRunOptions(m_frRunningLane);

				if (bIsPickErrorNoRetry)
				{
					if (GetPickUpMissCount(pickFeederID) < retryLimit && GetPartNGCount(pickFeederID) < retryNgLimit)
						ResetMispicks(pickFeederID);					
				}
				else if (rRunOptions.IsPreDump())
				{
					if (!bPickStisticsCounted)
						ResetMispicks(pickFeederID);
				}
				else
					ResetMispicks(pickFeederID);
					
				if (pickFeederID.Type() == eFEEDTYPE_TAPE)
				{
					CTapeFeeder* pFeeder = ::GetTapeFeeder( pickFeederID );
					pFeeder->ResetContinuousRetryError();
				}
			}
			else
			{
				//NOTE: This can be happened if this place is 'Virtual Pick' part.
				// Debug - do nothing line
				SYS_ASSERT( !pickFeederID.IsDefined());
			}
		}
		// On failure, if this is not a prefetch cycle (i.e. pd.pickedFeederID is defined) the 
		//	 count the error.  If this IS a PREFETCH cycle (i.e. pd.pickedFeederID==0 but hdPickFeederID!=0)
		//   then count the error in this case too.
		else if ( (pd.status==ePLACE_FAILED  ||  pd.status==ePLACE_FAILSKIP)
			   && (pickFeederID.IsDefined()  ||  hdPickFeederID.IsDefined()) )
		{
			ePartFailReason partFailReason	= pd.pHead->GetPartFailReason();
			CNozzleId		hdNozzleID		= pd.pHead->GetNozzle();

			// If the placement didn't record a part being picked, but the CHead has a failed picked part
			//   then use the Head's picked feeder as the part source.
			if ( !pickFeederID.IsDefined())
				pickFeederID = hdPickFeederID;

			// If the part is rejected due to Vision or Vacuum then process the failure.
			// If the part was picked, but not analyzed, then it's PICKED and NOT PLACED.
			if ( partFailReason != ePART_FAIL_NONE && partFailReason != ePART_FAIL_SKIP_PICK)
			{
				bool bSmartSplicing = pd.pHead->IsSpliceAreaPick()  &&  g_SysConst.IsUseSpliceZone();
				if ( !bSmartSplicing)
				{
					if ( pickFeederID.IsDefined())
					{
// TAPE Rule #3b,c,d: NG Pick, ~SplicePickArea:			misPick++  or  misCount++  or  misNGCount++,misCount=0
						IncMispicks( pickFeederID, partFailReason );
						if ( pickFeederID.Type() == eFEEDTYPE_STICK )
						{
							CStickFeeder* pStick = ::GetStickFeeder(pickFeederID);
							SYS_ASSERT( pStick );
							//pStick->SetErrorOccurredOnce();
							pStick->SetStickerBankStatus(eLABEL_FAIL, THIS_FILE, __LINE__);
							SetErrorOccurredOnce( pickFeederID );
						}
					}

					//Fixed do NOT count as pick/vision error for tray return part. <20070723 SAMEX>
					if ( !bSkipErrorCountForReturnTray || bSkipErrorCountForReturnTrayVisionErr)
// TAPE Rule #3e: NG Pick, ~SplicePickArea:				INC Stat BAD PICK Cache, LOG Stat Cache.
						UpdatePickStatisticsBad( idxPd, pd, partFailReason, pickFeederID, hdNozzleID, fdrIDtoPickBad[ pickFeederID]);
				}

				// If the feeder is already in an error state such as eTAPE_ERROR, eSTICK_EMPTY, eTRAY_EMPTY.
				if ( IsFeederError( pickFeederID)  || IsFeederPartEmptyIT(pickFeederID))
				{
// TAPE Rule #4a: NG Pick, ERROR Feeder:				DEC Stat Cache (via LOG), ERASE LOG Stat Cache.
					// UNDO the accumulated "failed part" statistics
					// eSTAT_ERASE_ERROR_OR_IT_EMPTY
					if (!bSkipErrorCountEver)
						UpdatePickStatisticsErr( pickFeederID, &fdrIDtoStatErr[ pickFeederID]);
				}
				else
				{
					//Fixed do NOT decrease error count in case part is really picked and retry limit
					//is setted as one time. <20070723 SAMEX>
					bool bSkipUndo = false;
					if ((( bVacVerify  &&  partFailReason == ePART_FAIL_VISION )
						|| bSkipErrorCountForSingleRetryCount ) 
						&&  retryLimit == 1 )
						bSkipUndo = true;

					if (bIsPickErrorNoRetry)
						bSkipUndo = true;

					//<2009.10.21 SAMEX>
					//Increse error count even if retry count is over.
					//TO maintain correct data of failed pickup count.
					if ( bSkipErrorCountEver )
						bSkipUndo = true;

					//Fixed do NOT count as pick/vision error for tray return part, so do NOT
					//decrease error count for not increased count. <20070723 SAMEX>
					if ( bSkipErrorCountForReturnTray )
						bSkipUndo = true;

					// Lead Scan Fail Error is Always Count.
					if (partFailReason == ePART_FAIL_LEAD_SCAN)
						bSkipUndo = true;

					if ( !bSmartSplicing)
					{
// TAPE Rule #5a,b: NG Pick, OK Feeder, ~SplicePickArea, RETRY==LIMIT, RetryFeeding ENABLED
						//			START Retry Feeding,  DEC Stat Cache (via LOG), ERASE LOG Stat Cache
						// Check if the feeder has failed due to excessive retries.
						eStatus status = CheckFeederRetryExceeded( pickFeederID, retryLimit, pd.profile.common.retryNgCount, &fdrIDtoStatErr[ pickFeederID], pd.profile.common.height, true);
						if ( status == eNG  &&  !bSkipUndo)
						{
// TAPE Rule #6: NG Pick, OK Feeder, ~SplicePickArea, OVER RETRY:	DEC Stat Cache (via LOG), ERASE LOG Stat Cache
							// This report forces the feeder into an error state, so UNDO the accumulated "failed part" statistics
							// eSTAT_ERASE_NOTSPLICEAREA_RETRYEXCEEDED
							UpdatePickStatisticsErr( pickFeederID, &fdrIDtoStatErr[ pickFeederID]);
						}
					}
				}
			}
			else
			{
				// Debug - do nothing lines
				if (partFailReason != ePART_FAIL_NONE && partFailReason != ePART_FAIL_SKIP_PICK)
					SYS_ASSERT(0);

				if ( !pd.placedFeederID.IsDefined())
					continue;
			}
		}
		else if ( pd.status==ePLACE_FAILED  ||  pd.status==ePLACE_FAILSKIP)
		{
			// Debug - do nothing line
			SYS_ASSERT( pd.status==ePLACE_FAILED  ||  pd.status==ePLACE_FAILSKIP);
		}
		else if ( pd.status == ePLACE_FAILSTOP)
		{
			// Debug - do nothing line
			SYS_ASSERT( pd.status==ePLACE_FAILSTOP);
		}

	}

	POS pos;
	// Emit telemetry for the PickBad feeders
	for (pos=fdrIDtoPickBad.GetStartPosition(); pos!=NULL;)
	{
		CFeederId	feederId;
		StStatisticsCycle& rCache = fdrIDtoPickBad.GetNextAssoc( pos, feederId);
		TelemetryPickErrors( "PickBad, Errors=", feederId, rCache);
	}

	// Telemetry the cumulative changes
	for (pos=fdrIDtoStatErr.GetStartPosition(); pos!=NULL;)
	{
		CFeederId	feederId;
		StStatisticsCycle& rCache = fdrIDtoStatErr.GetNextAssoc( pos, feederId);
		TelemetryPickErrors( "GLOBAL Erase=", feederId, rCache);
	}
}


void
CSeqCycle::UpdatePickStatisticsMultiPocket( )
{
	TPDOrder		pickDataOrder;
	TFdrIDtoCache	fdrIDtoPickBad;
	TFdrIDtoCache	fdrIDtoStatErr;

	// Apply any pick statistics failures in the pick order
	GetPickOrder( pickDataOrder);

	// If there were no picks in this cycle, then use the place order
	TPDOrder placeDataOrder;
	GetPlaceOrder( placeDataOrder);

	// Use all place information for updating pick count if count is different.
	if ( pickDataOrder.GetCount() < placeDataOrder.GetCount() )
		GetPlaceOrder( pickDataOrder);

	CFixedMap<CFeederId, int, SYS_MAX_HEAD> faultyMultiTapeMap;

	// Analyze the cycle in the pick order (not the place order).
	//
	for (int i=0; i<pickDataOrder.GetCount(); i++)
	{
		int idxPd = pickDataOrder[i];
		StPlaceData& pd = m_CycleData.placeData[ idxPd];
		long retryLimit = pd.profile.common.retryCount;
		bool bVacVerify = (pd.profile.handling.bVacuumVerify && g_SysConst.IsSysPickVacCheck());

		if ( pd.profile.IsVirtualPick() )
			continue;

		//
		//IMPORTANT NOTE: <SAMEX 20070723>
		//	Do NOT process any pick count if a head still has a part, next time
		//	this pick count can be reset by operator before placing or dumping
		//	and makes 'pick count < place count' situation.
		//
		double seatedHeight = pd.pHead->GetSeatedHeight();
		//	DO NOT count as pick or error until its placed or dumped if part is still on the head.
		if ( seatedHeight != 0.0  &&  seatedHeight != NaN )	// Part is on the head.
		{
			// Part is not failed and not aligned state, this means part
			// will be processed on next cycle, so skip it.
			ePartFailReason partFailReason = pd.pHead->GetPartFailReason();
			ePartVector		partVecStatus  = pd.pHead->GetPartAlignedStatus();
			if (	partFailReason == ePART_FAIL_NONE		// Not aligned yet, next time this part will be re-aligned.
				&&	partVecStatus  == ePART_VECTOR_UNALIGNED )
			{
				continue;
			}

			//NOTE: THE reason why commented...
			//		If part is already aligned with failure and got Error(EM or FREEZE), below will
			//		make error count is not counted on next time also. so, commented out.
			//x // Cycle will not be processed anymore including dump, do NOT count up as pick.
			//x if ( m_ReturnStatus >= eCYCLE_STOPPED )			// Part will not be dumped yet.
			//x 	continue;
		}

		bool bPickStisticsCounted	= pd.pHead->IsPickStatisticsCounted();
		CFeederId hdPickFeederID	= pd.pHead->GetPickedFeeder();
		CFeederId pickFeederID		= pd.pickedFeederID;

		//ADDED ON 20070511 - FIXING pick count and place count doesn't match.
		//Recursive update pick up count if this was not applied yet
		//due to manual pick up and then start auto production.
		//	If placement was successful, 'pd.placedFeederID' should be defined.
		//	else 'pickFeederID' or 'hdPickedFeederID' should be defined.
		if ( !pickFeederID.IsDefined() )
		{	//Part picked manually first or picked during previous production pick sequence
			//and stopped with EM, assign picked feeder ID.

			CFeederId placedFeederID = pd.placedFeederID;
			if ( placedFeederID.IsDefined() )	// successfully placed after manual pick up
				pickFeederID = placedFeederID;
			else								// failed to place after manual pick up
				pickFeederID = hdPickFeederID;
		}

		//THIS 'Not defined' situation can be happened, when part is picked manually and dumped.
		//And then start production, but hit the EM before picking up part.
		//SYS_ASSERT( pickFeederID.IsDefined() );

		//<20070723 SAMEX>
		//Fixed do NOT count as pick/vision error for tray return part because this part
		//can be re-used later. (Most of tray part failure is mis-teaching tray/part
		//or vision inspection failure due to vision algorithm.) 
		//Rule #1. Pick count should be increased and dump count will be increased
		//		   by MMI calculation.
		//Rule #2. Keep to increasing RT managed mis-pick count for detecting retry over stop.
		bool bSkipErrorCountForReturnTray = false;
		bool bSkipErrorCountForReturnTrayVisionErr = false;
		bool bSkipErrorCountForSingleRetryCount = false;
		bool bSkipErrorCountEver = false;
		if ( pickFeederID.IsDefined())
		{
			ePartFailReason partFailReason = pd.pHead->GetPartFailReason();

			if ( pickFeederID.Type() == eFEEDTYPE_TRAY)
			{
				CTray* pFeeder = ::GetTray( pickFeederID);
				DumpID dumpId = pFeeder ? pFeeder->GetDumpID() : (-1);
				if ( dumpId == 0 )	// Return to tray. 
				{
					bSkipErrorCountForReturnTray = !g_FacDebug.IsSupportErrorCountForReturnToTray();

					if ( g_FacDebug.IsSupportErrorCountForTrayOnlyVisionErr())
						if ( partFailReason == ePART_FAIL_VISION || partFailReason == ePART_FAIL_VIS_SHIFT_PICK 
							|| partFailReason == ePART_FAIL_SVS_VISION || partFailReason == ePART_FAIL_LEAD_SCAN)
							bSkipErrorCountForReturnTrayVisionErr = true;
				}
			}
			
			bSkipErrorCountForSingleRetryCount = (g_FacDebug.IsSupportIncErrorCountforSingleRetryCount() || g_FacDebug.IsPickupErrorNoRetry());
			bSkipErrorCountEver = g_FacDebug.IsSupportIncErrorCountEver();
			//Detect crash of Option
			SYS_ASSERT(!( bSkipErrorCountForSingleRetryCount && bSkipErrorCountEver ));
		}
		else
		{
			//NOTE: This can be happened if this place is 'Virtual Pick' part.
			// Debug - do nothing line
			SYS_ASSERT( !pickFeederID.IsDefined());
		}

		// Only collect pick statistics on executed picks, not skipped ones.
		// Update statistics with the pickup attempt.
		if ( pickFeederID.IsDefined())
		{
			if ( !bPickStisticsCounted )
			{
				UpdatePickStatistics( idxPd, pd, pickFeederID, pd.pHead->GetNozzle());

				//ADDED ON 20070511 - FIXING pick count and place count doesn't match.
				pd.pHead->SetPickStatisticsCounted( true);
			}
		}
		else
		{
			//NOTE: This can be happened if this place is 'Virtual Pick' part.
			// Debug - do nothing line
			pickFeederID = pd.pickedFeederID;
		}

		// On success, only reset mispicks on cycles that picked from the feeder (not prefetched 
		//	 parts) because a prefetch cycle would have already reset the mispicks if no pick 
		//	 problems occurred.
		if ( pd.status==ePLACE_OK)
		{
			if ( pickFeederID.IsDefined())
			{
				bool bProcessed = false;
				if ( pickFeederID.Type() == eFEEDTYPE_TAPE )
				{
					CTapeFeeder* pTape = ::GetTapeFeeder( pickFeederID );
					if ( pTape && pTape->IsMultiPocket() )
					{
						ResetMispicksTapeM( pickFeederID, pd.mPocketPickedIndex);
						bProcessed = true;
					}
				}
				
				if ( !bProcessed )
				{
					ResetMispicks( pickFeederID );
				}
			}
			else
			{
				//NOTE: This can be happened if this place is 'Virtual Pick' part.
				// Debug - do nothing line
				SYS_ASSERT( !pickFeederID.IsDefined());
			}
		}
		// On failure, if this is not a prefetch cycle (i.e. pd.pickedFeederID is defined) the 
		//	 count the error.  If this IS a PREFETCH cycle (i.e. pd.pickedFeederID==0 but hdPickFeederID!=0)
		//   then count the error in this case too.
		else if ( (pd.status==ePLACE_FAILED  ||  pd.status==ePLACE_FAILSKIP)
			   && (pickFeederID.IsDefined()  ||  hdPickFeederID.IsDefined()) )
		{
			ePartFailReason partFailReason	= pd.pHead->GetPartFailReason();
			CNozzleId		hdNozzleID		= pd.pHead->GetNozzle();

			// If the placement didn't record a part being picked, but the CHead has a failed picked part
			//   then use the Head's picked feeder as the part source.
			if ( !pickFeederID.IsDefined())
				pickFeederID = hdPickFeederID;

			// If the part is rejected due to Vision or Vacuum then process the failure.
			// If the part was picked, but not analyzed, then it's PICKED and NOT PLACED.
			if ( partFailReason != ePART_FAIL_NONE)
			{
				if ( pickFeederID.IsDefined())
				{
					bool bProcessed = false;
					if ( pickFeederID.Type() == eFEEDTYPE_TAPE )
					{
						CTapeFeeder* pTape = ::GetTapeFeeder( pickFeederID );
						if ( pTape && pTape->IsMultiPocket() )
						{
							IncMispicksTapeM( pickFeederID, pd.mPocketPickedIndex, partFailReason );
							bProcessed = true;
						}
					}
					
					if ( !bProcessed )
					{
						IncMispicks( pickFeederID, partFailReason );
					}

					if ( pickFeederID.Type() == eFEEDTYPE_STICK )
					{
						CStickFeeder* pStick = ::GetStickFeeder(pickFeederID);
						SYS_ASSERT( pStick );
						pStick->SetStickerBankStatus(eLABEL_FAIL, THIS_FILE, __LINE__);
						//pStick->SetErrorOccurredOnce();
						SetErrorOccurredOnce( pickFeederID );
					}
				}

					//Fixed do NOT count as pick/vision error for tray return part. <20070723 SAMEX>
					if ( !bSkipErrorCountForReturnTray || bSkipErrorCountForReturnTrayVisionErr)
						UpdatePickStatisticsBad( idxPd, pd, partFailReason, pickFeederID, hdNozzleID, fdrIDtoPickBad[ pickFeederID]);

				// If the feeder is already in an error state such as eTAPE_ERROR, eSTICK_EMPTY, eTRAY_EMPTY.
				if ( IsFeederError( pickFeederID)
//ITS_AUTO_LINK_SUPPORT_START	
					|| IsFeederPartEmptyIT(pickFeederID))
//ITS_AUTO_LINK_SUPPORT_END
				{
					//<2011.1.31>
					//ListUp errored multi pocket feeder
					if ( CTapeFeeder::IsMultiPocketTapeFeeder( pickFeederID ) )
					{
						if ( !faultyMultiTapeMap.IsExist(pickFeederID) )
							faultyMultiTapeMap[pickFeederID] = 1;
						else
							faultyMultiTapeMap[pickFeederID] +=1;
					}
					else
					{
						// UNDO the accumulated "failed part" statistics
						// eSTAT_ERASE_ERROR_OR_IT_EMPTY
						UpdatePickStatisticsErr( pickFeederID, &fdrIDtoStatErr[ pickFeederID]);
					}
				}
				else
				{
					//Fixed do NOT decrease error count in case part is really picked and retry limit
					//is setted as one time. <20070723 SAMEX>
					bool bSkipUndo = false;
					if ((( bVacVerify  &&  partFailReason == ePART_FAIL_VISION )
						|| bSkipErrorCountForSingleRetryCount ) 
						&&  retryLimit == 1 )
						bSkipUndo = true;

					//<2009.10.21 SAMEX>
					//Increse error count even if retry count is over.
					//TO maintain correct data of failed pickup count.
					if ( bSkipErrorCountEver )
						bSkipUndo = true;

					//Fixed do NOT count as pick/vision error for tray return part, so do NOT
					//decrease error count for not increased count. <20070723 SAMEX>
					if ( bSkipErrorCountForReturnTray )
						bSkipUndo = true;

					// Check if the feeder has failed due to excessive retries.
					eStatus status = CheckFeederRetryExceeded( pickFeederID, retryLimit, pd.profile.common.retryNgCount, &fdrIDtoStatErr[ pickFeederID], pd.profile.common.height, true);
					if ( status == eNG  &&  !bSkipUndo )
					{
						if ( CTapeFeeder::IsMultiPocketTapeFeeder( pickFeederID ) )
						{
							if ( !faultyMultiTapeMap.IsExist(pickFeederID) )
								faultyMultiTapeMap[pickFeederID] = 1;
							else
								faultyMultiTapeMap[pickFeederID] +=1;
						}
						else
						{
							// UNDO the accumulated "failed part" statistics
							// eSTAT_ERASE_NOTSPLICEAREA_RETRYEXCEEDED
							UpdatePickStatisticsErr( pickFeederID, &fdrIDtoStatErr[ pickFeederID]);
						}
					}
				}
			}
			else
			{
				// Debug - do nothing lines
				SYS_ASSERT( partFailReason == ePART_FAIL_NONE);
				if ( !pd.placedFeederID.IsDefined())
					continue;
			}
		}
		else if ( pd.status==ePLACE_FAILED  ||  pd.status==ePLACE_FAILSKIP)
		{
			// Debug - do nothing line
			SYS_ASSERT( pd.status==ePLACE_FAILED  ||  pd.status==ePLACE_FAILSKIP);
		}
		else if ( pd.status == ePLACE_FAILSTOP)
		{
			// Debug - do nothing line
			SYS_ASSERT( pd.status==ePLACE_FAILSTOP);
		}
	}

	CFeederId fID;
	CTapeFeeder* pTapeM = NULL;
	POS pos = NULL;

	for (pos=faultyMultiTapeMap.GetStartPosition(); pos!=NULL;)
	{
		faultyMultiTapeMap.GetNextAssoc( pos, fID);
		pTapeM = ::GetTapeFeeder( fID );
		if ( !pTapeM ) continue;

		if ( pTapeM->IsMultiPocketReelFaulty( ) )
		{
			//Decrease error count for a specific pocket Index
			UpdatePickStatisticsErrEx(fID, &fdrIDtoStatErr[ fID]);

			//<2011.07.14>
			//Set tape feeder as Error at this time.
			//Because, when dual gantry use same feeder, and opposite gantry success to pickup
			//then the opposite gantry will clear mis pick count 
			//and it will make engine keep continuing tape feeder picking 
			CTapeFeeder::AddRetryExceededList( m_SectionID, m_frRunningLane, fID);
			
			pTapeM->SetStatus( eTAPE_ERROR, false );

		}
		else
		{
			// ResetMispicks( fID );
			if ( fID.Type() == eFEEDTYPE_TAPE)
			{
				ClearMispickTape( fID );
			}
			//ResetMispicks( fID );
		}
	}


	// Emit telemetry for the PickBad feeders
	for (pos=fdrIDtoPickBad.GetStartPosition(); pos!=NULL;)
	{
		CFeederId	feederId;
		StStatisticsCycle& rCache = fdrIDtoPickBad.GetNextAssoc( pos, feederId);
		TelemetryPickErrors( "PickBad, Errors=", feederId, rCache);
	}

	// Telemetry the cumulative changes
	for (pos=fdrIDtoStatErr.GetStartPosition(); pos!=NULL;)
	{
		CFeederId	feederId;
		StStatisticsCycle& rCache = fdrIDtoStatErr.GetNextAssoc( pos, feederId);
		TelemetryPickErrors( "GLOBAL Erase=", feederId, rCache);
	}
}



void
CSeqCycle::UpdatePickStatistics( int aIdxPd, const StPlaceData& aPlaceData, CFeederId aPickedFeederId, CNozzleId aHdNozzle )
{
	if ( aPickedFeederId.Type() == eFEEDTYPE_TAPE && aPlaceData.status==ePLACE_OK )
	{
		bool& nonStopRetryTape = GetNonStopRetryTape( aPickedFeederId);
		nonStopRetryTape = false;
	}

	if ( aPickedFeederId.Type() == eFEEDTYPE_TAPE && GetNonStopRetryTape(aPickedFeederId) == true ) return;

	StStatCacheRec& rCacheRec = m_StatCacheRecs[ aIdxPd];
	
	eCameraGroupType camGrpType = aPlaceData.profile.visCommon.cameraGroupType;

	if ( camGrpType & eCAMERA_GROUP_UP )
	{
		rCacheRec.m_Head.pickUpwardCount++;
		rCacheRec.m_Nozzle.pickUpwardCount++;
		rCacheRec.m_Feeder.pickUpwardCount++;
	}
	else
	{
		// Consider non-UP alignment as FLY/LSO Alignment...
		rCacheRec.m_Head.pickFlyLsoCount++;
		rCacheRec.m_Nozzle.pickFlyLsoCount++;
		rCacheRec.m_Feeder.pickFlyLsoCount++;
	}

	// Below will keep track of these increments in the actual feeder object.
	// BECAUSE... if the feeder fails, then we will need to UNDO all of the increments
	//				directly related to the feeder's failure.
	// The data below is only needed when a feeder is forced into a failed state due to excessive retries.
	//
	eStatIncCat incCat = ((camGrpType & eCAMERA_GROUP_UP) ? eSTAT_INC_CAT_UP : eSTAT_INC_CAT_FLY_LSO);

	eFeederType feedType = aPickedFeederId.Type();
	int tapePocketIndex = -1;
	if ( CTapeFeeder::IsMultiPocketTapeFeeder( aPickedFeederId ) )
		tapePocketIndex = aPlaceData.mPocketPickedIndex;

	CHead* pHead = aPlaceData.pHead;
	SYS_ASSERT(pHead);
	ePartFailReason failReason = pHead->GetPartFailReason();

	StStatIncRec headInc( eSTAT_INC_HEAD,	aPlaceData.head, incCat, tapePocketIndex, failReason);
	StStatIncRec nozInc ( eSTAT_INC_NOZZLE,	aHdNozzle,		 incCat, tapePocketIndex, failReason);
	StStatIncRec feedInc( eSTAT_INC_FEEDER,	aPickedFeederId, incCat, tapePocketIndex, failReason );

	if ( feedType == eFEEDTYPE_TAPE)
	{
		AddStatisticsIncRecTape( aPickedFeederId, headInc);
		AddStatisticsIncRecTape( aPickedFeederId, nozInc );
		AddStatisticsIncRecTape( aPickedFeederId, feedInc);
	}
	else if ( feedType == eFEEDTYPE_STICK)
	{
		AddStatisticsIncRecStick( aPickedFeederId, headInc);
		AddStatisticsIncRecStick( aPickedFeederId, nozInc );
		AddStatisticsIncRecStick( aPickedFeederId, feedInc);
	}
	else if ( feedType == eFEEDTYPE_TRAY)
	{
		AddStatisticsIncRecTray( aPickedFeederId, headInc);
		AddStatisticsIncRecTray( aPickedFeederId, nozInc );
		AddStatisticsIncRecTray( aPickedFeederId, feedInc);
	}
}


void
CSeqCycle::ResetMispicks( CFeederId aPickFeederID )
{
	SYS_ASSERT( aPickFeederID.IsDefined());

	eFeederType	feedType = aPickFeederID.Type();
	
	// Discard the accumulated statistic increment records normally used to back out statistics data.
	// Reset the mispick count
	if ( feedType == eFEEDTYPE_TAPE)
	{
		ClearStatisticsIncTape(aPickFeederID);
		ClearMispickTape(aPickFeederID);
	}
	else if ( feedType == eFEEDTYPE_STICK)
	{
		ClearStatisticsIncStick( aPickFeederID);
		ClearMispickStick( aPickFeederID);
	}
	else if (feedType == eFEEDTYPE_TRAY)
	{
			ClearStatisticsIncTray(aPickFeederID);
			ClearMispickTray(aPickFeederID);
		}
}

void
CSeqCycle::IncMispicksTapeM( CFeederId aPickFeederID, int aPocketPickedIndex, ePartFailReason aPartFailReason )
{
	SYS_ASSERT( aPickFeederID.IsDefined());

	eFeederType	feedType = aPickFeederID.Type();

	if ( feedType == eFEEDTYPE_TAPE)
	{
		CTapeFeeder* pTape = ::GetTapeFeeder( aPickFeederID );

		SYS_ASSERT( pTape && pTape->IsMultiPocket() );
		
		pTape->NotifyPickedStatus( aPocketPickedIndex, eNG );

		IncMispickTape( aPickFeederID, aPartFailReason );
	}
}


void
CSeqCycle::ResetMispicksTapeM( CFeederId aPickFeederID, int aPocketPickedIndex )
{
	SYS_ASSERT( aPickFeederID.IsDefined());

	eFeederType	feedType = aPickFeederID.Type();

	// Discard the accumulated statistic increment records normally used to back out statistics data.
	// Reset the mispick count
	if ( feedType == eFEEDTYPE_TAPE)
	{
		// ClearStatisticsIncTape( aPickFeederID);
		ClearMispickTape( aPickFeederID);

		CTapeFeeder* pTape = ::GetTapeFeeder( aPickFeederID );

		SYS_ASSERT( pTape && pTape->IsMultiPocket() );
		
		pTape->NotifyPickedStatus( aPocketPickedIndex, eOK );

	}
//	else if ( feedType == eFEEDTYPE_STICK)
//	{
//		ClearStatisticsIncStick( aPickFeederID);
//		ClearMispickStick( aPickFeederID);
//	}
//	else if ( feedType == eFEEDTYPE_TRAY)
//	{
//		ClearStatisticsIncTray( aPickFeederID);
//		ClearMispickTray( aPickFeederID);
//	}
}


void
CSeqCycle::IncMispicks( CFeederId aPickFeederID, ePartFailReason aPartFailReason )
{
	if (aPartFailReason == ePART_FAIL_LCR_OK || aPartFailReason == ePART_FAIL_LCR_NG)
		return;

	SYS_ASSERT( aPickFeederID.IsDefined());

	eFeederType	feedType = aPickFeederID.Type();

	if ( feedType == eFEEDTYPE_TAPE)		IncMispickTape( aPickFeederID, aPartFailReason );
	else if ( feedType == eFEEDTYPE_STICK)	IncMispickStick( aPickFeederID, aPartFailReason );
	else if ( feedType == eFEEDTYPE_TRAY)	IncMispickTray( aPickFeederID, aPartFailReason );
}


eStatus
CSeqCycle::CheckFeederRetryExceeded( CFeederId aPickFeederID, long aRetryLimit, long aPartNgLimit, StStatisticsCycle* apFdrIdStatErr, double aPartHeight, bool abRetryFeed)
{
	char ctxt[50];
	SYS_ASSERT( aPickFeederID.IsDefined());

	long		retryLimit = aRetryLimit;
	long		partNgLimit = aPartNgLimit;
	long		totalMissPick		= 0;
	long		pickMissCnt			= 0;
	long		partNgCnt			= 0;
	eFeederType	feedType	= aPickFeederID.Type();

	bool		checkRetryFeed	= abRetryFeed;
	bool		retryOut = false;

	if (partNgLimit == 0)
		partNgLimit = retryLimit;

	if ( feedType == eFEEDTYPE_TAPE)
	{
		CTapeFeeder* pTape = ::GetTapeFeeder( aPickFeederID );
		SYS_ASSERT( pTape )

		long retryFeedingCount;

		long mispickTape		= GetMispickTape( aPickFeederID);			// All Error Count
		long pickMissCount		= GetPickUpMissCountTape( aPickFeederID);	// Pick Miss Count
		long partNGCount		= GetPartNGCountTape( aPickFeederID);		// Part NG Count

		// Not support retry feeding at multi vendor part.
		if ( g_SysConst.IsUseAutoMultivendor() && pTape->IsMainSubPartTape() && partNGCount >= partNgLimit)
			checkRetryFeed = false;

		if ( checkRetryFeed == true && (pickMissCount == retryLimit) && mispickTape != 1)
		{
			if ( CStepRetryFeedingStd::IsUseRetryFeeding( pTape, retryFeedingCount, m_pGantry, aPartHeight))
			{
				if ( !CStepRetryFeedingStd::IsRetryFeedingActive( pTape))
				{
					Telemetry( EV_CLASSHEAD, 0, "Stat:%s CheckRetry: MissCount=%d, RetryLimit=%d,  RetryFeed=Y,  START & GLOBAL Erase",
												aPickFeederID.Format( ctxt, sizeof(ctxt)),
												mispickTape,
												retryLimit);
					CStepRetryFeedingStd::StartRetryFeeding( pTape);
					
					//if ( partNGCount > 0)
					pTape->SetPocketTeachStatus( ePOCKET_TEACH_RETEACH );

					// eSTAT_ERASE_STARTRETRYFEEDING
					if ( !g_FacDebug.IsSupportIncErrorCountEver() )
						UpdatePickStatisticsErr( aPickFeederID, apFdrIdStatErr);

					return eOK;
				}
			}
		}

		long nonStopRetryCount = pTape->GetNonStopRetryCount();

		if ( ( feedType == eFEEDTYPE_TRAY ) ||
			 ( nonStopRetryCount < 1 ) ||
			 ( retryLimit < 3 ) ||
			 ( nonStopRetryCount <= retryLimit ) )
		{
		}
		else if ( mispickTape == retryLimit )
		{
			UpdatePickStatisticsErr( aPickFeederID, apFdrIdStatErr);

			bool& nonStopRetryTape = GetNonStopRetryTape( aPickFeederID);
			nonStopRetryTape = true;

			return eOK;
		}
		else if ( ( GetNonStopRetryTape(aPickFeederID) == true ) && ( mispickTape < nonStopRetryCount ) )
		{
			return eOK;
		}
	}

	switch( feedType)
	{
	case eFEEDTYPE_TAPE:
		totalMissPick	= GetMispickTape( aPickFeederID);
		pickMissCnt		= GetPickUpMissCountTape( aPickFeederID);
		partNgCnt		= GetPartNGCountTape(aPickFeederID);
		break;
	case eFEEDTYPE_STICK:
		totalMissPick	= GetMispickStick( aPickFeederID);
		pickMissCnt		= GetPickUpMissCountStick(aPickFeederID);
		partNgCnt		= GetPartNGCountStick(aPickFeederID);
		break;
	case eFEEDTYPE_TRAY:
		totalMissPick	= GetMispickTray( aPickFeederID);
		pickMissCnt		= GetPickUpMissCountTray( aPickFeederID);
		partNgCnt		= GetPartNGCountTray( aPickFeederID);
		break;
	default: ;	break;
	}

	if ( apFdrIdStatErr!=NULL)//retries >= aRetryLimit)
		Telemetry( EV_CLASSHEAD, 0, "Stat:%s CheckRetry: Retry=%d, RetryLimit=%d,%d MissCnt=%d, NgCnt= %d", aPickFeederID.Format( ctxt, sizeof(ctxt)), totalMissPick, retryLimit, partNgLimit, pickMissCnt, partNgCnt);

	if (feedType == eFEEDTYPE_STICK)
	{
		CStickFeeder* pStick = ::GetStickFeeder(aPickFeederID);
		if ( pStick && pStick->IsStickerTapeFeeder())
		{
			if (pickMissCnt >= retryLimit
				|| partNgCnt >= partNgLimit
				|| totalMissPick >= (retryLimit + partNgLimit))
			{
				/*pStick->SetIsRetryExceeded(true);*/
				//멀티라벨피더(=Sticker Tape Feeder)는 마지막 부품까지 Pick up시도 후,
				//부품이 더이상 공급될 수 없는 경우에만 생산을 중단한다.
				if (pStick->GetIsNeedToReportRetryExceeded())
				{
					retryOut = true;
				}

				return (retryOut ? eNG : eOK);
			}
		}
	}

	if (pickMissCnt >= retryLimit
		|| partNgCnt >= partNgLimit
		|| totalMissPick >= (retryLimit+partNgLimit))
		retryOut = true;

	return (retryOut ? eNG : eOK);
}

eStatus
CSeqCycle::CheckFeederMissPickRate ( const StPlaceData& aPlaceData, CFeederId aPickFeederID )
{
	SYS_ASSERT( aPickFeederID.IsDefined());
	
//	eFeederType	feedType	= aPickFeederID.Type();
	CTapeFeeder* pTape		= NULL;
	bool		bResult1	= false;
	bool		bResult2	= false;
	bool		bFinalResult= false;
	bool bUseMissPickTapeRate = g_FacDebug.IsUsePickUpMissTapeRate();
	bool bUseMissPickTapeRate2= g_FacDebug.IsUsePickUpMissTapeRate2();
	int			checkIndex	= GetPickUpMissTapeRateCheckCount(aPickFeederID);
	int			checkIndex2	= GetPickUpMissTapeRateCheckCount2(aPickFeederID);
	
	if ( /*feedType*/aPickFeederID.Type() == eFEEDTYPE_TAPE && (bUseMissPickTapeRate || bUseMissPickTapeRate2) )
	{

		pTape = ::GetTapeFeeder( aPickFeederID );
		SYS_ASSERT( pTape );

		bResult1 = CheckFeederMissPickRateDetail( aPickFeederID, checkIndex, bUseMissPickTapeRate, false);
		bResult2 = CheckFeederMissPickRateDetail( aPickFeederID, checkIndex2, false, bUseMissPickTapeRate2);
		
		bFinalResult = (bResult1 == true || bResult2 == true)? true:false;
		
		if ( bFinalResult )
		{
			if (pTape->GetMasterPartID() == 0 && !pTape->IsMultiVendorTape())
			{
				if ( g_FacConst.GetPickTeachXYMethod() && aPlaceData.profile.common.pocketTeachStatus && !GetPickUpMissTapePocketTeach(aPickFeederID) )
				{
					pTape->SetPocketTeachStatus(ePOCKET_TEACH_RETEACH);
					Telemetry( EV_CLASSFEEDER, aPlaceData.pHead->Unit(), "H%s Re-TeachPocket Feeder=%d:%d Idx:%d, %d", aPlaceData.pHead->GetName(), aPickFeederID.Unit(), aPickFeederID.No(), checkIndex, checkIndex2 ); 
				}
				else
				{
					ReportFeederMissPickRate ( aPlaceData, aPickFeederID );
				}
			}
			else //No use Multi-Vendor
			{
				ReportFeederMissPickRate ( aPlaceData, aPickFeederID );
			}
		}

	}
	return eOK;
}	


bool					
CSeqCycle::CheckFeederMissPickRateDetail( CFeederId aPickFeederID, int aIndex, bool bUseMissPickCheck1, bool bUseMissPickCheck2 )
{
	int missPickCnt = 0;
	int addLoop		= 0;
	int loopValue	= 0;
	bool bSatisfyMaxPick = false;
	int maxPick = GetPickUpTapeCount(aPickFeederID);

	if ( bUseMissPickCheck1 ) 
	{
		loopValue = aIndex+g_FacDebug.GetPickUpMissTapeRateMaxCount();
		bSatisfyMaxPick = (maxPick >= g_FacDebug.GetPickUpMissTapeRateMaxCount()) ? true:false;
	}
	else if ( bUseMissPickCheck2)
	{
		loopValue = aIndex+g_FacDebug.GetPickUpMissTapeRateMaxCount2();
		bSatisfyMaxPick = (maxPick >= g_FacDebug.GetPickUpMissTapeRateMaxCount2()) ? true:false;
	}

	if ( loopValue > eMAX_PICK_RATE_COUNT )
		addLoop = loopValue - eMAX_PICK_RATE_COUNT;
	
	if ( (bUseMissPickCheck1&&bSatisfyMaxPick) || (bUseMissPickCheck2&&bSatisfyMaxPick) )
	{
		if ( !addLoop )
		{
			for ( int i = aIndex ; i < loopValue; i++ )
			{
				ePartFailReason placestate = GetPickUpMissTapeRate(aPickFeederID, i);
				if ((placestate == ePART_FAIL_VAC_PICK || placestate == ePART_FAIL_SVS_PICK || placestate == ePART_FAIL_VISION || placestate == ePART_FAIL_SVS_VISION || placestate == ePART_FAIL_LEAD_SCAN))
					missPickCnt++;
			}
		}
		else
		{
			for ( int k = aIndex; k<eMAX_PICK_RATE_COUNT; k++)
			{
				ePartFailReason placestate = GetPickUpMissTapeRate(aPickFeederID, k);
				if ((placestate == ePART_FAIL_VAC_PICK || placestate == ePART_FAIL_SVS_PICK || placestate == ePART_FAIL_VISION || placestate == ePART_FAIL_SVS_VISION || placestate == ePART_FAIL_LEAD_SCAN))
					missPickCnt++;
			}
			
			for ( int j = 0; j < addLoop; j++)
			{
				ePartFailReason placestate = GetPickUpMissTapeRate(aPickFeederID, j);
				if ((placestate == ePART_FAIL_VAC_PICK || placestate == ePART_FAIL_SVS_PICK || placestate == ePART_FAIL_VISION || placestate == ePART_FAIL_SVS_VISION || placestate == ePART_FAIL_LEAD_SCAN))
					missPickCnt++;
			}
		}
		
		if ( bUseMissPickCheck1 )
		{
			if ( missPickCnt >= g_FacDebug.GetPickUpMissTapeRateCount())
			{
				Telemetry( EV_CLASSFEEDER, 0, "satisfyRate1 Feeder=%d:%d MissPick:%d, loop:%d, addLoop:%d ", aPickFeederID.Unit(), aPickFeederID.No(), missPickCnt, loopValue, addLoop ); 
				return true;
			}
		}
		if ( bUseMissPickCheck2 )
		{ 
			if ( missPickCnt >= g_FacDebug.GetPickUpMissTapeRateCount2())
			{
				Telemetry( EV_CLASSFEEDER, 0, "satisfyRate2 Feeder=%d:%d MissPick:%d, loop:%d, addLoop:%d ", aPickFeederID.Unit(), aPickFeederID.No(), missPickCnt, loopValue, addLoop );
				return true;
			}
		}
	}

	return false;
}

bool
CSeqCycle::IsFeederError( CFeederId aPickFeederID)
{
	SYS_ASSERT( aPickFeederID.IsDefined());

	eFeederType	feedType = aPickFeederID.Type();

	if ( feedType == eFEEDTYPE_TAPE)
	{
		CTapeFeeder* pFeeder = ::GetTapeFeeder( aPickFeederID);
		eTapeStatus status = pFeeder->GetStatus();
		return status == eTAPE_ERROR;
	}
	else if ( feedType == eFEEDTYPE_STICK)
	{
		CStickFeeder* pFeeder = ::GetStickFeeder( aPickFeederID);
		//eStickStatus status = pFeeder->GetStatus();
		//return (status == eSTICK_EMPTY || status == eSTICK_ERROR);
		return pFeeder->IsFeederError();
	}
	else if ( feedType == eFEEDTYPE_TRAY)
	{
		CTray* pFeeder = ::GetTray( aPickFeederID);
		eTrayStatus status = pFeeder->GetStatus();
		return status == eTRAY_EMPTY;
	}
	else
	{
		// The picked feeder should always be defined at this level.
		SYS_ASSERT(0);
	}
	return true;
}

//ITS_AUTO_LINK_SUPPORT_START
bool
CSeqCycle::IsFeederPartEmptyIT( CFeederId aPickFeederID)
{
	SYS_ASSERT( aPickFeederID.IsDefined());

	eFeederType	feedType = aPickFeederID.Type();

	if ( feedType == eFEEDTYPE_TAPE)
	{
		CTapeFeeder* pFeeder = ::GetTapeFeeder( aPickFeederID);
		if ( pFeeder->IsPartCountITEmpty() )
			return true;

		return false;		
	}
//	else if ( feedType == eFEEDTYPE_STICK)
//	{
//		CStickFeeder* pFeeder = ::GetStickFeeder( aPickFeederID);
//		eStickStatus status = pFeeder->GetStatus();
//		return status == eSTICK_EMPTY;
//	}
//	else if ( feedType == eFEEDTYPE_TRAY)
//	{
//		CTray* pFeeder = ::GetTray( aPickFeederID);
//		eTrayStatus status = pFeeder->GetStatus();
//		return status == eTRAY_EMPTY;
//	}
//	else
//	{
//		// The picked feeder should always be defined at this level.
//		SYS_ASSERT(0);
//	}
	return false;		
}
//ITS_AUTO_LINK_SUPPORT_EMD

void
CSeqCycle::SetErrorOccurredOnce( CFeederId aPickFeederID)
{
	SYS_ASSERT( aPickFeederID.IsDefined());

	eFeederType	feedType = aPickFeederID.Type();

	if ( feedType == eFEEDTYPE_TAPE)
	{
		CTapeFeeder* pFeeder = ::GetTapeFeeder( aPickFeederID);
		pFeeder->SetErrorOccurredOnce();
	}
	else if ( feedType == eFEEDTYPE_STICK)
	{
		CStickFeeder* pFeeder = ::GetStickFeeder( aPickFeederID);
		pFeeder->SetErrorOccurredOnce();
	}
//	else if ( feedType == eFEEDTYPE_TRAY)
//	{
//		CTray* pFeeder = ::GetTray( aPickFeederID);
//		eTrayStatus status = pFeeder->GetStatus();
//		return status == eTRAY_EMPTY;
//	}
//	else
//	{
//		// The picked feeder should always be defined at this level.
//		SYS_ASSERT(0);
//	}
	return;		
}


bool g_bStopOnlyGantryForNonIndepError = true;
bool
CSeqCycle::ReportFeederRetryExceeded( const StPlaceData& aPlaceData, ePlacementStatus& aPlaceStatus, CFeederId aPickedFeederId, bool& abReport)
{
	bool			bDoNotRetry		= false;
	bool			bPlaceStatusChgd= false;
	bool			bFeedersFound	= false;
	long			retryFeedingCount;
	CHead*			pHead			= aPlaceData.pHead;
	CFeederId		feederId		= aPickedFeederId; //aPlaceData.pickedFeederID;
	CFeederId		altFeederId;
	eFrontRear		frntRear		= eBOTH;
	eFeederType		feedType		= feederId.Type();
	eMachErrorCode	errCode			= eMEC_NOT_AN_ERROR;
	eErrorLevel		errLevel		= eERR_LEVEL_WARNING;
	CRunOptions&	rRunOptions		= *::GetRunOptions( m_frRunningLane);
	CStepPick*		pAssignedPickStep = NULL;
	bool			bStopOnlyGantryByError = g_bStopOnlyGantryForNonIndepError? true : g_StateReport.IsIndependentLanes();
	PartID			aCurrentPartID;
	PartID			aNewPartID;

	if( feedType == eFEEDTYPE_TAPE )
	{
		CTapeFeeder* pTape = ::GetTapeFeeder( aPickedFeederId );

		// SKIP reporting if Retry Feeding is handling an effective splice.
		if ( pTape && CStepRetryFeedingStd::IsUseRetryFeeding( pTape, retryFeedingCount, m_pGantry,aPlaceData.profile.common.height))
		{
			if ( CStepRetryFeedingStd::IsRetryFeedingActive( pTape))
				return bPlaceStatusChgd;
		}
	}

	SYS_ASSERT( feederId.IsDefined());

	StPlaceData::eASSIGNED_STEP_TYPE stepType = StPlaceData::eASSIGNED_STEP_PICK_TAPE;
	if ( feedType == eFEEDTYPE_TAPE )
		stepType = StPlaceData::eASSIGNED_STEP_PICK_TAPE;
	else if ( feedType == eFEEDTYPE_STICK )
		stepType = StPlaceData::eASSIGNED_STEP_PICK_STICK;
	else if ( feedType == eFEEDTYPE_TRAY )
		stepType = StPlaceData::eASSIGNED_STEP_PICK_TRAY;
	else
		SYS_ASSERT(0);

	pAssignedPickStep = aPlaceData.pStepPickAssigned[stepType];

	SYS_ASSERT( pAssignedPickStep );


	// Init the return placement status
	aPlaceStatus = aPlaceData.status;
	
	if ( feedType == eFEEDTYPE_TAPE)
	{
		CTapeFeeder* pFeeder = ::GetTapeFeeder( feederId );

		eStatus mainSubPartStatus = eNG;
		bool	bAutoLinkSkip = false;
		long	retriesPartNG = GetPartNGCountTape(feederId);
		
		if (g_SysConst.IsUseAutoMultivendor() && (aPlaceData.profile.common.retryCount <= retriesPartNG))
		{
			Telemetry( EV_CLASSFEEDER, pHead->Unit(), "H%s PartNGRetryExceeded: %s, PartID: &d, RetryCount: %d, PartNGCount: %d", pHead->GetName(), pFeeder->GetName(), 
														aPlaceData.partID, aPlaceData.profile.common.retryCount, retriesPartNG); 
			mainSubPartStatus = pFeeder->ChangeMainSubPartTapeData( aNewPartID );
			if ( mainSubPartStatus == eOK )
			{
				Telemetry( EV_CLASSFEEDER, pHead->Unit(), "H%s MultiVendorPartChanged: %s, PrevPartID: %d, NewPartID: %d", pHead->GetName(), pFeeder->GetName(), aPlaceData.partID, aNewPartID); 
				bDoNotRetry = false;
				bAutoLinkSkip = true;
				ResetMispicks( feederId );
			}
			else
			{
				CTapeFeeder::AddRetryExceededList( m_SectionID, m_frRunningLane, feederId);
				pFeeder->SetStatus( eTAPE_ERROR );
				bAutoLinkSkip = false;
			}
		}
		else
		{
			CTapeFeeder::AddRetryExceededList( m_SectionID, m_frRunningLane, feederId);
			pFeeder->SetStatus( eTAPE_ERROR );
			mainSubPartStatus = eNG;
			bAutoLinkSkip = false;
		}

		CStepPick::MapFdrIDtoN	usedFeederMap;

		frntRear = pFeeder->GetFrontOrRear();

//		CHeadBlock* pHB = pHead->GetHeadBlock();

		//<2011.02.21>
		//FindFeeder again and reset errored status if it failed to find avaiable feeder.
		
		// Retries are exceeded.  If there ARE alternates then downgrade and skip this part number.
		// NOTE: CFeeder should not make decisions about head independence.  Instead the caller should
		// handle this condition and convert this status if necessary.
		altFeederId = pAssignedPickStep->FindFeeder( aPlaceData.feederID,
												usedFeederMap, *pHead, 0,
												aPlaceData.profile.common.height-aPlaceData.profile.common.depthZ,
												aPlaceData.bAutoFeeder,
												bFeedersFound, errCode);

		if ( !altFeederId.IsDefined() && !bAutoLinkSkip)
		{
			if ( g_SysConst.GetTapeAutoLinkOption() == eFEEDER_LINK_AUTO_SUCCESSIVE )
			{
				Telemetry( EV_CLASSFEEDER, pHead->Unit(), "H%s ReportFeederRetryExceeded: %s, SuccessiveLineSearch", pHead->GetName(), pFeeder->GetName()); 

				CTapeFeeder::ResetErrorOccurredOnceFBaseOnlySameParts(aPlaceData.feederID);
				// Get the actual feeder to use, considering linked feeders too.
				altFeederId = pAssignedPickStep->FindFeeder( aPlaceData.feederID,
												usedFeederMap, *pHead, 0,
												aPlaceData.profile.common.height-aPlaceData.profile.common.depthZ,
												aPlaceData.bAutoFeeder,
												bFeedersFound, errCode);
			}
		}

//		errCode = eMEC_TAPE_NO_ACCESS;
		// If there are no alternate feeders...
		if ( !altFeederId.IsDefined() && !bAutoLinkSkip )
			bDoNotRetry = true;

		if ( mainSubPartStatus == eOK )
		{
			errCode = eMEC_TAPE_MULTIVENDOR_PART_CHANGED;
			aCurrentPartID = aNewPartID;
		}
		else if ( altFeederId.IsDefined() && !bAutoLinkSkip )
		{
			errCode = eMEC_LOW_PARTS_TAPE;
			aCurrentPartID = aPlaceData.partID;
		}
		else
		{
			errCode = eMEC_TAPE_RETRY_EXCEEDED;
			aCurrentPartID = aPlaceData.partID;
		}
			
		if ( !bStopOnlyGantryByError && rRunOptions.IsStopWhenPickError() && mainSubPartStatus != eOK)
			errLevel = eERR_LEVEL_FREEZE;

		if(abReport)
		{
			if ( errCode == eMEC_TAPE_RETRY_EXCEEDED || errCode == eMEC_LOW_PARTS_TAPE)
			{
				g_StateReport.ReportErrorEx2(errCode, errLevel, eMOTOR_OK,
					m_SectionID, frntRear, aPlaceData.head, HeadNone,
					aCurrentPartID, pFeeder->GetID(), 0, 0, 0, __LINE__, aPlaceData.cycle, m_frRunningLane);
			}
			else
			{
				g_StateReport.ReportErrorEx2(errCode, errLevel, eMOTOR_OK,
					m_SectionID, frntRear, aPlaceData.head, HeadNone,
					aCurrentPartID, pFeeder->GetID(), 0, 0, 0, __LINE__, aPlaceData.cycle);
			}
			abReport = false;
		}

		Telemetry( EV_CLASSFEEDER, pHead->Unit(), "H%s  Feeder=%d:%d  %d", pHead->GetName(), frntRear, pFeeder->Unit(), pFeeder->GetStatus()); 

		if ( bStopOnlyGantryByError && rRunOptions.IsStopWhenPickError() 
			|| rRunOptions.IsManualPlaceSteps() 
			|| (g_FacDebug.IsPickupErrorNoRetry() && aPlaceData.profile.common.retryCount == 1))
		{
			// Stop only this station
			g_Machine.StopRunning( pHead->Gantry()->GetSectionID(), frntRear, eSTOP_OPTION_GANTRY);
			ResetMispicks(pFeeder->GetID());
		}
	}
	else if ( feedType == eFEEDTYPE_STICK)
	{
		CStickFeeder* pFeeder = ::GetStickFeeder( feederId );

		CStickFeeder::AddRetryExceededList(feederId);

		pFeeder->SetStatus( eSTICK_EMPTY );

		// Multi Label Feeder의 경우, Unit 전체를 Error 처리한다.
		if ( pFeeder && pFeeder->IsStickerTapeFeeder())
		{
			long unit = feederId.Unit();
			for (int i = 1; i < _countof(g_StickFeeder[0]); i++)
			{
				CStickFeeder& stick = g_StickFeeder[unit][i];
				if (stick.IsInstalled())
					stick.SetStatus(eSTICK_EMPTY);
			}
		}

		frntRear = pFeeder->GetFrontRear();

		CStepPick::MapFdrIDtoN	usedFeederMap;

		// Retries are exceeded.  If there ARE alternates then downgrade and skip this part number.
		// NOTE: CFeeder should not make decisions about head independence.  Instead the caller should
		// handle this condition and convert this status if necessary.
		altFeederId = pAssignedPickStep->FindFeeder( aPlaceData.feederID, 
												usedFeederMap, *pHead, 0,
												aPlaceData.bAutoFeeder,
												bFeedersFound);

//		errCode = eMEC_STICK_NO_ACCESS;
		// If there are no alternate feeders...
		if ( !altFeederId.IsDefined())
			bDoNotRetry = true;

		if ( pFeeder && pFeeder->IsStickerTapeFeeder())
			errCode = (altFeederId.IsDefined() ? eMEC_PICK_STICKER_TAPE_FEEDER_PICK_FAILED : eMEC_STICK_RETRY_EXCEEDED);
		else
			errCode = (altFeederId.IsDefined() ? eMEC_LOW_PARTS_STICK : eMEC_STICK_RETRY_EXCEEDED);

		// The below code is commented to limit production stop to the Gantry concerned.
		if ( !bStopOnlyGantryByError && rRunOptions.IsStopWhenPickError() )
			errLevel = eERR_LEVEL_FREEZE;

		if (abReport)
		{
			if ( errCode == eMEC_STICK_RETRY_EXCEEDED || errCode == eMEC_LOW_PARTS_STICK)
			{
				g_StateReport.ReportErrorEx2(errCode, errLevel, eMOTOR_OK,
					m_SectionID, frntRear, aPlaceData.head, HeadNone,
					aPlaceData.partID, pFeeder->GetID(), 0, 0, 0, __LINE__, aPlaceData.cycle, m_frRunningLane);
			}
			else
			{
				g_StateReport.ReportErrorEx2(errCode, errLevel, eMOTOR_OK,
					m_SectionID, frntRear, aPlaceData.head, HeadNone,
					aPlaceData.partID, pFeeder->GetID(), 0, 0, 0, __LINE__, aPlaceData.cycle);
			}
			abReport = false;
		}

		Telemetry( EV_CLASSFEEDER, pHead->Unit(), "H%s  Feeder=%d:%d  %d", pHead->GetName(), frntRear, pFeeder->Unit(), pFeeder->GetStatus()); 
		
		if ( bStopOnlyGantryByError && rRunOptions.IsStopWhenPickError() 
			|| rRunOptions.IsManualPlaceSteps()
			|| (g_FacDebug.IsPickupErrorNoRetry() && aPlaceData.profile.common.retryCount == 1))
		{
			// Stop only this station
			g_Machine.StopRunning( pHead->Gantry()->GetSectionID(), frntRear, eSTOP_OPTION_GANTRY);
			ResetMispicks(pFeeder->GetID());
		}
	}
	else if ( feedType == eFEEDTYPE_TRAY)
	{
		CTray* pFeeder = ::GetTray( feederId );

		//In case tray feeder, retry exceed does not mean tray is Empty, the empty state
		//will be setted in tray when pocket count is really became Empty.
		//!!! NO !!! pFeeder->SetStatus( eTRAY_EMPTY);
		pFeeder->SetStatus( eTRAY_RETRY_OVER);

		CStepPick::MapFdrIDtoN	usedFeederMap;

		frntRear = pFeeder->GetFrontRear();

		// Retries are exceeded.  If there ARE alternates then downgrade and skip this part number.
		// NOTE: CFeeder should not make decisions about head independence.  Instead the caller should
		// handle this condition and convert this status if necessary.

		bool	bFeederFound;
		bool	bEmptyPallet;
		bool	bUpperDisable;
		bool	bLowerDisable;
		bool	bOutOfRange = false;
		bool	bUnclampDockingTray;
		CStepPickTrayFeeder::TListTrayRec	trays;

//		altFeederId = FindFeeder( aPlaceData.feederID,
//												usedFeederMap, *pHead, 0,
//												aPlaceData.bAutoFeeder, 
//												bFeedersFound);

		CStepPickTrayFeeder* pStepPickTFNxt = ::GetStepPickTrayFeeder( aPlaceData.pHead->GetHeadBlock()->GetGantryID(), aPlaceData.feederID.Unit());
		// Get the actual feeder to use, considering linked feeders too.
		altFeederId = pStepPickTFNxt->FindFeeder( aPlaceData.feederID, usedFeederMap,
												 *aPlaceData.pHead, 0,
												 aPlaceData.bAutoFeeder, trays, bFeederFound, bEmptyPallet, bUpperDisable, bLowerDisable, bOutOfRange, bUnclampDockingTray);

//		errCode = eMEC_TRAY_NO_ACCESS;
		// If there are no alternate feeders...
		if ( !altFeederId.IsDefined())
			bDoNotRetry = true;

		errCode = (altFeederId.IsDefined() ? eMEC_LOW_PARTS_TRAY : eMEC_TRAY_RETRY_EXCEEDED);

		// The below code is commented to limit production stop to the Gantry concerned.
		if ( !bStopOnlyGantryByError && rRunOptions.IsStopWhenPickError() )
			errLevel = eERR_LEVEL_FREEZE;

		if(abReport)
		{
			if ( errCode == eMEC_TRAY_RETRY_EXCEEDED || errCode == eMEC_LOW_PARTS_TRAY)
			{
				g_StateReport.ReportErrorEx2(errCode, errLevel, eMOTOR_OK,
					m_SectionID, frntRear, aPlaceData.head, HeadNone,
					aPlaceData.partID, pFeeder->GetID(), 0, 0, 0, __LINE__, aPlaceData.cycle, m_frRunningLane);
			}
			else
			{
				g_StateReport.ReportErrorEx2(errCode, errLevel, eMOTOR_OK,
					m_SectionID, frntRear, aPlaceData.head, HeadNone,
					aPlaceData.partID, pFeeder->GetID(), 0, 0, 0, __LINE__, aPlaceData.cycle);
			}
			abReport = false;
		}

		Telemetry( EV_CLASSFEEDER, pHead->Unit(), "H%s  Feeder=%d:%d  %d", pHead->GetName(), frntRear, pFeeder->Unit(), pFeeder->GetStatus()); 
		
		if ( bStopOnlyGantryByError && rRunOptions.IsStopWhenPickError() 
			|| rRunOptions.IsManualPlaceSteps()
			|| (g_FacDebug.IsPickupErrorNoRetry() && aPlaceData.profile.common.retryCount == 1))
		{
			// Stop only this station
			g_Machine.StopRunning( pHead->Gantry()->GetSectionID(), frntRear, eSTOP_OPTION_GANTRY);
			ResetMispicks(pFeeder->GetID());
		}
	}
	else
		SYS_ASSERT(0);

	if ( bDoNotRetry )
	{
		// Indicate retries will not fix this problem.
		if ( aPlaceStatus < ePLACE_FAILSKIP)
		{
			bPlaceStatusChgd = true;
			aPlaceStatus = ePLACE_FAILSKIP; //eMEC_COMP_VIS_ALIGN_ERROR
//			if ( bFeedersFound )
//			{
// 'eMEC_TAPE_RETRY_EXCEEDED' is already reported, there is no need to report below message again.
// ..._NO_ACCESS is not adequate because feederId is original accessible feeder.... 20060925
//			g_StateReport.ReportErrorEx2( errCode, eERR_LEVEL_WARNING, eMOTOR_OK,		//eMEC_PICK_NO_FEEDER
//										m_SectionID, frntRear, pHead->Unit(), HeadNone, aPlaceData.partID, 
//										feederId, 0, 0, 0, __LINE__, aPlaceData.cycle);
//			}
		}
	}
	else
	{
		// Indicate the placement should be retried
		if ( aPlaceStatus < ePLACE_FAILED)
		{
			bPlaceStatusChgd = true;
			aPlaceStatus = ePLACE_FAILED;
		}
	}

	return bPlaceStatusChgd;
}

bool			
CSeqCycle::ReportFeederMissPickRate ( const StPlaceData& aPlaceData, CFeederId aPickedFeederId )
{
	bool			bPlaceStatusChgd= false;
	CHead*			pHead			= aPlaceData.pHead;
	CFeederId		feederId		= aPickedFeederId; //aPlaceData.pickedFeederID;
	eFrontRear		frntRear		= pHead->Gantry()->IsFront() ? eFRONT : eREAR;
	eFeederType		feedType		= feederId.Type();
	eMachErrorCode	errCode			= eMEC_TAPE_RETRY_EXCEEDED;
	eErrorLevel		errLevel		= eERR_LEVEL_WARNING;
	bool			bUseMissPickTapeRate = g_FacDebug.IsUsePickUpMissTapeRate();
	bool			bUseMissPickTapeRate2= g_FacDebug.IsUsePickUpMissTapeRate2();
	bool			bReportError = false;

	if( feedType == eFEEDTYPE_TAPE )
	{
		CTapeFeeder* pTape = ::GetTapeFeeder( aPickedFeederId );

		if ( (bUseMissPickTapeRate || bUseMissPickTapeRate2) && pTape->GetPocketTeachStatus() == ePOCKET_TEACH_RETEACH && (pTape->GetMasterPartID() == 0 && !pTape->IsMultiVendorTape()))
				return bPlaceStatusChgd;
	}

	SYS_ASSERT( feederId.IsDefined());

	StPlaceData::eASSIGNED_STEP_TYPE stepType = StPlaceData::eASSIGNED_STEP_PICK_TAPE;
	if ( feedType == eFEEDTYPE_TAPE )
		stepType = StPlaceData::eASSIGNED_STEP_PICK_TAPE;

	else
		SYS_ASSERT(0);

	if ( feedType == eFEEDTYPE_TAPE)
	{
		CTapeFeeder* pFeeder = ::GetTapeFeeder( feederId );

		//Multi-Vendor Part 
		if ( pFeeder->GetMasterPartID() != 0 || pFeeder->IsMultiVendorTape())
			errCode = eMEC_FIND_TAPE_REEL_TYPE_FAILED;
		
		//No Multi-Vendor part  
		else if ( GetPickUpMissTapePocketTeach(feederId) )
			errCode = eMEC_TAPE_RETRY_EXCEEDED;
		
		if (pFeeder->GetStatus() != eTAPE_ERROR)
		{
			if ( errCode == eMEC_TAPE_RETRY_EXCEEDED)
			{
				g_StateReport.ReportErrorEx2(errCode, errLevel, eMOTOR_OK,
					m_SectionID, frntRear, 0, HeadNone,
					aPlaceData.partID, pFeeder->GetID(), 0, 0, 0, __LINE__, aPlaceData.cycle, m_frRunningLane);
			}
			else
			{
				g_StateReport.ReportErrorEx2(errCode, errLevel, eMOTOR_OK,
					m_SectionID, frntRear, 0, HeadNone,
					aPlaceData.partID, pFeeder->GetID(), 0, 0, 0, __LINE__, aPlaceData.cycle);
			}

		}

		Telemetry( EV_CLASSFEEDER, pHead->Unit(), "H%s Report MissPickRate Feeder=%d:%d check:%d ", pHead->GetName(), aPickedFeederId.Unit(), aPickedFeederId.No(), bUseMissPickTapeRate ? 1:2); 

		pFeeder->SetStatus(eTAPE_ERROR);

		CStepPick*		pAssignedPickStep = NULL;
		pAssignedPickStep = aPlaceData.pStepPickAssigned[stepType];
		
		if (pAssignedPickStep)
		{
			bool			bFeedersFound = false;
			CFeederId		altFeederId;
			CStepPick::MapFdrIDtoN	usedFeederMap;
			

			altFeederId = pAssignedPickStep->FindFeeder(aPlaceData.feederID,
														usedFeederMap, *pHead, 0,
														aPlaceData.profile.common.height - aPlaceData.profile.common.depthZ,
														aPlaceData.bAutoFeeder,
														bFeedersFound, errCode);
			
			if (!altFeederId.IsDefined())
				g_Machine.StopRunning( pHead->Gantry()->GetSectionID(), frntRear, eSTOP_OPTION_GANTRY);
		}
				
	}

	return bPlaceStatusChgd;
}


// Updates all required pickup statistic values when the pickup has failed.
void
CSeqCycle::UpdatePickStatisticsBad( int		aIdxPd,
						const StPlaceData&	aPlaceData,
							ePartFailReason	aPartFailReason,
								CFeederId	aPickedFeederId,
								CNozzleId	aHdNozzle,
						StStatisticsCycle&	arPickBadTel)
{
	if ( aPickedFeederId.Type() == eFEEDTYPE_TAPE && GetNonStopRetryTape(aPickedFeederId) == true ) return;
	
	StStatCacheRec& rCacheRec = m_StatCacheRecs[ aIdxPd];
	eStatIncCat		incMissCat = eSTAT_INC_CAT_NONE;

	switch ( aPartFailReason)
	{
	case ePART_FAIL_VAC_PICK:
	case ePART_FAIL_VIS_SHIFT_PICK:
		incMissCat = eSTAT_INC_CAT_MISPICK_VAC_PICK;
		rCacheRec.m_Head.pickMissVacPickCount++;
		rCacheRec.m_Nozzle.pickMissVacPickCount++;
		rCacheRec.m_Feeder.pickMissVacPickCount++;
		rCacheRec.m_Head.dumpCount++;
		break;

	case ePART_FAIL_VISION:
		incMissCat = eSTAT_INC_CAT_MISPICK_VISION;
		rCacheRec.m_Head.pickMissPartNGCount++;
		rCacheRec.m_Nozzle.pickMissPartNGCount++;
		rCacheRec.m_Feeder.pickMissPartNGCount++;
		rCacheRec.m_Head.dumpCount++;
		break;

	case ePART_FAIL_SVS_PICK:
		incMissCat = eSTAT_INC_CAT_MISPICK_SVS_PICK;
		rCacheRec.m_Head.pickMissSvsPickCount++;
		rCacheRec.m_Nozzle.pickMissSvsPickCount++;
		rCacheRec.m_Feeder.pickMissSvsPickCount++;
		rCacheRec.m_Head.dumpCount++;
		break;
		
	case ePART_FAIL_SVS_VISION:
		incMissCat = eSTAT_INC_CAT_MISPICK_SVS_VISION;
		rCacheRec.m_Head.pickMissSvsPartNGCount++;
		rCacheRec.m_Nozzle.pickMissSvsPartNGCount++;
		rCacheRec.m_Feeder.pickMissSvsPartNGCount++;
		rCacheRec.m_Head.dumpCount++;
		break;
		
	case ePART_FAIL_SVS:
		; // Do not record non-Pick SVS errors because they all stop the machine
		; // Do not add eSTAT_INC_CAT_NONE style records because they cannot be processed.
		return;
		
	//NOTE: MMI does NOT support below failure count yet.
	//K	case ePART_FAIL_VIS_SHIFT_PICK:
	//K		incMissCat = eSTAT_INC_CAT_MISPICK_VIS_SHIFT_PICK;
	//K		rCacheRec.m_Head.pickMissVisShiftPickCount++;
	//K		rCacheRec.m_Nozzle.pickMissVisShiftPickCount++;
	//K		rCacheRec.m_Feeder.pickMissVisShiftPickCount++;
	//K	break;
		
	case ePART_FAIL_VAC_PRE_PLACE:
		incMissCat = eSTAT_INC_CAT_MISPICK_VAC_PRE_PLACE;
		rCacheRec.m_Head.pickMissVacPrePlaceCount++;
		rCacheRec.m_Nozzle.pickMissVacPrePlaceCount++;
		rCacheRec.m_Feeder.pickMissVacPrePlaceCount++;
		rCacheRec.m_Head.dumpCount++;
		break;
		
	case ePART_FAIL_FORCE_PLACE:
		incMissCat = eSTAT_INC_CAT_MISPLACE_FORCE_PLACE;
		rCacheRec.m_Head.placeMissCount++;
		rCacheRec.m_Nozzle.placeMissCount++;
		rCacheRec.m_Feeder.placeMissCount++;
		rCacheRec.m_Head.dumpCount++;
		break;

	case ePART_FAIL_LEAD_SCAN:
		incMissCat = eSTAT_INC_CAT_MISPICK_LEAD_SCAN;
		rCacheRec.m_Head.pickMissLeadScanNGCount++;
		rCacheRec.m_Nozzle.pickMissLeadScanNGCount++;
		rCacheRec.m_Feeder.pickMissLeadScanNGCount++;
		rCacheRec.m_Head.dumpCount++;
		break;
		
	case ePART_FAIL_LCR_OK:
		incMissCat = eSTAT_INC_CAT_MISPICK_LCR_OK;
		rCacheRec.m_Head.pickMissLcrOKCount++;
		rCacheRec.m_Nozzle.pickMissLcrOKCount++;
		rCacheRec.m_Feeder.pickMissLcrOKCount++;
		rCacheRec.m_Head.dumpCount++;
		break;

	case ePART_FAIL_LCR_NG:
		incMissCat = eSTAT_INC_CAT_MISPICK_LCR_NG;
		rCacheRec.m_Head.pickMissLcrNGCount++;
		rCacheRec.m_Nozzle.pickMissLcrNGCount++;
		rCacheRec.m_Feeder.pickMissLcrNGCount++;
		rCacheRec.m_Head.dumpCount++;
		break;

	default:
		SYS_ASSERT(0);
		break;
	}

	// Below will keep track of these increments in the actual feeder object.
	// BECAUSE... if the feeder fails, then we will need to UNDO all of the increments
	//				directly related to the feeder's failure.
	// The data below is only needed when a feeder is forced into a failed state due to excessive retries.
	//
	eFeederType feedType = aPickedFeederId.Type();

	int tapePocketIndex = -1;
	if ( CTapeFeeder::IsMultiPocketTapeFeeder( aPickedFeederId ) )
		tapePocketIndex = aPlaceData.mPocketPickedIndex;

	StStatIncRec headInc( eSTAT_INC_HEAD,	aPlaceData.head, incMissCat, tapePocketIndex, aPartFailReason);
	StStatIncRec nozInc ( eSTAT_INC_NOZZLE,	aHdNozzle,		 incMissCat, tapePocketIndex, aPartFailReason);
	StStatIncRec feedInc( eSTAT_INC_FEEDER,	aPickedFeederId, incMissCat, tapePocketIndex, aPartFailReason);

	switch ( feedType)
	{
	case eFEEDTYPE_TAPE:
		AddStatisticsIncRecTape( aPickedFeederId, headInc);
		AddStatisticsIncRecTape( aPickedFeederId, nozInc );
		AddStatisticsIncRecTape( aPickedFeederId, feedInc);
		break;
		
	case eFEEDTYPE_STICK:
		AddStatisticsIncRecStick( aPickedFeederId, headInc);
		AddStatisticsIncRecStick( aPickedFeederId, nozInc );
		AddStatisticsIncRecStick( aPickedFeederId, feedInc);
		break;
		
	case eFEEDTYPE_TRAY:
		AddStatisticsIncRecTray( aPickedFeederId, headInc);
		AddStatisticsIncRecTray( aPickedFeederId, nozInc );
		AddStatisticsIncRecTray( aPickedFeederId, feedInc);
		break;
		
	default:
		SYS_ASSERT(0);
		break;
	}

	arPickBadTel.Accumulate( rCacheRec.m_Feeder);
}



// Updates all required pickup statistic values when the pickup forces a feeder into an error state.
void
CSeqCycle::UpdatePickStatisticsErr( CFeederId aPickedFeederId, StStatisticsCycle* apStatErrTel)
{
	CStatisticsInfo::CStatGetAndPutRes crit( g_StatisticsInfo );

	SYS_ASSERT( aPickedFeederId.IsDefined());

	int idxSt;
	TStatIncRecList* pStatIncList;

	// The Work Station index is the index for picks
	idxSt = ::GetConveyorObj()->GetWorkStationArrayIndx( m_CycleData.stationId );
	SYS_ASSERT( idxSt != NOT_AN_INDEX );

	// Get the increment accounting list from the feeder in error.
	switch( aPickedFeederId.Type() )
	{
	case eFEEDTYPE_TAPE:	pStatIncList = &GetStatisticsIncListTape( aPickedFeederId);		break;
	case eFEEDTYPE_STICK:	pStatIncList = &GetStatisticsIncListStick( aPickedFeederId);	break;
	case eFEEDTYPE_TRAY:	pStatIncList = &GetStatisticsIncListTray( aPickedFeederId);		break;
	// There should never be any uncertainty about a feeder ID down at this level
	default:	SYS_ASSERT(0);
	}

	// For each increment associated with this feeder, since the mispicks started accumulating,
	// UNDO any statistical increment associated with this feeder.
	//
	TStatIncRecList&	incList = *pStatIncList;
	StStatisticsCycle*	pStatCycle = NULL;

	for (int i=0; i<incList.GetCount(); i++)
	{
		const StStatIncRec& incRec = incList[i];

		ePartFailReason failReason = (ePartFailReason)incRec.failReason;
		if ( g_FacDebug.IsSupportResetRetryCountOnlyPickErr())
		{
			// the error of vision should not be deleted when retry over happen.
			if ( failReason == ePART_FAIL_VISION || failReason == ePART_FAIL_VIS_SHIFT_PICK 
				|| failReason == ePART_FAIL_SVS_VISION || failReason == ePART_FAIL_LEAD_SCAN)
				continue;
		}

		if (failReason == ePART_FAIL_LCR_NG || failReason == ePART_FAIL_LCR_OK)
			continue;

		// Determine the specific device that was being incremented because
		// that device was associated with a pick from this feeder,
		// AND get a pointer to the global statistics for that device.
		if ( incRec.incDev == eSTAT_INC_HEAD)
		{
			SYS_ASSERT( 1 <= incRec.headID  &&  incRec.headID <= SYS_MAX_HEAD);
			CHead*		pHead	= ::GetHead( incRec.headID);
			SYS_ASSERT( pHead != NULL );
			CGantry*	pGantry = pHead->Gantry();
			SYS_ASSERT( pGantry != NULL );
			int			spindle	  = pHead->GetSpindle();	// One(1) based
			GantryID	gantryId  = pGantry->Unit();		// One(1) based
			SectionID	sectionId = pGantry->GetSectionID();// One(1) based
			SYS_ASSERT( sectionId > 0 && sectionId <= SYS_MAX_GANTRY_PER_SECTION );
			int gantryIdx = ::GetGantryIdxbySection( gantryId, sectionId ); // One(1) based

			pStatCycle = &g_Statistics.statHead[ sectionId-1 ][idxSt][ gantryIdx-1 ][ spindle-1 ];
			//pStatCycle = &g_Statistics.statHead[ incRec.headID-1];
		}

		else if ( incRec.incDev == eSTAT_INC_NOZZLE)
			pStatCycle = &g_Statistics.statNozzle[idxSt][ incRec.nozzleID.Anc()-1][ incRec.nozzleID.HoleNum()-1];

		else if ( incRec.incDev == eSTAT_INC_FEEDER)
		{
			pStatCycle = &g_Statistics.statNozzle[idxSt][ incRec.nozzleID.Anc()-1][ incRec.nozzleID.HoleNum()-1];
			CFeederId fdID = incRec.feederID;
			switch( fdID.Type() )
			{
			case eFEEDTYPE_TAPE:
				pStatCycle = &g_Statistics.statTapeFeeder[ idxSt][ fdID.Unit()-1][ fdID.No()-1];
				break;
			case eFEEDTYPE_STICK:
				pStatCycle = &g_Statistics.statStickFeeder[ idxSt][ fdID.Unit()-1][ fdID.No()-1 ];
				break;
			case eFEEDTYPE_TRAY:
				pStatCycle = &g_Statistics.statTrayFeeder[ idxSt][ fdID.Unit()-1][ fdID.No()-1][ fdID.ExtNo()-1];
				break;
			// There should never be any uncertainty about a feeder ID down at this level
			default:	SYS_ASSERT(0);
			}
		}
		else
		{
			SYS_ASSERT(0);
		}

		// Decrement the specific "increment category" that had been incremented.
		switch( incRec.incCat)
		{
		case eSTAT_INC_CAT_MISPICK_VAC_PICK:		pStatCycle->pickMissVacPickCount--;			break;
		case eSTAT_INC_CAT_MISPICK_VISION:			pStatCycle->pickMissPartNGCount--;			break;
		case eSTAT_INC_CAT_MISPICK_VIS_SHIFT_PICK:	pStatCycle->pickMissVisShiftPickCount--;	break;
		case eSTAT_INC_CAT_MISPICK_VAC_PRE_PLACE:	pStatCycle->pickMissVacPrePlaceCount--;		break;
		case eSTAT_INC_CAT_MISPLACE_FORCE_PLACE:												break;
		case eSTAT_INC_CAT_MISPICK_SVS_PICK:		pStatCycle->pickMissSvsPickCount--;			break;
		case eSTAT_INC_CAT_MISPICK_SVS_VISION:		pStatCycle->pickMissSvsPartNGCount--;		break;
		case eSTAT_INC_CAT_MISPICK_LEAD_SCAN:		pStatCycle->pickMissLeadScanNGCount--;		break;
 		case eSTAT_INC_CAT_MISPICK_LCR_OK:			pStatCycle->pickMissLcrOKCount--;			break;
 		case eSTAT_INC_CAT_MISPICK_LCR_NG:			pStatCycle->pickMissLcrNGCount--;			break;

		case eSTAT_INC_CAT_FLY_LSO:					pStatCycle->pickFlyLsoCount--;				break;
		case eSTAT_INC_CAT_UP:						pStatCycle->pickUpwardCount--;				break;
		// There should never be any category problems.
		default:	
			Telemetry( EV_CLASSHEAD, 0, "Index[%d/%d], Cate[%d], HeadId[%d], NozId[%d], FeederId[%d]", 
				i, incList.GetCount(), incRec.incCat, incRec.headID, (long)incRec.nozzleID, (long)incRec.feederID);
			SYS_ASSERT(0);
		}

		// Telemetry accumulate the FEEDER changes.
		if ( apStatErrTel!=NULL  &&  incRec.incDev == eSTAT_INC_FEEDER)
		{
			switch( incRec.incCat)
			{
			case eSTAT_INC_CAT_MISPICK_VAC_PICK:		apStatErrTel->pickMissVacPickCount--;		break;
			case eSTAT_INC_CAT_MISPICK_VISION:			apStatErrTel->pickMissPartNGCount--;		break;
			case eSTAT_INC_CAT_MISPICK_VIS_SHIFT_PICK:	apStatErrTel->pickMissVisShiftPickCount--;	break;
			case eSTAT_INC_CAT_MISPICK_VAC_PRE_PLACE:	apStatErrTel->pickMissVacPrePlaceCount--;	break;
			case eSTAT_INC_CAT_MISPLACE_FORCE_PLACE:												break;
			case eSTAT_INC_CAT_MISPICK_SVS_PICK:		apStatErrTel->pickMissSvsPickCount--;		break;
			case eSTAT_INC_CAT_MISPICK_SVS_VISION:		apStatErrTel->pickMissSvsPartNGCount--;		break;
			case eSTAT_INC_CAT_MISPICK_LEAD_SCAN:		apStatErrTel->pickMissLeadScanNGCount--;	break;
 			case eSTAT_INC_CAT_MISPICK_LCR_OK:			apStatErrTel->pickMissLcrOKCount--;			break;
 			case eSTAT_INC_CAT_MISPICK_LCR_NG:			apStatErrTel->pickMissLcrNGCount--;			break;
			case eSTAT_INC_CAT_FLY_LSO:					apStatErrTel->pickFlyLsoCount--;			break;
			case eSTAT_INC_CAT_UP:						apStatErrTel->pickUpwardCount--;			break;
			// There should never be any category problems.
			default:
				;
			}
		}
	}

	// Clear out the increment records after they have been used
	// so they cannot be used twice.
	switch( aPickedFeederId.Type() )
	{
	case eFEEDTYPE_TAPE:	ClearStatisticsIncTape( aPickedFeederId);	break;
	case eFEEDTYPE_STICK:	ClearStatisticsIncStick( aPickedFeederId);	break;
	case eFEEDTYPE_TRAY:	ClearStatisticsIncTray( aPickedFeederId);	break;
	// There should never be any uncertainty about a feeder ID down at this level
	default:	SYS_ASSERT(0);
	}
}

void
CSeqCycle::UpdatePickStatisticsErrEx( CFeederId aPickedFeederId, StStatisticsCycle* apStatErrTel)
{
	CStatisticsInfo::CStatGetAndPutRes crit( g_StatisticsInfo );

	SYS_ASSERT( aPickedFeederId.IsDefined());

	int idxSt;
	TStatIncRecList* pStatIncList;

	// The Work Station index is the index for picks
	idxSt = ::GetConveyorObj()->GetWorkStationArrayIndx( m_CycleData.stationId );
	SYS_ASSERT( idxSt != NOT_AN_INDEX );

	// Get the increment accounting list from the feeder in error.
	switch( aPickedFeederId.Type() )
	{
	case eFEEDTYPE_TAPE:	pStatIncList = &GetStatisticsIncListTape( aPickedFeederId);		break;
	case eFEEDTYPE_STICK:	pStatIncList = &GetStatisticsIncListStick( aPickedFeederId);	break;
	case eFEEDTYPE_TRAY:	pStatIncList = &GetStatisticsIncListTray( aPickedFeederId);		break;
	// There should never be any uncertainty about a feeder ID down at this level
	default:	SYS_ASSERT(0);
	}

	TFaultyPocketIndexMap faultyPocketIndexMap;
	faultyPocketIndexMap.RemoveAll();
	CTapeFeeder* pTapeM = NULL;
	if ( aPickedFeederId.Type() == eFEEDTYPE_TAPE )
	{
		pTapeM = ::GetTapeFeeder( aPickedFeederId );
		pTapeM->GetFaultyPocketIndexList(faultyPocketIndexMap);
	}
	
	
	// For each increment associated with this feeder, since the mispicks started accumulating,
	// UNDO any statistical increment associated with this feeder.
	//
	TStatIncRecList&	gIncList = *pStatIncList;

	TStatIncRecList incListTest;
	incListTest.Append( gIncList );

	TStatIncRecList incList;
	incList.Append(incListTest);

	if ( pTapeM )
	{
		incList.RemoveAll();

		for (int j=0; j<incListTest.GetCount(); j++)
		{
			StStatIncRec& incRec = incListTest[j];

			if ( incRec.pocketIndex == -1 )	continue;
			if ( !faultyPocketIndexMap.IsExist(incRec.pocketIndex))	continue;
#ifdef WIN_SIM
			TRACE("UpdatePickStatisticsErrEx : DEV:0x%x, CAT:0x%x, H:%d, P:%d\n\r",incRec.incDev, incRec.incCat, incRec.headID, incRec.pocketIndex );
#endif
			incList.Append( incRec );
		}

		int pocketIndex = -1;
		for (POS pos=faultyPocketIndexMap.GetStartPosition(); pos!=NULL;)
		{
			faultyPocketIndexMap.GetNextAssoc( pos, pocketIndex);
			
			//Set Unknown this pocket to prevent begin used twice.
			pTapeM->ResetMPocketState( pocketIndex );
		}
	}

	StStatisticsCycle*	pStatCycle = NULL;

	for (int i=0; i<incList.GetCount(); i++)
	{
		const StStatIncRec& incRec = incList[i];

		ePartFailReason failReason = (ePartFailReason)incRec.failReason;
		if (failReason == ePART_FAIL_LCR_NG || failReason == ePART_FAIL_LCR_OK)
			continue;

		// Determine the specific device that was being incremented because
		// that device was associated with a pick from this feeder,
		// AND get a pointer to the global statistics for that device.
		if ( incRec.incDev == eSTAT_INC_HEAD)
		{
			SYS_ASSERT( 1 <= incRec.headID  &&  incRec.headID <= SYS_MAX_HEAD);
			CHead*		pHead	= ::GetHead( incRec.headID);
			SYS_ASSERT( pHead != NULL );
			CGantry*	pGantry = pHead->Gantry();
			SYS_ASSERT( pGantry != NULL );
			int			spindle	  = pHead->GetSpindle();	// One(1) based
			GantryID	gantryId  = pGantry->Unit();		// One(1) based
			SectionID	sectionId = pGantry->GetSectionID();// One(1) based
			SYS_ASSERT( sectionId > 0 && sectionId <= SYS_MAX_GANTRY_PER_SECTION );
			int gantryIdx = ::GetGantryIdxbySection( gantryId, sectionId ); // One(1) based

			pStatCycle = &g_Statistics.statHead[ sectionId-1 ][idxSt][ gantryIdx-1 ][ spindle-1 ];
			//pStatCycle = &g_Statistics.statHead[ incRec.headID-1];
		}

		else if ( incRec.incDev == eSTAT_INC_NOZZLE)
			pStatCycle = &g_Statistics.statNozzle[ idxSt][ incRec.nozzleID.Anc()-1][ incRec.nozzleID.HoleNum()-1];

		else if ( incRec.incDev == eSTAT_INC_FEEDER)
		{
			pStatCycle = &g_Statistics.statNozzle[ idxSt][ incRec.nozzleID.Anc()-1][ incRec.nozzleID.HoleNum()-1];
			CFeederId fdID = incRec.feederID;
			switch( fdID.Type() )
			{
			case eFEEDTYPE_TAPE:
				pStatCycle = &g_Statistics.statTapeFeeder[ idxSt][ fdID.Unit()-1][ fdID.No()-1];
				break;
			case eFEEDTYPE_STICK:
				pStatCycle = &g_Statistics.statStickFeeder[ idxSt][ fdID.Unit()-1][ fdID.No()-1 ];
				break;
			case eFEEDTYPE_TRAY:
				pStatCycle = &g_Statistics.statTrayFeeder[ idxSt][ fdID.Unit()-1][ fdID.No()-1][ fdID.ExtNo()-1];
				break;
			// There should never be any uncertainty about a feeder ID down at this level
			default:	SYS_ASSERT(0);
			}
		}
		else
		{
			SYS_ASSERT(0);
		}

		// Decrement the specific "increment category" that had been incremented.
		switch( incRec.incCat)
		{
		case eSTAT_INC_CAT_MISPICK_VAC_PICK:		pStatCycle->pickMissVacPickCount--;			break;
		case eSTAT_INC_CAT_MISPICK_VISION:			pStatCycle->pickMissPartNGCount--;			break;
		case eSTAT_INC_CAT_MISPICK_VIS_SHIFT_PICK:	pStatCycle->pickMissVisShiftPickCount--;	break;
		case eSTAT_INC_CAT_MISPICK_VAC_PRE_PLACE:	pStatCycle->pickMissVacPrePlaceCount--;		break;
		case eSTAT_INC_CAT_MISPLACE_FORCE_PLACE:												break;
		case eSTAT_INC_CAT_MISPICK_SVS_PICK:		pStatCycle->pickMissSvsPickCount--;			break;
		case eSTAT_INC_CAT_MISPICK_SVS_VISION:		pStatCycle->pickMissSvsPartNGCount--;		break;
		case eSTAT_INC_CAT_MISPICK_LEAD_SCAN:		pStatCycle->pickMissLeadScanNGCount--;		break;
 		case eSTAT_INC_CAT_MISPICK_LCR_OK:			pStatCycle->pickMissLcrOKCount--;			break;
 		case eSTAT_INC_CAT_MISPICK_LCR_NG:			pStatCycle->pickMissLcrNGCount--;			break;

		case eSTAT_INC_CAT_FLY_LSO:					pStatCycle->pickFlyLsoCount--;				break;
		case eSTAT_INC_CAT_UP:						pStatCycle->pickUpwardCount--;				break;
		// There should never be any category problems.
		default:	
			Telemetry( EV_CLASSHEAD, 0, "Index[%d/%d], Cate[%d], HeadId[%d], NozId[%d], FeederId[%d]", 
				i, incList.GetCount(), incRec.incCat, incRec.headID, (long)incRec.nozzleID, (long)incRec.feederID);
			SYS_ASSERT(0);
		}

		// Telemetry accumulate the FEEDER changes.
		if ( apStatErrTel!=NULL  &&  incRec.incDev == eSTAT_INC_FEEDER)
		{
			switch( incRec.incCat)
			{
			case eSTAT_INC_CAT_MISPICK_VAC_PICK:		apStatErrTel->pickMissVacPickCount--;		break;
			case eSTAT_INC_CAT_MISPICK_VISION:			apStatErrTel->pickMissPartNGCount--;		break;
			case eSTAT_INC_CAT_MISPICK_VIS_SHIFT_PICK:	apStatErrTel->pickMissVisShiftPickCount--;	break;
			case eSTAT_INC_CAT_MISPICK_VAC_PRE_PLACE:	apStatErrTel->pickMissVacPrePlaceCount--;	break;
			case eSTAT_INC_CAT_MISPLACE_FORCE_PLACE:												break;
			case eSTAT_INC_CAT_MISPICK_SVS_PICK:		apStatErrTel->pickMissSvsPickCount--;		break;
			case eSTAT_INC_CAT_MISPICK_SVS_VISION:		apStatErrTel->pickMissSvsPartNGCount--;		break;
			case eSTAT_INC_CAT_MISPICK_LEAD_SCAN:		apStatErrTel->pickMissLeadScanNGCount--;	break;
 			case eSTAT_INC_CAT_MISPICK_LCR_OK:			apStatErrTel->pickMissLcrOKCount--;			break;
 			case eSTAT_INC_CAT_MISPICK_LCR_NG:			apStatErrTel->pickMissLcrNGCount--;			break;
			case eSTAT_INC_CAT_FLY_LSO:					apStatErrTel->pickFlyLsoCount--;			break;
			case eSTAT_INC_CAT_UP:						apStatErrTel->pickUpwardCount--;			break;
			// There should never be any category problems.
			default:
				;
			}
		}
	}

	int pocketIndex = -1;
	POS pos;
	// Clear out the increment records after they have been used
	// so they cannot be used twice.
	switch( aPickedFeederId.Type() )
	{
	case eFEEDTYPE_TAPE:	
							pocketIndex = -1;
							for ( pos=faultyPocketIndexMap.GetStartPosition(); pos!=NULL;)
							{
								faultyPocketIndexMap.GetNextAssoc( pos, pocketIndex);
								ClearStatisticsIncTapeEx( aPickedFeederId, pocketIndex );
							}
							break;

	case eFEEDTYPE_STICK:	ClearStatisticsIncStick( aPickedFeederId);	break;
	case eFEEDTYPE_TRAY:	ClearStatisticsIncTray( aPickedFeederId);	break;
	// There should never be any uncertainty about a feeder ID down at this level
	default:	SYS_ASSERT(0);
	}
}


void
CSeqCycle::UpdatePlaceStatistics( )
{
	// Ignore cycles that don't place parts
	if ( m_PrefetchOp != eCYCLE_PREFETCH_NONE)
		return;

	for (int i=0; i<m_CycleData.nCount; i++)
	{
		StPlaceData& pd = m_CycleData.placeData[i];

		// Don't count placements that didn't place successfully
		if ( pd.status != ePLACE_OK)			continue;
		if ( !pd.placedFeederID.IsDefined())	continue;

		if ( pd.profile.IsVirtualPick() )
			continue;

		UpdatePlaceStatistics( i, pd, pd.pickedFeederID);
	}
}

//Set top priority pallet to move to buffer
void
CSeqCycle::SetBufferChangePalletNo()
{
	CPCBoard* pBoard = m_CycleData.pPcbBoard;
	if (!pBoard)
		return;

	TPcbID pcbID = pBoard->GetPcbID();

	for (int i = 1; i < _countof(g_TrayFeeder); i++)
	{
		CTrayFeeder* pTrayFeeder = ::GetTrayFeeder(i);
		if (!(pTrayFeeder && pTrayFeeder->IsInstalled()))
			continue;

		if (!::IsSectionCompatible(pTrayFeeder->GetSectionID(), m_SectionID))
			continue;

		if (!::IsFrontRearCompatible(pTrayFeeder->GetFrontOrRear(), m_frRunningLane))
			continue;

		if (pTrayFeeder->GetEmptyPalletCount() < 2)
			continue;

		if (pTrayFeeder->IsManualBuffOutCmdProcessing())
		{
			CFeederId trayFeederId = pTrayFeeder->GetManualBuffOutCmdFeederId();
			CTray* pTray = GetTray(trayFeederId);

			if (pTray->GetStatus() == eTRAY_READY)
				pTrayFeeder->SetManualBuffOutCmdProcessing(CFeederId(0), false);
			else
				continue;
		}

		TMapPartIdToCount mapEmptyPalletPartIdToCount;
		pTrayFeeder->GetPartCountOfEmptyPallet(mapEmptyPalletPartIdToCount);

		double lowerPartCountPerPCB = NaN;
		PartID TopPriorityPartID = -1;
		for (POS pos = mapEmptyPalletPartIdToCount.GetStartPosition(); pos != NULL; )
		{
			PartID partID;
			long count = mapEmptyPalletPartIdToCount.GetNextAssoc(pos, partID);
			long partCountUsedInPer1PCB = pTrayFeeder->GetPartCountUsedInPer1PCB(pcbID, partID);
			if (!partCountUsedInPer1PCB)
				continue;

			double partPerPCB = (double)count / partCountUsedInPer1PCB;
			if (lowerPartCountPerPCB > partPerPCB)
			{
				lowerPartCountPerPCB = partPerPCB;
				TopPriorityPartID = partID;
			}
		}
		if (TopPriorityPartID < 0)
			continue;
		pTrayFeeder->SetTopPriorityPalletOfBuffer(TopPriorityPartID);
	}
	return;
}

void
CSeqCycle::UpdatePlaceStatistics( int aIdxPd, const StPlaceData& aPlaceData, CFeederId aPickedFeederId )
{
	if ( aPickedFeederId.Type() == eFEEDTYPE_TAPE && GetNonStopRetryTape(aPickedFeederId) == true ) return;

	StStatCacheRec& rCacheRec = m_StatCacheRecs[ aIdxPd];
	// Record the successful alignment.

	eCameraGroupType camGrpType = aPlaceData.profile.visCommon.cameraGroupType;

	if ( camGrpType & eCAMERA_GROUP_UP )
	{
		rCacheRec.m_Head.placeUpwardCount++;
		rCacheRec.m_Nozzle.placeUpwardCount++;
		rCacheRec.m_Feeder.placeUpwardCount++;
	}
	else
	{
		// Consider non-UP alignment as FLY/LSO Alignment...
		rCacheRec.m_Head.placeFlyLsoCount++;
		rCacheRec.m_Nozzle.placeFlyLsoCount++;
		rCacheRec.m_Feeder.placeFlyLsoCount++;
	}
}



int
HeadCompare( const void *apPlace1, const void *apPlace2 )
{
	StPlaceData *p1 = *( StPlaceData** )apPlace1;
	StPlaceData *p2 = *( StPlaceData** )apPlace2;
	if ( p1==NULL  &&  p2==NULL)
		return 0;
	else if ( p1==NULL  &&  p2!=NULL)
		return -1;
	else if ( p1!=NULL  &&  p2==NULL)
		return 1;
	ASSERT( p1->pHead);
	ASSERT( p2->pHead);

	int unit1 = p1->pHead->Unit();
	int unit2 = p2->pHead->Unit();
	if ( unit1 > unit2 )
		return 1;
	if ( unit1 < unit2 )
		return -1;
	return 0;
//	return strcmp( p1->pHead->Name(), p2->pHead->Name());
}
