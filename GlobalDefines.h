////////////////////////////////////////////////////////////////////////////////
//			Copyright (c) 2006 Samsung TechWin Co., Ltd.
//					All rights reserved.
//
//	This source code contains confidential, trade secret material of
//	Samsung TechWin. Any attempt or participation in deciphering, decoding,
//	reverse engineering or in any way altering the source code is
//	strictly prohibited, unless the prior written consent of Samsung TechWin
//	is obtained.
//
//	File Version History
//		V0.1 : 2006-xx-xx First Release
//
//
#ifndef __GLOBALDEFINES_H__
#define __GLOBALDEFINES_H__

namespace smart 
{


// General status returned by many functions to indicate pass/fail operation.
//
// eOK		Indicates the operation was a success
// eNG		Indicates a process error occurred during the operation (its likely that the MMI would
//				want to use this opportunity to display a message box to the customer)
// eFAULT	Indicates that an error has been declared through CStateReporting and has 
//				resulted in a message board notification to the customer.
//			NOTE: CStateReporting can also send informational messages to the customer.
//			NOTE: This eFAULT value does not mean that the machine state is eMS_FAULT, it just
//			means that this function operation has been abandoned due to an error.
//
enum	eStatus { eOK, eNG, eFAULT };

#define PI									3.14159265358979323846
#define DEG_PER_RAD							(180/PI) 
#define RAD_PER_DEG							(1.0/DEG_PER_RAD) 
#define DtoR								(PI/180) 
#define MSEC_PER_SEC						1000.0

#define	NaN	(double)(1e38)	// sign(1).exp(all ones).mantissa(!=0)
							//	IEEE Not a Number - all calculations against this 
							//	value will result in NaN

// determine number of elements in an array (not bytes)
#ifndef _countof
#define _countof(array) (sizeof(array)/sizeof(array[0]))
#endif

#define NOT_AN_INDEX			-1				// Used to indicate an index is not refering 
												// to an actual array element. Equivalent to 
												// NULL for pointers.

#define NOT_AN_ID				0				// Used to indicate an ID (such as GantryID, DumpID, etc.)
												// is not refering to an actual device. An ID is used as
												// an index into an array of devices. The first array element
												// is not used. 
												// Equivalent to NULL for pointers.

#define SW_ERROR_TIMEOUT		15000			// Timeout value for S/W errors. This timeout 
												// should never expire unless a S/W bug exists.
												// All H/W timeouts must be shorter than this 
												// value.


	//                   Range (eHandlingSpeed)
	//                    1    2    3    4    5
	// Scale=2.2, Speed 100%  45%  20%   9%   4%   // Slowest is too slow??
	// Scale=2.0, Speed 100%  50%  25%  12%   6%   // Each step is 1/2
	// Scale=1.8, Speed 100%  55%  31%  17%  10%
	// Scale=1.6, Speed 100%  62%  39%  24%  15%   // 
	// Scale=1.4, Speed 100%  71%  51%  36%  26%   // Could this be the best??
	// Scale=1.2, Speed 100%  83%  69%  58%  48%   // Slowest is too fast??
	// Scale=1.0, Speed 100% 100% 100% 100% 100%   // No variations
#define	SPEED_SCALE				1.8				// This is the scale that converts eHandlingSpeed
												// values to a fraction of the machine's maximum
												// speed and acceleration.

#define	TOUCH_DOWN_DIST_DEFAULT	2.0

enum eDirection
{
	eDIR_DN,
	eDIR_UP
};


enum ePartOnState
{
	ePART_OFF,
	ePART_ON,
	ePART_SCAN
};


enum eFrontPanelButtons
{
	eFPB_NONE,
	eFPB_READY,
	eFPB_START,
	eFPB_STOP,
	eFPB_RESET,
	eFPB_CHANGE_1F,
	eFPB_CHANGE_1R,
	eFPB_SAFEGUARD,
	eFPB_MAX
};

enum eOpPanelButtons
{
	eOPB_NONE,
	eOPB_READY,
	eOPB_START,
	eOPB_ENABLE,		// Enables the eOPB_START
	eOPB_STOP,
	eOPB_RESET,
	eOPB_CHANGE,
	eOPB_SELECT,		// Selects the OpPanel F or R
	eOPB_SAFEGUARD,
	eOPB_DOOROPEN,
	eOPB_MAX
};

enum eGantryRegion
{
	eREGION_NONE, 
	eREGION_FIDUCIAL,
	eREGION_DUMP, 
	eREGION_NOZZLE, 
	eREGION_PICKUP, 
	eREGION_ALIGN, 
	eREGION_LEAD_CHECK,
	eREGION_PLACE, 
	eREGION_MAX
};


enum eHeadMail						// General HeadBlock task mail notifications
{
	eHEAD_MAIL_NONE,
	eHEAD_MAIL_DEBUG_WAKEUP,		// Periodic wakeup of the task to make setting debug break points easier
	eHEAD_MAIL_REQ,					// Application level request, (e.g. Move(...)) has been received
	eHEAD_MAIL_NEARLY_DONE_POS,		// Motor Controller has declared axis has met nearly done position criteria
	eHEAD_MAIL_NEARLY_DONE_TIME,	// Head Block CAxisReq timer notification indicates a motion is nearly done by time
	eHEAD_MAIL_DONE_DELAY,			// A motion done, delayed by request, is now done
	eHEAD_MAIL_AXIS_STARTED,		// Motor controller has declared an axis motion to be started.
	eHEAD_MAIL_AXIS_DONE,			// Motor controller has declared an axis motion to be done.

	eHEAD_MAIL_AXIS_MACRO_DONE,		// Motor controller has declared an axis 
									//Macro motion to be done.

	eHEAD_MAIL_SAFETY,				// Safety Device indicates that a safety state change should be reevaluated
	eHEAD_MAIL_TIMER,				// ?
	eHEAD_MAIL_BREAK_POINT,			// ?
	eHEAD_MAIL_TIMEOUT_MOTOR,		// Head Block CAxisReq timer notification indicates an axis motion has exceeded reasonable completion time.
	eHEAD_MAIL_PURGE				// Delayed PURGE should be processed.
};


enum eGantryMail					// General Gantry task mail notifications
{
	eGANTRY_MAIL_NONE,
	eGANTRY_MAIL_DEBUG_WAKEUP,		// Periodic wakeup of the task to make setting debug break points easier
	eGANTRY_MAIL_HOME,				// Application level request to home the gantry
	eGANTRY_MAIL_REQ,				// Application level request, (e.g. Move(...)) has been received
	eGANTRY_MAIL_NEARLY_DONE_POS,	// Motor Controller has declared axis has met nearly done position criteria
	eGANTRY_MAIL_NEARLY_DONE_TIME,	// Head Block CAxisReq timer notification indicates a motion is nearly done by time
	eGANTRY_MAIL_DONE_DELAY,		// A motion done, delayed by request, is now done
	eGANTRY_MAIL_AXIS_STARTED,		// Motor controller has declared an axis motion started.
	eGANTRY_MAIL_AXIS_DONE,			// Motor controller has declared an axis motion to be done.
	eGANTRY_MAIL_SAFETY,			// Safety Device indicates that a safety state change should be reevaluated
	eGANTRY_MAIL_TIMER,				// ?
	eGANTRY_MAIL_BREAK_POINT,		// ?
	eGANTRY_MAIL_TIMEOUT_MOTOR,		// Head Block CAxisReq timer notification indicates an axis motion has exceeded reasonable completion time.
	eGANTRY_MAIL_PROCESS_POST_MOTION_DONE,	//Restore axis gain when an axis motion is done.
	eGANTRY_MAIL_SHOW				// Display the position of the gantry
};


#define NOZZLE_LENGTH_TYP				13.5	// Nozzle length for most nozzles
//#define NOZZLE_LENGTH_NONE_OR_UNKNOWN	20.0	// Nozzle length allowance if nozzle type is unknown
#define NOZZLE_LENGTH_NONE_OR_UNKNOWN	13.5	// Qhs Requirement - Nozzle length allowance if nozzle type is unknown
												//	NOTE: Making this different form the TYP causes the nozzle tip height
												//		  in CHeadBlockQhs to produce incompatible Z-axis positions.
												//		  And CHeadBlockQhs logic is all based on nozzle tip
												//		  position, not Z-axis position
//#define NOZZLE_LENGTH_MAX				30.0	// Nozzle length sanity check
#define NOZZLE_LENGTH_MAX				35.0	// Nozzle length sanity check

#define NOZZLE_TIP_LENGTH_NONE_OR_UNKNOWN	6.7	// The length between flange and tip on SM nozzle.
#define NOZZLE_OUTER_DIAMETER_DEFAULT		1.0	// VN040, VN045
#define NOZZLE_BODY_DIAMETER_DEFAULT		7.0	// VN040, VN045
#define CN_NOZZLE_BODY_DIAMETER_DEFAULT		9.8	// CN Nozzle

enum eMoveOpt
{
	eMOVE_OPT_NONE					=0x00000000,
	eMOVE_OPT_NO_LOCK				=0x00000001,	// MP
	eMOVE_OPT_NO_INTERMEDIATE		=0x00000002,	// GantryDual
	eMOVE_OPT_DONE_DELAY			=0x00000004,	// MEI & CGantryQ - maybe implement as a separate request by 
													//	CGantryQ to MEI which will APPEND a null motion with a 
													//	pre-delay to maintain zero-latency execution.
	eMOVE_OPT_Z_LOW_FLY				=0x00000008,	// 	 XY Option - Allows Z moves below the 
													//		traveling height while XY is moving.
													//		e.g. Use this on the last XY motion 
													//		before Z down to placement.
	eMOVE_OPT_Z_OVERRIDE			=0x00000010,	// XY Option - Allows the gantry to move 
													//		even if Z is below the traveling 
													//		height.
													//		e.g. Use this over an UP Camera 
													//		during MFOV inspections.
	eMOVE_OPT_SAFE					=0x00000020,	// axis will be moving to a safe position
	eMOVE_OPT_SET_SPEEDLIMIT		=0x00000040,
	eMOVE_OPT_RESET_SPEEDLIMIT		=0x00000080,
	eMOVE_OPT_MINIMUM				=0x00000100,
	eMOVE_OPT_USE_VMAX_CONST		=0x00000200,
	eMOVE_OPT_STOP_MOTION			=0x00000400,
	eMOVE_OPT_USE_NEARLY_MIN_XY		=0x00000800,
	eMOVE_OPT_USE_NEARLY_MIN_AXIS	=0x00001000,
	eMOVE_OPT_HOLD					=0x00002000,
	eMOVE_OPT_BLENDED				=0x00004000,
	eMOVE_OPT_XY_SETTLE				=0x00008000,
	eMOVE_OPT_DEP					=0x00010000,	// This motion is dependent on other motions
	eMOVE_OPT_TRIG_MOTION_MODIFY	=0x00020000,	// This motion profile will be modified during motion by an input signal
	eMOVE_OPT_ADJUST				=0x00040000,
	eMOVE_OPT_RESET_REGIONS			=0x00080000,
	eMOVE_OPT_EXPAND_REGION			=0x00100000,
	eMOVE_OPT_FINE					=0x00200000,
	eMOVE_OPT_NEAR_LIMIT_MOVE		=0x00400000,	// Newly added to adjust following Gantry speed to apposite gantry.
	eMOVE_OPT_ENHANCED_SHORT_MOVE	=0x00800000,
	eMOVE_OPT_OPTIONS_MASK			=0x00FFFFFF,

	eMOVE_OPT_REGION_MASK			=0x07000000,// eGantryRegion 3-bits shift in macros below if this value is changed.
	eMOVE_OPT_SPEED_WEIGHT_MASK		=0xF8000000 // eSpeedWeight 5-bits shift in macros below if this value is changed.
};

#define REGION_MASK( aeGantryRegion) (aeGantryRegion << 24) & eMOVE_OPT_REGION_MASK
#define REGION_UNMASK( aeGantryRegion) ((unsigned long)(aeGantryRegion & eMOVE_OPT_REGION_MASK) >> 24)

#define WEIGHT_MASK( aSpeedWeight ) (aSpeedWeight << 27) & eMOVE_OPT_SPEED_WEIGHT_MASK
#define WEIGHT_UNMASK( aSpeedWeight ) ((unsigned long)(aSpeedWeight & eMOVE_OPT_SPEED_WEIGHT_MASK) >> 27)

enum eMoveOpt2
{
	eMOVE_OPT2_NONE					=0x00000000,
	eMOVE_OPT2_FIRST_PICK_MOVE		=0x00000001,
	eMOVE_OPT2_MOTION_MODIFY		=0x00000002,
	eMOVE_OPT2_T_ABS_MOVE			=0x00000004,
	eMOVE_OPT2_T_ALIGN_MOVE			=0x00000008
};

enum eStepType
{
	eSTEP_NONE,				// BEGIN Primary Steps

	eSTEP_CYCLE_OPTIMIZE,

	eSTEP_DUMP,

	eSTEP_VAC_STUFF,

	eSTEP_ANC_PUT_PRE_INSP,
	eSTEP_ANC_PUT,
	eSTEP_ANC_PUT_POST_INSP,

	eSTEP_ANC_GET_PRE_INSP,
	eSTEP_ANC_GET,
	eSTEP_ANC_GET_POST_INSP,
	eSTEP_ANC_GET_POST_NOZ,

	eSTEP_RETRY_FEEDING,
	eSTEP_POCKET_XY,

	eSTEP_PRE_PICK_INSP,

	eSTEP_PICK,

	eSTEP_PRE_FLUX,

	eSTEP_ALIGN,

	eSTEP_LEAD_CHECK,
	
	eSTEP_SIDE_VIEW,
	
	eSTEP_POST_FLUX,

	eSTEP_LCR,

	eSTEP_PLACE,

	eSTEP_PBI,

	eSTEP_MAX,				// Maximum number of steps for one head in a cycle

eSTEP_VARIATION_BEGIN,		// BEGIN Step Variations

	eSTEP_CYCLE_OPTIMIZE_TRAY_FLY,
	eSTEP_CYCLE_OPTIMIZE_TRAY_SMF,
	eSTEP_CYCLE_OPTIMIZE_TRAY,
	eSTEP_CYCLE_OPTIMIZE_QHS,
	eSTEP_CYCLE_OPTIMIZE_QMF,
	eSTEP_CYCLE_OPTIMIZE_RHS,
	eSTEP_CYCLE_OPTIMIZE_AP,
	eSTEP_CYCLE_OPTIMIZE_FLY,
	eSTEP_CYCLE_OPTIMIZE_SMF,
	eSTEP_CYCLE_OPTIMIZE_FLY_TWIN,
	eSTEP_CYCLE_OPTIMIZE_VM,
	eSTEP_CYCLE_OPTIMIZE_PMF,
	eSTEP_CYCLE_OPTIMIZE_STD,

	eSTEP_DUMP_STD,
	eSTEP_DUMP_FLY,
	eSTEP_DUMP_FLY_TWIN,
	eSTEP_DUMP_QHS,
	eSTEP_DUMP_QMF,
	eSTEP_DUMP_PMF,
	eSTEP_DUMP_SMF,
	eSTEP_DUMP_RHS,
	eSTEP_DUMP_AP,
	eSTEP_DUMP_VM,
	eSTEP_DUMP_TRAY,
	eSTEP_DUMP_TRAY_FLY,
	eSTEP_DUMP_TRAY_FLY_TWIN,
	eSTEP_DUMP_TRAY_VM,
	eSTEP_DUMP_TRAY_SMF,
	eSTEP_DUMP_TRAY_AP,
	eSTEP_DUMP_CONVEYOR,

	eSTEP_VAC_STUFF_STD,

	eSTEP_ANC_PUT_PRE_INSP_FLY,
	eSTEP_ANC_PUT_PRE_INSP_SMF,
	eSTEP_ANC_PUT_PRE_INSP_VM,
	eSTEP_ANC_PUT_PRE_INSP_QMF,
	eSTEP_ANC_PUT_PRE_INSP_PMF,

	eSTEP_ANC_PUT_STD,
	eSTEP_ANC_PUT_FLY,
	eSTEP_ANC_PUT_FLY_TWIN,
	eSTEP_ANC_PUT_VM,
	eSTEP_ANC_PUT_QHS,
	eSTEP_ANC_PUT_QMF,
	eSTEP_ANC_PUT_PMF,
	eSTEP_ANC_PUT_AP,
	eSTEP_ANC_PUT_SMF,
	eSTEP_ANC_PUT_RHS,
	
	eSTEP_ANC_PUT_POST_INSP_FLY,
	eSTEP_ANC_PUT_POST_INSP_SMF,
	eSTEP_ANC_PUT_POST_INSP_VM,
	eSTEP_ANC_PUT_POST_INSP_QMF,
	eSTEP_ANC_PUT_POST_INSP_PMF,

	eSTEP_ANC_GET_PRE_INSP_FLY,
	eSTEP_ANC_GET_PRE_INSP_SMF,
	eSTEP_ANC_GET_PRE_INSP_VM,
	eSTEP_ANC_GET_PRE_INSP_QMF,
	eSTEP_ANC_GET_PRE_INSP_PMF,

	eSTEP_ANC_GET_STD,
	eSTEP_ANC_GET_FLY,
	eSTEP_ANC_GET_FLY_TWIN,
	eSTEP_ANC_GET_VM,
	eSTEP_ANC_GET_QHS,
	eSTEP_ANC_GET_QMF,
	eSTEP_ANC_GET_PMF,
	eSTEP_ANC_GET_AP,
	eSTEP_ANC_GET_SMF,
	eSTEP_ANC_GET_RHS,

	eSTEP_ANC_GET_POST_INSP_FLY,
	eSTEP_ANC_GET_POST_INSP_SMF,
	eSTEP_ANC_GET_POST_INSP_RHS,
	eSTEP_ANC_GET_POST_INSP_VM,
	eSTEP_ANC_GET_POST_NOZ_FLY,
	eSTEP_ANC_GET_POST_INSP_QMF,
	eSTEP_ANC_GET_POST_INSP_PMF,

	eSTEP_RETRY_FEEDING_STD,
	eSTEP_POCKET_XY_STD,
	eSTEP_POCKET_XY_MULTI, // Ju! please check this. This may should be merged.

	eSTEP_PICK_TAPE_VM,
	eSTEP_PICK_TAPE_AP,
	eSTEP_PICK_TAPE_QHS,
	eSTEP_PICK_TAPE_QMF,
	eSTEP_PICK_TAPE_PMF,
	eSTEP_PICK_TAPE_FLY,
	eSTEP_PICK_TAPE_FLY_TWIN,
	eSTEP_PICK_TAPE_SMF,
	eSTEP_PICK_TAPE_RHS,	
	eSTEP_PICK_STICK,
	eSTEP_PICK_STICK_AP,
	eSTEP_PICK_TRAY_FLY,
	eSTEP_PICK_TRAY_AP,
	eSTEP_PICK_TRAY_VM,
	eSTEP_PICK_TRAY_RHS,
	eSTEP_PICK_TRAY,

	eSTEP_PRE_FLUX_FLY,
	eSTEP_PRE_FLUX_QMF,
	eSTEP_PRE_FLUX_PMF,
	eSTEP_PRE_FLUX_SMF,
	eSTEP_PRE_FLUX_VHP,
	eSTEP_PRE_FLUX_RHS,
	eSTEP_POST_FLUX_FLY,
	eSTEP_POST_FLUX_QMF,
	eSTEP_POST_FLUX_PMF, 
	eSTEP_POST_FLUX_SMF,
	eSTEP_POST_FLUX_VHP,
	eSTEP_POST_FLUX_RHS,
		
	eSTEP_ALIGN_AP,
	eSTEP_ALIGN_RHS,
	eSTEP_ALIGN_QHS,
	eSTEP_ALIGN_QMF,
	eSTEP_ALIGN_QMF_MFOV,
	eSTEP_ALIGN_FLY,
	eSTEP_ALIGN_FLY_TWIN,
	eSTEP_ALIGN_VM,
	eSTEP_ALIGN_SMF,
	eSTEP_ALIGN_LSO,
	eSTEP_ALIGN_LSO_VM,
	eSTEP_ALIGN_LSO_QMF,
	eSTEP_ALIGN_UP,
	eSTEP_ALIGN_UP_SYNC,
	eSTEP_ALIGN_UP_MFOV,
	eSTEP_ALIGN_PMF,
	eSTEP_ALIGN_PMF_MFOV,

	eSTEP_LEAD_CHECK_QHS,
	eSTEP_LEAD_CHECK_QMF,
	eSTEP_LEAD_CHECK_PMF,
	eSTEP_LEAD_CHECK_SMF,
	eSTEP_LEAD_CHECK_FLY,
	eSTEP_LEAD_CHECK_FLY_TWIN,
	eSTEP_LEAD_CHECK_VM,
	
	eSTEP_SIDE_VIEW_QHS,
	eSTEP_SIDE_VIEW_QMF,
	eSTEP_SIDE_VIEW_PMF,
	eSTEP_SIDE_VIEW_SMF,
	eSTEP_SIDE_VIEW_FLY,
	eSTEP_SIDE_VIEW_FLY_TWIN,
	eSTEP_SIDE_VIEW_VM,

	eSTEP_PLACE_VM,
	eSTEP_PLACE_AP,
	eSTEP_PLACE_QHS,
	eSTEP_PLACE_QMF_LCR,
	eSTEP_PLACE_QMF,
	eSTEP_PLACE_PMF_LCR,
	eSTEP_PLACE_PMF,
	eSTEP_PLACE_SMF,
	eSTEP_PLACE_FLY,
	eSTEP_PLACE_FLY_TWIN,
	eSTEP_PLACE_RHS_LCR,
	eSTEP_PLACE_RHS,

	eSTEP_PBI_SCM,
	eSTEP_PBI_STD,

	eSTEP_VARIATION_END		// END Step Variations
};


enum eCycleStatus
{
	eCYCLE_OK,
	eCYCLE_STOPPED,		// The Step wants to stop the cycle i.e. dump-stop.
	eCYCLE_FAILED,		// The Step encountered a failure and should not retry.
	eCYCLE_ABORT,		// The Step discovered an eMS_FAULT or eMS_FREEZE condition.
	eCYCLE_MAX
};

// belt error
#define BE_NOERROR				((BYTE) NOERROR)
#define BE_AXISHWFAULT			((BYTE)'\x01')
#define BE_AXISBUSY				((BYTE)'\x02')

// rail error
#define RE_NOERROR				((BYTE) NOERROR)
#define RE_AXISHWFAULT			((BYTE)'\x01')
#define RE_AXISBUSY				((BYTE)'\x02')
#define RE_NOTHOMED				((BYTE)'\x03')
#define RE_POSOUTOFRANGE		((BYTE)'\x04')
#define RE_SENSORBLOCKED		((BYTE)'\x05')
#define RE_HWLIMIT				((BYTE)'\x06')
#define RE_STATIONBUSY			((BYTE)'\x07')
#define RE_UNDEFINEDPARAM		((BYTE)'\x08')
#define RE_AMP_ALARM			((BYTE)'\x09')
#define RE_FOLLOWINGERROR		((BYTE)'\x0a')
#define RE_DATASETTINGERROR		((BYTE)'\x0b')
#define RE_INVALIDZONEPARAM		((BYTE)'\xf0')	/* CP-60 only */
#define RE_INVALIDZONECODE		((BYTE)'\xf1')	/* CP-60 only */
#define RE_NOTINMOSTINSTARTCAL	((BYTE)'\xf2')

// shuttle error
#define SE_NOERROR				((BYTE) NOERROR)
#define SE_AXISHWFAULT			((BYTE)'\x01')
#define SE_AXISBUSY				((BYTE)'\x02')
#define SE_NOTHOMED				((BYTE)'\x03')
#define SE_POSOUTOFRANGE		((BYTE)'\x04')
#define SE_ENTRYBLOCKED			((BYTE)'\x05')
#define SE_HWLIMIT				((BYTE)'\x06')
#define SE_STATIONBUSY			((BYTE)'\x07')
#define SE_UNDEFINEDPARAM		((BYTE)'\x08')
#define SE_AMP_ALARM			((BYTE)'\x09')
#define SE_FOLLOWINGERROR		((BYTE)'\x0a')
#define SE_DATASETTINGERROR		((BYTE)'\x0b')

typedef void*  PAxisReq;	// Motion tag for a CAxis request

#define	MAX_AGILE_CONTROLLER		10

#define GANTRYMAP_USED_FIDUCIALS	6
#define GANTRYMAP_USED_AREAS		2		// Number of Mapping Rectangles for one area
#define THERMALMAP_USED_FIDUCIALS	12
#define THERMALMAP_USED_AREAS		4		// Number of Mapping Rectangles for one area
#define MAX_HEADBLOCK_PER_GANTRY	1
#define MAX_HEADBLOCK				(MAX_HEADBLOCK_PER_GANTRY * SYS_MAX_GANTRY)
//Replaced with SYS_MAX_HEAD_CONNECTORS.   #define MAX_HEAD_CONNECTOR			(SYS_MAX_GANTRY * 2)
#define MAX_SEQ_ENGINE_PER_SECTION	SYS_MAX_GANTRY_PER_SECTION
#define MAX_SEQ_SECTION				SYS_MAX_SECTION
#define MAX_SEQ_ENGINE				SYS_MAX_GANTRY
#define MAX_SEQ_CYCLE				MAX_SEQ_ENGINE
#define MAX_GANTRYMAPPING			SYS_MAX_SECTION
#define MAX_OP_PANEL				2

// Specifies the max num of CStepDumpStd objects needed. One CStepDumpStd obj is needed for each possible 
// interaction between a gantry and a CDumpBox device. If each CDumpBox can be used only by one gantry,
// the number of interactions is limited to the number of CDumpBoxS. If a CDumpBox can be
// shared between 2 gantries then the count must be increased by the number of sharing gantry
// interactions.  
#define MAX_STEP_DUMP				SYS_MAX_DUMPBOX
#define MAX_STEP_VAC_STUFF_STD		SYS_MAX_GANTRY
#define MAX_STEP_ANC				SYS_MAX_ANC
#define MAX_STEP_NOZ_INSP			SYS_MAX_GANTRY
#define MAX_STEP_CAMERALSO			SYS_MAX_LSOPTICS
#define MAX_STEP_ALIGN_CAMERA_FLY	SYS_MAX_FLY_CAMERAS
#define MAX_STEP_ALIGN_CAMERA_QHS	SYS_MAX_STAGE_CAMERAS
#define MAX_STEP_ALIGN_CAMERA_UP	SYS_MAX_UP_CAMERAS
#define MAX_STEP_ALIGN_CAMERA_LSO	SYS_MAX_LSO_FLY_CAMERAS
#define MAX_STEP_ALIGN_CAMERA_LSO_VM SYS_MAX_LSO_VM_CAMERAS
#define MAX_STEP_RETRY_FEEDING_STD	SYS_MAX_FEEDER_BASE
#define MAX_STEP_POCKET_XY_STD		SYS_MAX_FEEDER_BASE
#define MAX_STEP_PRE_PICK_INSP		SYS_MAX_FEEDER_BASE
#define MAX_STEP_PICKTAPE			SYS_MAX_FEEDER_BASE
#define MAX_STEP_PICKSTICK			(SYS_MAX_GANTRY*2)	// Each ganty with possible front and rear
#define MAX_STEP_PICKTRAY			(SYS_MAX_TRAYUNIT*eBOTH)	// 2 gantries could access each tray unit.
#define MAX_STEP_FLUX				MAX_HEADBLOCK
#define MAX_STEP_LEAD_CHECK			MAX_HEADBLOCK
#define MAX_STEP_LCR				MAX_HEADBLOCK
#define MAX_STEP_PLACE				MAX_HEADBLOCK
#define MAX_STEP_PBI				MAX_HEADBLOCK
#define MAX_STEP_SIDEVIEW			MAX_HEADBLOCK

#define MAX_DUMP_BELT				1

#define SYS_MAX_NET_DEV				10


typedef long	TReqID;				// Request Tag ID, 0==Undefined, 1..999 Valid Values

// General purpose tag to be used where callers would be returned pointers to structures
// that are recycled after they are used.  By associating a unique ID with the request
// it prevents a caller from using a recycled request.
struct RQTag
{
	void*	pReq;	// Pointer to request structure or class
	TReqID	id;		// Unique id of the request
	inline	RQTag( )							{ pReq=0; id=0; }
	inline	RQTag( void* aReq, long anId)	{ pReq=aReq; id=anId; }
	inline	bool IsTag() const { return pReq!=NULL; }
	inline	void Null() { pReq=NULL; id=0; }
	inline	bool operator==( const RQTag& aTag) const { return aTag.pReq==pReq  &&  aTag.id==id; }

	inline	RQTag& operator=( const RQTag& aTag)
	{
		pReq = aTag.pReq; 
		id = aTag.id;
		return *this;
	}
	inline	void Set( void* aReq, long anId)	{ pReq=aReq; id=anId; }
};

#define ANTI_BACKLASH_ANGLE			15.0

// ID for CSeqEngine.
typedef long TSeqEngId;

// Used to post new run error counts
enum eRandomErrorDlgCounter
{
	eRANDOM_ERROR_PICKUP_COUNT,
	eRANDOM_ERROR_SVS_NOPART_COUNT,
	eRANDOM_ERROR_VISION_NOPART_COUNT,
	eRANDOM_ERROR_VISION_COUNT,
	eRANDOM_ERROR_FIDUCIAL_COUNT,
	eRANDOM_ERROR_BAD_COUNT,
	eRANDOM_ERROR_ACCEPT_COUNT
};

// ITS Mode 
enum eITSMode
{
	eMODE_CTRL_BY_FEEDER,			//0
	eMODE_REMOTE_SOLID,				//1
	eMODE_REMOTE_S_BLINK,			//2, S = Single
	eMODE_REMOTE_D_BLINK,			//3, D = Double
	eMODE_NEW_REMOTE_SOLID,			//4, When Feeder clamping but no need Feeder in PCB, 
									//	 ITS set Feeder LED off and this mode.   
	eMODE_NEW_REMOTE_S_BLINK,		//5
	eMODE_NEW_REMOTE_D_BLINK		//6
};


enum eReportCycleStyle
{
	eREPORT_STYLE_NONE,
	eREPORT_STYLE_WARNING_OK,
	eREPORT_STYLE_WARNING_FAILED,
	eREPORT_STYLE_FREEZE_ABORT,
};


enum eVacMode
{
	eVAC_MODE_NORMAL		= 0,
	eVAC_MODE_SMALL_CHIP	= 1,
	eVAC_MODE_GRIPPER		= 2,
	eVAC_MODE_LARGE_NOZZLE	= 3,
	eVAC_MODE_AFTER_ZDOWN	= 4
};


enum eVacMethod
{	//Default: 0x00000022 (34)
	eVAC_METHOD_NONE					= 0,
	eVAC_METHOD_OPEN_CYCLE_ENABLE		= 0x00000001,	// Inspect Open Level on every Cycle
	eVAC_METHOD_OPEN_ANC_ENABLE			= 0x00000002,	//*Inspect Open Level on every ANC Operation
	eVAC_METHOD_OPEN_SMALL_ENABLE		= 0x00000004,	//?
	eVAC_METHOD_OPEN_VAC_CHK_ENABLE		= 0x00000008,	// Inspect Open Level when vacuum check (pick Check)
	eVAC_METHOD_STUFF_CYCLE_ENABLE		= 0x00000010,	// Inspect Nozzle Stuff on every Cycle
	eVAC_METHOD_STUFF_DUMP_ENABLE		= 0x00000020,	//*Inspect Nozzle Stuff on every Dump Operation
	eVAC_METHOD_OPEN_PRE_PICK_ENABLE	= 0x00000040,	//?
	eVAC_METHOD_CHECK_PRE_MOUNT_ENABLE	= 0x00000080	//?
};


enum eAnyRTol
{
	////////////////////////////////////////////////////////////
	// AnyR     	PickInspR           	pick		Insp
	////////////////////////////////////////////////////////////
	// eANYR_OK 	ePIR_ANY_ANY        	anyAngle	anyAngle
	// eANYR_OK 	ePIR_PICK_ANY       	pickAngle	anyAngle
	// eANYR_NO 	ePIR_PICK_PLACE     	pickAngle	PlaceAngle
	// eANYR_TOL	ePIR_NPL_ANY_PL_ANY 	anyAngle	anyAngle
	// eANYR_NO 	ePIR_NEG_PLACE__PLACE	-PlaceAngle	PlaceAngle(0)
	// eANYR_ZERO	ePIR_PICK_ZERO      	pickAngle	0(Cam Angle)(F:0 R:180)
	////////////////////////////////////////////////////////////

	eANYR_NO,				// No tolerance for changing R from the defined best accuracy angle
	eANYR_TOL,				// Some tolerance
	eANYR_OK,				// Changing R is OK.
	eANYR_ZERO				// Insp Only Zero.
};


enum ePartVector
{
	ePART_VECTOR_OK,
	ePART_VECTOR_UNALIGNED,
	ePART_VECTOR_NG,
	ePART_VECTOR_NG_SKIP,
	ePART_VECTOR_NG_STOP
};


enum ePartFailReason
{
	ePART_FAIL_NONE,

	// <20080402> 'ePART_FAIL_VACUUM' was split with '_VACUUM_PICK,
	// _VIS_SHIFT_PICK, _VACUUM_PRE_PLACE' by SEC-VD request.
	ePART_FAIL_VAC_PICK,			// Vacuum check error was detected after picking up part
									// or Vision reported as part was not picked.
									// Previous 'ePART_FAIL_VACUUM' category

	ePART_FAIL_VISION,				// Vision failed to inspect a part.

	ePART_FAIL_VIS_SHIFT_PICK,		// Vision reported part align offset is too big.
									// Previous 'ePART_FAIL_VACUUM' category
									// This will be set as 'ePART_FAIL_VAC_POST_PICK'
									// RT Only - Not used yet in MMI
	ePART_FAIL_SVS_PICK,			// SVS "No Part" check error was detected after picking up part
	ePART_FAIL_SVS_VISION,			// SVS inspection check error was detected after picking up part

	ePART_FAIL_VAC_PRE_PLACE,		// Vacuum check error was detected before placing part
									// Previous 'ePART_FAIL_VACUUM' category
	ePART_FAIL_FORCE_PLACE,			// Force limit check error was detected while placing part
									
	ePART_FAIL_SVS,					// SVS check error was detected when checking the head

	ePART_FAIL_LEAD_SCAN,			// Lead Scan error was detected when checking the head

	ePART_FAIL_LCR_OK,				// LCR Result OK and dump Part.
	ePART_FAIL_LCR_NG,				// LCR Result NG and dump Part.

	ePART_FAIL_SKIP_PICK,			// HSH, SVS 5회 연속 Pick Miss 판정시 Pick 동작 Skip 됨.

	ePART_FAIL_MAX
};


enum ePartVector3D
{
	ePART_VECTOR_3D_OK,
	ePART_VECTOR_3D_UNALIGNED,
	ePART_VECTOR_3D_NG,
	ePART_VECTOR_3D_NG_SKIP,
	ePART_VECTOR_3D_NG_STOP,
	ePART_VECTOR_3D_NG_RETRY,
	ePART_VECTOR_3D_ROTATION_SCAN
};


enum eEngReason
{
	eENG_REASON_NONE,			// Not determined
	eENG_REASON_NOT_DONE,		// Engine not finished with board yet
	eENG_REASON_PREDONE,		// Engine prefetch is done.
	eENG_REASON_PREFAIL,		// Engine prefetch failed.
	eENG_REASON_PREMOV_FAIL,	// Engine pre-move failed.
	eENG_REASON_PREMOV_FAULT,	// Engine pre-move faulted.
	eENG_REASON_WORKDONE,		// Engine work really is done.
	eENG_REASON_DUMPSTOP,		// Engine stopped by dump stop.
	eENG_REASON_FIDUCIAL_BOARD,	// 
	eENG_REASON_FIDUCIAL_LOCAL,	// 
	eENG_REASON_NOTIFY_TIME,	// Notify Remaining Time failed
	eENG_REASON_TRAVEL_HGHT,	// Set machine traveling height failed
	eENG_REASON_FAILED,			// RunCycle returnd eNG
	eENG_REASON_FAIL_SKIP,		// 
	eENG_REASON_FAIL_STOP,		// 
	eENG_REASON_FAULT,			// 
	eENG_REASON_POST_OP_NG,		// 
	eENG_REASON_POST_OP_FAULT,	// 
	eENG_REASON_SKIP_TIL_END,	// Some placements were skipped until end
	eENG_REASON_STOP_LINELCR,	// 
	eENG_REASON_MAX
};


// wait times. 
enum eTimeOut
{
	eTIMEOUT_5000 = 5000,		// 5 seconds
	eTIMEOUT_10000 = 10000,	// 10 seconds
	eTIMEOUT_20000 = 20000,	// 20 seconds
	eTIMEOUT_30000 = 30000,	// 30 seconds
	eTIMEOUT_40000 = 40000,	// 40 seconds
	eTIMEOUT_50000 = 50000,	// 50 seconds
	eTIMEOUT_60000 = 60000,	// 60 seconds
};


enum eLEADSCAN_CMD
{
	eLEADSCAN_STARTSCAN  =1,
	eLEADSCAN_STOPSCAN   =2,
	eLEADSCAN_FLOWDATAON =3,
	eLEADSCAN_LIGHTLEVEL =4,
	eLEADSCAN_DATACOUNT  =5,
	eLEADSCAN_DATARESET  =6,
	eLEADSCAN_DISTANCE	 =7,
	eLEADSCAN_LIGHTLEVEL_GAIN   =8,
	eLEADSCAN_LIGHTLEVEL_POWER  =9,
	eLEADSCAN_EXPOSURE_TIME_SET = 10,	// param = usec
	eLEADSCAN_EXPOSURE_TIME_GET = 11,
	eLEADSCAN_USERDATA = 0x12345678
};

enum eRgbColor
{
	//rgb
	eRGB_WHITE,
	eRGB_BLUE,
	eRGB_GREEN,
	eRGB_CYAN,
	eRGB_RED,
	eRGB_MAGENTA,
	eRGB_YELLOW,
	eRGB_BLACK,
	eRGB_MAX
};

enum eNozzleTipInspType
{
	eTipPosAtPickPosZDown,				// Pick Place XY Position. (Head Offset) and Place Height(0mm). It's Matched Runout.
	eTipPosAtPickPosZDownCOR,			// It's Matched Head Offset. 
	
	eTipPosAtPickPosZUp,				// Pick Place XY Position. (Head Offset) and Align Height(6.5mm). It's Matched Align Runout.
	eTipPosAtPickPosZUpCOR,				// 

	eTipPosAtAlignPosZUp,				// Align XY Position. (Align Offset) and Align Height(6.5mm).
	eTipPosAtAlignPosZUpCOR,			// It's Matched Align Offset.
};

enum eGainType
{
	eGain_Normal = 0,
	eGain_Heavy = 1,
	eMAX_GainType = 2
};

} // namespace smart 

#endif // __GLOBALDEFINES_H__
