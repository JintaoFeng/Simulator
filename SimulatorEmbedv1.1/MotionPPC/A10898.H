#ifndef A10887_8_H
#define A10887_8_H

// 10897 & 10898 registers - short values
#define A9X_CONTROL                 0x00
#define A9X_STATUS                  0x00
#define A9X_LSR_SOURCE              0x04
#define A9X_OUT_CNTRL               0x08
#define A9X_COMMAND                 0x0C
#define A9X_INT_VECTOR              0x22
#define A9X_POS_OFF_MSB             0x82    // Must be written 1st
#define A9X_POS_OFF_LSB1            0x84    // Must be written 2nd
#define A9X_POS_OFF_LSB0            0x86    // Must be written last
#define A9X_POS_OFF_SE1             0x88    // Must be written 1st
#define A9X_POS_OFF_SE0             0x8A    // Must be written last
#define A9X_TEST_RATE_A             0x92
#define A9X_TEST_RATE_B             0x96
#define A9X_SMPL_DELAY              0x9A
#define A9X_SMPL_MODE_MASK          0x9E
#define A9X_SMPL_STATUS             0xA2
#define A9X_IRQ_ERR_MASK            0xA6
#define A9X_P2_ERR_MASK             0xAA
#define A9X_ERR_STAT_RESET          0xB2
#define A9X_OUT_HOLD_RATE           0xB6
#define A9X_FLTR_CNTRL              0xBA
#define A9X_OVFLW_LEV_OUT           0xBE

// 10897 & 10898 registers - long values
#define A9X_POS_OFF_LSB             0x84
#define A9X_POS_OFF_SE              0x88
#define A9X_CLIP_WIN_MASK           0x8c    // LSBs MUST be written last for D16 transfers
#define A9X_POS_REG(N,B)            (0x100 + ((N)-1) * 4 + (B) * 0x40)  // N={1:6}, B={0:3}
#define A9X_POS_REG_AUTO(N,B)       (0x120 + ((N)-1) * 4 + (B) * 0x40)  // N={1:6}, B={0:3} - Read LSBs 1st
#define A9X_VEL_REG(N)              (0x118 + ((N)-1) * 4)               // N={1:2}
#define A9X_VEL_REG_AUTO(N)         (0x138 + ((N)-1) * 4)               // N={1:2}          - Read LSBs 1st



#ifndef WORD
typedef unsigned short WORD;
#endif

#ifndef word
typedef short word;
#endif


#ifndef LONG
typedef unsigned long LONG;
#endif


#ifndef VMELONGDEF
#define VMELONGDEF
typedef union 
{
	struct {word msw; word lsw; } w;
	long l;
} VMELONG;
#endif

typedef struct 
{
	VMELONG Pos[6];
	VMELONG Vel[2];
   } POSVEL;

typedef enum 
{
	X=0,
	Y=1
}A10898_Axis;

#pragma pack(2)

typedef volatile struct {
	WORD Control;
	WORD Dummy0;    // offset 0x00
	WORD LsrSource;     WORD Dummy1;    // offset 0x04
    WORD OutControl;    WORD Dummy2;    // offset 0x08
	WORD Command;       WORD Dummy3;    // offset 0x0C
	LONG LDummyA0[4];   WORD Dummy4;    // offset 0x10
	WORD IntVector;                     // offset 0x22
	LONG LDummyA1[23];                  // offset 0x24
	struct {                            // offset 0x80
		VMELONG ms;
		VMELONG ls;
		VMELONG se;
		} PosOffset;
	VMELONG ClipWinMask;WORD Dummy6;    // offset 0x8C
    WORD TestRateA;     WORD Dummy7;    // offset 0x92
    WORD TestRateB;     WORD Dummy8;    // offset 0x96
	WORD SampleDelay;   WORD Dummy9;    // offset 0x9A
	WORD SampleModeMask;WORD DummyA;    // offset 0x9E
    WORD SampleStatus;  WORD DummyB;    // offset 0xA2
    WORD IRQerrMask;    WORD DummyC;    // offset 0xA6
    WORD P2errMask;     WORD DummyA0[3];// offset 0xAA
    WORD ErrStatusReset;WORD DummyD;    // offset 0xB2
    WORD OutHoldRate;   WORD DummyE;    // offset 0xB6
    WORD FilterControl; WORD DummyF;    // offset 0xBA
    WORD OvflLevelOut;						 // offset 0xBE
    LONG LDummyA2[16];

    POSVEL PVnorm;
    POSVEL PVauto;
    POSVEL PVnormS1;
    POSVEL PVautoS1;
    POSVEL PVnormS2;
    POSVEL PVautoS2;
    POSVEL PVnormS3;
    POSVEL PVautoS3;
    }A10898;
#pragma pack()


// 10897 & 10898 constants for register reads and writes
//  for A9X_CONTROL & A9X_STATUS registers
#define A9X_IPL_MASK        0x0007				//bit 0~2 in General Control & Status Register
#define A9X_IPL(L)          ((L)&A9X_IPL_MASK)
#define A9X_IRQ_ENABLE      0x0008			// bit 3 in General Control & Status Register
#define A9X_HW_SMPL_M0      0x0010			//bit 4
#define A9X_HW_SMPL_M1      0x0020			//bit 5
#define A9X_PRESET_EN       0x0040
#define A9X_USER_LED        0x0080
#define A9X_FORCE_0_DISABLE 0x0100
#define A9X_POS_RST_DISABLE 0x0200
//  for A9X_STATUS register
#define A9X_SMPL_PENDING    0x0400
#define A9X_MASKED_ERROR    0x0800
#define A9X_SIGNAL_A        0x1000
#define A9X_SIGNAL_B        0x2000
//#define  0x4000
//#define  0x8000

//  for A9X_LSR_SOURCE register
#define A9X_MEAS_A(S)       (((S) & 7) << 4)              // S={0:7} see next 9 lines
#define A9X_MEAS_B(S)       ((S) & 7)
#define   A9X_MS_MEAS_1       0x0000
#define   A9X_MS_MEAS_2       0x0001
#define   A9X_MS_COM1         0x0002
#define   A9X_MS_COM2         0x0003
#define   A9X_MS_COM3         0x0004
#define   A9X_MS_COM4         0x0005
#define   A9X_MS_TEST         0x0006
#define   A9X_MS_EXTERN       0x0007
#define A9X_COM_DRV(N,S)    (((S) & 3) << ((N) * 2)+6)// N={1:4}, S={0:2} see next 3 lines
#define   A9X_CD_OFF          0x0000
#define   A9X_CD_MEAS1        0x0001
#define   A9X_CD_MEAS2        0x0002
#define A9X_MEAS_A_MASK     0x0070
#define A9X_MEAS_B_MASK     0x0007
#define A9X_COM_MASK(N)     (0x0300 << (((N)-1)* 2) )

#define A9X_MS_VAL(V,N)     (7 & ((V) >> ((2 - (N)) * 4) ) )
#define A9X_COM_VAL(V,N)    (3 & ((V) >> ((N) * 2 + 6) ) )

//  for A9X_OUT_CNTRL register - OR the various items together
#define A9X_P2_ADD_MASK     0x000f
#define A9X_P2_ADD97(A)     ((A) & A9X_P2_ADD_MASK)     // 10897 ONLY

#define A9X_P2_ADD_MASK98   0x0200                  // 10898 ONLY
#define A9X_P2_ADD98(A)     (((A)&A9X_P2_ADD_MASK) | (((A)<<4)&A9X_P2_ADD_MASK98))
#define A9X_GET_P2_ADD98(V) ((((V)&A9X_P2_ADD_MASK98)>>4) | ((V)&A9X_P2_ADD_MASK))

#define A9X_ADDRES_MODE     0x0000                  // default and always for 10898
#define A9X_DIRECT_MODE     0x0010                  // 10897 ONLY

#define A9X_ERRMODE_MASK    0x00e0
#define A9X_ERROR_MODE(M)   (((M)<<5)&A9X_ERRMODE_MASK) // M={0:7} see next 8 lines
#define   A9X_EWM_OC_NT       0x0000
#define   A9X_EWM_TTL_NT      0x0001
#define   A9X_EWM_TTL_PT      0x0002
#define   A9X_EWM_3STATE      0x0003
#define   A9X_EWM_OC_NT_L     0x0004
#define   A9X_EWM_TTL_NT_L    0x0005
#define   A9X_EWM_TTL_PT_L    0x0006
#define   A9X_EWM_3STATE_L    0x0007

#define A9X_WINMODE_MASK97  0x0700                      // 10897 ONLY
#define A9X_WNDW_MODE97(M)  (((M)<<8)&A9X_WINMODE_MASK97)   // M={0:7} see EWM* constants

#define A9X_ASYNCMODE_MASK  0x3000
#define A9X_ASYNC_MODE(M)   (((M)<<12)&A9X_ASYNCMODE_MASK)
#define   A9X_AM_PASSIVE      0x0000                    // board does not drive hold line
#define   A9X_AM_HOLD         0x0001
#define   A9X_AM_CLOCK        0x0002
#define   A9X_AM_10MHZ        0x0003

#define A9X_CLOCKMODE_MASK  0xc000
#define A9X_CLOCK_MODE(M)   (((M)<<14)&A9X_CLOCKMODE_MASK)
#define   A9X_CM_INTERNAL     0x0000
#define   A9X_CM_EXTERNAL     0x0001
#define   A9X_CM_SHARED       0x0003

//  for A9X_COMMAND register (write only)
#define A9X_SAMPLE_POS(N)   ((0x0001<<((N)-1))&0x003f) // N={1:6}
#define A9X_SAMPLE_VEL(N)   ((0x0040<<((N)-1))&0x00c0) // N={1:2}
#define A9X_RESET_POSITION  0x0100
#define A9X_SAMPLE_DRV98(N) ((0x0200<<((N)-1))&0x1e00) // N={1:4} - 10898 ONLY

//  for A9X_CLIP_WIN_MASK register
#define A9X_CM_OFF          0xFFFF
#define A9X_CM_BITS(B)      (0xFFFF & ~(2^((B)-7) - 1))         // B={8:23}
#define A9X_WM_BITS(B)      ((0x7FFF & ~(2^((B)-1) - 1)) << 16) // B={1:16}
#define A9X_WM_MAG(V)       ((0x7FFF & ~((V) - 1)) << 16)       // V=(1,2,4,8,...,32768}

//  for A9X_TEST_RATE_A & A9X_TEST_RATE_B registers
#define A9X_TF_OFF          0x0000
#define A9X_TF_ON           0x0200
#define A9X_TR_DIV(Hz)      (((40000000 / (Hz)) - 1) & 0xFFFF)  // converts Hz to Divisor value
#define A9X_TR_FREQ(Div)    ((40000000 / (((Div) & 0xffff) + 1))// converts divisor value to Hz

//  for A9X_SMPL_DELAY register (only 8 bits valid)
#define A9X_SD100_P(N)      ((0x0001 << ((N)-1)) & 0x003f)     // N={1:6}
#define A9X_SD100_V(N)      ((0x0040 << ((N)-1)) & 0x00c0)     // N={1:2}

//  for A9X_SMPL_MODE_MASK register
#define A9X_SIRQEN_P(N)     ((0x0001 << ((N)-1)) & 0x003f)     // N={1:6}
#define A9X_SIRQEN_V(N)     ((0x0040 << ((N)-1)) & 0x00c0)     // N={1:2}
#define A9X_SM_NTRLK_P(N)   ((0x0100 << ((N)-1)) & 0x3f00)     // N={1:6}
#define A9X_SM_NTRLK_V(N)   ((0x4000 << ((N)-1)) & 0xc000)     // N={1:2}

//  for A9X_SMPL_STATUS register (only 8 bits valid - read only)
#define A9X_SS_P(N)         ((0x0001 << ((N)-1)) & 0x003f)     // N={1:6}
#define A9X_SS_V(N)         ((0x0040 << ((N)-1)) & 0x00c0)     // N={1:2}

//  for A9X_IRQ_ERR_MASK register (only 8 bits valid)
//  for A9X_P2_ERR_MASK register (only 8 bits valid)
//  for A9X_ERR_STAT_RESET register (only 8 bits valid)
#define A9X_ERR_VAP         0x0001
#define A9X_ERR_VAM         0x0002
#define A9X_ERR_VBP         0x0004
#define A9X_ERR_VBM         0x0008
#define A9X_ERR_GLITCH      0x0010
#define A9X_ERR_OVFL        0x0020
#define A9X_ERR_UNLOCK      0x0040
#define A9X_ERR_ALL         0x007F
#define A9X_ERR_RSTALL      0x003F
#define A9X_INTRP_RST       0x0080      // ERR_STAT_RESET reg only

//  for A9X_OUT_HOLD_RATE register (only 8 bits valid)
#define A9X_OHRG_OFF        0x0080
#define A9X_OHRG_ON         0x0000       // Can not be in ASYNC MODE 1 (AM_HOLD)
#define A9X_OHRG_DIV(Hz)    (((10000000 / (Hz)) - 1) & 0x007f)  // converts Hz to Divisor value
#define A9X_OHRG_HZ(Div)    (10000000 / (((Div) & 0x007f) + 1))// converts dicosr value to Hz

//  for A9X_FLTR_CNTRL register (only 8 bits valid)
#define A9X_FLTR_OFF        0x0000
#define A9X_FLTR_ON         0x0001
#define A9X_FLTR_KV         0x0002
#define A9X_FLTR_KP_MSK     0x000c
#define A9X_FLTR_KP(Kp)     (((Kp) << 2) & A9X_FLTR_KP_MSK)   // Kp={0:3}
#define A9X_FLTR_INIT       0x0010


//  for A9X_OVFLW_LEV_OUT register (only 8 bits valid)
#define A9X_OVFL_LEVEL_MASK 0x0003
#define A9X_OVFL_LEVEL(B)   (((B) - 32) & A9X_OVFL_LEVEL_MASK)  // B={32:35}
#define A9X_OUTPUT_2S_COMP  0x0000
#define A9X_OUTPUT_SGN_MAG  0x0004

void A10898_Init(char board,int address);
void A10898_Start(char board,A10898_Axis axis);

void A10898_Reset(char board,A10898_Axis axis);
int A10898_ReadPos(char board,A10898_Axis axis,char Channel);
int A10898_Auto_ReadPos(char board,A10898_Axis axis,char Channel);
int A10898_ReadVel(char board,A10898_Axis axis,char Channel);

int A10898_Auto_ReadVel(char board,A10898_Axis axis,char Channel);


#endif
