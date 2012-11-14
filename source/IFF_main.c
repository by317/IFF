#include "DSP2802x_Device.h"
#include "DSP2802x_Examples.h"
#include "easy2802x_sci_v7.3.h"
#include "DSP2802x_GlobalPrototypes.h"
#include "Piccolo_PWM.h"
#include "IQmathLib.h"

#define VIN_SCALE_INIT	430		//Scaling for Q15 Format
#define VIN_OFFSET_INIT 10
#define VBUS_SCALE_INIT	667		//Scaling for Q15 Format
#define	I_SCALE_INIT	100		//Scaling for Q15 Format
#define I_OFFSET_INIT		47
#define MPPT_UPDATE_PERIOD_MS 500
#define Q2_MAX_PULSE	30
#define Q1_MAX_PULSE	300
#define DEADTIME	8
#define MAX_PERIOD	3000

#define EPWM3_CMPA	Q1_PULSE
#define EPWM4_CMPA	Q1_PULSE + DEADTIME
#define EPWM4_CMPB	Q1_PULSE + DEADTIME + Q2_PULSE


#define GLOBAL_Q	15

#pragma CODE_SECTION(pwm_int, "ramfuncs");
#pragma CODE_SECTION(mppt_int, "ramfuncs");
#pragma CODE_SECTION(ms_delay, "ramfuncs");
#pragma CODE_SECTION(__IQmpy, "ramfuncs");
#pragma CODE_SECTION(_IQ15div, "ramfuncs");

interrupt void pwm_int(void);
interrupt void mppt_int(void);

void pwm_setup(void);

unsigned int x;
volatile unsigned int duty;
unsigned int period;

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

int32 Input_Voltage_Q15;
int32 Bus_Voltage_Q15;
int32 Input_Current_Q15;

unsigned int q1_pulse;
unsigned int q2_pulse;
unsigned int period_cmd;
int R_Factor;

extern void DSP28x_usDelay(Uint32);
void ms_delay(unsigned int);
void SetupAdc(void);
void initVariables(void);
void initialize_mppt_timer(void);
void initialize_Period_Table(void);

void main()
{
	initVariables();
	
	DINT;
	ms_delay(1);
	InitAdc();
	SetupAdc();
	InitSysCtrl();
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitPieVectTable();
	easyDSP_SCI_Init();
	InitEPwm3();
	InitEPwm4();
	pwm_setup();
	initialize_mppt_timer();
	EALLOW;
	PieVectTable.TINT2 = &mppt_int;
	PieVectTable.EPWM3_INT = &pwm_int;
	PieCtrlRegs.PIEIER3.bit.INTx3 = 0x1;
	EDIS;
	IER |= M_INT3;
	IER |= M_INT14;
	EINT;
	ms_delay(1);
	InitEPwm3Gpio();
	InitEPwm4Gpio();

	ERTM;
	for(;;)
	{

	}
}

interrupt void mppt_int()
{
	EINT;
	GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
	DINT;
	return;
}

interrupt void pwm_int()
{
	DINT;

	if (period_cmd > MAX_PERIOD)
	{
		period = MAX_PERIOD;
		R_Factor = period_cmd - MAX_PERIOD;
	}
	else
	{
		period = period_cmd;
		R_Factor = 0;
	}
	if (R_Factor > Q1_MAX_PULSE)
		R_Factor = Q1_MAX_PULSE;

	q1_pulse = Q1_MAX_PULSE - R_Factor;

	q2_pulse = ((long) (q1_pulse*100)>>10);

	EPwm3Regs.TBPRD = period;
	EPwm4Regs.TBPRD = period;

	EPwm3Regs.CMPA.half.CMPA = q2_pulse;
	EPwm4Regs.CMPA.half.CMPA = q2_pulse + DEADTIME;
	EPwm4Regs.CMPB = q1_pulse + q2_pulse + DEADTIME;

	EINT;
	EPwm3Regs.ETCLR.bit.INT = 0x1;  			//Clear the Interrupt Flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;  	//Acknowledge the interrupt
	return;
}

void pwm_setup()
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0x0;
	GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
	EDIS;

	EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0x0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0x0;
	GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1;
	GpioCtrlRegs.GPAPUD.bit.GPIO17 = 1;
	EDIS;

	EPwm3Regs.ETSEL.bit.INTEN = 0x1;
	EPwm3Regs.ETSEL.bit.INTSEL = 0x1;
	EPwm3Regs.ETPS.bit.INTPRD = 0x1;
	EPwm3Regs.ETSEL.bit.SOCAEN = 1;
	EPwm3Regs.ETSEL.bit.SOCASEL = ET_CTRD_CMPB;
	EPwm3Regs.ETPS.bit.SOCAPRD = 0x1;
}

void ms_delay(unsigned int wait_time)
{
	volatile unsigned int i;
	CpuTimer1Regs.PRD.all = 0x0000EA60;
	for (i=0; i < wait_time; i++)
	{
		CpuTimer1Regs.TCR.bit.TIF = 1;
		CpuTimer1Regs.TCR.bit.TRB = 1;
		CpuTimer1Regs.TCR.bit.TSS = 0;
		while (CpuTimer1Regs.TCR.bit.TIF == 0)
		{	
		}
	}
}

void InitAdc(void)
{
    // *IMPORTANT*
    // The Device_cal function, which copies the ADC calibration values from TI reserved
    // OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
    // Boot ROM. If the boot ROM code is bypassed during the debug process, the
    // following function MUST be called for the ADC to function according
    // to specification. The clocks to the ADC MUST be enabled before calling this
    // function.
    // See the device data manual and/or the ADC Reference
    // Manual for more information.

        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
        (*Device_cal)();
        EDIS;

    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
    // after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
    // CPU_RATE define statement in the DSP2802x_Examples.h file must
    // contain the correct CPU clock period in nanoseconds.
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCBGPWD  = 1;      // Power ADC BG
    AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;      // Power reference
    AdcRegs.ADCCTL1.bit.ADCPWDN   = 1;      // Power ADC
    AdcRegs.ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;      // Select interal BG
    EDIS;

    ms_delay(10);         // Delay before converting ADC channels
}

void SetupAdc(void)
{
	EALLOW;
	//AdcRegs.ADCSAMPLEMODE.bit.SIMULEN0 = 1;
	////Input Voltage Sampling on SOC0
	//AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0x07;
	//AdcRegs.ADCSOC0CTL.bit.CHSEL = 0x2;
	//AdcRegs.ADCSOC0CTL.bit.ACQPS = 0x6;
	//Output Voltage Sampling on SOC1
//	AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 0x00;
//	AdcRegs.ADCSOC1CTL.bit.CHSEL = 0x0;
//	AdcRegs.ADCSOC1CTL.bit.ACQPS = 0x6;
//	//Input Current Sampling on SOC4
//	AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 0x00;
//	AdcRegs.ADCSOC2CTL.bit.CHSEL = 0x4;
//	AdcRegs.ADCSOC2CTL.bit.ACQPS = 0x6;
	EDIS;
}

void initialize_mppt_timer(void)
{
	//Load the pre-scaler to 60000 clock cycles (1ms)
	CpuTimer2Regs.TPRH.all = 0x00EA;
	CpuTimer2Regs.TPR.all = 0x0060;
	CpuTimer2Regs.PRD.all = MPPT_UPDATE_PERIOD_MS;
	CpuTimer2Regs.TCR.bit.TRB = 1;
	CpuTimer2Regs.TCR.all = 0x4000; //Initialize to Interrupt Enabled and Free-Running
}

void initVariables (void)
{
	duty = 0;
	period_cmd = INITIAL_PERIOD;

}

void initialize_Period_Table(void)
{
}
