//###########################################################################
//
// FILE:   main.c
//
// TITLE:  Control de Rectificador Trif�sico con TMS320F2833x
//
// DESC:   Implementa el control para un rectificador trif�sico, incluyendo
//         PWMs s�ncronos, ADC disparado por PWM, m�quina de estados
//         (precarga, rampa, normal), y algoritmos de control en DSMC.
//
// Target: F28M35H52C (C28x core)
//
//###########################################################################

#include "DSP28x_Project.h"
#include <string.h>
#include <Stdlib.h>
#include <math.h>


typedef unsigned long ulong;
typedef unsigned int uint;
typedef unsigned char uchar;

// Definiciones para la configuraci�n del PWM y del sistema
#define SYSCLKOUT_FREQ_HZ   150000000UL // (150 MHz)
#define PWM_FREQ_HZ         20000UL     // (20 kHz)

// TBPRD se calcula para el modo ascendente-descendente:
#define TBPRD_VALUE         (SYSCLKOUT_FREQ_HZ / (2 * PWM_FREQ_HZ))

// Definici�n para el valor de la Banda Muerta en ciclos de TBCLK
#define DEAD_BAND_CYCLES    102

// Definiciones para el ADC
#define ADC_NUM_CHANNELS    7
#define ADC_ACQ_PS_VALUE    6           // Tiempo de adquisici�n

// Par�metros del Sistema
#define Ts (1.0f / 20000.0f)                // Periodo de muestreo (s)
#define L 2.94e-3f                          // Inductancia de l�nea (H)
#define K_max ((L / Ts))                  // Ganancia del control deslizante
volatile float K = 0.1*(K_max);
#define w  (2.0f * 3.14159265f * 50.0f)     // Frecuencia angular (rad/s)
#define Vg_rms 30.0f                        // Tension RMS de la red (V)
#define Po 288.0f                           // Potencia nominal (W)
#define fr 50.0f                            // Frecuencia de la red (Hz)
//#define Vo 240.0f                 // Tensi�n de salida de referencia nominal (V)
volatile float Vo = 240.0;

// Par�metros del Controlador PID
#define PID_KP        (0.4585f * 0.5)
#define PID_KI        25.4122f
#define PID_KD        2.2919e-6f

// Par�metros de Precarga
#define NUM_RAMP_CYCLES 100       // N�mero de cruces por cero para que cada fase

// Factores de Escala ADC
#define ADC_VOLTAGE_SCALE_FACTOR ((2.0f * 831.29f) / 4095.0f)               // Vref=3V
#define ADC_CURRENT_SCALE_FACTOR (((2.0f * 36.0f) / 4095.0f)*-1.0f)         // Vref=3V
#define ADC_VOLTAGE_OUT_SCALE_FACTOR (930.81f / 4095.0f)                    // Vref=3V
#define ADC_OFFSET 2047.5f

// Definici�n de los estados del sistema para la m�quina de estados
typedef enum {
    STATE_IDLE,                 // Estado de reposo, PWMs deshabilitados
    STATE_PRECHARGE_INIT,       // Precarga inicial
    STATE_PRECHARGE_RAMP,       // Rampa incremental de la corriente de referencia
    STATE_HOLD_CURRENT,         // Mantiene Ie_target constante antes de activar PID
    STATE_NORMAL_OPERATION,     // Funcionamiento normal con control PID de tensi�n
    STATE_FAULT                 // Estado de fallo, PWMs deshabilitados
} SystemState;

// Prototipos de funciones
#pragma CODE_SECTION(epwm1_isr, ".TI.ramfunc");
//#pragma CODE_SECTION(adc_isr, ".TI.ramfunc");
#pragma CODE_SECTION(dma_adc_isr, ".TI.ramfunc");
#pragma CODE_SECTION(ControlAlgorithm, ".TI.ramfunc");
#pragma CODE_SECTION(Update_ADC_Values, ".TI.ramfunc");
#pragma CODE_SECTION(Update_PWM_DutyCycles, ".TI.ramfunc");
#pragma CODE_SECTION(IeRampAlgorithm, ".TI.ramfunc");
//#pragma CODE_SECTION(epwm2_isr, ".TI.ramfunc"); // TEST


void Init_3phase_PWMs(void);        // Configura los 3 m�dulos ePWM y el trigger ADC
void Update_PWM_DutyCycles(void);   // Actualiza los registros CMPA de los ePWMs con da, db, dc globales
void Init_PWM_GPIO(void);           // Configura los pines GPIO para las salidas ePWM
void Init_ADC_SOC(void);            // Configura los canales del ADC, SOC y la interrupci�n
void Initialization(void);          // Inicializa par�metros de control
void ControlAlgorithm(void);        // Algoritmo de control principal
void IeRampAlgorithm(void);         // Algoritmo para la rampa de corriente de referencia
void Update_ADC_Values(void);       // Escala las lecturas raw del ADC a valores f�sicos
void ResetVariables(void);          // Resetea las varibales en caso que se pare el funcionamiento
void Init_DMA_ADC(void);
//__interrupt void adc_isr(void);       // Rutina de servicio de interrupci�n del ADC
__interrupt void epwm1_isr(void);       // Rutina de interrupci�n disparada por ePWM1 CMPB
//__interrupt void epwm2_isr(void);     // TEST
__interrupt void dma_adc_isr(void);


// Variables Globales:
// Ciclos de trabajo para los PWMs, calculados por ControlAlgorithm
volatile float da = 0.5f, db = 0.5f, dc = 0.5f;

// Valores escalados de las lecturas del ADC, actualizados en Update_ADC_Values()
volatile float iLa = 0.0f, iLb = 0.0f, iLc = 0.0f; // Corrientes de fase
volatile float vga = 0.0f, vgb = 0.0f, vgc = 0.0f; // Voltajes de fase de la red
volatile float vo  = 0.0f;                         // Voltaje de salida

volatile float Ro = 0.0f;        // Resistencia de carga equivalente calculada
volatile float Ie_target = 0.0f; // Amplitud objetivo final de la corriente de referencia

// Variables para la Precarga Individual por Fase
volatile float Iea_ramp = 0.0f, Ieb_ramp = 0.0f, Iec_ramp = 0.0f;           // Amplitudes actuales de la rampa
volatile float ramp_step = 0.0f;                                            // Tama�o del incremento de Ie_ramp en cada cruce por cero de esa fase
volatile Uint16 ramp_started_a = 0, ramp_started_b = 0, ramp_started_c = 0;  // Flags de inicio de rampa por fase
volatile Uint16 ramp_done_a = 0, ramp_done_b = 0, ramp_done_c = 0;           // Flags de finalizaci�n de rampa por fase
volatile Uint16 ramp_done = 0;                                               // Flag general de finalizaci�n de todas las rampas
volatile Uint16 zc_a = 0, zc_b = 0, zc_c = 0;                                // Flags de cruce por cero
volatile Uint16 start=0;

// Variables generacion referencia de corriente
volatile float Q = 0.0f;
volatile float invVg_peak = 0.0f;                       // Coeficiente para el generador de seno
volatile float mvga[2]={0.0f,0.0f};                     // Estados referencia de fase
volatile float mvgb[2]={0.0f,0.0f};
volatile float mvgc[2]={0.0f,0.0f};     // Referencias sinusoidales

// Variables para el Controlador PID
volatile float pid_q0 = 0.0f, pid_q1 = 0.0f, pid_q2 = 0.0f;     // Coeficientes del PID
volatile float pid_u_prev = 0.0f;                               // Salida anterior del PID u(n-1)
volatile float pid_e_curr = 0.0f;                               // Error actual e(n)
volatile float pid_e_prev1 = 0.0f;                              // Error anterior e(n-1)
volatile float pid_e_prev2 = 0.0f;                              // Error e(n-2)
volatile float pid_output = 0.0f;                               // Salida actual del PID u(n)

// Variables del ADC
volatile Uint16 AdcResults[ADC_NUM_CHANNELS];     // Array para almacenar los resultados del ADC
volatile Uint16 ConversionCount = 0;              // Contador de conversiones ADC
volatile Uint16 adc_ready = 0;                   // Flag activado por la ISR del ADC

volatile SystemState STATE = STATE_IDLE; // Estado inicial

volatile float dmin = 0.0f;
volatile Uint16 enable_pid = 0;



//***************************************************************************
// Funci�n Principal - main
//***************************************************************************
main()
{
    InitSysCtrl(); // Inicializar Control del Sistema

#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
    InitFlash();
#endif

    Init_PWM_GPIO(); // Configurar GPIOs para ePWM

    // Deshabilitar interrupciones e inicializar tabla de vectores de interrupci�n
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    // Mapear la ISR del ADC
    EALLOW;
    //PieVectTable.ADCINT1 = &adc_isr;
    PieVectTable.EPWM1_INT = &epwm1_isr;
    //PieVectTable.EPWM2_INT = &epwm2_isr; // TEST
    PieVectTable.DINTCH1 = &dma_adc_isr;
    EDIS;

    // Inicializar ADC
    InitAdc1();
    Init_ADC_SOC(); // Configurar los canales ADC, disparo y interrupci�n

    // Inicializar PWMs y el trigger para el ADC desde ePWM1
    Init_3phase_PWMs();

    // Inicializar el DMA para mover los datos del ADC a la RAM
    Init_DMA_ADC();

    // Habilitar interrupciones
    //PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Habilitar ADCINT (INT1.1) en el PIE
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // Habilitar EPWM1_INT (INT3.1) en el PIE
    //PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  // TEST
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1;  // Grupo 7, INT1 (DMA CH1)
    //IER |= M_INT1;                    // Habilitar INT1 a nivel de CPU
    IER |= M_INT3;                      // Habilitar INT3 a nivel de CPU
    IER |= M_INT7;
    EINT;                              // Habilitar Interrupciones Globales INTM
    ERTM;                              // Habilitar Interrupciones Globales en tiempo real DBGM

    // Inicializa par�metros de control
    Initialization();
    GpioG2DataRegs.GPESET.bit.GPIO131 = 1;      // Encender la ventilaci�n
    GpioDataRegs.GPCSET.bit.GPIO71 = 1;         //Desactivar la descarga de condensadores

    // --- Bucle Principal ---
    while(1)
    {
        if (adc_ready == 1)
        {
            adc_ready = 0;          // Limpiar flag

            Update_ADC_Values();    // Escalar lecturas ADC

            // --- M�quina de Estados ---
            switch(STATE)
            {
                case STATE_IDLE:
                    EALLOW; // Mantener PWMs apagados
                    EPwm1Regs.TZFRC.bit.OST = 1; EPwm2Regs.TZFRC.bit.OST = 1;  EPwm3Regs.TZFRC.bit.OST = 1;
                    EDIS;
                    break;

                case STATE_PRECHARGE_INIT:
                    if ((vga > 0.0f && mvga[0] < 0.0f)) zc_a++;
                    if ((vgb > 0.0f && mvgb[0] < 0.0f)) zc_b++;
                    if ((vgc > 0.0f && mvgc[0] < 0.0f)) zc_c++;

                    if ((zc_a >= 3) && (zc_b >= 3) && (zc_c >= 3) && start)
                     {
                         zc_a = 0; zc_b = 0; zc_c = 0;
                         STATE = STATE_PRECHARGE_RAMP;
                     }
                    break;

                case STATE_PRECHARGE_RAMP:          // Rampa de corriente
                    IeRampAlgorithm();
                    if (ramp_done) STATE = STATE_HOLD_CURRENT;
                    ControlAlgorithm();
                    break;

                case STATE_HOLD_CURRENT:        
                    if (ramp_done){
                       ramp_done=0;
                       dmin = 0.32f;
                       K = 0.25*(K_max);
                       EPwm1Regs.CMPB = 1070;
                       //Activar modo Immediato
                       EALLOW;
                       EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
                       EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
                       EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
                       EDIS;
                    }
                    ControlAlgorithm();
                    if (enable_pid == 1) STATE = STATE_NORMAL_OPERATION;
                    break;

                case STATE_NORMAL_OPERATION:
                    ControlAlgorithm();
                    break;

                case STATE_FAULT:                   // Estado de fallo.
                    EALLOW; // Forzar PWMs apagados.
                    EPwm1Regs.TZFRC.bit.OST = 1; EPwm2Regs.TZFRC.bit.OST = 1; EPwm3Regs.TZFRC.bit.OST = 1;
                    EDIS;
                    ResetVariables();
                    STATE = STATE_IDLE;
                    break;

                default:                            // Caso inesperado, ir a un estado seguro.
                    STATE = STATE_FAULT;
                    break;
            }

            mvga[1]=mvga[0]; mvga[0]=vga;
            mvgb[1]=mvgb[0]; mvgb[0]=vgb;
            mvgc[1]=mvgc[0]; mvgc[0]=vgc;

        }
    }
}


//***************************************************************************
// Funci�n Init_PWM_GPIO - Configuraci�n de pines GPIO para ePWM
//***************************************************************************
void Init_PWM_GPIO(void)
{
    // Configurar GPIOs para ePWM1, ePWM2, ePWM3
    // Salidas A: GPIO0, GPIO2, GPIO4
    // Salidas B: GPIO1, GPIO3, GPIO5

    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; // EPWM1A
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;  // Output
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1; // EPWM1B
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;  // Output

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; // EPWM2A
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;  // Output
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1; // EPWM2B
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;  // Output

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1; // EPWM3A
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;  // Output
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1; // EPWM3B
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;  // Output

    GpioG2CtrlRegs.GPEMUX1.bit.GPIO131 = 0;   // Fan
    GpioG2CtrlRegs.GPEDIR.bit.GPIO131 = 1;    // Output

    GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 0;     // Discharge Relay
    GpioCtrlRegs.GPCDIR.bit.GPIO71 = 1;      // Output
    EDIS;
}


//***************************************************************************
// Funci�n Init_3phase_PWMs - Configuraci�n de los m�dulos ePWM
//***************************************************************************
void Init_3phase_PWMs(void)
{
    EALLOW;
    // Deshabilitar la sincronizaci�n del reloj de la base de tiempo (TBCLK) para todos los ePWM
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    // Configuraci�n para ePWM1 (Master)
    //------------------------------------
    // Configuraci�n de la Base de Tiempo (TB)
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Modo de conteo: Ascendente-Descendente (sim�trico)
    EPwm1Regs.TBPRD = TBPRD_VALUE;                  // Establecer el per�odo del temporizador
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000;            // Fase  0
    EPwm1Regs.TBCTR = 0x0000;                       // Limpiar contador de la base de tiempo

    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // ePWM1 es maestro
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;     // Pulso de sincronizaci�n en TBCTR = 0

    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // TBCLK = SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Configuraci�n del Comparador
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     //CC_IMMEDIATE
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;      // No aplica en inmediato

    // Configuraci�n del Trigger de Interrupci�n
    EPwm1Regs.CMPB = 1400;                          // Set CMPB = 1400
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPB;      // Disparar INT cuando TBCTR = CMPB subiendo
    EPwm1Regs.ETSEL.bit.INTEN = 1;                  // Habilitar Interrupci�n ePWM
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;             // Generar interrupci�n en el primer evento

    // Configuraci�n del Calificador de Acci�n para ePWM1A
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;              // Poner en alto PWM1A cuando TBCTR = CMPA en conteo ascendente
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;            // Poner en bajo PWM1A cuando TBCTR = CMPA en conteo descendente

    // Configuraci�n de Banda Muerta para ePWM1
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // Habilita DB para ePWM1A y ePWM1B
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // ePWMxB es inv(ePWMxA) con DB
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;          // ePWMxA es la fuente para ambos retardos RED y FED
    EPwm1Regs.DBRED = DEAD_BAND_CYCLES;             // Tiempo muerto de subida
    EPwm1Regs.DBFED = DEAD_BAND_CYCLES;             // Tiempo muerto de bajada

    // Configuraci�n del Event Trigger para generar SOCA para el ADC
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;                 // Habilitar  SOCA
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;      // Disparar SOCA cuando TBCTR = 0
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;            // Generar SOCA en el primer evento


    // Configuraci�n para ePWM2 (Esclavo)
    //------------------------------------
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm2Regs.TBPRD = TBPRD_VALUE;
    EPwm2Regs.TBPHS.half.TBPHS = 0x0000;
    EPwm2Regs.TBCTR = 0x0000;

    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;          // ePWM2 es esclavo
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;      // Usar se�al de sincronizaci�n externa (SYNCIN)


    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

    // TEST
   //EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Disparar en Cero (Inicio ciclo)
   //EPwm2Regs.ETSEL.bit.INTEN = 1;                  // Habilitar
   //EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;


    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    // Configuraci�n de Banda Muerta para ePWM2
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED = DEAD_BAND_CYCLES;
    EPwm2Regs.DBFED = DEAD_BAND_CYCLES;

    // Configuraci�n para ePWM3 (Esclavo)
    //------------------------------------
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm3Regs.TBPRD = TBPRD_VALUE;
    EPwm3Regs.TBPHS.half.TBPHS = 0x0000;
    EPwm3Regs.TBCTR = 0x0000;

    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    // Configuraci�n de Banda Muerta para ePWM3
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBRED = DEAD_BAND_CYCLES;
    EPwm3Regs.DBFED = DEAD_BAND_CYCLES;


    // Forzar PWMs apagados inicialmente usando Trip Zone (TZ).
    EPwm1Regs.TZFRC.bit.OST = 1; EPwm2Regs.TZFRC.bit.OST = 1; EPwm3Regs.TZFRC.bit.OST = 1;  // Forzar One-Shot Trip
    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO;           // Salidas a BAJO en trip
    EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO;

    // Habilitar la sincronizaci�n del reloj de la base de tiempo (TBCLK) para todos los ePWM
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}


//***************************************************************************
// Funci�n Init_ADC_SOC - Configuraci�n del ADC
//***************************************************************************
void Init_ADC_SOC(void)
{
    EALLOW;
    Adc1Regs.ADCCTL2.bit.ADCNONOVERLAP = 0; // Evitar solapamiento de S/H y conversi�n //0
    Adc1Regs.ADCCTL1.bit.INTPULSEPOS = 1;   // Interrupci�n al final de la conversi�n

    Adc1Regs.INTSEL1N2.bit.INT1E = 1;       // Habilitar ADCINT1
    Adc1Regs.INTSEL1N2.bit.INT1CONT = 0;    // Deshabilitar modo continuo de ADCINT1
    Adc1Regs.INTSEL1N2.bit.INT1SEL = 5;     // EOC6 dispara ADCINT1

    AnalogSysctrlRegs.TRIG1SEL.all = 5;     // Trigger ePWM1SOCA

    // Configurar cada SOC para ser disparado por EPWM1SOCA
    Adc1Regs.ADCSOC0CTL.bit.TRIGSEL = 5;
    Adc1Regs.ADCSOC1CTL.bit.TRIGSEL = 5;
    Adc1Regs.ADCSOC2CTL.bit.TRIGSEL = 5;
    Adc1Regs.ADCSOC3CTL.bit.TRIGSEL = 5;
    Adc1Regs.ADCSOC4CTL.bit.TRIGSEL = 5;
    Adc1Regs.ADCSOC5CTL.bit.TRIGSEL = 5;
    Adc1Regs.ADCSOC6CTL.bit.TRIGSEL = 5;

    // Asignar canales a cada SOC
    Adc1Regs.ADCSOC0CTL.bit.CHSEL = 2;  // SOC0 -> ADCINA2 -> iLa
    Adc1Regs.ADCSOC1CTL.bit.CHSEL = 0;  // SOC1 -> ADCINA0 -> iLb
    Adc1Regs.ADCSOC2CTL.bit.CHSEL = 12; // SOC2 -> ADCINB4 -> iLc
    Adc1Regs.ADCSOC3CTL.bit.CHSEL = 3;  // SOC3 -> ADCINA3 -> vga
    Adc1Regs.ADCSOC4CTL.bit.CHSEL = 4;  // SOC4 -> ADCINA4 -> vgb
    Adc1Regs.ADCSOC5CTL.bit.CHSEL = 8;  // SOC5 -> ADCINB0 -> vgc
    Adc1Regs.ADCSOC6CTL.bit.CHSEL = 15; // SOC6 -> ADCINB7 -> vo

    // Configurar tiempo de adquisici�n para cada SOC
    Adc1Regs.ADCSOC0CTL.bit.ACQPS = ADC_ACQ_PS_VALUE;
    Adc1Regs.ADCSOC1CTL.bit.ACQPS = ADC_ACQ_PS_VALUE;
    Adc1Regs.ADCSOC2CTL.bit.ACQPS = ADC_ACQ_PS_VALUE;
    Adc1Regs.ADCSOC3CTL.bit.ACQPS = ADC_ACQ_PS_VALUE;
    Adc1Regs.ADCSOC4CTL.bit.ACQPS = ADC_ACQ_PS_VALUE;
    Adc1Regs.ADCSOC5CTL.bit.ACQPS = ADC_ACQ_PS_VALUE;
    Adc1Regs.ADCSOC6CTL.bit.ACQPS = ADC_ACQ_PS_VALUE;
    EDIS;

}


//***************************************************************************
// Funci�n Initialization - Inicializaci�n de par�metros de control
//***************************************************************************
void Initialization(void)
{
    // C�lculo de par�metros derivados del sistema
    Ro = (Vo * Vo) / Po;
    //Ie_target = (2.0f * Vo * Vo) / (3.0f * Vg_rms * sqrt(2) * Ro);
    Ie_target = 6.0f;
    Q = 2.0f * cos (w * Ts) + 1.0f;
    invVg_peak = 1.0f / (Vg_rms * sqrt(2));

    // Calcular el incremento de rampa por paso de muestreo
    float ramp_duration_seconds = (float)NUM_RAMP_CYCLES / fr;  // Duraci�n total de la rampa en segundos
    float ramp_total_steps = ramp_duration_seconds / Ts;        // N�mero total de pasos de simulaci�n para la rampa
    ramp_step = Ie_target / ramp_total_steps;

    // Inicializaci�n del Controlador PID
    pid_q0 = PID_KP + (Ts/2.0f)*PID_KI + PID_KD/Ts;
    pid_q1 = -PID_KP + (Ts/2.0f)*PID_KI - (2.0f*PID_KD/Ts);
    pid_q2 = PID_KD/Ts;
    pid_u_prev = Ie_target; // Inicializar salida previa del PID al valor objetivo de la rampa
    pid_output = Ie_target; // Inicializar salida del PID
}


//***************************************************************************
// Funci�n Update_ADC_Values - Escala las lecturas del ADC a valores f�sicos
//***************************************************************************
void Update_ADC_Values(void)
{
    iLa = (((float)AdcResults[0] - ADC_OFFSET) * ADC_CURRENT_SCALE_FACTOR);
    iLb = (((float)AdcResults[1] - ADC_OFFSET) * ADC_CURRENT_SCALE_FACTOR);
    iLc = (((float)AdcResults[2] - ADC_OFFSET) * ADC_CURRENT_SCALE_FACTOR);
    vga = (((float)AdcResults[3] - ADC_OFFSET) * ADC_VOLTAGE_SCALE_FACTOR);
    vgb = (((float)AdcResults[4] - ADC_OFFSET) * ADC_VOLTAGE_SCALE_FACTOR);
    vgc = (((float)AdcResults[5] - ADC_OFFSET) * ADC_VOLTAGE_SCALE_FACTOR);
    vo  = (((float)AdcResults[6]) * ADC_VOLTAGE_OUT_SCALE_FACTOR);
}


//***************************************************************************
// Funci�n Update_PWM_DutyCycles - Actualiza los registros CMPA de los ePWMs
//***************************************************************************
void Update_PWM_DutyCycles(void)
{

    float da_sat = __fmin(1.0f, __fmax(dmin, da));
    float db_sat = __fmin(1.0f, __fmax(dmin, db));
    float dc_sat = __fmin(1.0f, __fmax(dmin, dc));

    // Conversi�n a contadores
    Uint16 period = TBPRD_VALUE;

    EPwm1Regs.CMPA.half.CMPA = (Uint16)(da_sat * period);
    EPwm2Regs.CMPA.half.CMPA = (Uint16)(db_sat * period);
    EPwm3Regs.CMPA.half.CMPA = (Uint16)(dc_sat * period);
}


//***************************************************************************
// Funci�n IeRampAlgorithm - Implementa la rampa de corriente de referencia
//***************************************************************************
void IeRampAlgorithm(void)
{
    // Detecci�n de cruce por cero positivo para cada fase
    zc_a = (vga > 0.01f && mvga[0] < -0.01f);
    zc_b = (vgb > 0.01f && mvgb[0] < -0.01f);
    zc_c = (vgc > 0.01f && mvgc[0] < -0.01f);

    // Iniciar rampa para cada fase al detectar su primer cruce por cero
    if (zc_a && !ramp_started_a) {
        ramp_started_a = 1;
        EALLOW;
        EPwm1Regs.TZCLR.bit.OST = 1;
        EDIS;
    }
    if (zc_b && !ramp_started_b) {
        ramp_started_b = 1;
        EALLOW;
        EPwm2Regs.TZCLR.bit.OST = 1;
        EDIS;
    }
    if (zc_c && !ramp_started_c) {
        ramp_started_c = 1;
        EALLOW;
        EPwm3Regs.TZCLR.bit.OST = 1;
        EDIS;
    }

    // Incrementar la amplitud de la rampa para cada fase
    if (ramp_started_a && Iea_ramp < Ie_target && !ramp_done_a) {
        Iea_ramp += ramp_step;
        if (Iea_ramp >= Ie_target) { Iea_ramp = Ie_target; ramp_done_a = 1; }
    }
    if (ramp_started_b && Ieb_ramp < Ie_target && !ramp_done_b) {
        Ieb_ramp += ramp_step;
        if (Ieb_ramp >= Ie_target) { Ieb_ramp = Ie_target; ramp_done_b = 1; }
    }
    if (ramp_started_c && Iec_ramp < Ie_target && !ramp_done_c) {
        Iec_ramp += ramp_step;
        if (Iec_ramp >= Ie_target) { Iec_ramp = Ie_target; ramp_done_c = 1; }
    }

    // Comprobar si todas las rampas han finalizado
    if(ramp_done_a && ramp_done_b && ramp_done_c) {ramp_done=1; zc_a = 0; zc_b = 0; zc_c = 0;}
}


//***************************************************************************
// Funci�n ControlAlgorithm - Algoritmo de control principal
//***************************************************************************
void ControlAlgorithm(void)
{

    float ia_ref = (Q*(vga - mvga[0]) + mvga[1]) * invVg_peak;

    float ib_ref = (Q*(vgb - mvgb[0]) + mvgb[1]) * invVg_peak;

    float ic_ref = (Q*(vgc - mvgc[0]) + mvgc[1]) * invVg_peak;

    float inv_vo = __einvf32(vo);
    inv_vo = inv_vo*(2.0f - inv_vo*vo);
    inv_vo = inv_vo*(2.0f - inv_vo*vo);


    // Control usando las amplitudes de rampa Iea_ramp, Ieb_ramp, Iec_ramp
    if (STATE == STATE_PRECHARGE_RAMP)
    {
        // C�lculo de Referencias  (Solo si la rampa ha iniciado)
        if (ramp_started_a) {

            da = (K * inv_vo) * (Iea_ramp * ia_ref - iLa) - (vga * inv_vo) + 0.5f;
        }

        if (ramp_started_b) {

            db = (K * inv_vo) * (Ieb_ramp * ib_ref - iLb) - (vgb * inv_vo) + 0.5f;
        }

        if (ramp_started_c) {

            dc = (K * inv_vo) * (Iec_ramp * ic_ref - iLc) - (vgc * inv_vo) + 0.5f;
        }
    }


     if (STATE == STATE_HOLD_CURRENT)
    {

		/*
		if ((ConversionCount * Ts) >= 1)
        {
            ConversionCount = 0;
                if (up == 1) {Vo = 200;}
                if (up == -1) {Vo = 240;}
                up *= -1;
        }
        */
       da = (K * inv_vo) * (Ie_target * ia_ref - iLa) - (vga * inv_vo) + 0.5f;
       db = (K * inv_vo) * (Ie_target * ib_ref - iLb) - (vgb * inv_vo) + 0.5f;
       dc = (K * inv_vo) * (Ie_target * ic_ref - iLc) - (vgc * inv_vo) + 0.5f;
    }



    // Control PID para la tensi�n de salida
   if (STATE == STATE_NORMAL_OPERATION)
    {

		/*
		if ((ConversionCount * Ts) >= 1)
        {
            ConversionCount = 0;
                if (up == 1) {Vo = 200;}
                if (up == -1) {Vo = 240;}
                up *= -1;
        }
        */

       pid_e_curr = Vo - vo;
       pid_output = pid_u_prev + pid_q0 * pid_e_curr + pid_q1 * pid_e_prev1 + pid_q2 * pid_e_prev2;
       pid_u_prev = pid_output;
       pid_e_prev2 = pid_e_prev1;
       pid_e_prev1 = pid_e_curr;

       da = (K * inv_vo) * (pid_output * ia_ref - iLa) - (vga * inv_vo) + 0.5f;
       db = (K * inv_vo) * (pid_output * ib_ref - iLb) - (vgb * inv_vo) + 0.5f;
       dc = (K * inv_vo) * (pid_output * ic_ref - iLc) - (vgc * inv_vo) + 0.5f;
    };
}


//***************************************************************************
// Funci�n ResteVariables - Reset de las variables del sistema
//***************************************************************************
void ResetVariables(void)
{
    // Variables Globales:
    // Ciclos de trabajo para los PWMs, calculados por ControlAlgorithm
    da = 0.5f, db = 0.5f, dc = 0.5f;

    // Valores escalados de las lecturas del ADC, actualizados en Update_ADC_Values()
    iLa = 0.0f, iLb = 0.0f, iLc = 0.0f; // Corrientes de fase
    vga = 0.0f, vgb = 0.0f, vgc = 0.0f; // Voltajes de fase de la red
    vo  = 0.0f;                         // Voltaje de salida


    // Variables para la Precarga Individual por Fase
    Iea_ramp = 0.0f, Ieb_ramp = 0.0f, Iec_ramp = 0.0f;           // Amplitudes actuales de la rampa
    ramp_started_a = 0, ramp_started_b = 0, ramp_started_c = 0;  // Flags de inicio de rampa por fase
    ramp_done_a = 0, ramp_done_b = 0, ramp_done_c = 0;           // Flags de finalizaci�n de rampa por fase
    ramp_done = 0;                                               // Flag general de finalizaci�n de todas las rampas
    zc_a = 0, zc_b = 0, zc_c = 0;                                // Flags de cruce por cero
    start=0;

    // Inicializaci�n del Controlador PID
    pid_u_prev = Ie_target;                     // Salida anterior del PID u(n-1)
    pid_output = Ie_target;                     // Salida actual del PID u(n)
    pid_e_curr = 0.0f;                           // Error actual e(n)
    pid_e_prev1 = 0.0f;                          // Error anterior e(n-1)
    pid_e_prev2 = 0.0f;                          // Error e(n-2)

    // Variables del ADC
    ConversionCount = 0;        // Contador de conversiones ADC
    adc_ready = 0;              // Flag activado por la ISR del ADC

    EPwm1Regs.CMPB = 1400;
    dmin=0.0f;
    EALLOW;
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EDIS;
   enable_pid = 0;
   K = 0.1*(K_max);
}



//***************************************************************************
// ISR del EPWM1 - Se dispara cuando TBCTR == CMPB (subida) a los 8us
//***************************************************************************
__interrupt void epwm1_isr(void)
{
    float da_sat = __fmin(1.0f, __fmax(dmin, da));   //TEST
    float db_sat = __fmin(1.0f, __fmax(dmin, db));
    float dc_sat = __fmin(1.0f, __fmax(dmin, dc));

    // Conversi�n a contadores
    Uint16 period = TBPRD_VALUE;

    EPwm1Regs.CMPA.half.CMPA = (Uint16)(da_sat * period);
    EPwm2Regs.CMPA.half.CMPA = (Uint16)(db_sat * period);
    EPwm3Regs.CMPA.half.CMPA = (Uint16)(dc_sat * period);

    //GpioDataRegs.GPCCLEAR.bit.GPIO71 = 1;

    EPwm1Regs.ETCLR.bit.INT = 1;            // Limpiar flag de interrupci�n de EPWM1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; // Acknowledge interrupt to PIE
}


__interrupt void dma_adc_isr(void)
{

    //GpioDataRegs.GPCCLEAR.bit.GPIO71 = 1;

    if (STATE == STATE_NORMAL_OPERATION) ConversionCount++;
    adc_ready=1;

    // Limpiar flags para la siguiente interrupci�n
    EALLOW;
    Adc1Regs.ADCINTFLGCLR.bit.ADCINT1 = 1;   // Limpiar el flag del perif�rico ADC
    DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;   // Limpiar flag de interrupci�n del canal 1 del DMA
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;  // Acknowledge al Grupo 7 del PIE
    EDIS;
}

void Init_DMA_ADC(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR3.bit.DMAENCLK = 1; // Habilitar el reloj del m�dulo DMA
    EDIS;

    // Reset por hardware del DMA
    EALLOW;
    DmaRegs.DMACTRL.bit.HARDRESET = 1;
    __asm(" NOP");

    // Configuraci�n del Canal 1 del DMA
    // BURST: Cantidad de palabras a transferir por cada disparo del ADC (N-1)
    DmaRegs.CH1.BURST_SIZE.all = 6;      // Queremos 7 conversiones (0 a 6)
    DmaRegs.CH1.SRC_BURST_STEP = 1;      // Incremento de la direcci�n de origen (pasar al siguiente ADCRESULT)
    DmaRegs.CH1.DST_BURST_STEP = 1;      // Incremento de la direcci�n de destino (pasar al siguiente �ndice del array)

    // TRANSFER: Cu�ntas r�fagas componen una transferencia completa (N-1)
    DmaRegs.CH1.TRANSFER_SIZE = 0;       // 1 sola r�faga por evento
    DmaRegs.CH1.SRC_TRANSFER_STEP = 0;   // No se usa en este caso
    DmaRegs.CH1.DST_TRANSFER_STEP = 0;   // No se usa en este caso

    // Punteros de origen y destino
    DmaRegs.CH1.SRC_ADDR_SHADOW = (Uint32)&Adc1Result.ADCRESULT0;
    DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32)&AdcResults[0];

    // Wrap (Reinicio de punteros tras la transferencia)
    DmaRegs.CH1.SRC_WRAP_SIZE = 0xFFFF;  // Sin Wrap (mayor que el tama�o de transferencia)
    DmaRegs.CH1.DST_WRAP_SIZE = 0xFFFF;  // Sin Wrap

    // Configuraci�n del Modo de Operaci�n del CH1
    DmaRegs.CH1.MODE.bit.PERINTSEL = 1;  // Disparo del DMA: 1 = ADC1INT1 (Comprueba la tabla de tu TRM)
    DmaRegs.CH1.MODE.bit.PERINTE = 1;    // Habilitar el disparo por perif�rico
    DmaRegs.CH1.MODE.bit.ONESHOT = 0;    // One-shot deshabilitado (transfiere un burst por cada trigger)
    DmaRegs.CH1.MODE.bit.CONTINUOUS = 1; // Modo continuo habilitado (los punteros se auto-recargan)
    DmaRegs.CH1.MODE.bit.DATASIZE = 0;   // Tama�o de dato: 16-bits

    // Configuraci�n de la Interrupci�n hacia la CPU
    DmaRegs.CH1.MODE.bit.CHINTMODE = 1;  // Generar interrupci�n al FINAL de la transferencia completa
    DmaRegs.CH1.MODE.bit.CHINTE = 1;     // Habilitar la interrupci�n de este canal

    // Limpiar flags pendientes y arrancar el canal
    DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;
    DmaRegs.CH1.CONTROL.bit.ERRCLR = 1;
    DmaRegs.CH1.CONTROL.bit.RUN = 1;
    EDIS;
}

