#include "stm32f10x.h"
#include <stdint.h>
#include <string.h>

// =====================================================================================
// Configuration and compatibility layer
// =====================================================================================

#ifndef Log
#define Log(...) ((void)0)
#endif

#ifndef STATE_NORMAL
#define STATE_NORMAL 0
#endif
#ifndef CONFIG_POWER_ON
#define CONFIG_POWER_ON 1
#endif
#ifndef MODE_NORMAL
#define MODE_NORMAL 0
#endif

#ifndef NULL
#define NULL ((void*)0)
#endif

// Weak external variables so project-level symbols (if any) can override these
typedef struct {
    uint8_t ucNetworkControl;
} AppCfg_t;

__attribute__((weak)) uint8_t g_ucErr = STATE_NORMAL;
__attribute__((weak)) uint8_t g_ucWorkMode = MODE_NORMAL;
__attribute__((weak)) static AppCfg_t g_defaultAppCfg = { .ucNetworkControl = CONFIG_POWER_ON };
__attribute__((weak)) AppCfg_t *g_ptAppCfg = &g_defaultAppCfg;

// =====================================================================================
// Pin map (adjust to your hardware)
// =====================================================================================
// Input switches
#define SWITCH1_PIN            GPIO_Pin_9   // PB9  Pure tank present
#define SWITCH2_PIN            GPIO_Pin_6   // PA6  Pure tank high level (example mapping from comments)
#define SWITCH3_PIN            GPIO_Pin_5   // PB5  Pure tank low level

#define LOW_SW_PIN             GPIO_Pin_7   // PA7  Mineral tank low level
#define HIGH_SW_PIN            GPIO_Pin_4   // PC4  Mineral tank high level

#define HIGH_SWITCH_PIN        GPIO_Pin_4   // PB4  Raw tank present
#define LOW_SWITCH_PIN         GPIO_Pin_3   // PB3  Raw tank level

// Actuators
#define FLUSH_VALVE_PIN        GPIO_Pin_13  // PC13 Flush/backflow valve
#define PURE_VALVE_PIN         GPIO_Pin_0   // PA0  Pure water valve
#define MINERAL_VALVE_PIN      GPIO_Pin_1   // PA1  Mineral water valve
#define PUMP_EN_PIN            GPIO_Pin_2   // PA2  Booster pump enable
#define BUZZ_PIN               GPIO_Pin_3   // PA3  Buzzer

// Optional: Mineral processing control (if hardware exists)
#define MINERALIZE_CTRL_PIN    GPIO_Pin_8   // PC8  Mineralization process control

// =====================================================================================
// Time base flags (1ms, 10ms, 50ms, 100ms, 200ms, 500ms, 1s)
// These flags should be set/cleared from a timer ISR (not implemented here)
// =====================================================================================

typedef uint8_t U8;

typedef struct {
    U8 b0 :1; // 1ms
    U8 b1 :1; // 10ms
    U8 b2 :1; // 50ms
    U8 b3 :1; // 100ms
    U8 b4 :1; // 200ms
    U8 b5 :1; // 500ms
    U8 b6 :1; // 1s
    U8 b7 :1; // reserved
} TS_Bit8_Def;

typedef union {
    U8 byte;
    TS_Bit8_Def bit_;
} TU_UIbyte_Def;

static TU_UIbyte_Def G_Time_Bit;

#define FLAG_1ms_For_SYS      G_Time_Bit.bit_.b0
#define FLAG_10ms_For_SYS     G_Time_Bit.bit_.b1
#define FLAG_50ms_For_SYS     G_Time_Bit.bit_.b2
#define FLAG_100ms_For_SYS    G_Time_Bit.bit_.b3
#define FLAG_200ms_For_SYS    G_Time_Bit.bit_.b4
#define FLAG_500ms_For_SYS    G_Time_Bit.bit_.b5
#define FLAG_1s_For_SYS       G_Time_Bit.bit_.b6
#define Clear_SYS_Time_Bit()  (G_Time_Bit.byte = 0)

// =====================================================================================
// Types and state
// =====================================================================================

typedef enum {
    TANK_ABSENT = 0,
    TANK_LOW_WATER,
    TANK_MID_WATER,
    TANK_HIGH_WATER
} TankLevel_t;

typedef enum {
    LED_OFF = 0,
    LED_FAST_BLINK,
    LED_SLOW_BLINK
} LedState_t;

typedef enum {
    MODE_IDLE = 0,
    MODE_FLUSHING,
    MODE_REVERSE_FLOW,
    MODE_PURE_MAKING,
    MODE_MINERAL_MAKING,
    MODE_MINERALIZING,
    MODE_SLEEP
} MachineMode_t;

typedef enum {
    WASH_TYPE_BEFORE_WATER = 0,
    WASH_TYPE_AFTER_WATER
} WashType_t;

typedef WashType_t WashType_e; // alias for compatibility

typedef struct {
    // Pure, Raw, Mineral tank states and auxiliary flags
    struct {
        TankLevel_t status;
        LedState_t ledState;
        uint8_t lastPresent;
        uint8_t needRefill;
        uint8_t lastHighWater;     // debounced high-water signal (repurposed var name to match original usage)
        uint8_t highWaterBreak;     // edge detection: high->mid transition occurred
        uint8_t changing;           // transition flag
        uint8_t debounceCounter;    // 100ms tick counter *5 => 500ms
        uint16_t dispenseTimer;     // seconds for 20s fetch timer
    } pureTank;

    struct {
        TankLevel_t status;
        LedState_t ledState;
        uint8_t lastPresent;
        uint8_t changing;
        uint8_t changed;
        uint8_t refillDelay;        // seconds delay after tank change
    } rawTank;

    struct {
        TankLevel_t status;
        uint8_t needRefill;
        uint8_t mineralizing;
        uint16_t mineralTimer;      // seconds
    } mineralTank;

    MachineMode_t machineMode;

    uint8_t reverseFlowNeeded;
    uint32_t noMakingTimer;         // 100ms ticks (for 6h)

    uint32_t idleTimer;             // seconds in idle for sleep
    uint16_t modeTimer;             // seconds in current mode for various timeouts

    uint8_t needStartMineralizing;  // set when mineral making completes, to start mineralization after post-flush
} SystemState_t;

static SystemState_t sysState;

// =====================================================================================
// Wash module light implementation (timed flushing)
// =====================================================================================

#define FLUSH_BEFORE_SECONDS   30
#define FLUSH_AFTER_SECONDS    30

static struct {
    uint8_t active;
    WashType_t currentType;
    uint16_t secondsRemaining;
} g_washCtx;

static inline void Wash_InternalStart(WashType_t washType, uint16_t durationSeconds) {
    g_washCtx.active = 1;
    g_washCtx.currentType = washType;
    g_washCtx.secondsRemaining = durationSeconds;

    // Common: open flush valve and run pump
    GPIO_SetBits(GPIOA, PUMP_EN_PIN);
    GPIO_SetBits(GPIOC, FLUSH_VALVE_PIN);
}

void Wash_Init(void) {
    // Close all valves
    GPIO_ResetBits(GPIOC, FLUSH_VALVE_PIN);
    GPIO_ResetBits(GPIOA, PURE_VALVE_PIN | MINERAL_VALVE_PIN | PUMP_EN_PIN);
}

void Wash_Start(WashType_e washType) {
    Wash_Init();
    if (washType == WASH_TYPE_BEFORE_WATER) {
        Wash_InternalStart(washType, FLUSH_BEFORE_SECONDS);
    } else {
        Wash_InternalStart(washType, FLUSH_AFTER_SECONDS);
    }
}

uint8_t Wash_IsActive(void) {
    return g_washCtx.active;
}

WashType_t Wash_GetCurrentType(void) {
    return g_washCtx.currentType;
}

// call every 1s
static void Wash_Process1s(void) {
    if (!g_washCtx.active) return;
    if (g_washCtx.secondsRemaining > 0) {
        g_washCtx.secondsRemaining--;
    }
    if (g_washCtx.secondsRemaining == 0) {
        // stop flushing
        GPIO_ResetBits(GPIOC, FLUSH_VALVE_PIN);
        GPIO_ResetBits(GPIOA, PUMP_EN_PIN);
        g_washCtx.active = 0;
    }
}

// =====================================================================================
// Preconditions for water making
// =====================================================================================

uint8_t CheckWaterMakingCondition(void) {
    // 1. Raw tank present (SWITCH2_PIN PA6 in original comment, but raw present uses HIGH_SWITCH_PIN PB4 in this file)
    if (GPIO_ReadInputDataBit(GPIOB, HIGH_SWITCH_PIN) != Bit_RESET) { // high => absent
        Log("Raw water tank not in place");
        return 0;
    }

    // 2. Low water level float (mineral tank low level as example check)
    if (GPIO_ReadInputDataBit(GPIOA, LOW_SW_PIN) != Bit_RESET) {
        Log("Low water level float not detected");
        return 0;
    }

    // 3. System state must be normal
    if (g_ucErr != STATE_NORMAL || g_ptAppCfg->ucNetworkControl != CONFIG_POWER_ON || g_ucWorkMode != MODE_NORMAL) {
        Log("Machine not in normal state");
        return 0;
    }

    return 1;
}

// =====================================================================================
// State init and updates
// =====================================================================================

static void System_StateInit(void) {
    memset(&sysState, 0, sizeof(sysState));

    sysState.machineMode = MODE_IDLE;
    sysState.pureTank.ledState = LED_OFF;
    sysState.rawTank.ledState = LED_OFF;
    sysState.mineralTank.mineralizing = 0;
    sysState.reverseFlowNeeded = 0;

    sysState.pureTank.lastHighWater = 1; // assume high level ok initially
}

static void UpdatePureTankState(void) {
    // Input pull-up: low=closed/active
    uint8_t present = (GPIO_ReadInputDataBit(GPIOB, SWITCH1_PIN) == Bit_RESET);
    uint8_t high    = (GPIO_ReadInputDataBit(GPIOA, SWITCH2_PIN) == Bit_RESET);
    uint8_t low     = (GPIO_ReadInputDataBit(GPIOB, SWITCH3_PIN) == Bit_RESET);

    // Debounce "low" as the variable used as lastHighWater in original code
    if (low != sysState.pureTank.lastHighWater) {
        sysState.pureTank.debounceCounter = 5; // 5 x 100ms = 500ms
    } else if (sysState.pureTank.debounceCounter > 0 && FLAG_100ms_For_SYS) {
        sysState.pureTank.debounceCounter--;
    }

    uint8_t stableLow = sysState.pureTank.lastHighWater;
    if (sysState.pureTank.debounceCounter == 0) {
        stableLow = low;
    }

    uint8_t lastPresent = sysState.pureTank.lastPresent;
    sysState.pureTank.lastPresent = present;

    if (!present) {
        sysState.pureTank.status = TANK_ABSENT;
        sysState.pureTank.ledState = LED_FAST_BLINK;
        sysState.pureTank.needRefill = 0;
    } else {
        if (high) { // high level indicates full
            sysState.pureTank.status = TANK_HIGH_WATER;
            sysState.pureTank.ledState = LED_OFF;
            sysState.pureTank.needRefill = 0;
        } else if (stableLow) { // low level indicates lack of water
            sysState.pureTank.status = TANK_LOW_WATER;
            sysState.pureTank.ledState = LED_SLOW_BLINK;
            sysState.pureTank.needRefill = 1;
        } else {
            sysState.pureTank.status = TANK_MID_WATER;
            sysState.pureTank.ledState = LED_OFF;
            if (sysState.pureTank.lastHighWater && !stableLow) {
                sysState.pureTank.highWaterBreak = 1;
                sysState.pureTank.dispenseTimer = 0;
            }
        }
        sysState.pureTank.lastHighWater = stableLow; // track last "low" stable value per original logic

        if (!lastPresent && present && sysState.pureTank.status != TANK_HIGH_WATER) {
            sysState.pureTank.needRefill = 1;
        }
    }

    if ((lastPresent && !present) || (!lastPresent && present)) {
        sysState.pureTank.changing = 1;
    }

    // 20 seconds water dispensing window after high->mid edge
    if (sysState.pureTank.highWaterBreak && FLAG_1s_For_SYS) {
        if (sysState.pureTank.dispenseTimer < 20) {
            sysState.pureTank.dispenseTimer++;
        } else {
            sysState.pureTank.needRefill = 1;
            sysState.pureTank.highWaterBreak = 0;
        }
    }
}

static void UpdateRawTankState(void) {
    // Input pull-up: low=closed/active
    uint8_t present = (GPIO_ReadInputDataBit(GPIOB, HIGH_SWITCH_PIN) == Bit_RESET);
    uint8_t water   = (GPIO_ReadInputDataBit(GPIOB, LOW_SWITCH_PIN)  == Bit_RESET);

    uint8_t lastPresent = sysState.rawTank.lastPresent;
    sysState.rawTank.lastPresent = present;

    if (present) {
        sysState.rawTank.status = water ? TANK_MID_WATER : TANK_LOW_WATER;
    } else {
        sysState.rawTank.status = TANK_ABSENT;
    }

    if (lastPresent && !present) {
        sysState.rawTank.changing = 1;
        sysState.rawTank.changed = 1;
        sysState.rawTank.refillDelay = 5;
        sysState.rawTank.ledState = LED_FAST_BLINK;
    } else if (!lastPresent && present) {
        sysState.rawTank.changing = 1;
        sysState.rawTank.changed = 1;
        sysState.rawTank.refillDelay = 5;
        sysState.rawTank.ledState = LED_FAST_BLINK;
    }

    if (sysState.rawTank.refillDelay > 0 && FLAG_1s_For_SYS) {
        sysState.rawTank.refillDelay--;
        if (sysState.rawTank.refillDelay == 0) {
            sysState.rawTank.changing = 0;
            if (sysState.rawTank.status == TANK_LOW_WATER) {
                sysState.rawTank.ledState = LED_SLOW_BLINK;
            } else {
                sysState.rawTank.ledState = LED_OFF;
            }
        }
    }
}

static void UpdateMineralTankState(void) {
    // Input pull-up: low=closed/active
    uint8_t low  = (GPIO_ReadInputDataBit(GPIOA, LOW_SW_PIN)  == Bit_RESET);
    uint8_t high = (GPIO_ReadInputDataBit(GPIOC, HIGH_SW_PIN) == Bit_RESET);

    if (high) {
        sysState.mineralTank.status = TANK_HIGH_WATER;
        sysState.mineralTank.needRefill = 0;
    } else if (low) {
        sysState.mineralTank.status = TANK_LOW_WATER;
        sysState.mineralTank.needRefill = 1;
    } else {
        sysState.mineralTank.status = TANK_MID_WATER;
        sysState.mineralTank.needRefill = 0;
    }
}

// =====================================================================================
// Actuators control
// =====================================================================================

static void StartPureMaking(void) {
    sysState.machineMode = MODE_PURE_MAKING;
    sysState.modeTimer = 0;
    Wash_Init();

    GPIO_SetBits(GPIOA, PURE_VALVE_PIN | PUMP_EN_PIN);
}

static void StartMineralMaking(void) {
    sysState.machineMode = MODE_MINERAL_MAKING;
    sysState.modeTimer = 0;
    Wash_Init();

    GPIO_SetBits(GPIOA, MINERAL_VALVE_PIN | PUMP_EN_PIN);
}

static void StopWaterMaking(uint8_t isError) {
    GPIO_ResetBits(GPIOA, PURE_VALVE_PIN | MINERAL_VALVE_PIN | PUMP_EN_PIN);
    GPIO_ResetBits(GPIOC, FLUSH_VALVE_PIN);

    if (isError) {
        GPIO_SetBits(GPIOA, BUZZ_PIN);
        uint8_t cnt = 0;
        while (cnt < 20) {
            if (FLAG_10ms_For_SYS) {
                FLAG_10ms_For_SYS = 0;
                cnt++;
            }
        }
        GPIO_ResetBits(GPIOA, BUZZ_PIN);
    }

    sysState.machineMode = MODE_IDLE;
    sysState.modeTimer = 0;
}

static void StartReverseFlow(void) {
    sysState.machineMode = MODE_REVERSE_FLOW;
    sysState.modeTimer = 0;
    Wash_Init();

    GPIO_SetBits(GPIOA, PUMP_EN_PIN);
    GPIO_SetBits(GPIOC, FLUSH_VALVE_PIN);
}

static void StartMineralizing(void) {
    sysState.machineMode = MODE_MINERALIZING;
    sysState.mineralTank.mineralizing = 1;
    sysState.mineralTank.mineralTimer = 0;

    // close water-making valves
    GPIO_ResetBits(GPIOA, PURE_VALVE_PIN | MINERAL_VALVE_PIN);
    GPIO_ResetBits(GPIOC, FLUSH_VALVE_PIN);

    // Optional: enable mineralization hardware
#ifdef MINERALIZE_CTRL_PIN
    GPIO_SetBits(GPIOC, MINERALIZE_CTRL_PIN);
#endif
}

static void StopMineralizing(void) {
    sysState.mineralTank.mineralizing = 0;
#ifdef MINERALIZE_CTRL_PIN
    GPIO_ResetBits(GPIOC, MINERALIZE_CTRL_PIN);
#endif
    sysState.machineMode = MODE_IDLE;
}

static void ControlLeds(void) {
    // Hook to update LEDs based on sysState.rawTank.ledState and sysState.pureTank.ledState
    // Left unimplemented; map to GPIOs as needed
    (void)sysState;
}

static void EnterSleepMode(void) {
    sysState.machineMode = MODE_SLEEP;
    __WFI();
}

static void ExitSleepMode(void) {
    sysState.machineMode = MODE_IDLE;
    sysState.idleTimer = 0;
}

// =====================================================================================
// Main state machine
// =====================================================================================

static void ProcessMachineState(void) {
    // 6 hours no-making detection based on 100ms ticks
    if (FLAG_100ms_For_SYS) {
        sysState.noMakingTimer++;
        if (sysState.noMakingTimer >= 216000UL) { // 216000 * 100ms = 6h
            sysState.reverseFlowNeeded = 1;
            sysState.noMakingTimer = 0;
        }
    }

    uint8_t canMakeWater = (
        sysState.rawTank.status != TANK_ABSENT &&
        sysState.rawTank.status != TANK_LOW_WATER &&
        sysState.rawTank.refillDelay == 0
    );

    switch (sysState.machineMode) {
        case MODE_IDLE:
            if (canMakeWater) {
                if (sysState.pureTank.needRefill) {
                    Wash_Start(WASH_TYPE_BEFORE_WATER);
                    sysState.machineMode = MODE_FLUSHING;
                } else if (sysState.mineralTank.needRefill) {
                    Wash_Start(WASH_TYPE_BEFORE_WATER);
                    sysState.machineMode = MODE_FLUSHING;
                } else if (sysState.reverseFlowNeeded) {
                    // optional: perform reverse flow even if tanks full
                    StartReverseFlow();
                    sysState.reverseFlowNeeded = 0;
                }
            }
            break;

        case MODE_FLUSHING:
            if (!Wash_IsActive()) {
                WashType_t washType = Wash_GetCurrentType();
                if (washType == WASH_TYPE_BEFORE_WATER) {
                    if (sysState.pureTank.needRefill && canMakeWater) {
                        StartPureMaking();
                    } else if (sysState.mineralTank.needRefill && canMakeWater) {
                        StartMineralMaking();
                    } else if (sysState.reverseFlowNeeded) {
                        StartReverseFlow();
                        sysState.reverseFlowNeeded = 0;
                    } else {
                        sysState.machineMode = MODE_IDLE;
                    }
                } else { // WASH_TYPE_AFTER_WATER
                    sysState.machineMode = MODE_IDLE;
                    if (sysState.needStartMineralizing) {
                        StartMineralizing();
                        sysState.needStartMineralizing = 0;
                    }
                }
            }
            break;

        case MODE_REVERSE_FLOW:
            if (FLAG_1s_For_SYS) {
                sysState.modeTimer++;
                if (sysState.modeTimer >= 30) { // 30s reverse flow
                    Wash_Start(WASH_TYPE_BEFORE_WATER);
                    sysState.machineMode = MODE_FLUSHING;
                    sysState.modeTimer = 0;
                }
            }
            break;

        case MODE_PURE_MAKING:
            if (FLAG_1s_For_SYS) sysState.modeTimer++;
            if (sysState.rawTank.status == TANK_ABSENT ||
               (sysState.rawTank.status == TANK_LOW_WATER && sysState.modeTimer >= 150)) {
                StopWaterMaking(1);
            } else if (sysState.pureTank.status == TANK_ABSENT) {
                if (sysState.mineralTank.needRefill && canMakeWater) {
                    StartMineralMaking();
                } else {
                    StopWaterMaking(1);
                }
            } else if (sysState.pureTank.status == TANK_HIGH_WATER && sysState.modeTimer >= 60) {
                StopWaterMaking(0);
                Wash_Start(WASH_TYPE_AFTER_WATER);
                sysState.machineMode = MODE_FLUSHING;
            }
            break;

        case MODE_MINERAL_MAKING:
            if (FLAG_1s_For_SYS) sysState.modeTimer++;
            if (sysState.rawTank.status == TANK_ABSENT ||
               (sysState.rawTank.status == TANK_LOW_WATER && sysState.modeTimer >= 150)) {
                StopWaterMaking(1);
            } else if (sysState.mineralTank.status == TANK_HIGH_WATER && sysState.modeTimer >= 30) {
                StopWaterMaking(0);
                Wash_Start(WASH_TYPE_AFTER_WATER);
                sysState.machineMode = MODE_FLUSHING;
                sysState.needStartMineralizing = 1; // start mineralization after post-flush
            }
            break;

        case MODE_MINERALIZING:
            if (FLAG_1s_For_SYS) {
                sysState.mineralTank.mineralTimer++;
                if (sysState.mineralTank.mineralTimer >= 300) { // 5 minutes
                    StopMineralizing();
                }
            }
            break;

        case MODE_SLEEP:
            // nothing
            break;
    }

    // Idle -> sleep management
    if (sysState.machineMode == MODE_IDLE && FLAG_1s_For_SYS) {
        sysState.idleTimer++;
        if (sysState.idleTimer >= 120) {
            EnterSleepMode();
        }
    }
}

// =====================================================================================
// Optional: Main loop sketch (disabled by default)
// =====================================================================================
#ifdef BUILD_STANDALONE_EXAMPLE
int main(void) {
    // TODO: Hardware init (GPIO, SysTick for time flags, etc.)

    System_StateInit();

    for (;;) {
        // Example scheduler: user must set FLAG_* from SysTick/Timer ISR
        if (FLAG_100ms_For_SYS) {
            FLAG_100ms_For_SYS = 0;
            UpdatePureTankState();
            UpdateRawTankState();
            UpdateMineralTankState();
            ControlLeds();
        }

        if (FLAG_1s_For_SYS) {
            FLAG_1s_For_SYS = 0;
            Wash_Process1s();
        }

        ProcessMachineState();

        if (sysState.machineMode == MODE_SLEEP) {
            __WFI();
        }
    }
}
#endif