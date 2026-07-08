#pragma once

/**
 * @file agent_state.h — Agent state machine definitions
 */

typedef enum {
    AGENT_WAITING,      /* listening for wake word "小智" */
    AGENT_IDLE,         /* idle (legacy, same as waiting) */
    AGENT_LISTENING,    /* capturing mic audio after wake */
    AGENT_THINKING,     /* LLM request in flight */
    AGENT_SPEAKING,     /* playing TTS response */
    AGENT_EXECUTING,    /* executing motor command on P4 */
    AGENT_ERROR,        /* error state, recoverable */
} agent_state_t;

typedef enum {
    TRIGGER_UART_CMD,   /* P4 sent a framed text command */
    TRIGGER_WAKE_WORD,  /* future: local wake word */
    TRIGGER_MANUAL,     /* button press or software */
} agent_trigger_t;
