#pragma once

#include <stdint.h>
#include "agent_state.h"

/**
 * @file voice_engine.h — C5 AI Agent Engine
 *
 * Multi-modal agent orchestrator:
 *   - WiFi cloud LLM → text response → TTS → speaker
 *   - UART text input from P4 (fallback when no mic)
 *   - Motor command relay to P4
 *   - Voice activity → ASR → LLM pipeline (when mic connected)
 *
 * All orchestration runs in a single FreeRTOS task (voice_agent).
 */

/**
 * @brief Initialize the agent engine and start the main task.
 *        Must be called after WiFi, I2S, TTS, UART are initialized.
 */
void voice_engine_init(void);

/**
 * @brief Get current agent state.
 */
agent_state_t voice_engine_get_state(void);

/**
 * @brief Manually trigger the agent (e.g., from a button or P4 command).
 */
void voice_engine_trigger(agent_trigger_t trigger);

/**
 * @brief Feed text input to the agent (when mic is unavailable).
 *        This bypasses ASR and sends text directly to the LLM.
 */
void voice_engine_feed_text(const char *text);

/**
 * @brief Trigger wake from external (touchscreen button, P4 wake event).
 *        Switches agent to LISTENING/recording state (same as wake word).
 */
void voice_engine_trigger_wake(void);
