//
// Yosys slang frontend — Loom multi-cycle FSM extractor
//
// Copyright 2024-2025 ETH Zurich and Loom contributors
// SPDX-License-Identifier: Apache-2.0
//
// Extracts synthesizable FSMs from simulation-style always blocks
// that contain inner @(posedge clk) timing controls.
//
#pragma once

#include "slang_frontend.h"

namespace slang_frontend {

// Check whether a statement subtree contains any inner timing controls
// (i.e. @(posedge clk) suspension points inside the body of an always block).
bool contains_inner_timing(const ast::Statement &stmt);

// A single FSM state with per-transition actions.
// Transitions carry the actions that execute when taking that transition,
// ensuring post-while statements execute in the SAME cycle as the condition.
struct FsmState {
	int id;
	struct Transition {
		const ast::Expression *condition; // nullptr = unconditional
		bool condition_polarity;          // true = advance when condition is true
		int target_state;
		std::vector<const ast::Statement *> actions;

		// Counter operations for repeat loops.
		// counter_load >= 0: load counter[id] with its init expression on this transition
		// counter_check >= 0: use counter[id] > 0 as the condition (ignores condition/polarity)
		// counter_dec >= 0: decrement counter[id] on this transition
		int counter_load = -1;
		int counter_check = -1;
		int counter_dec = -1;
	};
	std::vector<Transition> transitions;
};

// Result of analyzing a multi-cycle always block.
struct FsmGraph {
	std::vector<FsmState> states;

	// Counter registers needed for repeat() loops.
	struct CounterReg {
		const ast::Expression *init_expr; // the repeat count expression
	};
	std::vector<CounterReg> counters;
};

// Analyze a multi-cycle always block body and build a state graph.
FsmGraph analyze_multicycle_block(
	NetlistContext &netlist,
	const ast::SignalEventControl &outer_clock,
	const ast::Statement &sync_body);

// Emit the FSM as RTLIL: creates state register, combinational process
// with case tree on state, and DFFs for all driven signals.
void emit_multicycle_fsm(
	NetlistContext &netlist,
	const ast::ProceduralBlockSymbol &symbol,
	const ast::SignalEventControl &clock,
	const ast::StatementBlockSymbol *prologue_block,
	const ast::Statement &sync_body,
	ProcessTiming &timing);

} // namespace slang_frontend
