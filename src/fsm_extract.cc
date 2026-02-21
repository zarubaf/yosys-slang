//
// Yosys slang frontend — Loom multi-cycle FSM extractor
//
// Copyright 2024-2025 ETH Zurich and Loom contributors
// SPDX-License-Identifier: Apache-2.0
//
// Extracts synthesizable FSMs from simulation-style always blocks
// that contain inner @(posedge clk) timing controls.
//
#include "slang/ast/ASTVisitor.h"
#include "slang/ast/TimingControl.h"
#include "slang/ast/Statement.h"
#include "slang/ast/statements/MiscStatements.h"
#include "slang/ast/statements/ConditionalStatements.h"
#include "slang/ast/statements/LoopStatements.h"
#include "slang/ast/expressions/MiscExpressions.h"
#include "slang/ast/symbols/BlockSymbols.h"
#include "slang/ast/symbols/VariableSymbols.h"

#include "kernel/rtlil.h"
#include "kernel/utils.h"

#include "fsm_extract.h"
#include "statements.h"
#include "diag.h"
#include "variables.h"

namespace slang_frontend {

// ============================================================================
// Phase 0: contains_inner_timing — AST walker to detect inner @(...) controls
// ============================================================================

namespace {

struct TimingDetector : public ast::ASTVisitor<TimingDetector, true, false>
{
	bool found = false;

	void handle(const ast::TimedStatement &) { found = true; }
	void handle(const ast::Statement &) {}
	void handle(const ast::BlockStatement &stmt) { stmt.body.visit(*this); }
	void handle(const ast::StatementList &list) {
		for (auto s : list.list) {
			s->visit(*this);
			if (found) return;
		}
	}
	void handle(const ast::ConditionalStatement &stmt) {
		stmt.ifTrue.visit(*this);
		if (!found && stmt.ifFalse) stmt.ifFalse->visit(*this);
	}
	void handle(const ast::WhileLoopStatement &stmt) { stmt.body.visit(*this); }
	void handle(const ast::RepeatLoopStatement &stmt) { stmt.body.visit(*this); }
	void handle(const ast::ForLoopStatement &stmt) { stmt.body.visit(*this); }
};

} // anonymous namespace

bool contains_inner_timing(const ast::Statement &stmt)
{
	TimingDetector det;
	stmt.visit(det);
	return det.found;
}

// ============================================================================
// Phase 1: analyze_multicycle_block — build FsmGraph from AST
// ============================================================================

namespace {

// Helper: check if a SignalEventControl matches the outer clock.
bool clock_matches(const ast::SignalEventControl &outer,
                   const ast::SignalEventControl &inner)
{
	if (outer.edge != inner.edge)
		return false;
	if (!ast::ValueExpressionBase::isKind(outer.expr.kind) ||
	    !ast::ValueExpressionBase::isKind(inner.expr.kind))
		return false;
	return &outer.expr.as<ast::ValueExpressionBase>().symbol ==
	       &inner.expr.as<ast::ValueExpressionBase>().symbol;
}

bool is_matching_clock(const ast::TimingControl &tc,
                       const ast::SignalEventControl &outer_clock)
{
	if (tc.kind != ast::TimingControlKind::SignalEvent)
		return false;
	return clock_matches(outer_clock, tc.as<ast::SignalEventControl>());
}

bool is_bare_clock_wait(const ast::Statement &stmt,
                        const ast::SignalEventControl &outer_clock)
{
	if (stmt.kind != ast::StatementKind::Timed)
		return false;
	auto &timed = stmt.as<ast::TimedStatement>();
	if (!is_matching_clock(timed.timing, outer_clock))
		return false;
	return timed.stmt.kind == ast::StatementKind::Empty;
}

// State graph builder with per-transition action tracking.
//
// The key invariant: "pending actions" always point to the action list of
// the transition currently being built. When we encounter a state boundary
// (@posedge clk or while-wait), we finalize that transition and start a new one.
struct FsmBuilder {
	NetlistContext &netlist;
	const ast::SignalEventControl &outer_clock;
	FsmGraph &graph;
	bool error = false;

	// Current state being built
	int current_state;
	// Buffer for actions being collected for the current (not yet finalized) transition
	std::vector<const ast::Statement *> action_buffer;

	FsmBuilder(NetlistContext &netlist, const ast::SignalEventControl &outer_clock, FsmGraph &graph)
		: netlist(netlist), outer_clock(outer_clock), graph(graph), current_state(0) {}

	int new_state() {
		int id = (int)graph.states.size();
		graph.states.push_back({id, {}});
		return id;
	}

	// Flatten block/list statements into a sequence of child statements.
	std::vector<const ast::Statement *> flatten_block(const ast::Statement &stmt) {
		std::vector<const ast::Statement *> result;
		const ast::Statement *s = &stmt;
		while (s->kind == ast::StatementKind::Block)
			s = &s->as<ast::BlockStatement>().body;
		if (s->kind == ast::StatementKind::List) {
			for (auto child : s->as<ast::StatementList>().list)
				result.push_back(child);
		} else {
			result.push_back(s);
		}
		return result;
	}

	// Check if the current state has a pending (incomplete) advance transition
	// from a while-wait (target_state == -1).
	bool has_pending_advance() {
		auto &transitions = graph.states[current_state].transitions;
		if (transitions.empty()) return false;
		return transitions.back().target_state == -1;
	}

	// Finalize a transition to target_state.
	// If there's a pending advance transition from a while-wait, complete it.
	// Otherwise, create a new unconditional transition.
	void finalize_to(int target_state) {
		if (has_pending_advance()) {
			auto &adv_tr = graph.states[current_state].transitions.back();
			adv_tr.target_state = target_state;
			adv_tr.actions = std::move(action_buffer);
		} else {
			graph.states[current_state].transitions.push_back(
				{nullptr, true, target_state, std::move(action_buffer)});
		}
		action_buffer.clear();
		current_state = target_state;
	}

	void process_statements(const std::vector<const ast::Statement *> &stmts);
	void process_statement(const ast::Statement &stmt);
};

void FsmBuilder::process_statement(const ast::Statement &stmt)
{
	if (error) return;

	switch (stmt.kind) {
	case ast::StatementKind::Block: {
		auto children = flatten_block(stmt);
		process_statements(children);
		return;
	}

	case ast::StatementKind::List: {
		auto &list = stmt.as<ast::StatementList>();
		std::vector<const ast::Statement *> stmts(list.list.begin(), list.list.end());
		process_statements(stmts);
		return;
	}

	case ast::StatementKind::Timed: {
		auto &timed = stmt.as<ast::TimedStatement>();
		if (!is_matching_clock(timed.timing, outer_clock)) {
			netlist.add_diag(diag::FsmClockMismatch, timed.timing.sourceRange);
			error = true;
			return;
		}

		// State boundary: finalize current transition, create next state
		int next_state = new_state();
		finalize_to(next_state);

		// If the timed statement has a non-empty body, it runs in the new state
		if (timed.stmt.kind != ast::StatementKind::Empty) {
			action_buffer.push_back(&timed.stmt);
		}
		return;
	}

	case ast::StatementKind::WhileLoop: {
		auto &loop = stmt.as<ast::WhileLoopStatement>();

		if (!contains_inner_timing(loop.body)) {
			// Regular synthesis loop — treat as action
			action_buffer.push_back(&stmt);
			return;
		}

		// while(!cond) @(posedge clk); pattern.
		// The while body should contain a @(posedge clk) with optional
		// pre-wait and post-wait statements.

		auto body_stmts = flatten_block(loop.body);

		// Validate: find the @(posedge clk) in the body
		bool found_wait = false;
		for (auto s : body_stmts) {
			if (s->kind == ast::StatementKind::Timed) {
				auto &timed = s->as<ast::TimedStatement>();
				if (!is_matching_clock(timed.timing, outer_clock)) {
					netlist.add_diag(diag::FsmClockMismatch, timed.timing.sourceRange);
					error = true;
					return;
				}
				found_wait = true;
			}
		}

		if (!found_wait) {
			netlist.add_diag(diag::FsmUnsupportedPattern, loop.sourceRange);
			error = true;
			return;
		}

		// Create the wait state.
		// Optimization: if the current state has no pending actions and no
		// existing transitions (e.g., it was just created by a preceding
		// @(posedge clk)), reuse it as the wait state instead of creating
		// an empty pass-through state.
		int wait_state;
		if (action_buffer.empty() &&
		    graph.states[current_state].transitions.empty() &&
		    !has_pending_advance()) {
			wait_state = current_state;
		} else {
			wait_state = new_state();
			finalize_to(wait_state);
		}

		// The wait state has two transitions:
		// 1. condition false (stay): execute while body's pre-wait actions, loop back
		// 2. condition true (advance): collect subsequent statements until next @(posedge clk)

		// Collect pre-wait actions from while body (statements before @(posedge clk))
		std::vector<const ast::Statement *> stay_actions;
		for (auto s : body_stmts) {
			if (s->kind == ast::StatementKind::Timed)
				break; // stop at the timing control
			stay_actions.push_back(s);
		}

		// Stay transition: while condition TRUE → keep looping
		graph.states[wait_state].transitions.push_back(
			{&loop.cond, true, wait_state, std::move(stay_actions)});

		// Advance transition: while condition FALSE → exit loop.
		// Placeholder with target=-1; subsequent statements accumulate in
		// action_buffer and finalize_to() completes the target.
		graph.states[wait_state].transitions.push_back(
			{&loop.cond, false, -1, {}});

		current_state = wait_state;

		return;
	}

	case ast::StatementKind::RepeatLoop: {
		auto &loop = stmt.as<ast::RepeatLoopStatement>();
		if (!contains_inner_timing(loop.body)) {
			action_buffer.push_back(&stmt);
			return;
		}

		// repeat(N) { body; @(posedge clk); } → counter-based wait state.
		// Validate: body must contain exactly one @(posedge clk).
		auto body_stmts = flatten_block(loop.body);
		bool found_wait = false;
		for (auto s : body_stmts) {
			if (s->kind == ast::StatementKind::Timed) {
				auto &timed = s->as<ast::TimedStatement>();
				if (!is_matching_clock(timed.timing, outer_clock)) {
					netlist.add_diag(diag::FsmClockMismatch, timed.timing.sourceRange);
					error = true;
					return;
				}
				found_wait = true;
			}
		}
		if (!found_wait) {
			netlist.add_diag(diag::FsmUnsupportedPattern, loop.sourceRange);
			error = true;
			return;
		}

		// Allocate a counter register
		int ctr_id = (int)graph.counters.size();
		graph.counters.push_back({&loop.count});

		// Create the repeat state. Always create a new state so the
		// entry transition can load the counter register.
		int repeat_state = new_state();
		{
			// Entry transition loads the counter
			if (has_pending_advance()) {
				auto &adv_tr = graph.states[current_state].transitions.back();
				adv_tr.target_state = repeat_state;
				adv_tr.actions = std::move(action_buffer);
				adv_tr.counter_load = ctr_id;
			} else {
				FsmState::Transition tr;
				tr.condition = nullptr;
				tr.condition_polarity = true;
				tr.target_state = repeat_state;
				tr.actions = std::move(action_buffer);
				tr.counter_load = ctr_id;
				graph.states[current_state].transitions.push_back(std::move(tr));
			}
			action_buffer.clear();
			current_state = repeat_state;
		}

		// Collect pre-wait actions from body (before @(posedge clk))
		std::vector<const ast::Statement *> stay_actions;
		for (auto s : body_stmts) {
			if (s->kind == ast::StatementKind::Timed)
				break;
			stay_actions.push_back(s);
		}

		// Stay transition: counter > 0 → execute body, decrement, loop back
		{
			FsmState::Transition tr;
			tr.condition = nullptr;
			tr.condition_polarity = true;
			tr.target_state = repeat_state;
			tr.actions = std::move(stay_actions);
			tr.counter_check = ctr_id;  // condition: counter > 0
			tr.counter_dec = ctr_id;    // decrement counter
			graph.states[repeat_state].transitions.push_back(std::move(tr));
		}

		// Advance transition: counter == 0 → exit
		{
			FsmState::Transition tr;
			tr.condition = nullptr;
			tr.condition_polarity = false;
			tr.target_state = -1;  // TBD
			tr.counter_check = ctr_id;  // same counter, but this is the "else" branch
			graph.states[repeat_state].transitions.push_back(std::move(tr));
		}

		current_state = repeat_state;
		return;
	}

	case ast::StatementKind::Conditional: {
		auto &cond = stmt.as<ast::ConditionalStatement>();
		bool true_has_timing = contains_inner_timing(cond.ifTrue);
		bool false_has_timing = cond.ifFalse && contains_inner_timing(*cond.ifFalse);

		if (!true_has_timing && !false_has_timing) {
			// No timing in either branch — regular action
			action_buffer.push_back(&stmt);
			return;
		}

		if (false_has_timing) {
			netlist.add_diag(diag::FsmUnsupportedPattern, cond.sourceRange);
			error = true;
			return;
		}

		// Only true branch has timing.
		// Create a conditional fork: true path processes the body with
		// sub-states, false path skips to the join point.

		// Move to the condition-check state
		int cond_state;
		if (action_buffer.empty() &&
		    graph.states[current_state].transitions.empty() &&
		    !has_pending_advance()) {
			cond_state = current_state;
		} else {
			cond_state = new_state();
			finalize_to(cond_state);
		}

		// Create a join state (where both paths converge after the if)
		int join_state = new_state();

		// False path: skip to join (no actions, or else-body without timing)
		{
			FsmState::Transition tr;
			tr.condition = cond.conditions[0].expr;
			tr.condition_polarity = false; // taken when cond is false
			tr.target_state = join_state;
			if (cond.ifFalse) {
				auto else_stmts = flatten_block(*cond.ifFalse);
				for (auto s : else_stmts)
					tr.actions.push_back(s);
			}
			graph.states[cond_state].transitions.push_back(std::move(tr));
		}

		// True path: use pending-advance pattern so the true body's
		// first state boundary (e.g. repeat or @posedge) completes
		// the transition directly from cond_state — no extra cycle.
		{
			FsmState::Transition tr;
			tr.condition = cond.conditions[0].expr;
			tr.condition_polarity = true; // taken when cond is true
			tr.target_state = -1;         // TBD by finalize_to
			graph.states[cond_state].transitions.push_back(std::move(tr));
		}

		// Process true body — accumulate actions, finalize_to will
		// complete the pending true transition when a state boundary is hit.
		current_state = cond_state;
		action_buffer.clear();
		auto true_stmts = flatten_block(cond.ifTrue);
		process_statements(true_stmts);
		if (error) return;

		// Finalize true path to the join state
		finalize_to(join_state);

		current_state = join_state;
		// action_buffer is already clear — post-if statements accumulate here
		return;
	}

	default:
		action_buffer.push_back(&stmt);
		return;
	}
}

void FsmBuilder::process_statements(const std::vector<const ast::Statement *> &stmts)
{
	for (auto s : stmts) {
		process_statement(*s);
		if (error) return;
	}
}

} // anonymous namespace

FsmGraph analyze_multicycle_block(
	NetlistContext &netlist,
	const ast::SignalEventControl &outer_clock,
	const ast::Statement &sync_body)
{
	FsmGraph graph;
	FsmBuilder builder(netlist, outer_clock, graph);

	// State 0 = the entry point (outer @(posedge clk))
	builder.new_state();

	auto stmts = builder.flatten_block(sync_body);
	builder.process_statements(stmts);

	if (!builder.error) {
		// Finalize: remaining actions loop back to S0
		int final_state = builder.current_state;
		auto &final_actions = builder.action_buffer;

		// Check if the final state has an incomplete advance transition
		// (from a while-wait that was the last thing in the block)
		bool has_incomplete_advance = false;
		if (!graph.states[final_state].transitions.empty()) {
			auto &last_tr = graph.states[final_state].transitions.back();
			if (last_tr.target_state == -1) {
				has_incomplete_advance = true;
			}
		}

		if (has_incomplete_advance) {
			// Complete the advance transition: target = S0, actions = remaining
			auto &adv_tr = graph.states[final_state].transitions.back();
			adv_tr.target_state = 0;
			adv_tr.actions = std::move(final_actions);
		} else {
			// Simple unconditional loop-back
			graph.states[final_state].transitions.push_back(
				{nullptr, true, 0, std::move(final_actions)});
		}
	}

	// Fix up any remaining -1 targets
	for (auto &state : graph.states) {
		for (auto &tr : state.transitions) {
			if (tr.target_state == -1)
				tr.target_state = 0;
		}
	}

	// Eliminate empty pass-through states: a state with exactly one
	// unconditional transition, no actions, and no counter ops is
	// redundant — redirect all transitions targeting it to its target.
	bool changed = true;
	while (changed) {
		changed = false;
		for (int si = 0; si < (int)graph.states.size(); si++) {
			auto &s = graph.states[si];
			if (s.transitions.size() != 1) continue;
			auto &tr = s.transitions[0];
			if (tr.condition != nullptr) continue;
			if (!tr.actions.empty()) continue;
			if (tr.counter_load >= 0 || tr.counter_check >= 0 || tr.counter_dec >= 0) continue;
			int bypass_target = tr.target_state;
			if (bypass_target == si) continue; // self-loop, not a pass-through
			// Redirect all references to si → bypass_target
			for (auto &s2 : graph.states) {
				for (auto &tr2 : s2.transitions) {
					if (tr2.target_state == si) {
						tr2.target_state = bypass_target;
						changed = true;
					}
				}
			}
		}
	}

	return graph;
}

// ============================================================================
// Phase 2: emit_multicycle_fsm — emit RTLIL for the FSM
// ============================================================================

void emit_multicycle_fsm(
	NetlistContext &netlist,
	const ast::ProceduralBlockSymbol &symbol,
	const ast::SignalEventControl &clock,
	const ast::StatementBlockSymbol *prologue_block,
	const ast::Statement &sync_body,
	ProcessTiming &timing)
{
	FsmGraph graph = analyze_multicycle_block(netlist, clock, sync_body);
	if (graph.states.empty())
		return;

	int num_states = (int)graph.states.size();
	int state_width = std::max(1, ceil_log2(num_states));

	log("Loom FSM: extracted %d states, %d counters\n",
	    num_states, (int)graph.counters.size());

	// --- Create state register ---
	RTLIL::Wire *state_wire = netlist.canvas->addWire(
		netlist.new_id("loom_fsm_state"), state_width);
	RTLIL::Wire *state_next = netlist.canvas->addWire(
		netlist.new_id("loom_fsm_state_next"), state_width);
	state_wire->attributes[ID::init] = RTLIL::Const(0, state_width);
	netlist.add_dff(
		netlist.canvas->uniquify("$loom_fsm_state_dff"),
		timing.triggers[0].signal, state_next, state_wire,
		timing.triggers[0].edge_polarity);

	// --- Create counter registers ---
	// Each counter has a current-value wire (DFF Q) and a next-value wire (DFF D).
	std::vector<std::pair<RTLIL::Wire *, RTLIL::Wire *>> ctr_wires;
	for (int ci = 0; ci < (int)graph.counters.size(); ci++) {
		// Determine width from the count expression type
		int ctr_width = graph.counters[ci].init_expr->type->getBitstreamWidth();
		if (ctr_width < 1) ctr_width = 32;

		auto *ctr_cur = netlist.canvas->addWire(
			netlist.new_id(Yosys::stringf("loom_fsm_ctr%d", ci)), ctr_width);
		auto *ctr_nxt = netlist.canvas->addWire(
			netlist.new_id(Yosys::stringf("loom_fsm_ctr%d_next", ci)), ctr_width);
		ctr_cur->attributes[ID::init] = RTLIL::Const(0, ctr_width);
		netlist.add_dff(
			netlist.canvas->uniquify(Yosys::stringf("$loom_fsm_ctr%d_dff", ci)),
			timing.triggers[0].signal, ctr_nxt, ctr_cur,
			timing.triggers[0].edge_polarity);
		ctr_wires.push_back({ctr_cur, ctr_nxt});
	}

	// --- Build unified ProceduralContext with state switch ---
	ProceduralContext fsm_ctx(netlist, timing);
	EnterAutomaticScopeGuard scope_guard(fsm_ctx.eval, prologue_block);

	StatementExecutor::SwitchHelper state_sw(
		fsm_ctx.current_case, fsm_ctx.vstate, state_wire);

	for (int si = 0; si < num_states; si++) {
		auto &state = graph.states[si];
		state_sw.enter_branch({RTLIL::Const(si, state_width)});

		if (state.transitions.size() == 1 &&
		    state.transitions[0].condition == nullptr &&
		    state.transitions[0].counter_check < 0) {
			// Unconditional transition (no condition, no counter check)
			auto &tr = state.transitions[0];

			// Execute actions unconditionally
			for (auto action : tr.actions)
				action->visit(StatementExecutor(fsm_ctx));

			if (tr.counter_load >= 0) {
				// Repeat entry with zero-bypass.
				// count > 0: load counter = count - 1, enter repeat state (N cycles)
				// count == 0: bypass repeat state, emit exit actions (0 cycles)
				log_assert(tr.counter_load < (int)ctr_wires.size());
				log_assert(tr.counter_load < (int)graph.counters.size());
				auto [ctr_cur, ctr_nxt] = ctr_wires[tr.counter_load];
				RTLIL::SigSpec load_val = fsm_ctx.eval(
					*graph.counters[tr.counter_load].init_expr);
				RTLIL::SigSpec count_nonzero = netlist.ReduceBool(load_val);

				// Find repeat state's exit transition (counter_check but no counter_dec)
				const FsmState::Transition *exit_tr = nullptr;
				for (auto &rt : graph.states[tr.target_state].transitions)
					if (rt.counter_check >= 0 && rt.counter_dec < 0) {
						exit_tr = &rt;
						break;
					}
				log_assert(exit_tr != nullptr);
				log_assert(exit_tr->target_state >= 0);

				StatementExecutor::SwitchHelper bypass_sw(
					fsm_ctx.current_case, fsm_ctx.vstate, count_nonzero);

				bypass_sw.enter_branch({RTLIL::S1});
				RTLIL::SigSpec dec_val = netlist.Biop(
					ID($sub), load_val, RTLIL::Const(1, ctr_nxt->width),
					false, false, ctr_nxt->width);
				fsm_ctx.current_case->aux_actions.push_back(
					RTLIL::SigSig(ctr_nxt, dec_val));
				fsm_ctx.current_case->aux_actions.push_back(
					RTLIL::SigSig(state_next,
						RTLIL::Const(tr.target_state, state_width)));
				bypass_sw.exit_branch();

				bypass_sw.enter_branch({});
				// Emit exit transition's actions (post-repeat statements)
				for (auto action : exit_tr->actions)
					action->visit(StatementExecutor(fsm_ctx));
				fsm_ctx.current_case->aux_actions.push_back(
					RTLIL::SigSig(state_next,
						RTLIL::Const(exit_tr->target_state, state_width)));
				bypass_sw.exit_branch();

				bypass_sw.finish(netlist);
			} else {
				fsm_ctx.current_case->aux_actions.push_back(
					RTLIL::SigSig(state_next, RTLIL::Const(tr.target_state, state_width)));
			}

		} else if (state.transitions.size() == 2) {
			// Two-way conditional state.
			auto &tr0 = state.transitions[0];
			auto &tr1 = state.transitions[1];

			// Both transitions must reference the same condition (or counter)
			log_assert(tr0.condition == tr1.condition);
			log_assert(tr0.counter_check == tr1.counter_check);
			// Target states must be valid
			log_assert(tr0.target_state >= 0 && tr0.target_state < num_states);
			log_assert(tr1.target_state >= 0 && tr1.target_state < num_states);

			// Determine the condition signal for the switch
			RTLIL::SigSpec cond_sig;
			if (tr0.counter_check >= 0) {
				// Counter-based: switch on counter > 0
				log_assert(tr0.counter_check < (int)ctr_wires.size());
				auto [ctr_cur, ctr_nxt] = ctr_wires[tr0.counter_check];
				cond_sig = netlist.ReduceBool(ctr_cur);
			} else {
				log_assert(tr0.condition != nullptr);
				cond_sig = netlist.ReduceBool(
					fsm_ctx.eval(*tr0.condition));
			}

			StatementExecutor::SwitchHelper cond_sw(
				fsm_ctx.current_case, fsm_ctx.vstate, cond_sig);

			// Helper lambda to emit one branch
			auto emit_branch = [&](const FsmState::Transition &tr,
			                       std::vector<RTLIL::SigSpec> compare) {
				cond_sw.enter_branch(compare);

				// Execute actions first (unconditionally within this branch)
				for (auto action : tr.actions)
					action->visit(StatementExecutor(fsm_ctx));

				if (tr.counter_load >= 0) {
					// Repeat entry with zero-bypass.
					log_assert(tr.counter_load < (int)ctr_wires.size());
					log_assert(tr.counter_load < (int)graph.counters.size());
					auto [ctr_cur, ctr_nxt] = ctr_wires[tr.counter_load];
					RTLIL::SigSpec load_val = fsm_ctx.eval(
						*graph.counters[tr.counter_load].init_expr);
					RTLIL::SigSpec count_nonzero = netlist.ReduceBool(load_val);

					// Find repeat state's exit transition
					const FsmState::Transition *exit_tr = nullptr;
					for (auto &rt : graph.states[tr.target_state].transitions)
						if (rt.counter_check >= 0 && rt.counter_dec < 0) {
							exit_tr = &rt;
							break;
						}
					log_assert(exit_tr != nullptr);
					log_assert(exit_tr->target_state >= 0);

					StatementExecutor::SwitchHelper bypass_sw(
						fsm_ctx.current_case, fsm_ctx.vstate, count_nonzero);

					bypass_sw.enter_branch({RTLIL::S1});
					RTLIL::SigSpec dec_val = netlist.Biop(
						ID($sub), load_val, RTLIL::Const(1, ctr_nxt->width),
						false, false, ctr_nxt->width);
					fsm_ctx.current_case->aux_actions.push_back(
						RTLIL::SigSig(ctr_nxt, dec_val));
					fsm_ctx.current_case->aux_actions.push_back(
						RTLIL::SigSig(state_next,
							RTLIL::Const(tr.target_state, state_width)));
					bypass_sw.exit_branch();

					bypass_sw.enter_branch({});
					// Emit exit transition's actions (post-repeat statements)
					for (auto action : exit_tr->actions)
						action->visit(StatementExecutor(fsm_ctx));
					fsm_ctx.current_case->aux_actions.push_back(
						RTLIL::SigSig(state_next,
							RTLIL::Const(exit_tr->target_state, state_width)));
					bypass_sw.exit_branch();

					bypass_sw.finish(netlist);
				} else {
					if (tr.counter_dec >= 0) {
						log_assert(tr.counter_dec < (int)ctr_wires.size());
						auto [ctr_cur, ctr_nxt] = ctr_wires[tr.counter_dec];
						RTLIL::SigSpec dec = netlist.Biop(
							ID($sub), ctr_cur, RTLIL::Const(1, ctr_cur->width),
							false, false, ctr_cur->width);
						fsm_ctx.current_case->aux_actions.push_back(
							RTLIL::SigSig(ctr_nxt, dec));
					}
					fsm_ctx.current_case->aux_actions.push_back(
						RTLIL::SigSig(state_next,
							RTLIL::Const(tr.target_state, state_width)));
				}

				cond_sw.exit_branch();
			};

			// Determine match values for each branch.
			// tr0 is the "explicit match" branch, tr1 is the "default".
			RTLIL::SigSpec match0;
			if (tr0.counter_check >= 0) {
				// For counter: tr0 (stay) matches when counter > 0 (cond_sig=1)
				// tr1 (advance) is the default (counter == 0)
				match0 = RTLIL::S1;
			} else {
				match0 = tr0.condition_polarity ? RTLIL::S1 : RTLIL::S0;
			}

			emit_branch(tr0, {match0});
			emit_branch(tr1, {});  // default branch

			cond_sw.finish(netlist);
		} else {
			log_error("Loom FSM: state S%d has unsupported transition count %zu\n",
			          si, state.transitions.size());
		}

		state_sw.exit_branch();
	}

	state_sw.finish(netlist);

	// --- Copy case tree into RTLIL process ---
	RTLIL::Process *proc = netlist.canvas->addProcess(netlist.new_id("loom_fsm"));
	transfer_attrs(netlist, symbol.getBody().as<ast::TimedStatement>().stmt, proc);

	// Defaults: hold state and counter values
	proc->root_case.actions.push_back(RTLIL::SigSig(state_next, state_wire));
	for (auto [ctr_cur, ctr_nxt] : ctr_wires)
		proc->root_case.actions.push_back(RTLIL::SigSig(ctr_nxt, ctr_cur));

	fsm_ctx.copy_case_tree_into(proc->root_case);

	// --- Create DFFs for driven signals ---
	VariableBits driven = fsm_ctx.all_driven();
	for (VariableChunk driven_chunk : driven.chunks()) {
		const ast::Type *type = &driven_chunk.variable.get_symbol()->getType();
		RTLIL::SigSpec assigned = fsm_ctx.vstate.evaluate(netlist, driven_chunk);

		for (auto [named_chunk, name] : generate_subfield_names(driven_chunk, type)) {
			log_assert(named_chunk.variable.get_symbol() != nullptr);
			std::string base_name = Yosys::stringf("$loom_fsm$%s%s",
				RTLIL::unescape_id(netlist.id(*named_chunk.variable.get_symbol())).c_str(),
				name.c_str());

			netlist.add_dff(netlist.canvas->uniquify(base_name),
				timing.triggers[0].signal,
				assigned.extract(named_chunk.base - driven_chunk.base,
					named_chunk.bitwidth()),
				netlist.convert_static(named_chunk),
				timing.triggers[0].edge_polarity);
		}
	}
}

} // namespace slang_frontend
