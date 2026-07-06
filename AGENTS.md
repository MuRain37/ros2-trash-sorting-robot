# AGENTS.md

Project-level instructions for coding agents working in this repository.

These guidelines reduce common LLM coding mistakes and should be merged with task-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Do not assume. Do not hide confusion. Surface tradeoffs.**

Before implementing:
- State assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them instead of choosing silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop, name what is confusing, and ask.

## 2. Simplicity First

**Use the minimum code that solves the problem. Do not add speculative work.**

- Add no features beyond what was requested.
- Add no abstractions for single-use code.
- Add no flexibility or configurability that was not requested.
- Add no error handling for impossible scenarios.
- If a solution is much longer than necessary, simplify it.

Ask: "Would a senior engineer say this is overcomplicated?"

If yes, simplify.

## 3. Surgical Changes

**Touch only what must change. Clean up only your own changes.**

When editing existing code:
- Do not improve adjacent code, comments, or formatting unless needed for the task.
- Do not refactor code that is not broken.
- Match the existing style, even if you would choose a different one.
- If you notice unrelated dead code, mention it instead of deleting it.

When your changes create unused code:
- Remove imports, variables, functions, or files made unused by your changes.
- Do not remove pre-existing dead code unless explicitly asked.

Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" -> write tests for invalid inputs, then make them pass.
- "Fix the bug" -> write a test that reproduces it, then make it pass.
- "Refactor X" -> ensure tests pass before and after.

For multi-step tasks, state a brief plan:
1. Step -> verify: check.
2. Step -> verify: check.
3. Step -> verify: check.

Strong success criteria allow independent progress. Weak criteria such as "make it work" require clarification.
