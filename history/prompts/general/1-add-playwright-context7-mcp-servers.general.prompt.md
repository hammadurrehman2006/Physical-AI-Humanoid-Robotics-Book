---
id: 1
title: Add Playwright and Context7 MCP Servers
stage: general
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Muhammad Hammad ur Rehman
command: claude mcp add commands
labels: [mcp, playwright, context7, configuration]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - /home/hammadurrehman2006/.claude.json
 - history/prompts/general/1-add-playwright-context7-mcp-servers.general.prompt.md
tests:
 - npx @playwright/mcp@latest --help
 - npx @upstash/context7-mcp --help
---

## Prompt

Add Playwright and Context7 MCP servers to Claude Code:
1. claude mcp add --transport stdio playwright npx @playwright/mcp@latest
2. claude mcp add --transport stdio context7 npx @upstash/context7-mcp

## Response snapshot

Successfully added both MCP servers to the Claude Code configuration in ~/.claude.json. The Playwright and Context7 servers are now configured for the project but need to be running to be accessible. Both packages were verified to be available via npx.

## Outcome

- ✅ Impact: MCP servers configured for Claude Code integration
- 🧪 Tests: Both packages verified with --help commands
- 📁 Files: ~/.claude.json updated with server configurations
- 🔁 Next prompts: Start the MCP servers to make them available for use
- 🧠 Reflection: Configuration was successful but servers need to be started to be accessible

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - Configuration completed successfully
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Start the MCP servers to make them accessible